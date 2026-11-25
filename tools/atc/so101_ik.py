"""Inverse kinematics for the SO-ARM101 (5 DOF), for move_bit.

The motion planner's generic kinematic_chain IK does not converge for this
5-DOF arm away from its seed, and its constraint-based IK only supports known
robots. So move_bit solves the SO-101's IK here and sends joint targets, which
the planner still plans and collision-checks.

Kinematics: TheRobotStudio so101_new_calib.urdf (Apache-2.0), the same joints
as models/so101/build_so101.py. The tool frame is the URDF gripper_frame_link
(= so101_gripper.tool_frame). Poses are 4x4 matrices in the arm's base frame
(the so101_arm object frame).
"""

import numpy as np

# (origin xyz, origin rpy, (lower, upper)) per revolute joint, base to wrist.
JOINTS = (
  ((0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159), (-1.91986, 1.91986)),
  ((-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0), (-1.74533, 1.74533)),
  ((-0.11257, -0.028, 0.0), (0.0, 0.0, 1.5708), (-1.69, 1.69)),
  ((-0.1349, 0.0052, 0.0), (0.0, 0.0, -1.5708), (-1.65806, 1.65806)),
  ((0.0, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), (-2.74385, 2.84121)),
)
TIP = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))
# Named configuration "home" in models/so101/so101_limits.updates.pbtxt: tool
# 13 cm above the cart, clear of the Carvera and the grid.
HOME = (0.6, -1.1, 0.5, 1.6, 0.0)
# Application limits in so101_limits.updates.pbtxt are 99% of the range; stay
# a little inside them.
LIMIT_SCALE = 0.985


def _rpy(r: float, p: float, y: float) -> np.ndarray:
  cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
  return np.array([
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ])


def _tf(xyz, rpy) -> np.ndarray:
  m = np.eye(4)
  m[:3, :3] = _rpy(*rpy)
  m[:3, 3] = xyz
  return m


def _rz(q: float) -> np.ndarray:
  m = np.eye(4)
  m[:2, :2] = [[np.cos(q), -np.sin(q)], [np.sin(q), np.cos(q)]]
  return m


_ORIGINS = [_tf(xyz, rpy) for xyz, rpy, _ in JOINTS]
_TIP = _tf(*TIP)
LIMITS = np.array([lim for _, _, lim in JOINTS]) * LIMIT_SCALE


def fk(q) -> np.ndarray:
  """Tool frame pose in the arm base frame."""
  m = np.eye(4)
  for origin, qi in zip(_ORIGINS, q):
    m = m @ origin @ _rz(qi)
  return m @ _TIP


def _error(q, target: np.ndarray) -> np.ndarray:
  m = fk(q)
  dp = target[:3, 3] - m[:3, 3]
  r = target[:3, :3] @ m[:3, :3].T
  dr = 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
  return np.concatenate([dp, dr])


def _solve_from(seed, target, iters=200, tol=1e-6):
  q = np.clip(np.asarray(seed, float), LIMITS[:, 0], LIMITS[:, 1])
  lam = 1e-3
  prev = np.inf
  for it in range(iters):
    e = _error(q, target)
    n = np.linalg.norm(e)
    if n < tol:
      return q
    if it % 10 == 9:
      if n > 0.9 * prev:   # stalled on a limit or out of reach
        break
      prev = n
    jac = np.zeros((6, 5))
    for i in range(5):
      dq = np.zeros(5)
      dq[i] = 1e-6
      jac[:, i] = (e - _error(q + dq, target)) / 1e-6
    step = np.linalg.solve(jac.T @ jac + lam * np.eye(5), jac.T @ e)
    q = np.clip(q + step, LIMITS[:, 0], LIMITS[:, 1])
  return q if np.linalg.norm(_error(q, target)) < 1e-4 else None


ELBOW_DOWN_PENALTY = 10.0


def solve(target: np.ndarray, seed, attempts: int = 64, elbow_up: bool = False):
  """Joint positions placing the tool frame at `target`, closest to `seed`.

  Returns None when the pose is out of reach (the 5-DOF arm can still reach
  any yaw about a vertical tool axis, through shoulder_pan and wrist_roll).
  elbow_up: prefer solutions with elbow_flex > 0 (lower arm raised, as at
  home). Far targets such as the ATC also have elbow-down solutions that put
  the lower arm into the machine.
  """
  seed = np.asarray(seed, float)
  rng = np.random.default_rng(0)
  seeds = [seed] + [rng.uniform(LIMITS[:, 0], LIMITS[:, 1]) for _ in range(attempts)]
  best = None
  for s in seeds:
    q = _solve_from(s, target)
    if q is None:
      continue
    d = np.linalg.norm(q - seed)
    if elbow_up and q[2] <= 0:
      d += ELBOW_DOWN_PENALTY
    if best is None or d < best[0]:
      best = (d, q)
    if d < 0.5:
      break
  return None if best is None else best[1]


def pose_matrix(translation, quaternion_xyzw) -> np.ndarray:
  x, y, z, w = quaternion_xyzw
  m = np.eye(4)
  m[:3, :3] = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ]
  m[:3, 3] = translation
  return m
