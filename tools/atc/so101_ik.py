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
  (
    (0.0388353, -8.97657e-09, 0.0624),
    (3.14159, 4.18253e-17, -3.14159),
    (-1.91986, 1.91986),
  ),
  (
    (-0.0303992, -0.0182778, -0.0542),
    (-1.5708, -1.5708, 0.0),
    (-1.74533, 1.74533),
  ),
  ((-0.11257, -0.028, 0.0), (0.0, 0.0, 1.5708), (-1.69, 1.69)),
  ((-0.1349, 0.0052, 0.0), (0.0, 0.0, -1.5708), (-1.65806, 1.65806)),
  ((0.0, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), (-2.74385, 2.84121)),
)
TIP = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))
# Named configuration "home" in models/so101/so101_limits.updates.pbtxt: arm
# over the pedestal (world +x), gripper 11 cm above the seats and nearly
# vertical, turned away from the Carvera.
HOME = (-1.571, -0.743, 0.43, 1.618, 0.0)
# Application limits in so101_limits.updates.pbtxt are 99% of the range; stay
# a little inside them.
LIMIT_SCALE = 0.985


def _rpy(r: float, p: float, y: float) -> np.ndarray:
  cr, sr, cp, sp, cy, sy = (
    np.cos(r),
    np.sin(r),
    np.cos(p),
    np.sin(p),
    np.cos(y),
    np.sin(y),
  )
  return np.array(
    [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
    ]
  )


def _tf(xyz, rpy) -> np.ndarray:
  m = np.eye(4)
  m[:3, :3] = _rpy(*rpy)
  m[:3, 3] = xyz
  return m


_ORIGINS = [_tf(xyz, rpy) for xyz, rpy, _ in JOINTS]
_TIP = _tf(*TIP)
LIMITS = np.array([lim for _, _, lim in JOINTS]) * LIMIT_SCALE

# The solver works on a batch of configurations at once (one row per seed):
# forward kinematics as stacked 3x3 rotations, and the Jacobian taken
# analytically from the joint axes, so one pass through the chain per
# iteration for every seed.
_O_ROT = np.array([o[:3, :3] for o in _ORIGINS])
_O_POS = np.array([o[:3, 3] for o in _ORIGINS])
_TIP_ROT, _TIP_POS = _TIP[:3, :3], _TIP[:3, 3]
# Random restarts, the same for every call (deterministic results).
_RANDOM_SEEDS = np.random.default_rng(0).uniform(
  LIMITS[:, 0], LIMITS[:, 1], size=(64, 5)
)


def _chain(q: np.ndarray):
  """Batched FK: q (N, 5) -> tool rotation (N, 3, 3), tool position (N, 3),
  joint axes (N, 5, 3) and joint origins (N, 5, 3), all in the base frame."""
  n = q.shape[0]
  rot = np.broadcast_to(np.eye(3), (n, 3, 3))
  pos = np.zeros((n, 3))
  axes = np.empty((n, 5, 3))
  origins = np.empty((n, 5, 3))
  for i in range(5):
    pos = pos + rot @ _O_POS[i]
    rot = rot @ _O_ROT[i]
    axes[:, i] = rot[:, :, 2]
    origins[:, i] = pos
    c, s = np.cos(q[:, i])[:, None], np.sin(q[:, i])[:, None]
    r0, r1 = rot[:, :, 0], rot[:, :, 1]
    rot = np.stack([c * r0 + s * r1, c * r1 - s * r0, rot[:, :, 2]], axis=2)
  pos = pos + rot @ _TIP_POS
  rot = rot @ _TIP_ROT
  return rot, pos, axes, origins


def fk(q) -> np.ndarray:
  """Tool frame pose in the arm base frame."""
  rot, pos, _, _ = _chain(np.asarray(q, float).reshape(1, 5))
  m = np.eye(4)
  m[:3, :3] = rot[0]
  m[:3, 3] = pos[0]
  return m


def _pose_residual(target: np.ndarray):
  """Residual and Jacobian for a full tool pose; `target` (4, 4), or (N, 4, 4)
  with one target per row of the batch. Rotation residual: the full
  matrix difference. The usual small-angle (skew-part) error is zero for a
  half turn, which let the solver accept the tool yawed 180 deg about its
  axis - the gripper is not symmetric."""
  target = np.asarray(target, float)
  t_rot, t_pos = target[..., :3, :3], target[..., :3, 3]

  def f(q, rows):
    if target.ndim == 3:
      tr, tp = t_rot[rows], t_pos[rows]
    else:
      tr, tp = t_rot, t_pos
    rot, pos, axes, origins = _chain(q)
    e = np.concatenate([tp - pos, 0.5 * (tr - rot).reshape(-1, 9)], axis=1)
    # Columns are -de/dq_i: z_i x (p - o_i) for the position, 0.5 * z_i x R
    # (each column of R) for the rotation.
    jp = np.cross(axes, (pos[:, None, :] - origins))  # (N, 5, 3)
    jr = 0.5 * np.cross(
      axes[:, :, None, :], np.swapaxes(rot, 1, 2)[:, None, :, :]
    )  # (N, 5, col, row)
    jac = np.concatenate([jp, np.swapaxes(jr, 2, 3).reshape(-1, 5, 9)], axis=2)
    return e, np.swapaxes(jac, 1, 2)  # (N, 12, 5)

  return f


def _down_residual(point, local, down):
  """Residual and Jacobian for tool point `local` at `point`, tool z along `down`."""

  def f(q, rows):
    rot, pos, axes, origins = _chain(q)
    tip = rot @ local + pos
    z = rot[:, :, 2]
    e = np.concatenate([point - tip, down - z], axis=1)
    jac = np.concatenate(
      [
        np.cross(axes, tip[:, None, :] - origins),
        np.cross(axes, z[:, None, :]),
      ],
      axis=2,
    )
    return e, np.swapaxes(jac, 1, 2)  # (N, 6, 5)

  return f


def _solve_batch(seeds: np.ndarray, residual, iters=200, tol=1e-6):
  """Damped least squares from every seed at once. Each seed stops when it
  converges or stalls (on a limit or out of reach); returns (N, 5) solutions
  and a mask of those within 1e-4."""
  q = np.clip(np.asarray(seeds, float), LIMITS[:, 0], LIMITS[:, 1])
  n = q.shape[0]
  active = np.ones(n, bool)
  prev = np.full(n, np.inf)
  lam = 1e-3 * np.eye(5)
  for it in range(iters):
    idx = np.flatnonzero(active)
    if idx.size == 0:
      break
    e, jac = residual(q[idx], idx)
    norm = np.linalg.norm(e, axis=1)
    keep = norm >= tol
    if it % 10 == 9:
      keep &= ~(norm > 0.9 * prev[idx])
      prev[idx] = norm
    active[idx[~keep]] = False
    idx, e, jac = idx[keep], e[keep], jac[keep]
    if idx.size == 0:
      break
    jt = np.swapaxes(jac, 1, 2)
    step = np.linalg.solve(jt @ jac + lam, (jt @ e[:, :, None]))[:, :, 0]
    q[idx] = np.clip(q[idx] + step, LIMITS[:, 0], LIMITS[:, 1])
  e, _ = residual(q, np.arange(n))
  return q, np.linalg.norm(e, axis=1) < 1e-4


ELBOW_DOWN_PENALTY = 10.0
# Above this elbow_flex the arm is folded so tightly that, near its base, the
# gripper runs into the shoulder (e.g. grid cell 0,0).
ELBOW_FOLD_MAX = 1.1


def _closest(seed, residual, attempts, elbow_up, targets=1):
  """The solution closest to `seed` (elbow-up preferred if asked): the seed
  itself first, then `attempts` random restarts, all solved together. With
  `targets` > 1 the residual holds that many targets and the closest
  solution over all of them comes back."""
  seed = np.asarray(seed, float)

  def pick(qs, ok, best):
    for q in qs[ok]:
      d = np.linalg.norm(q - seed)
      if elbow_up and not 0 < q[2] <= ELBOW_FOLD_MAX:
        d += ELBOW_DOWN_PENALTY
      if best is None or d < best[0]:
        best = (d, q)
    return best

  best = pick(*_solve_batch(np.tile(seed, (targets, 1)), residual), None)
  if best is not None and best[0] < 0.5:
    return best[1]
  # Row k * attempts + j: target k from random seed j.
  restarts = np.tile(_RANDOM_SEEDS[:attempts], (targets, 1))
  rows = np.repeat(np.arange(targets), attempts)
  best = pick(
    *_solve_batch(restarts, lambda q, idx: residual(q, rows[idx])), best
  )
  return None if best is None else best[1]


def solve(target: np.ndarray, seed, attempts: int = 64, elbow_up: bool = False):
  """Joint positions placing the tool frame at `target`, closest to `seed`.

  Returns None when the pose is out of reach (the 5-DOF arm can still reach
  any yaw about a vertical tool axis, through shoulder_pan and wrist_roll).
  elbow_up: prefer solutions with 0 < elbow_flex <= ELBOW_FOLD_MAX (lower
  arm raised, not folded back). Far targets such as the ATC also have
  elbow-down solutions that put the lower arm into the machine; near ones
  have folded solutions that put the gripper into the shoulder.
  """
  return _closest(
    seed, _pose_residual(np.asarray(target, float)), attempts, elbow_up
  )


def solve_any(targets, seed, attempts: int = 64, elbow_up: bool = False):
  """Like solve(), for several alternative targets (e.g. the jaw turned to
  different directions): the solution closest to `seed` over all of them."""
  targets = np.asarray(targets, float)
  return _closest(
    seed, _pose_residual(targets), attempts, elbow_up, len(targets)
  )


def solve_down(
  point: np.ndarray,
  local,
  down,
  seed,
  attempts: int = 64,
  elbow_up: bool = False,
):
  """Joint positions putting tool-frame point `local` at `point` with the tool
  z axis along `down` (all in the arm base frame), at any yaw about it.

  For a round shank the approach yaw does not matter, and leaving it free
  matters for this arm: wrist_roll's limits and the jaw's offset from the
  roll axis rule out some yaws at some places.
  """
  residual = _down_residual(
    np.asarray(point, float), np.asarray(local, float), np.asarray(down, float)
  )
  return _closest(seed, residual, attempts, elbow_up)


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
