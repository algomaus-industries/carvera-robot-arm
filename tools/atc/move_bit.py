# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Moves Carvera bits between the bit grid and the Makera Carvera ATC.

Slots are written as `grid:R,C` (bit_grid cell, R and C in 0..9) or `atc:N`
(Carvera ATC slot, N in 1..6; slot 0 holds the probe). The holder sitting at
the source slot is found from the belief world, so the tool needs no state of
its own. The bits are models/makera_bits (bit_<id>); their shank sizes come
from makera_bits.json. Works in both cells:

  UR5e + Hand-E (default):  --config=configs/omts/app_config.yaml
  SO-101 (--//:setup=carvera_so101):  --config=configs/carvera_so101/app_config.yaml

  bazel run -c opt //tools/atc:move_bit -- --move=grid:0,0:atc:1
  bazel run -c opt //tools/atc:move_bit -- --fill_atc     # grid row 0 -> T1..T6
  bazel run -c opt //tools/atc:move_bit -- --empty_atc    # T1..T6 -> free cells
  bazel run -c opt //tools/atc:move_bit -- --list

Each move: open the Carvera lid if an ATC slot is involved, open the gripper,
approach above the source seat (ANY), descend (LINEAR), close, attach the
holder, lift (LINEAR), approach above the target seat (ANY), descend (LINEAR),
open, detach, lift (LINEAR). Finishes at the `view` frame (UR5e) or the
the SO-101 cell's own `view` frame, so the arm is out of the machine.

Grippers (app_config gripper.type):
  robotiq  Hand-E: tool frame centred between the fingers.
  so101    SO-ARM101 jaw: a separate world object (robot.tool_object_name)
           whose jaw_joint is set with update_world; its tool frame is on the
           FIXED jaw's face, so the shank axis sits half a shank diameter
           along the tool's -x.
"""

import argparse
import dataclasses
import json
import math
import os
import re
import sys
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments, execution
from intrinsic.world.proto import object_world_refs_pb2, object_world_updates_pb2
from intrinsic.world.python import object_world_resources

from src.core.config import AppConfig, load_app_config
from src.hardware.gripper import GripperInterface, RobotiqGripper
from src.hardware.machine import DioCncMachine
from src.hardware.robot import Robot
from tools.atc import so101_ik

GRID_OBJECT = "bit_grid"
GRID_N = 10
ATC_OBJECT = "cnc_enclosure"
ATC_SLOTS = range(1, 7)
HOLDER_PREFIXES = ("bit_", "tool_holder_")

SAME_SLOT_TOL_M = 0.004
BITS_JSON = "models/makera_bits/makera_bits.json"
DEFAULT_SHANK_M = 0.125 * 0.0254


@dataclasses.dataclass(frozen=True)
class GraspProfile:
  """Where the tool frame goes relative to a seat frame, gripper pointing down.

  grasp_z: tool frame height above the seat when closing (just above the
      5 mm bit collar, so the fingers close on the shank).
  pre_z: carrying height; lifts the cutter (up to ~40 mm below the collar)
      out of the seat bore.
  off_centre: True if the tool frame is on one jaw's face rather than centred
      between the jaws (then the tool moves half a shank along +x).
  joint_ik: True to solve the arm's IK here (tools/atc/so101_ik.py) and send
      joint targets, for arms the motion planner's IK cannot solve.
  """

  grasp_z: float
  pre_z: float
  off_centre: bool
  joint_ik: bool = False


PROFILES = {
    "robotiq": GraspProfile(grasp_z=0.006, pre_z=0.096, off_centre=False),
    # The SO-101's fixed jaw reaches ~7 mm past its tool frame, so the tool
    # frame stays 14 mm above the seat: jaw tips just above the 5 mm collar.
    "so101": GraspProfile(grasp_z=0.014, pre_z=0.064, off_centre=True, joint_ik=True),
}
# Hand-E finger position when opening at a seat (12 mm per finger): clears a
# 6 mm shank and stays off the neighbours on the 35 mm grid.
HANDE_OPEN_M = 0.012


class So101Jaw(GripperInterface):
  """SO-ARM101 moving jaw, set through the world (update_world).

  Jaw gap between the finger faces vs jaw_joint angle, measured on the
  official meshes by models/so101/build_so101.py --calibrate-jaw: about
  15.8 mm at 0 rad and 7.3 mm per 0.1 rad (linear over -0.17..0.1 rad).
  """

  GAP_AT_ZERO_M = 0.0158
  GAP_PER_RAD_M = 0.073
  LIMITS = (-0.1745, 1.745)

  def __init__(self, solution: Any, object_name: str, open_gap_m: float, close_gap_m: float):
    self._skill = solution.skills.ai.intrinsic.update_world
    self._object = object_name
    self._open = self.angle_for_gap(open_gap_m)
    self._close = self.angle_for_gap(close_gap_m)

  @classmethod
  def angle_for_gap(cls, gap_m: float) -> float:
    a = (gap_m - cls.GAP_AT_ZERO_M) / cls.GAP_PER_RAD_M
    return min(max(a, cls.LIMITS[0]), cls.LIMITS[1])

  def _task(self, angle: float, name: str) -> bt.Node:
    update = object_world_updates_pb2.ObjectWorldUpdate(
      update_object_joints=object_world_updates_pb2.UpdateObjectJointsRequest(
        object=object_world_refs_pb2.ObjectReference(
          by_name=object_world_refs_pb2.ObjectReferenceByName(object_name=self._object)
        ),
        joint_positions=[angle],
      )
    )
    return bt.Task(action=self._skill(update=update), name=name)

  def build_open_task(self, name: str | None = None) -> bt.Node:
    return self._task(self._open, name or "Open SO-101 jaw")

  def build_close_task(self, name: str | None = None) -> bt.Node:
    return self._task(self._close, name or "Close SO-101 jaw")


def load_bit_catalog(path: str = BITS_JSON) -> dict[str, dict]:
  """Makera bit catalogue keyed by world object name (bit_<id>)."""
  if not os.path.exists(path):
    return {}
  with open(path) as fh:
    data = json.load(fh)
  return {"bit_" + b["id"]: b for b in data["bits"]}


def shank_m(holder: str, catalog: dict[str, dict]) -> float:
  return float(catalog.get(holder, {}).get("shank", DEFAULT_SHANK_M))


def make_gripper(solution: Any, config: AppConfig, shank: float) -> GripperInterface:
  if config.gripper.type == "so101":
    return So101Jaw(solution, config.robot.tool_object_name,
                    open_gap_m=shank + 0.005, close_gap_m=shank - 0.0003)
  return RobotiqGripper(
    solution=solution,
    config=dataclasses.replace(config.gripper, open_position=HANDE_OPEN_M),
  )


def parse_slot(spec: str) -> tuple[str, str]:
  """Returns (object_name, frame_name) for 'grid:R,C' or 'atc:N'."""
  m = re.fullmatch(r"grid:(\d+),(\d+)", spec)
  if m:
    r, c = int(m.group(1)), int(m.group(2))
    if not (0 <= r < GRID_N and 0 <= c < GRID_N):
      raise ValueError(f"Grid cell out of range: {spec}")
    return GRID_OBJECT, f"cell_{r}_{c}"
  m = re.fullmatch(r"atc:(\d+)", spec)
  if m:
    n = int(m.group(1))
    if n not in ATC_SLOTS:
      raise ValueError(f"ATC slot must be 1..6 (0 is the probe): {spec}")
    return ATC_OBJECT, f"atc_slot_{n}"
  raise ValueError(f"Slot must be 'grid:R,C' or 'atc:N', got {spec!r}")


def parse_move(spec: str) -> tuple[str, str]:
  """Splits 'grid:0,0:atc:1' into ('grid:0,0', 'atc:1')."""
  m = re.fullmatch(r"((?:grid:\d+,\d+)|(?:atc:\d+)):((?:grid:\d+,\d+)|(?:atc:\d+))", spec)
  if not m:
    raise ValueError(f"Move must be '<slot>:<slot>', e.g. grid:0,0:atc:1, got {spec!r}")
  return m.group(1), m.group(2)


def _frame_ref(object_name: str, frame_name: str | None) -> Any:
  by_name = object_world_refs_pb2.TransformNodeReferenceByName(
    frame=object_world_refs_pb2.FrameReferenceByName(
      object_name=object_name, frame_name=frame_name
    )
  ) if frame_name else object_world_refs_pb2.TransformNodeReferenceByName(
    object=object_world_refs_pb2.ObjectReferenceByName(object_name=object_name)
  )
  return object_world_refs_pb2.TransformNodeReference(by_name=by_name)


def _position(world: Any, object_name: str, frame_name: str | None = None):
  pose = world.get_transform(
    world.root, world.get_transform_node(_frame_ref(object_name, frame_name))
  )
  return tuple(float(v) for v in pose.translation)


def holder_positions(world: Any) -> dict[str, tuple[float, float, float]]:
  """All tool holders in the belief world and their positions in root."""
  out = {}
  for obj in world.list_objects():
    name = getattr(obj, "name", str(obj))
    if name.startswith(HOLDER_PREFIXES):
      out[name] = _position(world, name)
  return out


def holder_at(world: Any, slot: str, holders=None) -> str | None:
  """Name of the holder seated at `slot`, or None if the slot is empty."""
  obj, frame = parse_slot(slot)
  p = _position(world, obj, frame)
  for name, q in (holders or holder_positions(world)).items():
    if math.dist(p, q) < SAME_SLOT_TOL_M:
      return name
  return None


def all_slots() -> list[str]:
  return [f"grid:{r},{c}" for r in range(GRID_N) for c in range(GRID_N)] + [
    f"atc:{n}" for n in ATC_SLOTS
  ]


def _offset(z: float, yaw_deg: float, x_tool: float = 0.0):
  """Tool pose in the seat frame: pointing down (180 deg about an axis in the
  seat's XY plane), yawed by yaw_deg, and shifted x_tool along its own x.
  The tool's x axis then lies along (cos yaw, -sin yaw, 0) in the seat frame."""
  h = math.radians(yaw_deg) / 2
  yaw = math.radians(yaw_deg)
  return ((x_tool * math.cos(yaw), -x_tool * math.sin(yaw), z),
          (math.cos(h), -math.sin(h), 0.0, 0.0))


# Steps per LINEAR segment when moving by joint targets: the vertical approach
# and lift stay within about a millimetre of the straight line.
JOINT_IK_LINEAR_STEPS = 6


class JointTargets:
  """Turns seat-relative tool poses into SO-101 joint targets (so101_ik).

  Solves in the arm's base frame from the live belief world, seeding each
  target with the previous one so consecutive targets stay on one branch.
  """

  def __init__(self, world: Any, arm_name: str):
    self._world = world
    arm = world.get_kinematic_object(arm_name)
    self._base = _matrix(world.get_transform(world.root, arm))
    self._base_inv = _inverse(self._base)
    self.q = list(arm.joint_positions)
    self._last = None   # (obj, frame, z) of the previous target

  def via_home(self) -> None:
    """The next target is solved from the home configuration."""
    self.q = list(so101_ik.HOME)
    self._last = None

  def to(self, obj: str, frame: str, z: float, offset, linear: bool) -> list[list[float]]:
    seat = _matrix(self._world.get_transform(
      self._world.root, self._world.get_transform_node(_frame_ref(obj, frame))))
    zs = [z]
    if linear and self._last and self._last[:2] == (obj, frame):
      z0 = self._last[2]
      zs = [z0 + (z - z0) * (i + 1) / JOINT_IK_LINEAR_STEPS for i in range(JOINT_IK_LINEAR_STEPS)]
    out = []
    # The shank axis between the jaws (tool frame -x by half a shank, see
    # GraspProfile.off_centre) goes onto the seat axis, tool straight down;
    # the yaw about the shank is left free (round shank).
    (ox, oy, _), _ = offset(0.0)
    x_tool = math.hypot(ox, oy)
    down = self._base_inv[:3, :3] @ (-seat[:3, 2])
    for zi in zs:
      point = (self._base_inv @ seat @ [0.0, 0.0, zi, 1.0])[:3]
      # Free (ANY) targets take the elbow-up branch; LINEAR steps stay on the
      # branch of the step before.
      q = so101_ik.solve_down(point, (-x_tool, 0.0, 0.0), down, self.q,
                              elbow_up=len(zs) == 1)
      if q is None:
        raise ValueError(
          f"SO-101 cannot reach {obj}.{frame} at z={zi:.3f} m with the gripper "
          "vertical (out of reach of this arm from where it stands)")
      self.q = [float(v) for v in q]
      if os.environ.get("MOVE_BIT_PRINT_JOINTS"):
        print(f"    {obj}.{frame} z={zi:.3f}: tool at "
              f"{[round(float(v), 4) for v in (self._base @ so101_ik.fk(q))[:3, 3]]}"
              f" q={[round(v, 3) for v in self.q]}")
      out.append(self.q)
    self._last = (obj, frame, z)
    return out


def _matrix(pose: Any):
  return so101_ik.pose_matrix(
    [float(v) for v in pose.translation],
    [float(v) for v in pose.rotation.quaternion.xyzw])


def _inverse(m):
  import numpy as np  # pylint: disable=import-outside-toplevel
  return np.linalg.inv(m)


def _seat_in_world(world: Any, holder: str, slot: str) -> None:
  """Puts `holder` exactly on `slot` in the belief world.

  The SO-101's jaw is set through update_world only, so in Gazebo it never
  closes on the bit and the bit stays where it was; detaching then reads that
  Gazebo pose back. Seating the bit here keeps the belief world right.
  """
  obj, frame = parse_slot(slot)
  seat = world.get_transform_node(_frame_ref(obj, frame))
  bit = world.get_object(holder)
  world.update_transform(node_a=world.root, node_b=bit,
                         a_t_b=world.get_transform(world.root, seat), node_to_update=bit)


def build_move_tree(
  robot: Robot,
  gripper: GripperInterface,
  machine: DioCncMachine | None,
  config: AppConfig,
  holder: str,
  src: str,
  dst: str,
  yaw_deg: float,
  profile: GraspProfile,
  shank: float,
  joints: JointTargets | None = None,
) -> bt.Node:
  """Behavior tree moving `holder` from slot `src` to slot `dst`.

  With `joints`, every motion is a joint move (or several, for the LINEAR
  ones) to IK solutions computed here instead of a Cartesian move.
  """
  tool = config.robot.tool_object_name
  x_tool = shank / 2 if profile.off_centre else 0.0
  PRE_Z, GRASP_Z = profile.pre_z, profile.grasp_z
  src_obj, src_frame = parse_slot(src)
  dst_obj, dst_frame = parse_slot(dst)
  held = [(tool, holder)]

  def move(obj, frame, z, motion, pairs, label):
    if joints is not None:
      qs = joints.to(obj, frame, z, lambda zi: _offset(zi, yaw_deg, x_tool),
                     linear=motion == "LINEAR")
      # The jaw is hinged inside the gripper body (so101_arm.gripper_link), so
      # the two always overlap; the planner would reject every configuration.
      # The fixed jaw is part of the arm (gripper_link), so contacts allowed
      # for the gripper object are allowed for the arm as well.
      arm = config.robot.arm_part_name
      jaw_pairs = [(tool, arm)] + list(pairs or []) + [
        (arm, b) for a, b in (pairs or []) if a == tool]
      steps = [robot.build_move_joint_task(
                 object_world_resources.JointConfiguration(joint_position=q),
                 name=f"{label} ({i + 1}/{len(qs)})",
                 excluded_collision_pairs=jaw_pairs,
                 motion_type="JOINT" if motion == "LINEAR" else "ANY")
               for i, q in enumerate(qs)]
      return steps[0] if len(steps) == 1 else bt.Sequence(name=label, children=steps)
    return robot.build_move_cartesian_task(
      target_frame_name=frame,
      target_object_name=obj,
      motion_type=motion,
      target_frame_offset=_offset(z, yaw_deg, x_tool),
      excluded_collision_pairs=pairs,
      name=label,
    )

  def via_home(label):
    # SO-101: the planner's free path between grid and ATC can swing the
    # gripper into the Carvera, so carries go through the (high) home pose.
    joints.via_home()
    arm = config.robot.arm_part_name
    return robot.build_move_joint_task(
      "home", name=label, motion_type="ANY",
      excluded_collision_pairs=[(tool, arm), (arm, holder)] + held)

  tasks: list[bt.Node] = []
  if machine is not None and ATC_OBJECT in (src_obj, dst_obj):
    tasks.append(machine.build_open_door_task(name="Open Carvera Lid"))
  tasks += [
    gripper.build_open_task(name="Open Gripper"),
    # held: a stopped earlier run may have left the gripper touching this bit.
    move(src_obj, src_frame, PRE_Z, "ANY", held, f"Above {src}"),
    move(src_obj, src_frame, GRASP_Z, "LINEAR", held, f"Descend onto {holder}"),
    gripper.build_close_task(name=f"Grip {holder}"),
    robot.build_attach_object_task(object_name=holder, name=f"Attach {holder}"),
    move(src_obj, src_frame, PRE_Z, "LINEAR", held + [(holder, src_obj)],
         f"Lift {holder} out of {src}"),
    *([] if joints is None else [via_home(f"Carry {holder} via home")]),
    move(dst_obj, dst_frame, PRE_Z, "ANY", held, f"Carry {holder} above {dst}"),
    move(dst_obj, dst_frame, GRASP_Z, "LINEAR", held + [(holder, dst_obj)],
         f"Seat {holder} in {dst}"),
    gripper.build_open_task(name=f"Release {holder}"),
    robot.build_detach_object_task(object_name=holder, name=f"Detach {holder}"),
    move(dst_obj, dst_frame, PRE_Z, "LINEAR", held + [(holder, dst_obj)],
         f"Lift clear of {dst}"),
  ]
  return bt.Sequence(name=f"Move {holder}: {src} -> {dst}", children=tasks)


def run_moves(solution: Any, config: AppConfig, moves, yaw_deg: float,
              home_frame: str | None) -> bool:
  robot = Robot(solution=solution, config=config.robot)
  profile = PROFILES.get(config.gripper.type, PROFILES["robotiq"])
  catalog = load_bit_catalog()
  machine = (
    DioCncMachine(solution=solution, config=config.machine)
    if config.machine is not None else None
  )
  world = solution.world
  for src, dst in moves:
    holders = holder_positions(world)
    holder = holder_at(world, src, holders)
    if holder is None:
      print(f"[!] No tool holder at {src}; skipping.")
      return False
    occupant = holder_at(world, dst, holders)
    if occupant is not None:
      print(f"[!] {dst} is occupied by {occupant}; skipping.")
      return False
    shank = shank_m(holder, catalog)
    title = catalog.get(holder, {}).get("title", holder)
    print(f"Moving {holder} ({title}): {src} -> {dst} ...")
    gripper = make_gripper(solution, config, shank)
    joints = (JointTargets(world, config.robot.arm_part_name)
              if profile.joint_ik else None)
    try:
      solution.executive.run(
        build_move_tree(robot, gripper, machine, config, holder, src, dst, yaw_deg,
                        profile, shank, joints)
      )
      if profile.joint_ik:
        _seat_in_world(world, holder, dst)
      print(f"[✓] {holder} is now in {dst}")
    except (execution.ExecutionFailedError, ValueError) as e:
      print(f"[!] Move failed: {e}")
      if hasattr(solution.executive, "get_errors"):
        print(f"Executive errors: {solution.executive.get_errors()}")
      return False
  if profile.joint_ik and getattr(solution, "is_simulated", False) and solution.simulator:
    # Copy the belief world (bits on their new seats) into Gazebo.
    solution.simulator.reset()
  if home_frame and profile.joint_ik:
    solution.executive.run(robot.build_move_joint_task(
      "home", name="Return to home", motion_type="ANY",
      excluded_collision_pairs=[(config.robot.tool_object_name, config.robot.arm_part_name)]))
  elif home_frame:
    solution.executive.run(
      robot.build_move_cartesian_task(
        target_frame_name=home_frame, target_object_name="root",
        motion_type="ANY", name=f"Return to {home_frame}",
      )
    )
  return True


def plan_fill_atc(world: Any) -> list[tuple[str, str]]:
  """Fills empty ATC slots T1..T6 from the first occupied grid cells."""
  holders = holder_positions(world)
  sources = [s for s in all_slots() if s.startswith("grid:") and holder_at(world, s, holders)]
  targets = [f"atc:{n}" for n in ATC_SLOTS if holder_at(world, f"atc:{n}", holders) is None]
  return list(zip(sources, targets))


def plan_empty_atc(world: Any) -> list[tuple[str, str]]:
  """Returns holders in T1..T6 to the first free grid cells."""
  holders = holder_positions(world)
  sources = [f"atc:{n}" for n in ATC_SLOTS if holder_at(world, f"atc:{n}", holders)]
  free = [s for s in all_slots() if s.startswith("grid:") and holder_at(world, s, holders) is None]
  return list(zip(sources, free))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--address", default="localhost:17080")
  parser.add_argument("--config", default="configs/omts/app_config.yaml")
  parser.add_argument("--move", action="append", default=[],
                      help="'<src>:<dst>', e.g. grid:0,0:atc:1 (repeatable).")
  parser.add_argument("--fill_atc", action="store_true",
                      help="Fill empty ATC slots T1..T6 from the grid.")
  parser.add_argument("--empty_atc", action="store_true",
                      help="Return all holders in T1..T6 to free grid cells.")
  parser.add_argument("--list", action="store_true",
                      help="Print which holder sits in which slot and exit.")
  parser.add_argument("--yaw_deg", type=float, default=0.0,
                      help="Gripper rotation about the tool axis at the seats.")
  parser.add_argument("--home_frame", default="view",
                      help="Root frame to return to afterwards ('' to stay).")
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  config = load_app_config(args.config)
  solution = deployments.connect(address=args.address)
  world = solution.world

  if args.list:
    holders = holder_positions(world)
    for slot in all_slots():
      name = holder_at(world, slot, holders)
      if name or slot.startswith("atc:"):
        print(f"  {slot:10s} {name or '-'}")
    return

  # Each batch is planned on the world as the batch before left it, so
  # --fill_atc / --empty_atc see the slots the explicit --moves filled.
  batches = [lambda: [parse_move(m) for m in args.move]]
  if args.fill_atc:
    batches.append(lambda: plan_fill_atc(world))
  if args.empty_atc:
    batches.append(lambda: plan_empty_atc(world))
  ran = False
  for plan in batches:
    moves = plan()
    if not moves:
      continue
    if not run_moves(solution, config, moves, args.yaw_deg, args.home_frame or None):
      sys.exit(1)
    ran = True
  if not ran:
    print("Nothing to do: give --move, --fill_atc, --empty_atc or --list.")


if __name__ == "__main__":
  main()
