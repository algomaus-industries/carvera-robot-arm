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

"""Moves Makera bits between the bit carousel and the Makera Carvera ATC.

For the SO-101 cell (--//:setup=carvera_so101). Slots are written as
`grid:R,C` (frame cell_R_C of bit_grid, the carousel: ring R, seat C) or
`atc:N` (Carvera ATC slot, N in 1..6; slot 0 holds the probe). The bit at
the source slot is found from the belief world, so the tool needs no state
of its own. The bits are models/makera_bits (bit_<id>); their shank sizes
come from makera_bits.json.

  bazel run -c opt //tools/atc:move_bit -- --move=grid:0,0:atc:1
  bazel run -c opt //tools/atc:move_bit -- --fill_atc     # carousel -> empty T1..T6
  bazel run -c opt //tools/atc:move_bit -- --empty_atc    # T1..T6 -> free seats
  bazel run -c opt //tools/atc:move_bit -- --list
  bazel run -c opt //tools/atc:move_bit -- --turn=grid:2,10  # turn a seat to its pick position
  bazel run -c opt //tools/atc:move_bit -- --park   # arm home, cart away, hood shut
  bazel run -c opt //tools/atc:move_bit -- --dock   # hood open, cart in

Each move: open the Carvera lid if an ATC slot is involved, open the jaw,
above the source seat beside the shank (ANY), down (LINEAR), sideways onto
the shank (LINEAR), close, attach the bit, up (LINEAR), via the home
configuration, above the target seat (ANY), down (LINEAR), open, detach,
sideways off the shank (LINEAR), up (LINEAR); afterwards back to home.
Beside the shank: APPROACH_OFFSET_M (tools/atc/so101_jaws.py) towards the
fixed finger's side, so neither finger touches it on the way down or up.

The SO-ARM101 jaw is a separate world object (robot.tool_object_name) whose
jaw_joint is set with update_world; its tool frame is on the FIXED jaw's
face. The jaws are the V-groove ones (fab/jaws): the shank lies in the fixed
finger's V, its axis GraspProfile.v_depth - 0.707 d along the tool's +x
(the jaw numbers come from fab/gen_fab.py through the generated
tools/atc/so101_jaws.py). The
planner's IK does not converge for this 5-axis arm, so the joint targets
come from tools/atc/so101_ik.py (gripper vertical, yaw free).

SO-101 cell: bit_grid is the bit carousel (models/bit_carousel), a turning
seat plate. Before a move, its carousel_joint is turned so the seat involved
comes to its ring's pick position (carousel_layout.platter_angle_for), inside
the arm's reach; bits are attached to their seat frames so they turn with it.
A move between two carousel seats goes through a free ATC slot.
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
from intrinsic.world.proto import (
  object_world_refs_pb2,
  object_world_updates_pb2,
)
from intrinsic.world.python import object_world_resources

from models.bit_carousel import carousel_layout
from src.core.config import AppConfig, load_app_config
from src.hardware.gripper import GripperInterface
from src.hardware.machine import DioCncMachine
from src.hardware.robot import Robot
from tools.atc import gcode_tools, so101_ik, so101_jaws

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
  v_depth: depth of a 90 deg V along the fixed jaw (apex behind the tool
      frame's face, fab/jaws/jaws.scad V_DEPTH); 0 for a flat jaw. The shank
      lies in the V, see tool_x_from_shank_axis.
  joint_ik: True to solve the arm's IK here (tools/atc/so101_ik.py) and send
      joint targets, for arms the motion planner's IK cannot solve.
  approach_offset: how far off the shank (tool +x) the gripper goes down and
      comes up, moving sideways onto and off the shank at grasp_z.
  """

  grasp_z: float
  pre_z: float
  off_centre: bool
  joint_ik: bool = False
  v_depth: float = 0.0
  approach_offset: float = 0.0

  def tool_x_from_shank_axis(self, shank: float) -> float:
    """How far the tool frame is from the gripped shank's axis, along tool +x.

    Flat jaw: half a shank. V jaw: the axis is sqrt(2) radii in front of the
    V's apex, so the frame is 0.707 d - v_depth from it (negative for a
    1/8" shank in a 3 mm V: that axis is behind the finger's face).
    """
    if not self.off_centre:
      return 0.0
    if self.v_depth:
      return shank / math.sqrt(2.0) - self.v_depth
    return shank / 2


PROFILES = {
  # The SO-101's V-groove fingers (fab/jaws): grasp_z keeps the fixed
  # fingertip 3 mm above the collar (fab/gen_fab.py JAW).
  "so101": GraspProfile(
    grasp_z=so101_jaws.GRASP_Z_M,
    pre_z=0.064,
    off_centre=True,
    joint_ik=True,
    v_depth=so101_jaws.V_DEPTH_M,
    approach_offset=so101_jaws.APPROACH_OFFSET_M,
  ),
}


class So101Jaw(GripperInterface):
  """SO-ARM101 moving jaw, set through the world (update_world).

  V-groove jaws (fab/jaws): the "gap" is the diameter of the shank that,
  lying in the fixed finger's V, the moving finger's pad rib just touches
  (fab/gen_fab.py jaw_gap_table, generated into so101_jaws): about 8.95 mm
  at 0 rad and 6.19 mm per 0.1 rad. A 1/8" shank is gripped at -0.093 rad,
  a 6 mm one at -0.048 rad (fab/jaws/check_jaws.py).

  In the sim the jaw is set to a position CLOSE_SQUEEZE_M past contact. The
  real driver must close against a torque/current limit instead, commanding
  about CLOSE_PAST_CONTACT_REAL_M past contact: a print error of +-0.2 mm
  moves the contact by up to +-0.5 mm, more than the squeeze.
  """

  GAP_AT_ZERO_M = so101_jaws.GAP_AT_ZERO_M
  GAP_PER_RAD_M = so101_jaws.GAP_PER_RAD_M
  LIMITS = (-0.1745, 1.745)

  def __init__(
    self, solution: Any, object_name: str, open_gap_m: float, close_gap_m: float
  ):
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
          by_name=object_world_refs_pb2.ObjectReferenceByName(
            object_name=self._object
          )
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


def make_gripper(
  solution: Any, config: AppConfig, shank: float
) -> GripperInterface:
  return So101Jaw(
    solution,
    config.robot.tool_object_name,
    open_gap_m=shank + so101_jaws.OPEN_EXTRA_M,
    close_gap_m=shank - so101_jaws.CLOSE_SQUEEZE_M,
  )


def parse_slot(spec: str) -> tuple[str, str]:
  """Returns (object_name, frame_name) for 'grid:R,C' or 'atc:N'."""
  m = re.fullmatch(r"grid:(\d+),(\d+)", spec)
  if m:
    r, c = int(m.group(1)), int(m.group(2))
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
  m = re.fullmatch(
    r"((?:grid:\d+,\d+)|(?:atc:\d+)):((?:grid:\d+,\d+)|(?:atc:\d+))", spec
  )
  if not m:
    raise ValueError(
      f"Move must be '<slot>:<slot>', e.g. grid:0,0:atc:1, got {spec!r}"
    )
  return m.group(1), m.group(2)


def _frame_ref(object_name: str, frame_name: str | None) -> Any:
  by_name = (
    object_world_refs_pb2.TransformNodeReferenceByName(
      frame=object_world_refs_pb2.FrameReferenceByName(
        object_name=object_name, frame_name=frame_name
      )
    )
    if frame_name
    else object_world_refs_pb2.TransformNodeReferenceByName(
      object=object_world_refs_pb2.ObjectReferenceByName(
        object_name=object_name
      )
    )
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


def grid_cells(world: Any = None) -> list[tuple[int, int]]:
  """(R, C) of every cell_R_C frame on the rack, in R, C order.

  The square grids have 10 x 10 cells; the SO-101 cell's curved rack
  (models/bit_rack_arc) has rings of different lengths, so the cells are read
  from the rack's frames in the world when there is one.
  """
  if world is not None:
    try:
      cells = []
      for f in world.get_object(GRID_OBJECT).frame_names:
        m = re.fullmatch(r"cell_(\d+)_(\d+)", str(f))
        if m:
          cells.append((int(m.group(1)), int(m.group(2))))
      if cells:
        return sorted(cells)
    except Exception:  # pylint: disable=broad-except
      pass
  return [(r, c) for r in range(GRID_N) for c in range(GRID_N)]


def all_slots(world: Any = None) -> list[str]:
  return [f"grid:{r},{c}" for r, c in grid_cells(world)] + [
    f"atc:{n}" for n in ATC_SLOTS
  ]


def _offset(z: float, yaw_deg: float, x_tool: float = 0.0):
  """Tool pose in the seat frame: pointing down (180 deg about an axis in the
  seat's XY plane), yawed by yaw_deg, and shifted x_tool along its own x.
  The tool's x axis then lies along (cos yaw, -sin yaw, 0) in the seat frame."""
  h = math.radians(yaw_deg) / 2
  yaw = math.radians(yaw_deg)
  return (
    (x_tool * math.cos(yaw), -x_tool * math.sin(yaw), z),
    (math.cos(h), -math.sin(h), 0.0, 0.0),
  )


# Steps per LINEAR segment when moving by joint targets. Each step is its own
# planned move, so fewer is faster; with 3 over the 50 mm approach the
# gripper stays within 0.25 mm of the vertical line (6 steps: 0.21 mm, 2:
# 0.47 mm, 1: 1.8 mm at carousel seat 2,23 and ATC slot 2). Shorter
# segments get one step per LINEAR_STEP_M (the 2.5 mm moves onto and off a
# shank: one).
JOINT_IK_LINEAR_STEPS = 3
LINEAR_STEP_M = 0.017
GRID_CELL_PREFIX = "cell_"
# How far the open jaws reach from the shank axis along the jaw axis (half
# the open gap plus the ~10 mm fingertip, see models/so101/build_so101.py), the
# room a neighbouring bit (holder) needs around its own axis, and how much
# clearance (below the best direction's) a jaw direction may give up so the
# IK can pick the one nearest the previous configuration.
JAW_REACH = 0.020
PUCK_CLEAR = 0.012
JAW_AXIS_SLACK = 0.002


def _segment_distance(p, d, half_length):
  """Distance from point p to the segment [-half_length, half_length] * d."""
  t = max(-half_length, min(half_length, float(p @ d)))
  return float(((p - t * d) ** 2).sum() ** 0.5)


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
    self._last = None  # (obj, frame, z, x_tool) of the previous target
    self._seats = None  # rack seat name -> world xy

  def via_home(self) -> None:
    """The next target is solved from the home configuration."""
    self.q = list(so101_ik.HOME)
    self._last = None

  def to(
    self,
    obj: str,
    frame: str,
    z: float,
    offset,
    linear: bool,
    x_tool: float | None = None,
  ) -> list[list[float]]:
    seat = _matrix(
      self._world.get_transform(
        self._world.root, self._world.get_transform_node(_frame_ref(obj, frame))
      )
    )
    # The shank axis between the jaws (x_tool along the tool's -x, see
    # GraspProfile.tool_x_from_shank_axis; it can be negative) goes onto the
    # seat axis, tool straight down; the yaw about the shank is left free
    # (round shank).
    if x_tool is None:
      (ox, oy, _), _ = offset(0.0)
      x_tool = math.hypot(ox, oy)
    # A LINEAR segment from the previous target at the same seat: in steps
    # along z (down, up) and x_tool (sideways onto or off a shank).
    steps = [(z, x_tool)]
    follow = bool(linear and self._last and self._last[:2] == (obj, frame))
    if follow:
      z0, x0 = self._last[2:]
      n = max(
        1,
        min(
          JOINT_IK_LINEAR_STEPS,
          math.ceil(math.hypot(z - z0, x_tool - x0) / LINEAR_STEP_M - 1e-9),
        ),
      )
      steps = [
        (z0 + (z - z0) * (i + 1) / n, x0 + (x_tool - x0) * (i + 1) / n)
        for i in range(n)
      ]
    out = []
    down = self._base_inv[:3, :3] @ (-seat[:3, 2])
    # In the bit rack the neighbours stand only one pitch away, closer than
    # the bulky moving jaw reaches when it opens, so the jaw axis (tool x, the
    # moving jaw on -x) goes where the neighbouring seats leave the most room:
    # the diagonal of a square grid, a gap between rings in the curved rack.
    diagonals = []
    if frame.startswith(GRID_CELL_PREFIX) and obj == GRID_OBJECT:
      diagonals = [
        self._base_inv[:3, :3] @ [*d, 0.0] for d in self._jaw_axes(frame, seat)
      ]
    for zi, xi in steps:
      point = (self._base_inv @ seat @ [0.0, 0.0, zi, 1.0])[:3]
      # Free (ANY) targets take the elbow-up branch; LINEAR steps stay on the
      # branch of the step before.
      if diagonals:
        q = self._solve_diagonal(
          point, xi, down, diagonals, elbow_up=not follow
        )
      else:
        q = so101_ik.solve_down(
          point, (-xi, 0.0, 0.0), down, self.q, elbow_up=not follow
        )
      if q is None:
        raise ValueError(
          f"SO-101 cannot reach {obj}.{frame} at z={zi:.3f} m with the gripper "
          "vertical (out of reach of this arm from where it stands)"
        )
      self.q = [float(v) for v in q]
      if os.environ.get("MOVE_BIT_PRINT_JOINTS"):
        print(
          f"    {obj}.{frame} z={zi:.3f}: tool at "
          f"{[round(float(v), 4) for v in (self._base @ so101_ik.fk(q))[:3, 3]]}"
          f" q={[round(v, 3) for v in self.q]}"
        )
      out.append(self.q)
    self._last = (obj, frame, z, x_tool)
    return out

  def _rack_seats(self):
    """World xy of every rack seat, read once per move from the world (on the
    carousel the seat frames turn with its platter)."""
    if self._seats is None:
      rack = self._world.get_object(GRID_OBJECT)
      self._seats = {}
      for name in rack.frame_names:
        name = str(name)
        if name.startswith(GRID_CELL_PREFIX):
          t = self._world.get_transform(
            self._world.root,
            self._world.get_transform_node(_frame_ref(GRID_OBJECT, name)),
          )
          self._seats[name] = _matrix(t)[:2, 3]
    return self._seats

  def _jaw_axes(self, frame, seat):
    """World xy jaw axes (both signs) that keep the jaws clearest of the
    neighbouring seats: within JAW_AXIS_SLACK of the best clearance."""
    import numpy as np  # pylint: disable=import-outside-toplevel

    centre = seat[:2, 3]
    near = [
      p - centre
      for name, p in self._rack_seats().items()
      if name != frame and np.linalg.norm(p - centre) < JAW_REACH + PUCK_CLEAR
    ]
    scored = []
    for i in range(36):
      th = math.pi * i / 36
      d = np.array([math.cos(th), math.sin(th)])
      clear = min(
        (_segment_distance(n, d, JAW_REACH) for n in near), default=1.0
      )
      scored.append((clear, d))
    best = max(c for c, _ in scored)
    if os.environ.get("MOVE_BIT_PRINT_JOINTS"):
      print(
        f"    {frame}: jaw clearance to the nearest seat {best * 1000:.1f} mm"
      )
    axes = []
    for c, d in scored:
      if c >= best - JAW_AXIS_SLACK:
        axes += [d, -d]
    return axes

  def _solve_diagonal(self, point, x_tool, down, diagonals, elbow_up):
    import numpy as np  # pylint: disable=import-outside-toplevel

    z_axis = np.asarray(down, float)
    targets = []
    for x_axis in diagonals:
      x_axis = np.asarray(x_axis, float)
      target = np.eye(4)
      target[:3, :3] = np.column_stack(
        [x_axis, np.cross(z_axis, x_axis), z_axis]
      )
      # Shank axis at tool -x by x_tool: the tool frame goes +x of it.
      target[:3, 3] = np.asarray(point, float) + x_tool * x_axis
      targets.append(target)
    # All jaw directions in one batched solve; the closest solution wins.
    return so101_ik.solve_any(targets, self.q, elbow_up=elbow_up)


def _matrix(pose: Any):
  return so101_ik.pose_matrix(
    [float(v) for v in pose.translation],
    [float(v) for v in pose.rotation.quaternion.xyzw],
  )


def _inverse(m):
  import numpy as np  # pylint: disable=import-outside-toplevel

  return np.linalg.inv(m)


def carousel(world: Any) -> Any:
  """The bit carousel (bit_grid with a carousel_joint), or None."""
  try:
    obj = world.get_kinematic_object(GRID_OBJECT)
  except Exception:  # pylint: disable=broad-except
    return None
  return obj if len(list(obj.joint_positions)) == 1 else None


def carousel_angle(world: Any) -> float | None:
  obj = carousel(world)
  return None if obj is None else float(list(obj.joint_positions)[0])


# How the carousel turns, like its STS3215 in position mode: one turn of
# travel (0..2 pi, no wrapping past the end stop), a smooth start and stop,
# about CAROUSEL_PEAK_SPEED at most (conservative for the loaded 352 mm
# plate; the servo itself does ~4.7 rad/s unloaded).
CAROUSEL_PEAK_SPEED = 1.2  # rad/s
CAROUSEL_MIN_TURN_TIME = 0.5  # s
CAROUSEL_UPDATE_HZ = 20


def _turn_smoothly(world: Any, obj: Any, start: float, goal: float) -> None:
  import time  # pylint: disable=import-outside-toplevel

  dist = abs(goal - start)
  if dist < 1e-4:
    return
  # Smoothstep: peak speed 1.5x the mean.
  duration = max(CAROUSEL_MIN_TURN_TIME, 1.5 * dist / CAROUSEL_PEAK_SPEED)
  t0 = time.monotonic()
  while True:
    t = min(1.0, (time.monotonic() - t0) / duration)
    s = t * t * (3 - 2 * t)
    world.update_joint_positions(obj, [start + (goal - start) * s])
    if t >= 1.0:
      return
    time.sleep(1.0 / CAROUSEL_UPDATE_HZ)


def turn_carousel(world: Any, slot: str) -> None:
  """Turns the carousel so the seat of grid slot `slot` is at its ring's
  pick position (belief world; the real carousel's servo follows it)."""
  obj = carousel(world)
  _, frame = parse_slot(slot)
  r, c = (int(v) for v in frame[len(GRID_CELL_PREFIX) :].split("_"))
  angle = carousel_layout.platter_angle_for(r, c)
  print(f"  turning the carousel to {math.degrees(angle):.1f} deg for {slot}")
  _turn_smoothly(world, obj, float(list(obj.joint_positions)[0]), angle)
  p = world.get_transform(
    world.root, world.get_transform_node(_frame_ref(GRID_OBJECT, frame))
  )
  print(f"  {slot} now at world {[round(float(v), 4) for v in p.translation]}")


def _seat_in_world(world: Any, holder: str, slot: str) -> None:
  """Puts `holder` exactly on `slot` in the belief world.

  The SO-101's jaw is set through update_world only, so in Gazebo it never
  closes on the bit and the bit stays where it was; detaching then reads that
  Gazebo pose back. Seating the bit here keeps the belief world right.
  """
  obj, frame = parse_slot(slot)
  seat = world.get_transform_node(_frame_ref(obj, frame))
  bit = world.get_object(holder)
  world.update_transform(
    node_a=world.root,
    node_b=bit,
    a_t_b=world.get_transform(world.root, seat),
    node_to_update=bit,
  )
  # Attached to the seat, so a bit on the carousel turns with it.
  world.reparent_object(bit, seat)


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
  joints: JointTargets,
) -> bt.Node:
  """Behavior tree moving `holder` from slot `src` to slot `dst`.

  Every motion is a joint move (or several, for the LINEAR ones) to IK
  solutions computed here (`joints`, tools/atc/so101_ik.py).
  """
  tool = config.robot.tool_object_name
  x_tool = profile.tool_x_from_shank_axis(shank)
  x_beside = (
    x_tool + profile.approach_offset
  )  # tool off the shank, fixed finger away
  PRE_Z, GRASP_Z = profile.pre_z, profile.grasp_z
  src_obj, src_frame = parse_slot(src)
  dst_obj, dst_frame = parse_slot(dst)
  held = [(tool, holder)]

  def move(obj, frame, z, motion, pairs, label, x=x_tool):
    qs = joints.to(
      obj,
      frame,
      z,
      lambda zi: _offset(zi, yaw_deg, x),
      linear=motion == "LINEAR",
      x_tool=x,
    )
    # The jaw is hinged inside the gripper body (so101_arm.gripper_link), so
    # the two always overlap; the planner would reject every configuration.
    # The fixed jaw is part of the arm (gripper_link), so contacts allowed
    # for the gripper object are allowed for the arm as well.
    arm = config.robot.arm_part_name
    jaw_pairs = (
      [(tool, arm)]
      + list(pairs or [])
      + [(arm, b) for a, b in (pairs or []) if a == tool]
    )
    steps = [
      robot.build_move_joint_task(
        object_world_resources.JointConfiguration(joint_position=q),
        name=f"{label} ({i + 1}/{len(qs)})",
        excluded_collision_pairs=jaw_pairs,
        motion_type="JOINT" if motion == "LINEAR" else "ANY",
      )
      for i, q in enumerate(qs)
    ]
    return (
      steps[0] if len(steps) == 1 else bt.Sequence(name=label, children=steps)
    )

  def via_home(label):
    # SO-101: the planner's free path between grid and ATC can swing the
    # gripper into the Carvera, so carries go through the (high) home pose.
    joints.via_home()
    arm = config.robot.arm_part_name
    return robot.build_move_joint_task(
      "home",
      name=label,
      motion_type="ANY",
      excluded_collision_pairs=[(tool, arm), (arm, holder)] + held,
    )

  tasks: list[bt.Node] = []
  if machine is not None and ATC_OBJECT in (src_obj, dst_obj):
    tasks.append(machine.build_open_door_task(name="Open Carvera Lid"))
  tasks += [
    gripper.build_open_task(name="Open Gripper"),
    # held: a stopped earlier run may have left the gripper touching this bit.
    move(src_obj, src_frame, PRE_Z, "ANY", held, f"Above {src}", x=x_beside),
    move(
      src_obj,
      src_frame,
      GRASP_Z,
      "LINEAR",
      held,
      f"Descend beside {holder}",
      x=x_beside,
    ),
    move(src_obj, src_frame, GRASP_Z, "LINEAR", held, f"Onto {holder}'s shank"),
    gripper.build_close_task(name=f"Grip {holder}"),
    robot.build_attach_object_task(object_name=holder, name=f"Attach {holder}"),
    move(
      src_obj,
      src_frame,
      PRE_Z,
      "LINEAR",
      held + [(holder, src_obj)],
      f"Lift {holder} out of {src}",
    ),
    via_home(f"Carry {holder} via home"),
    move(dst_obj, dst_frame, PRE_Z, "ANY", held, f"Carry {holder} above {dst}"),
    move(
      dst_obj,
      dst_frame,
      GRASP_Z,
      "LINEAR",
      held + [(holder, dst_obj)],
      f"Seat {holder} in {dst}",
    ),
    gripper.build_open_task(name=f"Release {holder}"),
    robot.build_detach_object_task(object_name=holder, name=f"Detach {holder}"),
    move(
      dst_obj,
      dst_frame,
      GRASP_Z,
      "LINEAR",
      held + [(holder, dst_obj)],
      f"Off {holder}'s shank",
      x=x_beside,
    ),
    move(
      dst_obj,
      dst_frame,
      PRE_Z,
      "LINEAR",
      held + [(holder, dst_obj)],
      f"Lift clear of {dst}",
      x=x_beside,
    ),
  ]
  return bt.Sequence(name=f"Move {holder}: {src} -> {dst}", children=tasks)


def slot_name(slot: str) -> str:
  """grid:2,23 -> "carousel ring 2, seat 23"; atc:3 -> "T3"."""
  kind, _, where = slot.partition(":")
  if kind == "atc":
    return f"T{where}"
  if kind == "grid":
    r, _, c = where.partition(",")
    return f"carousel ring {r}, seat {c}"
  return slot


def progress(i: int, n: int, phase: str, move: str, text: str) -> None:
  """One machine-readable progress line (the ATC picker shows them): move i of
  n (0-based; i == n after the last), the phase and what is going on."""
  print(
    "@progress "
    + json.dumps({"i": i, "n": n, "phase": phase, "move": move, "text": text}),
    flush=True,
  )


def run_moves(
  solution: Any,
  config: AppConfig,
  moves,
  yaw_deg: float,
  home_frame: str | None,
) -> bool:
  robot = Robot(solution=solution, config=config.robot)
  if config.gripper.type != "so101":
    raise SystemExit(
      "move_bit drives the SO-101 cell (--config=configs/carvera_so101/app_config.yaml)."
    )
  profile = PROFILES["so101"]
  catalog = load_bit_catalog()
  machine = (
    DioCncMachine(solution=solution, config=config.machine)
    if config.machine is not None
    else None
  )
  world = solution.world
  if not getattr(solution, "is_simulated", False) and any(
    s.startswith("atc:") for move in moves for s in move
  ):
    # After a job the Carvera may keep a bit in the spindle; its ATC slot then
    # looks empty to the arm, which would put another bit there.
    input(
      "Make sure the Carvera's spindle holds no tool (Makera Studio / controller: "
      "drop the tool), then press Enter: "
    )
  n = len(moves)
  for i, (src, dst) in enumerate(moves):
    holders = holder_positions(world)
    holder = holder_at(world, src, holders)
    if holder is None:
      print(f"[!] No tool holder at {src}; skipping.")
      return False
    occupant = holder_at(world, dst, holders)
    if occupant is not None:
      print(f"[!] {dst} is occupied by {occupant}; skipping.")
      return False
    on_carousel = (
      [s for s in (src, dst) if s.startswith("grid:")]
      if carousel(world)
      else []
    )
    if len(on_carousel) == 2:
      print(
        f"[!] {src} -> {dst}: both seats are on the carousel; move the bit to a "
        "free ATC slot and from there to the other seat."
      )
      return False
    shank = shank_m(holder, catalog)
    title = catalog.get(holder, {}).get("title", holder)
    what = f"{title}: {slot_name(src)} → {slot_name(dst)}"
    if on_carousel:
      progress(
        i,
        n,
        "turn",
        what,
        f"turning the carousel to {slot_name(on_carousel[0])}",
      )
      turn_carousel(world, on_carousel[0])
    print(f"Moving {holder} ({title}): {src} -> {dst} ...")
    progress(i, n, "plan", what, "working out the arm's joint path")
    gripper = make_gripper(solution, config, shank)
    joints = JointTargets(world, config.robot.arm_part_name)
    try:
      tree = build_move_tree(
        robot,
        gripper,
        machine,
        config,
        holder,
        src,
        dst,
        yaw_deg,
        profile,
        shank,
        joints,
      )
      progress(i, n, "arm", what, "arm: pick, carry, place")
      solution.executive.run(tree)
      _seat_in_world(world, holder, dst)
      print(f"[✓] {holder} is now in {dst}", flush=True)
      progress(i + 1, n, "done", what, f"{slot_name(dst)} done")
      save_state(world)
    except (execution.ExecutionFailedError, ValueError) as e:
      print(f"[!] Move failed: {e}")
      if hasattr(solution.executive, "get_errors"):
        print(f"Executive errors: {solution.executive.get_errors()}")
      return False
  if getattr(solution, "is_simulated", False) and solution.simulator:
    # Copy the belief world (bits on their new seats) into Gazebo.
    progress(n, n, "sim", "", "updating the simulator")
    solution.simulator.reset()
  if home_frame:
    progress(n, n, "home", "", "arm back to home")
    solution.executive.run(
      robot.build_move_joint_task(
        "home",
        name="Return to home",
        motion_type="ANY",
        excluded_collision_pairs=[
          (config.robot.tool_object_name, config.robot.arm_part_name)
        ],
      )
    )
  return True


# ---------------------------------------------------------------- cart
# SO-101 cell: the rolling cart (arm, carousel and bits ride on it) stands in
# front of the Carvera while the hood is open, and rolls away before the hood
# closes. Docked position relative to the Carvera (cnc_enclosure origin),
# from configs/carvera_so101/gen_cell_configs.py; parked: PARK_DX further out.
CART_OBJECT = "robot_cart"
CART_DOCK_FROM_CARVERA = (0.4425, 0.0775)
PARK_DX = 0.45
HOOD_OPEN = 1.4866
ANIM_STEPS = 40


def _cart_target(world: Any, parked: bool):
  from intrinsic.math.python import data_types  # pylint: disable=import-outside-toplevel

  cnc = world.get_transform(
    world.root, world.get_object(ATC_OBJECT)
  ).translation
  cart = world.get_transform(world.root, world.get_object(CART_OBJECT))
  x = float(cnc[0]) + CART_DOCK_FROM_CARVERA[0] + (PARK_DX if parked else 0.0)
  y = float(cnc[1]) + CART_DOCK_FROM_CARVERA[1]
  return cart, data_types.Pose3(
    rotation=cart.rotation, translation=[x, y, float(cart.translation[2])]
  )


def _animate(n: int, step, pause: float = 0.03) -> None:
  import time  # pylint: disable=import-outside-toplevel

  for i in range(1, n + 1):
    # ease in and out
    t = i / n
    step(t * t * (3 - 2 * t))
    time.sleep(pause)


def roll_cart(solution: Any, config: AppConfig, parked: bool) -> None:
  """Rolls the cart to its parked or docked position (sim: animated in the
  world; the real cart is pushed by hand)."""
  from intrinsic.math.python import data_types  # pylint: disable=import-outside-toplevel

  world = solution.world
  start, goal = _cart_target(world, parked)
  if not getattr(solution, "is_simulated", False):
    input(
      f"Push the cart {'away from' if parked else 'up to'} the Carvera, then press Enter: "
    )
  cart = world.get_object(CART_OBJECT)
  a, b = start.translation, goal.translation

  def step(t):
    world.update_transform(
      node_a=world.root,
      node_b=cart,
      node_to_update=cart,
      a_t_b=data_types.Pose3(
        rotation=goal.rotation,
        translation=[a[k] + (b[k] - a[k]) * t for k in range(3)],
      ),
    )

  _animate(ANIM_STEPS, step)


def set_hood(solution: Any, open_: bool) -> None:
  """Opens or closes the Carvera's hood (sim: animated in the world; the
  real hood is lifted by hand)."""
  world = solution.world
  if not getattr(solution, "is_simulated", False):
    input(
      f"{'Open' if open_ else 'Close'} the Carvera's hood, then press Enter: "
    )
  cnc = world.get_kinematic_object(ATC_OBJECT)
  q0 = float(list(cnc.joint_positions)[0])
  q1 = HOOD_OPEN if open_ else 0.0
  _animate(
    ANIM_STEPS,
    lambda t: world.update_joint_positions(cnc, [q0 + (q1 - q0) * t]),
  )


def park(solution: Any, config: AppConfig) -> None:
  """Arm home, cart away from the Carvera, hood closed."""
  robot = Robot(solution=solution, config=config.robot)
  tool = config.robot.tool_object_name
  solution.executive.run(
    robot.build_move_joint_task(
      "home",
      name="Arm home",
      motion_type="ANY",
      excluded_collision_pairs=[(tool, config.robot.arm_part_name)],
    )
  )
  print("Rolling the cart away ...")
  roll_cart(solution, config, parked=True)
  print("Closing the hood ...")
  set_hood(solution, open_=False)
  print("[✓] Parked")


def dock(solution: Any, config: AppConfig) -> None:
  """Hood open, cart up to the Carvera."""
  print("Opening the hood ...")
  set_hood(solution, open_=True)
  print("Rolling the cart in ...")
  roll_cart(solution, config, parked=False)
  print("[✓] Docked")


def plan_fill_atc(world: Any) -> list[tuple[str, str]]:
  """Fills empty ATC slots T1..T6 from the first occupied grid cells."""
  holders = holder_positions(world)
  sources = [
    s
    for s in all_slots(world)
    if s.startswith("grid:") and holder_at(world, s, holders)
  ]
  targets = [
    f"atc:{n}"
    for n in ATC_SLOTS
    if holder_at(world, f"atc:{n}", holders) is None
  ]
  return list(zip(sources, targets, strict=False))


def plan_empty_atc(world: Any) -> list[tuple[str, str]]:
  """Returns holders in T1..T6 to the first free grid cells."""
  holders = holder_positions(world)
  sources = [
    f"atc:{n}" for n in ATC_SLOTS if holder_at(world, f"atc:{n}", holders)
  ]
  free = [
    s
    for s in all_slots(world)
    if s.startswith("grid:") and holder_at(world, s, holders) is None
  ]
  return list(zip(sources, free, strict=False))


def occupancy(world: Any) -> dict[str, str | None]:
  """Slot -> bit (or None) for every carousel seat and ATC slot."""
  holders = holder_positions(world)
  return {slot: holder_at(world, slot, holders) for slot in all_slots(world)}


def home_seat(bit: str, catalog: dict[str, dict]) -> str | None:
  """The carousel seat a bit starts on (configs/carvera_so101/bits.updates.pbtxt:
  catalogue order, ring by ring from the inside)."""
  order = list(catalog)
  if bit not in order:
    return None
  cells = carousel_layout.cells()
  i = order.index(bit)
  return f"grid:{cells[i][0]},{cells[i][1]}" if i < len(cells) else None


def parse_set_atc(spec: str) -> dict[int, str | None]:
  """'1=bit_ofl4_22,2=,3=bit_x' -> {1: 'bit_ofl4_22', 2: None, 3: 'bit_x'}."""
  out = {}
  for part in filter(None, spec.split(",")):
    n, _, bit = part.partition("=")
    n = int(n.strip().lstrip("Tt"))
    if n not in ATC_SLOTS:
      raise ValueError(f"ATC slot must be 1..6: {part}")
    bit = bit.strip()
    out[n] = (bit if bit.startswith("bit_") else "bit_" + bit) if bit else None
  return out


def gcode_targets(
  path: str, maps: list[str], accept_closest: bool
) -> dict[int, str | None]:
  """ATC targets for a Makera Studio job (--from_gcode), or exits with why not.

  Nothing moves unless every tool of the job has its bit: a wrong bit in a
  slot is worse than an empty one.
  """
  job = gcode_tools.plan(path, gcode_tools.parse_map(maps))
  print(
    f"Tools of {os.path.basename(path)}:\n{gcode_tools.report(job)}", flush=True
  )
  if job.problems:
    sys.exit(
      "Not loading the ATC: fix the lines marked '!' above (--gcode_map N=<bit id>)."
    )
  if job.guesses and not accept_closest:
    slots = ", ".join(f"T{n}" for n in job.guesses)
    sys.exit(
      f"Not loading the ATC: {slots} are custom tools (~). Name their bits with "
      "--gcode_map N=<bit id>, or pass --accept_closest to take the closest bits."
    )
  return parse_set_atc(job.spec)


def plan_set_atc(
  world: Any, targets: dict[int, str | None], catalog: dict[str, dict]
) -> list[tuple[str, str]]:
  """Moves that leave ATC slot n holding targets[n] (None: empty).

  First every slot that holds a bit it should not goes back to the
  carousel (the bit's own seat if free), then each wanted bit is fetched
  from wherever it is, carousel or another ATC slot.
  """
  occ = occupancy(world)
  moves = []

  def move(src, dst):
    moves.append((src, dst))
    occ[dst], occ[src] = occ[src], None

  def free_seat(bit):
    home = home_seat(bit, catalog)
    if home in occ and occ[home] is None:
      return home
    for slot, held in occ.items():
      if slot.startswith("grid:") and held is None:
        return slot
    raise ValueError("no free carousel seat")

  for bit in [b for b in targets.values() if b]:
    if bit not in occ.values():
      raise ValueError(f"{bit} is not on the carousel or in the ATC")
  for n in sorted(targets):
    cur = occ[f"atc:{n}"]
    if cur and cur != targets[n]:
      move(f"atc:{n}", free_seat(cur))
  for n in sorted(targets):
    want = targets[n]
    if want and occ[f"atc:{n}"] != want:
      src = next(slot for slot, held in occ.items() if held == want)
      move(src, f"atc:{n}")
  return moves


# ---------------------------------------------------------------- cell state
# The cell's state on disk: which bit is in which ATC slot and carousel seat,
# the carousel's angle, the cart and the hood. move_bit writes it after
# every move, turn, park and dock. On the real cell it is the record that
# outlives a restart of the solution (the world model starts from the
# configs' layout); --restore_state puts the world back to it, and
# --set_state records bits changed by hand. The sim forgets on every reset,
# so there the picker offers to restore it.
STATE_PATH = os.environ.get(
  "CARVERA_CELL_STATE",
  os.path.expanduser("~/.local/state/carvera_so101/cell_state.json"),
)


def cart_parked(world: Any) -> bool:
  cart, docked = _cart_target(world, parked=False)
  return float(cart.translation[0]) - float(docked.translation[0]) > PARK_DX / 2


def hood_open(world: Any) -> bool:
  return (
    float(list(world.get_kinematic_object(ATC_OBJECT).joint_positions)[0])
    > HOOD_OPEN / 2
  )


def snapshot(world: Any) -> dict:
  import datetime  # pylint: disable=import-outside-toplevel

  return {
    "slots": {slot: bit for slot, bit in occupancy(world).items() if bit},
    "carousel_rad": carousel_angle(world),
    "cart": "parked" if cart_parked(world) else "docked",
    "hood": "open" if hood_open(world) else "closed",
    "updated": datetime.datetime.now().isoformat(timespec="seconds"),
  }


def load_state() -> dict | None:
  try:
    with open(STATE_PATH) as fh:
      return json.load(fh)
  except (OSError, json.JSONDecodeError):
    return None


def save_state(world: Any, snap: dict | None = None) -> dict:
  snap = snap or snapshot(world)
  os.makedirs(os.path.dirname(STATE_PATH), exist_ok=True)
  tmp = STATE_PATH + ".tmp"
  with open(tmp, "w") as fh:
    json.dump(snap, fh, indent=1, sort_keys=True)
  os.replace(tmp, STATE_PATH)
  return snap


def state_differences(saved: dict | None, now: dict) -> list[str]:
  """What differs between the saved state and the world, for people."""
  if saved is None:
    return []
  out = []
  for slot in sorted(set(saved["slots"]) | set(now["slots"])):
    if saved["slots"].get(slot) != now["slots"].get(slot):
      out.append(
        f"{slot}: saved {saved['slots'].get(slot) or 'empty'}, "
        f"world {now['slots'].get(slot) or 'empty'}"
      )
  for key in ("cart", "hood"):
    if saved.get(key) != now.get(key):
      out.append(f"{key}: saved {saved.get(key)}, world {now.get(key)}")
  return out


def restore_state(solution: Any, state: dict) -> None:
  """Puts the world in the saved state, without moving the arm."""
  world = solution.world
  if state.get("carousel_rad") is not None and carousel(world) is not None:
    world.update_joint_positions(carousel(world), [state["carousel_rad"]])
  cart, goal = _cart_target(world, parked=state.get("cart") == "parked")
  obj = world.get_object(CART_OBJECT)
  world.update_transform(
    node_a=world.root, node_b=obj, node_to_update=obj, a_t_b=goal
  )
  world.update_joint_positions(
    world.get_kinematic_object(ATC_OBJECT),
    [HOOD_OPEN if state.get("hood") == "open" else 0.0],
  )
  for slot, bit in state["slots"].items():
    _seat_in_world(world, bit, slot)
  if getattr(solution, "is_simulated", False) and solution.simulator:
    solution.simulator.reset()


def record_set_atc(
  solution: Any, targets: dict[int, str | None]
) -> list[tuple[str, str]]:
  """Records that the ATC was changed by hand: the same moves as --set_atc,
  but the bits are only put there in the world, the arm does not move."""
  world = solution.world
  moves = plan_set_atc(world, targets, load_bit_catalog())
  holders = holder_positions(world)
  for src, dst in moves:
    _seat_in_world(world, holder_at(world, src, holders), dst)
    holders = holder_positions(world)
    print(f"[✓] recorded {src} -> {dst}")
  if getattr(solution, "is_simulated", False) and solution.simulator:
    solution.simulator.reset()
  return moves


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--address", default="localhost:17080")
  parser.add_argument(
    "--config", default="configs/carvera_so101/app_config.yaml"
  )
  parser.add_argument(
    "--move",
    action="append",
    default=[],
    help="'<src>:<dst>', e.g. grid:0,0:atc:1 (repeatable).",
  )
  parser.add_argument(
    "--fill_atc",
    action="store_true",
    help="Fill empty ATC slots T1..T6 from the grid.",
  )
  parser.add_argument(
    "--empty_atc",
    action="store_true",
    help="Return all holders in T1..T6 to free grid cells.",
  )
  parser.add_argument(
    "--list",
    action="store_true",
    help="Print which holder sits in which slot and exit.",
  )
  parser.add_argument(
    "--set_atc",
    default="",
    help="Make the ATC hold these bits, e.g. '1=bit_ofl4_22,2=,6=bn4_12' "
    "(empty = empty slot; slots not named stay as they are).",
  )
  parser.add_argument(
    "--from_gcode",
    default="",
    help="Load the ATC for a Makera Studio job: the G-code file's header "
    "says which bit is tool 1..6 (tools/atc/gcode_tools.py). Slots the "
    "job does not use stay as they are.",
  )
  parser.add_argument(
    "--gcode_map",
    action="append",
    default=[],
    help="With --from_gcode: the bit for a tool number, e.g. 2=fe18_1p5x8 "
    "(repeatable); needed for custom tools.",
  )
  parser.add_argument(
    "--accept_closest",
    action="store_true",
    help="With --from_gcode: for a custom tool, load the closest bit "
    "instead of stopping.",
  )
  parser.add_argument(
    "--list_json",
    action="store_true",
    help="Print the ATC slots' bits and the carousel angle as JSON and exit.",
  )
  parser.add_argument(
    "--park",
    action="store_true",
    help="SO-101 cell: arm home, roll the cart away, close the hood.",
  )
  parser.add_argument(
    "--dock",
    action="store_true",
    help="SO-101 cell: open the hood, roll the cart up to the Carvera.",
  )
  parser.add_argument(
    "--set_state",
    default="",
    help="Record bits changed by hand, same form as --set_atc: the world "
    "is set to match, the arm does not move.",
  )
  parser.add_argument(
    "--save_state",
    action="store_true",
    help=f"Write the world's cell state to {STATE_PATH} and exit.",
  )
  parser.add_argument(
    "--restore_state",
    action="store_true",
    help="Put the world back to the saved cell state (no arm motion) and exit.",
  )
  parser.add_argument(
    "--turn",
    default="",
    help="Carousel only: turn seat grid:R,C to its pick position and exit.",
  )
  parser.add_argument(
    "--yaw_deg",
    type=float,
    default=0.0,
    help="Gripper rotation about the tool axis at the seats.",
  )
  parser.add_argument(
    "--home_frame",
    default="view",
    help="Root frame to return to afterwards ('' to stay).",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  config = load_app_config(args.config)
  solution = deployments.connect(address=args.address)
  world = solution.world

  if args.turn:
    turn_carousel(world, args.turn)
    save_state(world)
    return
  if args.park:
    park(solution, config)
    save_state(world)
    return
  if args.dock:
    dock(solution, config)
    save_state(world)
    return
  if args.save_state:
    save_state(world)
    print(f"[✓] saved the cell state to {STATE_PATH}")
    return
  if args.restore_state:
    state = load_state()
    if state is None:
      raise SystemExit(f"No saved cell state at {STATE_PATH}.")
    restore_state(solution, state)
    print(f"[✓] restored the cell state of {state.get('updated')}")
    return
  if args.set_state:
    record_set_atc(solution, parse_set_atc(args.set_state))
    save_state(world)
    return

  if args.list_json:
    now = snapshot(world)
    saved = load_state()
    angle = now["carousel_rad"]
    print(
      json.dumps(
        {
          "atc": {str(n): now["slots"].get(f"atc:{n}") for n in ATC_SLOTS},
          "carousel_deg": None
          if angle is None
          else round(math.degrees(angle), 1),
          "cart": now["cart"],
          "hood": now["hood"],
          "saved": None if saved is None else saved.get("updated"),
          "differs": state_differences(saved, now),
        }
      )
    )
    return

  if args.list:
    angle = carousel_angle(world)
    if angle is not None:
      print(f"  carousel at {math.degrees(angle):.1f} deg")
    holders = holder_positions(world)
    for slot in all_slots(world):
      name = holder_at(world, slot, holders)
      if name or slot.startswith("atc:"):
        print(f"  {slot:10s} {name or '-'}")
    return

  # Each batch is planned on the world as the batch before left it, so
  # --fill_atc / --empty_atc see the slots the explicit --moves filled.
  batches = [lambda: [parse_move(m) for m in args.move]]
  if args.from_gcode or args.set_atc:
    targets = {}
    if args.from_gcode:
      targets.update(
        gcode_targets(args.from_gcode, args.gcode_map, args.accept_closest)
      )
    targets.update(parse_set_atc(args.set_atc))  # slots named by hand win

    def plan_set():
      moves = plan_set_atc(world, targets, load_bit_catalog())
      print(
        f"Plan: {len(moves)} moves"
        + "".join(f"\n  {a} -> {b}" for a, b in moves),
        flush=True,
      )
      return moves

    batches.append(plan_set)
  if args.fill_atc:
    batches.append(lambda: plan_fill_atc(world))
  if args.empty_atc:
    batches.append(lambda: plan_empty_atc(world))
  ran = False
  for plan in batches:
    moves = plan()
    if not moves:
      continue
    if not run_moves(
      solution, config, moves, args.yaw_deg, args.home_frame or None
    ):
      sys.exit(1)
    ran = True
  if not ran:
    print(
      "[✓] The ATC already holds those bits."
      if args.set_atc or args.from_gcode
      else "Nothing to do: give --move, --set_atc, --from_gcode, --fill_atc, --empty_atc, "
      "--park, --dock or --list."
    )


if __name__ == "__main__":
  main()
