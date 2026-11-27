"""Writes the bit-placement and SO-101 cell world updates.

Generates configs/carvera_so101/cell.updates.pbtxt and configs/carvera_so101/
bits.updates.pbtxt from the bit catalogue (models/makera_bits/makera_bits.json),
the carousel layout and the SO-101 kinematics.

  python3 configs/carvera_so101/gen_cell_configs.py
"""

import json
import math
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "atc"))
sys.path.insert(0, os.path.join(REPO, "models", "bit_carousel"))
import carousel_layout  # noqa: E402
from so101_ik import fk  # noqa: E402

os.chdir(REPO)

bits = json.load(open("models/makera_bits/makera_bits.json"))["bits"]
HDR = (
  "# proto-file: intrinsic/world/public/proto/object_world_updates.proto\n"
  "# proto-message: intrinsic_proto.world.ObjectWorldUpdates\n#\n"
)


def ref(obj, frame=None):
  return (
    f'by_name {{ frame {{ object_name: "{obj}" frame_name: "{frame}" }} }}'
    if frame
    else f'by_name {{ object {{ object_name: "{obj}" }} }}'
  )


def place(parent, child, xyz=(0, 0, 0), q=(0, 0, 0, 1), frame=None):
  x, y, z = xyz
  qx, qy, qz, qw = q
  return (
    f"updates {{\n  update_transform {{\n    node_a {{ {ref(parent, frame)} }}\n"
    f"    node_b {{ {ref(child)} }}\n    node_to_update {{ {ref(child)} }}\n"
    f"    a_t_b {{\n      position {{ x: {x:.5f} y: {y:.5f} z: {z:.5f} }}\n"
    f"      orientation {{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }}\n    }}\n  }}\n}}\n"
  )


def reparent(child, parent, frame):
  return (
    "updates {\n  reparent_object {\n"
    f'    object {{ by_name {{ object_name: "{child}" }} }}\n'
    f'    parent_frame {{ by_name {{ object_name: "{parent}" frame_name: "{frame}" }} }}\n'
    "  }\n}\n"
  )


def bits_in_rack():
  """Bits on the carousel's seat frames, ring by ring from the inside."""
  cells = carousel_layout.cells()
  assert len(bits) <= len(cells), (len(bits), len(cells))
  # Placing a bit at a seat frame does not attach it: reparent it so it turns
  # with the carousel's platter.
  return "".join(
    place("bit_grid", "bit_" + b["id"], frame=f"cell_{r}_{c}")
    + "updates {\n  reparent_object {\n"
    f'    object {{ by_name {{ object_name: "bit_{b["id"]}" }} }}\n'
    f'    parent_frame {{ by_name {{ object_name: "bit_grid" frame_name: "cell_{r}_{c}" }} }}\n'
    "  }\n}\n"
    for b, (r, c, *_rest) in zip(bits, cells, strict=False)
  )


# SO-101 cell: Carvera on its own bench, arm + grid on a rolling cart.
BENCH_Z = 0.84  # carvera_bench (Kuepper, 60 x 60 cm) top
CART_Z = (
  BENCH_Z + 0.150 - carousel_layout.SEAT_Z
)  # robot_cart top: carousel seats level with the ATC seats
CARVERA = (-0.46, -0.23)
# The 60 cm bench sits 4 cm further out than the Carvera's centre so its
# arm-side edge (x -0.20) is flush with the Carvera and 5 mm from the cart.
BENCH_C = (
  -0.50,
  -0.23,
)  # turned 90 deg like the Carvera: door under the Carvera's front (+x)
CART_C = (
  -0.0175,
  -0.1525,
)  # robot_cart top centre (x -0.195..0.16, y -0.37..0.065)
# SO-101 pan axis: on the pedestal's bracket over the Carvera's front lip,
# just inside the open hood's front and 2.5 cm in front of the right-hand side
# panel's edge (see models/so101/build_so101.py). The bit carousel
# (models/bit_carousel) stands beside it, its axis at carousel_layout.CENTER_IN_PAN.
STAND = np.array([-0.22, -0.025, CART_Z])
ARM_YAW = math.radians(270)
ARM_H = 0.103 + carousel_layout.SEAT_Z - 0.047
base = np.eye(4)
base[:3, :3] = [
  [math.cos(ARM_YAW), -math.sin(ARM_YAW), 0],
  [math.sin(ARM_YAW), math.cos(ARM_YAW), 0],
  [0, 0, 1],
]
base[:3, 3] = STAND + [
  -0.0388353 * math.cos(ARM_YAW),
  -0.0388353 * math.sin(ARM_YAW),
  ARM_H,
]
HOME = [-1.571, -0.743, 0.43, 1.618, 0.0]
view = base @ fk(HOME)


def quat(R):
  w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
  x = math.copysign(
    math.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2]
  )
  y = math.copysign(
    math.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0]
  )
  z = math.copysign(
    math.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1]
  )
  return x, y, z, w


vq = quat(view[:3, :3])
s2 = 0.70710678
cell = (
  HDR
  + f"""# SO-101 cell (--//:setup=carvera_so101): the Carvera on a Kuepper workshop
# bench (models/carvera_bench, 60 x 60 cm, top {BENCH_Z} m) and, in front of it, a rolling cart
# (models/robot_cart) carrying the SO-101 on a plywood pedestal and the
# bit carousel (models/bit_carousel, instance bit_grid) with every bit. The
# carousel's seat plate is level with the ATC seats; the pedestal's bracket
# puts the pan axis at world (-0.22, -0.025), just inside the open hood, and
# the arm base 5 cm above the grip height: ATC T1 0.27 m .. T6 0.12 m and the
# carousel's pick positions 0.16..0.225 m from the pan axis, all inside the
# SO-101's vertical-gripper reach. move_bit turns the carousel
# (carousel_joint) to bring a seat to its ring's pick position.
"""
  + place(
    "root", "carvera_bench", (BENCH_C[0], BENCH_C[1], 0.0), (0, 0, s2, s2)
  )
  + place(
    "root", "cnc_enclosure", (CARVERA[0], CARVERA[1], BENCH_Z), (0, 0, s2, s2)
  )
  + """updates {
  update_object_joints {
    object { by_name { object_name: "cnc_enclosure" } }
    joint_positions: 1.4866   # Carvera hood fully open
  }
}
"""
  + place("root", "robot_cart", (CART_C[0], CART_C[1], 0.0))
  + place("root", "so101_stand", tuple(STAND))
  + place("so101_stand", "so101_arm", frame="arm_mount")
  + place("so101_arm", "so101_gripper", frame="flange")
  # Placing it at the flange does not attach it: reparent so the gripper
  # (jaw, tool frame and anything attached to it) moves with the arm.
  + """updates {
  reparent_object {
    object { by_name { object_name: "so101_gripper" } }
    parent_frame { by_name { object_name: "so101_arm" frame_name: "flange" } }
  }
}
"""
  + place("so101_stand", "bit_grid", frame="grid_mount")
  # Everything on the cart rides with it (move_bit --park / --dock): the
  # pedestal on the cart top, the arm and the carousel on the pedestal.
  + reparent("so101_stand", "robot_cart", "top_center")
  + reparent("so101_arm", "so101_stand", "arm_mount")
  + reparent("bit_grid", "so101_stand", "grid_mount")
  + f"""# Home: tool frame at the arm's start / home configuration {HOME}
# (gripper 13 cm up, turned over the grid). move_bit returns here.
updates {{
  create_frame {{
    new_frame_name: "view"
    parent_t_new_frame {{
      position {{ x: {view[0, 3]:.5f} y: {view[1, 3]:.5f} z: {view[2, 3]:.5f} }}
      orientation {{ x: {vq[0]:.6f} y: {vq[1]:.6f} z: {vq[2]:.6f} w: {vq[3]:.6f} }}
    }}
    parent_object_with_filter {{ reference {{ by_name {{ object_name: "root" }} }} }}
  }}
}}
"""
)
open("configs/carvera_so101/cell.updates.pbtxt", "w").write(cell)
open("configs/carvera_so101/bits.updates.pbtxt", "w").write(
  HDR
  + "# Bits on the SO-101 cell's carousel (models/bit_carousel, instance bit_grid),\n"
  "# in catalogue order on its seat frames cell_R_C, ring R from the inside.\n"
  + bits_in_rack()
)
print(
  "cart top",
  CART_Z,
  "arm base",
  base[2, 3],
  "view",
  np.round(view[:3, 3], 4),
  "approach",
  np.round(view[:3, 2], 2),
  "bits",
  len(bits),
)
