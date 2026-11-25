"""Writes the bit-placement and SO-101 cell world updates.

Generates configs/carvera_so101/cell.updates.pbtxt, configs/carvera_so101/
bits.updates.pbtxt and configs/omts/bit_grid.updates.pbtxt from the bit
catalogue (models/makera_bits/makera_bits.json) and the SO-101 kinematics.

  python3 configs/carvera_so101/gen_cell_configs.py
"""
import json, math, os, sys
import numpy as np
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "atc"))
from so101_ik import fk
os.chdir(REPO)

bits = json.load(open("models/makera_bits/makera_bits.json"))["bits"]
SEAT_Z = 0.047
HDR = ("# proto-file: intrinsic/world/public/proto/object_world_updates.proto\n"
       "# proto-message: intrinsic_proto.world.ObjectWorldUpdates\n#\n")

def ref(obj, frame=None):
  return (f'by_name {{ frame {{ object_name: "{obj}" frame_name: "{frame}" }} }}' if frame
          else f'by_name {{ object {{ object_name: "{obj}" }} }}')

def place(parent, child, xyz=(0, 0, 0), q=(0, 0, 0, 1), frame=None):
  x, y, z = xyz; qx, qy, qz, qw = q
  return (f"updates {{\n  update_transform {{\n    node_a {{ {ref(parent, frame)} }}\n"
          f"    node_b {{ {ref(child)} }}\n    node_to_update {{ {ref(child)} }}\n"
          f"    a_t_b {{\n      position {{ x: {x:.5f} y: {y:.5f} z: {z:.5f} }}\n"
          f"      orientation {{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }}\n    }}\n  }}\n}}\n")

def bits_in_grid(pitch):
  out = []
  for i, b in enumerate(bits):
    r, c = divmod(i, 10)
    out.append(place("bit_grid", "bit_" + b["id"], ((c - 4.5) * pitch, (r - 4.5) * pitch, SEAT_Z)))
  return "".join(out)

def bit_comment(pitch):
  return (f"# The {len(bits)} Makera bits (models/makera_bits), bit_<id>, in catalogue order\n"
          f"# row by row from cell_0_0 ({pitch * 1000:.0f} mm pitch, seats {SEAT_Z * 1000:.0f} mm above the\n"
          "# grid base). //tools/atc:move_bit moves them to and from the Carvera ATC.\n")

# UR5e cell: same grid placement as before, new bits.
open("configs/omts/bit_grid.updates.pbtxt", "w").write(
    HDR + "# Tool-bit storage grid (models/bit_grid) on the table, robot side.\n" + bit_comment(0.035)
    + place("enclosure", "bit_grid", (0.135, 0.305, 1.025)) + bits_in_grid(0.035))

# SO-101 cell: Carvera on its own bench, arm + grid on a rolling cart.
BENCH_Z = 0.84                         # carvera_bench (Kuepper, 60 x 60 cm) top
CART_Z = BENCH_Z + 0.150 - 0.047       # robot_cart top: grid seats level with the ATC seats
CARVERA = (-0.46, -0.23)
# The 60 cm bench sits 4 cm further out than the Carvera's centre so its
# arm-side edge (x -0.20) is flush with the Carvera and 5 mm from the cart.
BENCH_C = (-0.50, -0.23)   # turned 90 deg like the Carvera: door under the Carvera's front (+x)
CART_C = (-0.0175, 0.0825)             # robot_cart top centre (x -0.195..0.16, y -0.135..0.30)
STAND = np.array([-0.175, -0.025, CART_Z])
ARM_YAW = math.radians(135); ARM_H = 0.103
base = np.eye(4); base[:3, :3] = [[math.cos(ARM_YAW), -math.sin(ARM_YAW), 0], [math.sin(ARM_YAW), math.cos(ARM_YAW), 0], [0, 0, 1]]
base[:3, 3] = STAND + [-0.0388353 * math.cos(ARM_YAW), -0.0388353 * math.sin(ARM_YAW), ARM_H]
HOME = [0.6, -1.1, 0.5, 1.6, 0.0]
view = base @ fk(HOME)
def quat(R):
  w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
  x = math.copysign(math.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
  y = math.copysign(math.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
  z = math.copysign(math.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
  return x, y, z, w
vq = quat(view[:3, :3])
s2 = 0.70710678
cell = (HDR +
f"""# SO-101 cell (--//:setup=carvera_so101): the Carvera on a Kuepper workshop
# bench (models/carvera_bench, 60 x 60 cm, top {BENCH_Z} m) and, in front of it, a rolling cart
# (models/robot_cart) carrying the SO-101 on a plywood pedestal and the 21 mm
# bit grid. The cart top is level with the ATC seats; the pedestal puts the
# arm base 5 cm above the grip height, so the ATC and the grid are inside the
# SO-101's top-down reach (7..33 cm from the pan axis). Pan axis at world
# (-0.175, -0.025): ATC T1 0.316 m, T6 0.166 m, grid cells 0.08..0.33 m.
# The vise comes from configs/omts/schunk.updates.pbtxt (relative to the Carvera).
""" + place("root", "carvera_bench", (BENCH_C[0], BENCH_C[1], 0.0), (0, 0, s2, s2))
    + place("root", "cnc_enclosure", (CARVERA[0], CARVERA[1], BENCH_Z), (0, 0, s2, s2))
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
""")
open("configs/carvera_so101/cell.updates.pbtxt", "w").write(cell)
open("configs/carvera_so101/bits.updates.pbtxt", "w").write(
    HDR + "# Bits in the SO-101 cell's 21 mm grid (models/bit_grid_21, instance bit_grid).\n"
    + bit_comment(0.021) + bits_in_grid(0.021))
print("cart top", CART_Z, "arm base", base[2,3], "view", np.round(view[:3, 3], 4), "approach", np.round(view[:3, 2], 2), "bits", len(bits))
