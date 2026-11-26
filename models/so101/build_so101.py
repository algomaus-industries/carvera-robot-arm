"""SO-ARM101 arm, jaw and riser stand as Intrinsic scene objects.

Run headless:
  blender -b --python build_so101.py -- <models_dir>            # build assets
  Q=q1,..,q5 JAW=a blender -b --python build_so101.py -- <dir> out.png   # preview
  blender -b --python build_so101.py -- <dir> --calibrate-jaw   # jaw gap table

Writes <models_dir>/so101/{so101.sdf, so101_<link>.glb},
       <models_dir>/so101_gripper/{so101_gripper.sdf, so101_jaw.glb},
       <models_dir>/so101_stand/{so101_stand.sdf, so101_stand.glb}.

Kinematics, visual placements and meshes are TheRobotStudio's official SO-101
model: SO-ARM100 repo, Simulation/SO101/so101_new_calib.urdf and assets/*.stl
(Apache License 2.0, copy in official_meshes/LICENSE). The meshes are
decimated and coloured here (printed parts white, STS3215 servos black).
Collision geometry is boxes fitted to slices of each mesh, 1 mm proud.

The URDF's gripper joint (moving jaw) is split off into its own scene object,
so101_gripper, attached at the arm's flange frame: ICON then controls exactly
the five arm joints, and the jaw is set through the world (update_world), like
the Carvera hood. so101_gripper.tool_frame is the URDF gripper_frame_link: the
point between the fingertips, z along the approach direction.
"""

import math
import os
import sys

import bmesh
import bpy
from mathutils import Euler, Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.join(HERE, "official_meshes")

# ---------------------------------------------------------------- kinematics
# (name, parent, child, origin xyz, origin rpy, (lower, upper)), verbatim URDF.
JOINTS = [
    ("shoulder_pan", "base_link", "shoulder_link",
     (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159), (-1.91986, 1.91986)),
    ("shoulder_lift", "shoulder_link", "upper_arm_link",
     (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0), (-1.74533, 1.74533)),
    ("elbow_flex", "upper_arm_link", "lower_arm_link",
     (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708), (-1.69, 1.69)),
    ("wrist_flex", "lower_arm_link", "wrist_link",
     (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708), (-1.65806, 1.65806)),
    ("wrist_roll", "wrist_link", "gripper_link",
     (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), (-2.74385, 2.84121)),
]
MASS = {"base_link": 0.147, "shoulder_link": 0.100, "upper_arm_link": 0.103,
        "lower_arm_link": 0.104, "wrist_link": 0.079, "gripper_link": 0.087}
LINKS = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link",
         "gripper_link"]
JAW_ORIGIN = ((0.0202, 0.0188, -0.0234), (1.5708, -5.24284e-08, -1.41553e-15))
JAW_LIMITS = (-0.174533, 1.74533)
TIP = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))

# Collision boxes along each finger. The fingers taper to ~8 x 10 mm at the
# tip; with 4 boxes the tip box was ~20 x 19 mm and the open jaw "hit" bits
# a seat away in the rack that the real jaw clears.
GRIPPER_SLICES = 14

# Visuals: link -> [(stl, xyz, rpy, colour, collision slices)], verbatim URDF.
WHITE = (0.94, 0.94, 0.92, 1.0)        # printed PLA, as on Seeed's kit photos
BLACK = (0.06, 0.06, 0.07, 1.0)        # Feetech STS3215
VISUALS = {
    "base_link": [
        ("base_motor_holder_so101_v1", (-0.00636471, -9.94414e-05, -0.0024),
         (1.5708, -1.67685e-15, 1.5708), WHITE, 2),
        ("base_so101_v2", (-0.00636471, -8.97657e-09, -0.0024), (1.5708, -2.78073e-29, 1.5708),
         WHITE, 2),
        ("sts3215_03a_v1", (0.0263353, -8.97657e-09, 0.0437), (-8.21148e-16, 7.84513e-18, 1.249e-15),
         BLACK, 0),
        ("waveshare_mounting_plate_so101_v2", (-0.0309827, -0.000199441, 0.0474),
         (1.5708, -1.35493e-14, 1.5708), WHITE, 1),
    ],
    "shoulder_link": [
        ("sts3215_03a_v1", (-0.0303992, 0.000422241, -0.0417), (1.5708, 1.5708, 0), BLACK, 1),
        ("motor_holder_so101_base_v1", (-0.0675992, -0.000177759, 0.0158499), (1.5708, -1.5708, 0),
         WHITE, 2),
        ("rotation_pitch_so101_v1", (0.0122008, 2.22413e-05, 0.0464), (-1.5708, 2.35221e-33, 0),
         WHITE, 2),
    ],
    "upper_arm_link": [
        ("sts3215_03a_v1", (-0.11257, -0.0155, 0.0187), (-3.14159, -5.27356e-16, -1.5708), BLACK, 1),
        ("upper_arm_so101_v1", (-0.065085, 0.012, 0.0182), (3.14159, 0, -1.30911e-30), WHITE, 3),
    ],
    "lower_arm_link": [
        ("under_arm_so101_v1", (-0.0648499, -0.032, 0.0182), (3.14159, 0, 6.67202e-31), WHITE, 3),
        ("motor_holder_so101_wrist_v1", (-0.0648499, -0.032, 0.018),
         (-3.14159, -2.55351e-15, -1.83387e-30), WHITE, 1),
        ("sts3215_03a_v1", (-0.1224, 0.0052, 0.0187), (-3.14159, -7.88861e-31, -3.14159), BLACK, 1),
    ],
    "wrist_link": [
        ("sts3215_03a_no_horn_v1", (8.32667e-17, -0.0424, 0.0306), (1.5708, 1.5708, 0), BLACK, 1),
        ("wrist_roll_pitch_so101_v2", (0, -0.028, 0.0181), (-1.5708, -1.5708, 0), WHITE, 2),
    ],
    "gripper_link": [
        ("sts3215_03a_v1", (0.0077, 0.0001, -0.0234), (-1.5708, -5.19179e-17, -1.66533e-16),
         BLACK, 1),
        ("wrist_roll_follower_so101_v1", (8.32667e-17, -0.000218214, 0.000949706),
         (-3.14159, -5.55112e-17, 0), WHITE, GRIPPER_SLICES),
    ],
}
JAW_VISUAL = ("moving_jaw_so101_v1", (-5.55112e-17, -5.55112e-17, 0.0189),
              (9.53145e-17, 6.93889e-18, 1.24077e-24), WHITE, GRIPPER_SLICES)
DECIMATE = 0.35                        # official meshes are ~30k triangles each

# Joint limits handed to ICON (STS3215: ~0.22 s / 60 deg no load at 7.4 V,
# 2.9 N m stall). Conservative: a hobby servo arm, not a cobot.
MAX_VEL, MAX_ACC, MAX_JERK, MAX_EFFORT = 1.5, 6.0, 200.0, 2.5


def xform(xyz, rpy):
  return Matrix.Translation(Vector(xyz)) @ Euler(rpy, "XYZ").to_matrix().to_4x4()


def rot_z(q):
  return Matrix.Rotation(q, 4, "Z")


def link_frames(q):
  frames = {"base_link": Matrix.Identity(4)}
  for (name, parent, child, xyz, rpy, _), qi in zip(JOINTS, q):
    frames[child] = frames[parent] @ xform(xyz, rpy) @ rot_z(qi)
  return frames


# ---------------------------------------------------------------- blender
def material(color):
  key = "m_%02x%02x%02x%02x" % tuple(int(c * 255) for c in color)
  m = bpy.data.materials.get(key)
  if m:
    return m
  m = bpy.data.materials.new(key)
  m.use_nodes = True
  bsdf = m.node_tree.nodes["Principled BSDF"]
  bsdf.inputs["Base Color"].default_value = color
  bsdf.inputs["Roughness"].default_value = 0.55 if color == WHITE else 0.35
  m.diffuse_color = color              # what the Workbench preview shows
  return m


_mesh_cache = {}


def load_stl(stem):
  """Imported, decimated mesh datablock for an official STL (cached)."""
  if stem in _mesh_cache and _mesh_cache[stem].name in bpy.data.meshes:
    return _mesh_cache[stem]
  bpy.ops.import_mesh.stl(filepath=os.path.join(MESHES, stem + ".stl"))
  o = bpy.context.object
  mod = o.modifiers.new("dec", "DECIMATE")
  mod.ratio = DECIMATE
  bpy.ops.object.modifier_apply(modifier="dec")
  bpy.ops.object.shade_smooth()
  o.data.use_auto_smooth = True
  o.data.auto_smooth_angle = math.radians(35)
  me = o.data
  me.name = stem
  bpy.data.objects.remove(o)
  _mesh_cache[stem] = me
  return me


def place(stem, m, color, name=None):
  o = bpy.data.objects.new(name or stem, load_stl(stem))
  bpy.context.scene.collection.objects.link(o)
  o.matrix_world = m
  if not o.data.materials:
    o.data.materials.append(material(color))
  return o


def reset():
  bpy.ops.wm.read_factory_settings(use_empty=True)
  _mesh_cache.clear()


def export_link(entries, path, extra=Matrix.Identity(4)):
  reset()
  for stem, xyz, rpy, color, _ in entries:
    place(stem, extra @ xform(xyz, rpy), color)
  bpy.ops.export_scene.gltf(filepath=os.path.abspath(path), export_format="GLB",
                            export_yup=False, export_apply=True)


def mesh_slices(stem, n):
  """Boxes (centre, size) in the mesh frame covering the mesh: n slabs along
  its longest axis, each the bounding box of the vertices in it."""
  me = load_stl(stem)
  vs = [v.co.copy() for v in me.vertices]
  lo = Vector([min(v[i] for v in vs) for i in range(3)])
  hi = Vector([max(v[i] for v in vs) for i in range(3)])
  ax = max(range(3), key=lambda i: hi[i] - lo[i])
  out = []
  for k in range(n):
    a0 = lo[ax] + (hi[ax] - lo[ax]) * k / n
    a1 = lo[ax] + (hi[ax] - lo[ax]) * (k + 1) / n
    sel = [v for v in vs if a0 - 1e-6 <= v[ax] <= a1 + 1e-6]
    if not sel:
      continue
    slo = Vector([min(v[i] for v in sel) for i in range(3)])
    shi = Vector([max(v[i] for v in sel) for i in range(3)])
    slo[ax], shi[ax] = a0, a1
    out.append(((slo + shi) / 2, shi - slo))
  return out


# ---------------------------------------------------------------- stand
STAND_COLOR = (0.62, 0.48, 0.32, 1.0)  # plywood block (xTool-cut box)
# Arm pedestal on the robot cart. Origin on the cart top under the arm's pan
# axis, axes along the world's. The pedestal overhangs the cart's front edge
# by 4.5 cm as a bracket over the Carvera's front lip, so the pan axis sits
# just inside the open hood's front, 2.5 cm in front of the right-hand side
# panel's edge: from there the SO-101 reaches all six ATC slots with its
# gripper vertical. The arm faces 270 deg (world -y), so its +-110 deg
# shoulder_pan range runs from the ATC (-x) across the open front of the
# machine to the bit carousel (models/bit_carousel) and never turns
# towards the panel. The carousel's seat plate is level with the ATC seats;
# the pedestal lifts the arm base 5 cm above the grip height (= seat + 6 mm).
ARM_YAW = math.radians(270)
sys.path.insert(0, os.path.join(HERE, "..", "bit_carousel"))
import carousel_layout  # noqa: E402  pylint: disable=wrong-import-position
# The cart top sits carousel_layout.SEAT_Z below the ATC seats (the carousel's
# seat plate is level with them), and the pedestal brings the arm base back to
# 5 cm above the grip height: 0.103 m over the old 47 mm rack seats.
ARM_BASE_H = 0.103 + carousel_layout.SEAT_Z - 0.047
PEDESTAL = dict(lo=(-0.020, -0.035, 0.0), hi=(0.050, 0.075, ARM_BASE_H))
GRID_CENTER = carousel_layout.CENTER_IN_PAN   # bit_carousel turntable axis, on the cart top


# The printed pedestal (fab/pedestal) rests on the cart only from its front
# edge (pan frame x 0.025) on, so a foot reaches back onto the cart to x 0.085.
PEDESTAL_FOOT = dict(lo=(0.025, -0.027, 0.0), hi=(0.085, 0.067, 0.006))


def stand_boxes():
  return [(n, (Vector(b["lo"]) + Vector(b["hi"])) / 2, Vector(b["hi"]) - Vector(b["lo"]))
          for n, b in (("pedestal", PEDESTAL), ("pedestal_foot", PEDESTAL_FOOT))]


def export_stand(path):
  reset()
  for name, c, sz in stand_boxes():
    bpy.ops.mesh.primitive_cube_add(size=1, location=c)
    o = bpy.context.object
    o.scale = sz
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    mod = o.modifiers.new("bevel", "BEVEL")
    mod.width, mod.segments = 0.003, 3
    o.name = name
    o.data.materials.append(material(STAND_COLOR))
  bpy.ops.export_scene.gltf(filepath=os.path.abspath(path), export_format="GLB",
                            export_yup=False, export_apply=True)


# ---------------------------------------------------------------- benches
# carvera_bench: fixed bench just larger than the Carvera (580 x 520 mm).
# robot_cart: rolling cart in front of it, top level with the ATC seats
# (bench top + 150 mm), carrying the arm pedestal and the bit grid.
# Both: origin on the floor at the centre of the top; frame top_center.
BEECH = (0.80, 0.62, 0.42, 1.0)
FRAME = (0.22, 0.23, 0.25, 1.0)
RUBBER = (0.10, 0.10, 0.11, 1.0)
# carvera_bench: Sarah's Kuepper workshop bench, 60 cm model with one door and
# one drawer (retailer listing: 60 x 60 cm, 84 cm working height, 30 mm solid
# beech top, adjustable feet). Panel thicknesses, drawer / door heights and
# the plinth recess are ESTIMATED from product photos. Front (door) faces -y.
BENCH_TOP_Z = 0.84
BENCHES = {
    "carvera_bench": dict(w=0.60, d=0.60, h=BENCH_TOP_Z, casters=False, style="kuepper",
                          drawers=1),
    "robot_cart": dict(w=0.355, d=0.435, h=BENCH_TOP_Z + 0.150 - carousel_layout.SEAT_Z, casters=True),
}
KUEPPER_GREY = (0.47, 0.50, 0.53, 1.0)
KUEPPER_RED = (0.60, 0.07, 0.06, 1.0)
CHROME = (0.80, 0.81, 0.83, 1.0)


def kuepper_boxes(b):
  """Kuepper bench: steel cabinet (grey frame, red door and drawers), beech top."""
  w, d, h = b["w"], b["d"], b["h"]
  top_t, over = 0.030, 0.020            # top thickness, overhang past the body
  bw, bd = w - 2 * over, d - 2 * over   # cabinet body
  foot, plinth = 0.035, 0.095           # adjustable feet; recessed plinth height
  side_t, top_rail = 0.030, 0.035
  zb = foot                              # body bottom
  zt = h - top_t                         # body top
  fy = -bd / 2                           # front face of the body
  out = [("top", "box", (0, 0, h - top_t / 2), (w, d, top_t), BEECH, True)]
  # Side frames (full height, standing on the feet) and back panel.
  for sx in (-1, 1):
    out.append((f"side_{sx}", "box", (sx * (bw - side_t) / 2, 0, (zb + zt) / 2),
                (side_t, bd, zt - zb), KUEPPER_GREY, False))
  out.append(("back", "box", (0, bd / 2 - 0.008, (zb + plinth + zt) / 2),
              (bw - 2 * side_t, 0.016, zt - zb - plinth), KUEPPER_GREY, False))
  # Plinth: recessed front, grey cross rail on top of it.
  out.append(("plinth", "box", (0, 0.03, zb + plinth / 2),
              (bw - 2 * side_t, bd - 0.06, plinth), KUEPPER_GREY, False))
  out.append(("bottom_rail", "box", (0, fy + 0.015, zb + plinth + 0.015),
              (bw - 2 * side_t, 0.03, 0.03), KUEPPER_GREY, False))
  # Front: top rail, two drawers, middle rail, door.
  inner_w = bw - 2 * side_t - 0.006
  z = zt - top_rail
  out.append(("top_rail", "box", (0, fy + 0.015, z + top_rail / 2),
              (bw - 2 * side_t, 0.03, top_rail), KUEPPER_GREY, False))
  n = b.get("drawers", 1)
  for i in range(n):
    dh = 0.085 if n == 1 else 0.068
    z -= dh + 0.004
    out.append((f"drawer_{i}", "box", (0, fy - 0.004, z + dh / 2), (inner_w, 0.02, dh),
                KUEPPER_RED, False))
    out.append((f"handle_{i}", "box", (0, fy - 0.022, z + dh / 2), (0.14, 0.018, 0.018),
                CHROME, False))
    out.append((f"label_{i}", "box", (-0.13, fy - 0.0145, z + dh / 2), (0.05, 0.003, 0.018),
                CHROME, False))
  mid = 0.035
  z -= mid + 0.004
  out.append(("mid_rail", "box", (0, fy + 0.015, z + mid / 2),
              (bw - 2 * side_t, 0.03, mid), KUEPPER_GREY, False))
  door_bot = zb + plinth + 0.035
  out.append(("door", "box", (0, fy - 0.004, (door_bot + z - 0.004) / 2),
              (inner_w, 0.02, z - 0.004 - door_bot), KUEPPER_RED, False))
  out.append(("lock", "cyl", (-inner_w / 2 + 0.035, fy - 0.016, z - 0.05), (0.008, 0.008),
              CHROME, False))
  # Body as one collision box ("collision_only": not drawn).
  out.append(("collision_only_body", "box", (0, 0, (zb + zt) / 2), (bw, bd, zt - zb),
              KUEPPER_GREY, True))
  for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
    out.append((f"foot_{i}", "box", (sx * (bw / 2 - side_t / 2), sy * (bd / 2 - 0.03), foot / 2),
                (0.035, 0.035, foot), RUBBER, False))
  return out


def bench_boxes(b):
  """(name, kind, centre, size, colour, collide); kind box or cyl (size r, len, axis y)."""
  if b.get("style") == "kuepper":
    return kuepper_boxes(b)
  # Rolling cart, as built from fab/cart: 18 mm birch-ply top, aprons and
  # shelf, 40 x 40 mm legs, four 50 mm braked swivel casters (75 mm high).
  w, d, h, top_t, leg = b["w"], b["d"], b["h"], 0.018, 0.040
  wheel = 0.075 if b["casters"] else 0.0
  hw, hd, ins, apron = w / 2, d / 2, 0.035, 0.060
  out = [("top", "box", (0, 0, h - top_t / 2), (w, d, top_t), BEECH, True)]
  for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
    x, y = sx * (hw - ins), sy * (hd - ins)
    out.append((f"leg_{i}", "box", (x, y, (h - top_t + wheel) / 2), (leg, leg, h - top_t - wheel),
                BEECH, True))
    if b["casters"]:
      # plate, swivel housing, fork, 50 mm wheel (axle along x), brake pedal
      out.append((f"caster_plate_{i}", "box", (x, y, wheel - 0.002), (0.060, 0.045, 0.004), CHROME, False))
      out.append((f"caster_swivel_{i}", "cyl", (x, y, wheel - 0.010), (0.022, 0.012, "z"), CHROME, False))
      for fx in (-1, 1):
        out.append((f"caster_fork_{i}_{fx}", "box", (x + fx * 0.013, y + 0.008, 0.042),
                    (0.003, 0.030, 0.042), CHROME, False))
      out.append((f"caster_wheel_{i}", "cyl", (x, y + 0.008, 0.025), (0.025, 0.020, "x"), RUBBER, False))
      out.append((f"caster_hub_{i}", "cyl", (x, y + 0.008, 0.025), (0.010, 0.022, "x"), CHROME, False))
      out.append((f"caster_brake_{i}", "box", (x, y - 0.016, 0.052), (0.016, 0.018, 0.004),
                  KUEPPER_RED, False))
  for sy in (-1, 1):
    out.append((f"apron_{sy}", "box", (0, sy * (hd - ins + leg / 2 - top_t / 2), h - top_t - apron / 2),
                (w - 2 * ins - leg, top_t, apron), BEECH, False))
  for sx in (-1, 1):
    out.append((f"apron_x{sx}", "box", (sx * (hw - ins + leg / 2 - top_t / 2), 0, h - top_t - apron / 2),
                (top_t, d - 2 * ins - leg, apron), BEECH, False))
  out.append(("shelf", "box", (0, 0, 0.150 + top_t / 2), (w - 2 * ins + 0.03, d - 2 * ins + 0.03, top_t),
              BEECH, True))
  return out


def export_bench(b, path):
  reset()
  for name, kind, c, sz, color, _ in bench_boxes(b):
    if name.startswith("collision_only"):
      continue
    if kind == "box":
      bpy.ops.mesh.primitive_cube_add(size=1, location=c)
      o = bpy.context.object
      o.scale = sz
      bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
      mod = o.modifiers.new("bevel", "BEVEL")
      mod.width, mod.segments = (0.004 if color == BEECH else 0.002), 3
    else:
      axis = sz[2] if len(sz) > 2 else "y"
      rot = {"x": (0, math.pi / 2, 0), "y": (math.pi / 2, 0, 0), "z": (0, 0, 0)}[axis]
      bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=sz[0], depth=sz[1], location=c,
                                          rotation=rot)
      o = bpy.context.object
    o.name = name
    o.data.materials.append(material(color))
  bpy.ops.export_scene.gltf(filepath=os.path.abspath(path), export_format="GLB",
                            export_yup=False, export_apply=True)


def bench_sdf(name, b):
  cols = "\n".join(
      f'      <collision name="{n}_collision">\n        <pose>{f3(c)} 0 0 0</pose>\n'
      f'        <geometry><box><size>{f3(sz)}</size></box></geometry>\n      </collision>'
      for n, kind, c, sz, _, col in bench_boxes(b) if col and kind == "box")
  what = ("Rolling cart on four casters" if b["casters"] else
          "Kuepper workshop bench (one door, one drawer)" if b.get("style") == "kuepper" else
          "Workshop bench")
  return f"""<?xml version="1.0" ?>
<!-- Generated by build_so101.py. {what}, {b['w']} x {b['d']} m, top at
     {b['h']:.3f} m: beech top, steel frame. Origin on the floor
     under the centre of the top; frame top_center on the work surface. -->
<sdf version="1.10" xmlns:intrinsic="https://intrinsic.ai/">
  <model name="{name}">
    <static>true</static>
    <link name="base_link">
{visual("bench_visual", name + ".glb")}
{cols}
    </link>
    <frame name="top_center" attached_to="base_link" intrinsic:create_entity="true">
      <pose>0 0 {b['h']:.4f} 0 0 0</pose>
    </frame>
  </model>
</sdf>
"""


# ---------------------------------------------------------------- sdf
HEADER = """<?xml version="1.0" ?>
<!--
Generated by build_so101.py - edit that script, not this file.
Kinematics and meshes: TheRobotStudio SO-ARM100, Simulation/SO101
(so101_new_calib.urdf, assets/*.stl), Apache License 2.0.
-->"""


def f3(v):
  return " ".join("%.6f" % x for x in v)


def pose_of(m):
  return f"{f3(m.translation)} {f3(m.to_euler('XYZ'))}"


def collisions(entries, indent="      ", base=Matrix.Identity(4)):
  out = []
  for stem, xyz, rpy, _, n in entries:
    m = base @ xform(xyz, rpy)
    for k, (c, s) in enumerate(mesh_slices(stem, n) if n else []):
      out.append(f'{indent}<collision name="{stem}_{k}_collision">\n'
                 f'{indent}  <pose>{pose_of(m @ Matrix.Translation(c))}</pose>\n'
                 f'{indent}  <geometry><box><size>{f3([v + 0.002 for v in s])}</size></box></geometry>\n'
                 f'{indent}</collision>')
  return "\n".join(out)


def visual(name, uri, indent="      "):
  return (f'{indent}<visual name="{name}">\n'
          f'{indent}  <geometry><mesh><uri>{uri}</uri></mesh></geometry>\n'
          f'{indent}</visual>')


def inertial(mass, indent="      "):
  i = mass * 0.05 ** 2 / 6
  return (f"{indent}<inertial><mass>{mass}</mass><inertia><ixx>{i:.3e}</ixx><iyy>{i:.3e}</iyy>"
          f"<izz>{i:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


DIO_PLUGIN = """    <!-- Digital IO blocks as on the UR model, so the Carvera door / vise
         handshake (DioCncMachine) works unchanged in this cell. -->
    <plugin filename="static://intrinsic::simulation::DigitalInputOutput" name="DigitalInputOutput">
      <input_block name="standard_digital_input_status" num_bits="8" />
      <input_block name="standard_digital_output_status" num_bits="8" />
      <output_block name="standard_digital_output_command" num_bits="8" />
    </plugin>"""


def arm_sdf():
  links, joints = [], []
  for l in LINKS:
    jn = [j[0] for j in JOINTS if j[2] == l]
    pose = f'      <pose relative_to="{jn[0]}">0 0 0 0 0 0</pose>\n' if jn else ""
    links.append(f'    <link name="{l}">\n{pose}{inertial(MASS[l])}\n'
                 f'{visual(l + "_visual", "so101_" + l + ".glb")}\n'
                 f'{collisions(VISUALS[l])}\n    </link>')
  for name, parent, child, xyz, rpy, (lo, hi) in JOINTS:
    joints.append(f"""    <joint name="{name}" type="revolute">
      <pose relative_to="{parent}">{f3(xyz)} {f3(rpy)}</pose>
      <parent>{parent}</parent>
      <child>{child}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>{lo}</lower><upper>{hi}</upper><effort>{MAX_EFFORT}</effort><velocity>{MAX_VEL}</velocity></limit>
      </axis>
    </joint>""")
  return f"""{HEADER}
<!-- SO-ARM101 follower arm, 5 joints. flange: gripper_link origin (wrist-roll
     output); attach so101_gripper there with an identity transform. -->
<sdf version="1.10" xmlns:intrinsic="https://intrinsic.ai/">
  <model name="so101">
    <intrinsic:ik_solver>kinematic_chain</intrinsic:ik_solver>
    <intrinsic:cartesian_limits>
      <intrinsic:min_translational_position>-5 -5 -5</intrinsic:min_translational_position>
      <intrinsic:max_translational_position>5 5 5</intrinsic:max_translational_position>
    </intrinsic:cartesian_limits>
{chr(10).join(links)}
{chr(10).join(joints)}
    <frame name="flange" attached_to="gripper_link" intrinsic:create_attachment_entity="true"/>
{DIO_PLUGIN}
  </model>
</sdf>
"""


def gripper_sdf():
  return f"""{HEADER}
<!-- SO-ARM101 moving jaw (URDF joint "gripper"). Attach base_link to
     so101.flange with an identity transform. jaw_joint: URDF convention,
     see the jaw table printed by build_so101.py for gap vs angle.
     tool_frame: URDF gripper_frame_link, between the fingertips, z along the
     approach direction. -->
<sdf version="1.10" xmlns:intrinsic="https://intrinsic.ai/">
  <model name="so101_gripper">
    <link name="base_link">
{inertial(0.001)}
    </link>
    <link name="jaw_link">
      <pose relative_to="jaw_joint">0 0 0 0 0 0</pose>
{inertial(0.012)}
{visual("jaw_visual", "so101_jaw.glb")}
{collisions([JAW_VISUAL])}
    </link>
    <joint name="jaw_joint" type="revolute">
      <pose relative_to="base_link">{f3(JAW_ORIGIN[0])} {f3(JAW_ORIGIN[1])}</pose>
      <parent>base_link</parent>
      <child>jaw_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>{JAW_LIMITS[0]}</lower><upper>{JAW_LIMITS[1]}</upper><effort>2.5</effort><velocity>3.0</velocity></limit>
      </axis>
    </joint>
    <frame name="tool_frame" attached_to="base_link" intrinsic:create_entity="true">
      <pose>{f3(TIP[0])} {f3(TIP[1])}</pose>
    </frame>
  </model>
</sdf>
"""


def stand_sdf():
  cols = "\n".join(
      f'      <collision name="{n}_collision">\n        <pose>{f3(c)} 0 0 0</pose>\n'
      f'        <geometry><box><size>{f3(s)}</size></box></geometry>\n      </collision>'
      for n, c, s in stand_boxes())
  return f"""{HEADER}
<!-- Plywood arm pedestal on the SO-101 robot cart. Origin on the cart top
     under the arm's pan axis, world-aligned. arm_mount: where so101 goes
     (yaw 270 deg); grid_mount: bit_carousel's turntable axis on the cart top. -->
<sdf version="1.10" xmlns:intrinsic="https://intrinsic.ai/">
  <model name="so101_stand">
    <static>true</static>
    <link name="stand">
{visual("stand_visual", "so101_stand.glb")}
{cols}
    </link>
    <frame name="arm_mount" attached_to="stand" intrinsic:create_entity="true">
      <pose>{-0.0388353 * math.cos(ARM_YAW):.5f} {-0.0388353 * math.sin(ARM_YAW):.5f} {ARM_BASE_H} 0 0 {ARM_YAW:.6f}</pose>
    </frame>
    <frame name="grid_mount" attached_to="stand" intrinsic:create_entity="true">
      <pose>{GRID_CENTER[0]} {GRID_CENTER[1]} 0 0 0 0</pose>
    </frame>
  </model>
</sdf>
"""


# ---------------------------------------------------------------- preview / jaw
def build_scene(q, jaw):
  reset()
  fr = link_frames(q)
  for l in LINKS:
    for stem, xyz, rpy, color, _ in VISUALS[l]:
      place(stem, fr[l] @ xform(xyz, rpy), color, f"{l}:{stem}")
  stem, xyz, rpy, color, _ = JAW_VISUAL
  jaw_m = fr["gripper_link"] @ xform(*JAW_ORIGIN) @ rot_z(jaw)
  place(stem, jaw_m @ xform(xyz, rpy), color, "jaw")
  return fr


def jaw_gap(jaw):
  """Gap between the jaws along the tool x axis, measured in the band 2..20 mm
  behind the fingertips (where a holder would be gripped)."""
  fr = build_scene([0] * 5, jaw)
  tip = (fr["gripper_link"] @ xform(*TIP))
  inv = tip.inverted()
  bpy.context.view_layer.update()

  def pts(obj_name):
    o = bpy.data.objects[obj_name]
    return [inv @ (o.matrix_world @ v.co) for v in o.data.vertices]

  fixed = pts("gripper_link:wrist_roll_follower_so101_v1")
  moving = pts("jaw")
  band = lambda p: -0.020 <= p.z <= -0.002 and abs(p.y) < 0.012
  fx = [p.x for p in fixed if band(p)]
  mx = [p.x for p in moving if band(p)]
  if not fx or not mx:
    return None
  # Jaws sit on opposite sides of x = 0; take the facing surfaces.
  if sum(fx) / len(fx) < sum(mx) / len(mx):
    return max(fx), min(mx)
  return min(fx), max(mx)


def preview(png, q, jaw):
  fr = build_scene(q, jaw)
  tip = fr["gripper_link"] @ xform(*TIP)
  bpy.ops.mesh.primitive_uv_sphere_add(radius=0.002, location=tip.translation)
  bpy.context.object.data.materials.append(material((1, 0.2, 0.1, 1)))
  cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
  bpy.context.scene.collection.objects.link(cam)
  cam.location = tuple(float(v) for v in os.environ.get("CAM", "0.45,-0.55,0.38").split(","))
  t = bpy.data.objects.new("t", None)
  bpy.context.scene.collection.objects.link(t)
  t.location = tuple(float(v) for v in os.environ.get("TARGET", "0.16,0,0.13").split(","))
  c = cam.constraints.new("TRACK_TO")
  c.target, c.track_axis, c.up_axis = t, "TRACK_NEGATIVE_Z", "UP_Y"
  s = bpy.context.scene
  s.camera = cam
  s.render.engine = "BLENDER_WORKBENCH"
  s.display.shading.light = "STUDIO"
  s.display.shading.show_cavity = True
  s.display.shading.color_type = "MATERIAL"
  s.render.resolution_x, s.render.resolution_y = 1000, 800
  s.render.filepath = os.path.abspath(png)
  bpy.ops.render.render(write_still=True)


def main():
  args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["out"]
  out = args[0]
  if len(args) > 1 and args[1] == "--calibrate-jaw":
    for k in range(-2, 18):
      a = k * 0.05
      g = jaw_gap(a)
      print("JAW %.2f" % a, "fixed %.4f moving %.4f gap %.1f mm" % (g[0], g[1], abs(g[1] - g[0]) * 1000)
            if g else "n/a")
    return
  if len(args) > 1:
    q = [float(v) for v in os.environ.get("Q", "0,0,0,0,0").split(",")]
    preview(args[1], q, float(os.environ.get("JAW", "0.3")))
    return
  adir, gdir, sdir = (os.path.join(out, d) for d in ("so101", "so101_gripper", "so101_stand"))
  for d in [adir, gdir, sdir] + [os.path.join(out, n) for n in BENCHES]:
    os.makedirs(d, exist_ok=True)
  for l in LINKS:
    export_link(VISUALS[l], os.path.join(adir, f"so101_{l}.glb"))
  export_link([JAW_VISUAL], os.path.join(gdir, "so101_jaw.glb"))
  export_stand(os.path.join(sdir, "so101_stand.glb"))
  for n, b in BENCHES.items():
    export_bench(b, os.path.join(out, n, n + ".glb"))
    with open(os.path.join(out, n, n + ".sdf"), "w") as fh:
      fh.write(bench_sdf(n, b))
  reset()
  with open(os.path.join(adir, "so101.sdf"), "w") as fh:
    fh.write(arm_sdf())
  with open(os.path.join(gdir, "so101_gripper.sdf"), "w") as fh:
    fh.write(gripper_sdf())
  with open(os.path.join(sdir, "so101_stand.sdf"), "w") as fh:
    fh.write(stand_sdf())


main()
