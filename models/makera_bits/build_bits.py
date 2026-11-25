"""Makera's Carvera bit catalogue as scene objects, one per distinct bit.

Run headless:  blender -b --python build_bits.py -- <models_dir> [preview.png]
Writes <models_dir>/makera_bits/{<id>.sdf, <id>.glb, bits.bzl, makera_bits.json}.

Catalogue: makera.com store (products.json and the CNC Bits collection),
read 2026-11-25. Taken from the store: bit families, cutting diameters,
shank diameters, cutting lengths, overall lengths, tip widths and angles,
thread sizes, TiN coating. ESTIMATED (store does not say): overall length
where marked est_len, helix pitch, the collar (OD / height / position on the
shank), coating colour of the "for Metal" bits, thread-mill cutting
diameters, corn-bit cutting lengths, the solder-mask tool's body.
The 1/8" engraving (plain and for Metal), Spiral O for Metal and ball-nose
(plain and for Metal) sizes and their ~38 mm length are from those product
pages, fetched later after rate limiting.



Each bit stands in an ATC / grid seat the way the Carvera holds it: a collar
pressed on the shank rests on the seat, the cutter hangs down into the bore,
the shank sticks up. Origin: collar bottom (= seat top), z up. Collision is
collar + shank only (above the seat). Frame grip_frame: middle of the shank
above the collar, where the gripper closes.
"""

import json
import math
import os
import sys

import bpy
from mathutils import Vector

IN = 0.0254
SH18 = 0.125 * IN                       # 1/8" shank

# Collar (ESTIMATED): ring pressed on the shank, OD = shank + 6 mm, 5 mm tall;
# 15 mm of shank stick up above it for the collet / gripper.
COLLAR_H = 0.005
STICK_UP = 0.015

CARBIDE = (0.58, 0.59, 0.61, 1.0)
METAL_COAT = (0.24, 0.22, 0.30, 1.0)    # "for Metal" bits: dark AlTiN-style (estimated)
TIN = (0.86, 0.69, 0.27, 1.0)           # TiN, as stated on the store
COLLAR = (0.12, 0.12, 0.13, 1.0)
STEEL = (0.75, 0.76, 0.78, 1.0)
GROOVE = (0.20, 0.20, 0.22, 1.0)


def bit(id_, title, kind, d, shank, cut, overall, flutes=2, coat=CARBIDE, est_len=False, **kw):
  return dict(id=id_, title=title, kind=kind, d=d / 1000, shank=shank, cut=cut / 1000,
              overall=overall / 1000, flutes=flutes, coat=coat, est_len=est_len, **kw)


def catalog():
  b = []
  for c, o in ((22, 45), (25, 50), (32, 55)):
    b.append(bit(f"ofl4_{c}", f'Spiral O Single Flute 4mm x {c}mm', "endmill", 4, 0.004, c, o, 1))
  for c in (12, 17, 22):
    b.append(bit(f"oflm4_{c}", f'Spiral O Single Flute for Metal 4mm x {c}mm', "endmill", 4,
                 0.004, c, 50, 1, METAL_COAT))
  for c, o in ((22, 50), (25, 55), (32, 60)):
    b.append(bit(f"ofl6_{c}", f'Spiral O Single Flute 6mm x {c}mm', "endmill", 6, 0.006, c, o, 1))
  for c, o in ((17, 50), (25, 55), (30, 60)):
    b.append(bit(f"oflm6_{c}", f'Spiral O Single Flute for Metal 6mm x {c}mm', "endmill", 6,
                 0.006, c, o, 1, METAL_COAT))
  for c in (25, 42):
    b.append(bit(f"ofl18_{c}", f'Spiral O Single Flute 1/8" x {c}mm', "endmill", 3.175, SH18, c,
                 max(38, c + 18), 1, est_len=True))
  for d, c in ((3.175, 12), (2.5, 12), (2, 8), (1.5, 6), (1, 3)):
    b.append(bit(f"oflm18_{d:g}x{c}".replace(".", "p"), f'Spiral O Single Flute for Metal 1/8" {d:g}x{c}mm',
                 "endmill", d, SH18, c, 38, 1, METAL_COAT))
  for t in (0.1, 0.2, 0.3, 0.4, 0.5):
    b.append(bit(f"eng4_30x{t:g}".replace(".", "p"), f"Single Flute Engraving 4mm 30deg x {t:g}mm",
                 "vbit", t, 0.004, 0, 42, 1, angle=30))
  for t in (0.1, 0.2, 0.3, 0.4, 0.5):   # store page: 3.175 mm shank, ~38 mm overall
    b.append(bit(f"eng18_30x{t:g}".replace(".", "p"), f'Single Flute Engraving 1/8" 30deg x {t:g}mm',
                 "vbit", t, SH18, 0, 38, 1, angle=30))
  for t in (0.3, 0.4, 0.5):
    b.append(bit(f"engm4_30x{t:g}".replace(".", "p"), f"Single Flute Engraving for Metal 4mm 30deg x {t:g}mm",
                 "vbit", t, 0.004, 0, 45, 1, METAL_COAT, angle=30))
  for a, t in ((60, 0.1), (30, 0.2), (30, 0.3), (30, 0.4), (30, 0.5)):
    b.append(bit(f"engm18_{a}x{t:g}".replace(".", "p"),
                 f'Single Flute Engraving for Metal 1/8" {a}deg x {t:g}mm', "vbit", t, SH18, 0, 38, 1,
                 METAL_COAT, angle=a))
  for t in (0.3, 0.4, 0.5):
    b.append(bit(f"eng6_20x{t:g}".replace(".", "p"), f"Two Flute Engraving 6mm 20deg x {t:g}mm",
                 "vbit", t, 0.006, 0, 50, 2, angle=20))
  for d in (0.5, 1, 2):
    b.append(bit(f"bneng6_{d:g}".replace(".", "p"), f"Two Flute Ball Nose Engraving 6mm, {d:g}mm",
                 "tapered_ball", d, 0.006, 30.5, 50, 2, est_len=True))
  for c, o in ((12, 45), (22, 45), (32, 60)):
    b.append(bit(f"bn4_{c}", f"Two Flute Ball Nose 4mm x {c}mm", "ballnose", 4, 0.004, c, o))
  for c, o in ((12, 50), (22, 50), (32, 60)):
    b.append(bit(f"bn6_{c}", f"Two Flute Ball Nose 6mm x {c}mm", "ballnose", 6, 0.006, c, o))
  b.append(bit("bnm4_12", "Two Flute Ball Nose for Metal 4mm x 12mm", "ballnose", 4, 0.004, 12, 50,
               coat=METAL_COAT))
  b.append(bit("bnm6_18", "Two Flute Ball Nose for Metal 6mm x 18mm", "ballnose", 6, 0.006, 18, 50,
               coat=METAL_COAT))
  for d, c in ((3.175, 22), (2.5, 15), (2, 12), (1.5, 8), (1, 4)):
    b.append(bit(f"bn18_{d:g}x{c}".replace(".", "p"), f'Two Flute Ball Nose 1/8" {d:g}x{c}mm',
                 "ballnose", d, SH18, c, 38))
  for d, c in ((3.175, 10), (2.5, 7.5), (2, 6), (1.5, 4.5), (1, 3)):
    b.append(bit(f"bnm18_{d:g}x{c:g}".replace(".", "p"), f'Two Flute Ball Nose for Metal 1/8" {d:g}x{c:g}mm',
                 "ballnose", d, SH18, c, 38, coat=METAL_COAT))
  for s, o in ((SH18, 38), (0.004, 50), (0.006, 50)):
    b.append(bit(f"chamfer90_{s * 1000:.3g}".replace(".", "p"), f"Chamfering Bit 90deg, {s * 1000:.3g}mm shank",
                 "chamfer", 0.3, s, 0, o, 2, angle=90))
  for m, d, c, p in (("M6", 4.8, 18, 1.0), ("M8", 6.0, 24, 1.25)):
    b.append(bit(f"thread6_{m.lower()}", f"Thread Milling 6mm {m} (d{d:g} x {c}mm)", "threadmill", d,
                 0.006, c, 50, 3, pitch=p))
  for m, d, c, p in (("M1", 0.75, 3, 0.25), ("M2", 1.5, 6, 0.4), ("M2.5", 1.9, 7.5, 0.45),
                     ("M3", 2.3, 9, 0.5), ("M4", 3.1, 12, 0.7), ("M5", 3.1, 15, 0.8)):
    b.append(bit(f"thread18_{m.lower().replace('.', 'p')}", f'Thread Milling 1/8" {m} x {c:g}mm',
                 "threadmill", d, SH18, c, 38, 3, est_len=True, pitch=p))
  for d in (1, 2, 2.5, 3):
    b.append(bit(f"drill18_{d:g}".replace(".", "p"), f'TiN Coating Drill 1/8" {d:g}mm', "drill", d,
                 SH18, 4 * d + 4, 38, 2, TIN, est_len=True))
  for d in (0.6, 1, 2, 3):
    b.append(bit(f"corn18_{d:g}".replace(".", "p"), f'TiN Coating Corn 1/8" {d:g}mm', "corn", d,
                 SH18, 3 * d + 3, 38, 8, TIN, est_len=True))
  b.append(bit("uv_soldermask", 'UV Solder Mask Removal Tool 1/8"', "spring_tool", 0.3, SH18, 0, 50,
               1, STEEL, est_len=True, angle=30))
  return b


# ---------------------------------------------------------------- geometry
def material(color):
  key = "m_%02x%02x%02x%02x" % tuple(int(c * 255) for c in color)
  m = bpy.data.materials.get(key)
  if m:
    return m
  m = bpy.data.materials.new(key)
  m.use_nodes = True
  bsdf = m.node_tree.nodes["Principled BSDF"]
  bsdf.inputs["Base Color"].default_value = color
  bsdf.inputs["Metallic"].default_value = 0.85 if color not in (COLLAR, GROOVE) else 0.3
  bsdf.inputs["Roughness"].default_value = 0.3
  m.diffuse_color = color
  return m


def _finish(o, color):
  o.data.materials.append(material(color))
  for f in getattr(o.data, "polygons", []):
    f.use_smooth = True
  return o


def cyl(r, z0, z1, color, verts=32):
  bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=z1 - z0,
                                      location=(0, 0, (z0 + z1) / 2))
  o = _finish(bpy.context.object, color)
  for f in o.data.polygons:
    f.use_smooth = len(f.vertices) == 4
  return o


def cone(r_top, r_bot, z_top, z_bot, color, verts=32):
  bpy.ops.mesh.primitive_cone_add(vertices=verts, radius1=r_bot, radius2=r_top,
                                  depth=z_top - z_bot, location=(0, 0, (z_top + z_bot) / 2))
  return _finish(bpy.context.object, color)


def sphere(r, z, color):
  bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=r, location=(0, 0, z))
  return _finish(bpy.context.object, color)


def helix(r, z0, z1, pitch, tube, color, phase=0.0, hand=1):
  turns = max(0.3, (z1 - z0) / pitch)
  n = max(8, int(turns * 24))
  cu = bpy.data.curves.new("flute", "CURVE")
  cu.dimensions = "3D"
  cu.bevel_depth = tube
  cu.bevel_resolution = 2
  sp = cu.splines.new("POLY")
  sp.points.add(n - 1)
  for i in range(n):
    t = i / (n - 1)
    a = phase + hand * 2 * math.pi * turns * t
    sp.points[i].co = (r * math.cos(a), r * math.sin(a), z1 - (z1 - z0) * t, 1)
  o = bpy.data.objects.new("flute", cu)
  bpy.context.scene.collection.objects.link(o)
  bpy.context.view_layer.objects.active = o
  for ob in bpy.context.selected_objects:
    ob.select_set(False)
  o.select_set(True)
  bpy.ops.object.convert(target="MESH")
  return _finish(o, color)


def fluted(r, z_bot, z_top, flutes, pitch, coat, hand=1):
  """Cutting section: core cylinder plus raised helical lands."""
  cyl(r * 0.78, z_bot, z_top, coat)
  for k in range(flutes):
    helix(r * 0.80, z_bot, z_top, pitch, r * 0.22, coat, 2 * math.pi * k / flutes, hand)


def build_bit(b):
  r, rs = b["d"] / 2, b["shank"] / 2
  below = b["overall"] - COLLAR_H - STICK_UP        # length below the collar (to the tip)
  tip = -below
  coat = b["coat"]
  kind = b["kind"]
  cut = min(b["cut"], below - 0.002) if b["cut"] else 0
  # Shank: from the top of the cutting part (or neck) to STICK_UP above the collar.
  neck_top = tip + cut
  if kind in ("vbit", "chamfer", "spring_tool", "tapered_ball"):
    neck_top = tip                                   # handled below
  if kind == "endmill" or kind == "drill" or kind == "corn" or kind == "threadmill" or kind == "ballnose":
    if r < rs - 0.0003:
      # Reduced-diameter cutter: a taper from the shank down to the cutter.
      taper = min(0.006, (rs - r) * 4)
      cone(rs, r, neck_top + taper, neck_top, CARBIDE)
      neck_top += taper
    if kind == "endmill":
      fluted(r, tip, tip + cut, b["flutes"], max(3 * b["d"], 0.004), coat)
    elif kind == "ballnose":
      fluted(r, tip + r, tip + cut, 2, max(2.5 * b["d"], 0.003), coat)
      sphere(r * 0.98, tip + r, coat)
    elif kind == "drill":
      fluted(r, tip + r * 0.6, tip + cut, 2, 3.5 * b["d"], coat)
      cone(r * 0.98, 0.0, tip + r * 0.6, tip, coat)
    elif kind == "corn":
      cyl(r * 0.85, tip, tip + cut, coat, verts=24)
      for k in range(4):
        helix(r * 0.88, tip, tip + cut, max(1.2 * b["d"], 0.0012), r * 0.14, coat,
              k * math.pi / 2, 1)
        helix(r * 0.88, tip, tip + cut, max(1.2 * b["d"], 0.0012), r * 0.14, coat,
              k * math.pi / 2, -1)
    elif kind == "threadmill":
      p = b["pitch"] / 1000
      cyl(r * 0.72, tip, tip + cut, coat, verts=24)
      z = tip
      while z + p <= tip + cut + 1e-9:
        cone(r * 0.72, r, z + p / 2, z, coat, verts=24)
        cone(r, r * 0.72, z + p, z + p / 2, coat, verts=24)
        z += p
    cyl(rs, neck_top, COLLAR_H + STICK_UP, CARBIDE)
  elif kind in ("vbit", "chamfer"):
    half = math.radians(b["angle"] / 2)
    length = max(0.0005, (rs - r / 2) / math.tan(half))
    cone(rs, r / 2, tip + length, tip, coat)
    if b["flutes"] == 1:     # single flute: the cone is ground half away
      bpy.ops.mesh.primitive_cube_add(size=1, location=(-rs * 0.55, 0, tip + length * 0.55))
      c = bpy.context.object
      c.scale = (rs * 0.9, rs * 2.2, length * 1.1)
      _finish(c, GROOVE)
    cyl(rs, tip + length, COLLAR_H + STICK_UP, CARBIDE)
  elif kind == "tapered_ball":
    taper = 0.020
    sphere(r, tip + r, coat)
    cone(rs, r, tip + r + taper, tip + r, coat)
    helix(rs * 0.6, tip + r, tip + r + taper, 0.008, max(r * 0.25, 0.0003), coat)
    cyl(rs, tip + r + taper, COLLAR_H + STICK_UP, CARBIDE)
  elif kind == "spring_tool":
    shell_r = 0.004
    cone(0.0008, 0.0, tip + 0.004, tip, STEEL)
    cyl(0.0008, tip + 0.004, tip + 0.010, STEEL)
    cone(shell_r, 0.0012, tip + 0.016, tip + 0.010, STEEL)
    cyl(shell_r, tip + 0.016, -0.002, STEEL)
    cyl(rs, -0.002, COLLAR_H + STICK_UP, CARBIDE)
  # Collar on the seat.
  bpy.ops.mesh.primitive_cylinder_add(vertices=40, radius=rs + 0.003, depth=COLLAR_H,
                                      location=(0, 0, COLLAR_H / 2))
  c = bpy.context.object
  mod = c.modifiers.new("bevel", "BEVEL")
  mod.width, mod.segments = 0.0005, 2
  _finish(c, COLLAR)
  return tip


# ---------------------------------------------------------------- sdf
HEADER = """<?xml version="1.0" ?>
<!-- Generated by build_bits.py - edit that script, not this file.
     Makera bit dimensions from makera.com (2026-11-25); collar and anything
     marked estimated in build_bits.py are estimates. -->"""


def grip_z():
  return COLLAR_H + STICK_UP / 2


def bit_sdf(b):
  rs = b["shank"] / 2
  return f"""{HEADER}
<!-- {b['title']}{' (overall length estimated)' if b['est_len'] else ''}.
     Origin: collar bottom on the seat; cutter below, shank above. -->
<sdf xmlns:intrinsic="https://intrinsic.ai/" version="1.9">
  <model name="makera_bit_{b['id']}">
    <link name="base_link">
      <inertial><mass>0.01</mass></inertial>
      <visual name="bit_visual">
        <geometry><mesh><uri>{b['id']}.glb</uri></mesh></geometry>
      </visual>
      <collision name="collar_collision">
        <pose>0 0 {COLLAR_H / 2 + 0.0005:.5f} 0 0 0</pose>
        <geometry><cylinder><radius>{rs + 0.003:.5f}</radius><length>{COLLAR_H - 0.001:.5f}</length></cylinder></geometry>
      </collision>
      <collision name="shank_collision">
        <pose>0 0 {COLLAR_H + STICK_UP / 2:.5f} 0 0 0</pose>
        <geometry><cylinder><radius>{rs:.5f}</radius><length>{STICK_UP:.5f}</length></cylinder></geometry>
      </collision>
    </link>
    <frame name="grip_frame" attached_to="base_link" intrinsic:create_entity="true">
      <pose>0 0 {grip_z():.5f} 0 0 0</pose>
    </frame>
  </model>
</sdf>
"""


def preview(png, bits):
  bpy.ops.wm.read_factory_settings(use_empty=True)
  cols = 12
  for i, b in enumerate(bits):
    before = set(bpy.data.objects)
    build_bit(b)
    for o in set(bpy.data.objects) - before:
      o.location.x += (i % cols) * 0.016
      o.location.y += (i // cols) * 0.030
  cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
  bpy.context.scene.collection.objects.link(cam)
  cam.location = (0.09, -0.24, 0.10)
  t = bpy.data.objects.new("t", None)
  bpy.context.scene.collection.objects.link(t)
  t.location = (0.088, 0.07, -0.01)
  c = cam.constraints.new("TRACK_TO")
  c.target, c.track_axis, c.up_axis = t, "TRACK_NEGATIVE_Z", "UP_Y"
  s = bpy.context.scene
  s.camera = cam
  s.render.engine = "BLENDER_WORKBENCH"
  s.display.shading.light = "STUDIO"
  s.display.shading.show_cavity = True
  s.display.shading.color_type = "MATERIAL"
  s.render.resolution_x, s.render.resolution_y = 1400, 900
  s.render.filepath = os.path.abspath(png)
  bpy.ops.render.render(write_still=True)


def main():
  args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["out"]
  bits = catalog()
  assert len({b["id"] for b in bits}) == len(bits), "duplicate bit ids"
  if len(args) > 1:
    preview(args[1], bits)
    return
  out = os.path.join(args[0], "makera_bits")
  os.makedirs(out, exist_ok=True)
  for b in bits:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    build_bit(b)
    bpy.ops.export_scene.gltf(filepath=os.path.abspath(os.path.join(out, b["id"] + ".glb")),
                              export_format="GLB", export_yup=False, export_apply=True)
    with open(os.path.join(out, b["id"] + ".sdf"), "w") as fh:
      fh.write(bit_sdf(b))
    with open(os.path.join(out, b["id"] + "_manifest.textproto"), "w") as fh:
      fh.write("# proto-file: intrinsic/assets/scene_objects/proto/scene_object_manifest.proto\n"
               "# proto-message: intrinsic_proto.scene_objects.SceneObjectManifest\n\n"
               f'metadata {{\n  id {{\n    name: "makera_bit_{b["id"]}"\n    package: "ai.intrinsic"\n  }}\n'
               f'  vendor {{\n    display_name: "intrinsic"\n  }}\n'
               f'  display_name: "Makera {b["title"].replace(chr(34), " in")}"\n}}\n')
  with open(os.path.join(out, "bits.bzl"), "w") as fh:
    fh.write('"""Generated by build_bits.py: Makera bit ids, catalogue order."""\n\n'
             "MAKERA_BITS = [\n" + "".join(f'    "{b["id"]}",\n' for b in bits) + "]\n")
  with open(os.path.join(out, "makera_bits.json"), "w") as fh:
    json.dump({"collar_h": COLLAR_H, "stick_up": STICK_UP, "grip_z": grip_z(),
               "bits": [{k: (v if not isinstance(v, tuple) else None) for k, v in b.items()
                         if k != "coat"} for b in bits]}, fh, indent=1)
  print("BITS", len(bits))


main()
