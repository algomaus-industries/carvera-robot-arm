"""Generates the Makera Carvera scene objects for OMTS (GLB visuals + SDF).

Run with Blender 3.4+ (headless):

    blender -b --python build_carvera.py -- <models_dir>

Writes <models_dir>/makera_carvera/{makera_carvera.sdf,carvera_body.glb,carvera_lid.glb}
and <models_dir>/carvera_tool_holder/{carvera_tool_holder.sdf,carvera_tool_holder.glb}.

One parts list drives both the visual meshes and the SDF collision geometry, so
the two cannot drift apart. Visuals are shaped after Makera's product photos
(low plinth with CARVERA lettering and power button, side panels with a slanted
front edge, blue-tinted hood with a large front radius and side wings, hinged at
the rear). Collision is boxes only: profiles are sliced into conservative
horizontal bands, the hood shell into thin rotated plates. All lengths in m.

Model frame (makera_carvera): origin at the centre of the footprint on the
table, +X to the machine's right (operator view), +Y to the back, +Z up. The
front (hood / opening) faces -Y.

Published (makera.com product page / manual):
  footprint 580 W x 520 D x 540 H mm, max height with lid open 840 mm,
  3-axis work area 360 x 240 x 140 mm, gantry clearance 120 mm, 6-tool ATC
  plus probe slot. ATC slot pitch / order from the stock firmware slot table
  (M889: all slots at one X, T0 at Y-24.48, T1 at Y-84.48, T2..T6 every 30 mm
  to Y-234.48).
Everything else (plinth height, hood radius, hinge line, side-panel slope,
gantry / spindle shape, rack size, tool holder shape) is ESTIMATED from photos:
check against the real machine and adjust the constants below.
"""

import math
import os
import sys

import bmesh
import bpy

# ---------------------------------------------------------------- parameters
# Published.
W, D, H = 0.580, 0.520, 0.541          # footprint, lid closed
H_LID_OPEN_PUBLISHED = 0.840           # max height, lid open (does not match the photos)
BED_X, BED_Y = 0.360, 0.240            # work area X/Y
GANTRY_CLEARANCE = 0.120
SLOT_MCS_Y = {0: -0.02448, 1: -0.08448, 2: -0.11448, 3: -0.14448,
              4: -0.17448, 5: -0.20448, 6: -0.23448}

# Estimated from photos.
FOOT_H = 0.012                         # rubber feet
BASE_H = 0.085                         # top of the white plinth
BASE_FRONT_LIP = 0.025                 # plinth sticks out in front of the hood
PANEL_T = 0.022                        # side panel thickness
SHELL_T = 0.005                        # hood acrylic
HOOD_BULGE = 0.030                     # hood front: convex curve, this far proud of a
HOOD_ROLL = 0.200                      # straight line; rolls into the flat top this far
                                       # behind the front (Makera / workshop photos)
SIDE_GAP = 0.032                       # side panel front edge follows the hood curve,
                                       # this far behind it
HINGE_Y = 0.248                        # hood hinge line: the rear edge (hood covers the whole top)
H_LID_OPEN = 0.980                     # open hood height: photos show the hood top near
                                       # vertical; capped below the OMTS cell roof
BED_TOP = 0.120
BED_T = 0.016
BED_CX, BED_CY = -0.065, -0.010        # bed centre (left of centre: ATC on right)
RACK_CX = 0.190                        # ATC rack centre X
RACK_W, RACK_TOP = 0.050, 0.150
MCS_Y0 = 0.115                         # model Y of machine Y=0 (rear home)

hx, hy = W / 2, D / 2
FRONT_Y = -hy + BASE_FRONT_LIP         # hood front face
TOP_Z = H

SILVER = (0.74, 0.75, 0.77, 1.0)       # brushed aluminium shell
WHITE = (0.92, 0.92, 0.92, 1.0)
DARK = (0.06, 0.06, 0.07, 1.0)
GREY = (0.30, 0.31, 0.33, 1.0)
ALU = (0.70, 0.71, 0.73, 1.0)
HOOD = (0.20, 0.22, 0.25, 0.45)        # smoked acrylic
STEEL = (0.62, 0.63, 0.65, 1.0)
BRASS = (0.80, 0.65, 0.30, 1.0)
HOSE = (0.55, 0.56, 0.58, 1.0)
LED = (0.55, 0.85, 1.00, 1.0)


# ---------------------------------------------------------------- geometry
def box(name, x0, x1, y0, y1, z0, z1, color, collide=True, bevel=0.0):
  return dict(kind="box", name=name, lo=(x0, y0, z0), hi=(x1, y1, z1),
              color=color, collide=collide, bevel=bevel)


def cyl(name, cx, cy, z0, z1, r, color, collide=True, axis="z"):
  return dict(kind="cyl", name=name, c=(cx, cy), z=(z0, z1), r=r,
              color=color, collide=collide, axis=axis)


def prism(name, x0, x1, poly, color, collide=True, bevel=0.0, bands=5):
  """Polygon in the YZ plane, extruded along X from x0 to x1."""
  return dict(kind="prism", name=name, x=(x0, x1), poly=poly, color=color,
              collide=collide, bevel=bevel, bands=bands)


def arc(cy, cz, r, a0, a1, n):
  return [(cy + r * math.cos(a0 + (a1 - a0) * i / n),
           cz + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


# Hood front: one smooth convex curve from the plinth lip up and back into
# the flat top (cubic Bezier), then flat back to the hinge at the rear edge.
def hood_curve(n=16, back=0.0):
  p0 = (FRONT_Y + 0.012, BASE_H + 0.004)
  p1 = (FRONT_Y - HOOD_BULGE, BASE_H + 0.45 * (TOP_Z - BASE_H))
  p2 = (FRONT_Y + 0.02, TOP_Z)
  p3 = (FRONT_Y + HOOD_ROLL, TOP_Z)
  pts = []
  for i in range(n + 1):
    t = i / n
    c = [(1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t ** 3]
    pts.append((sum(w * p[0] for w, p in zip(c, (p0, p1, p2, p3))) + back,
                sum(w * p[1] for w, p in zip(c, (p0, p1, p2, p3)))))
  return pts


HOOD_CURVE = hood_curve()
HOOD_LINE = HOOD_CURVE + [(HINGE_Y, TOP_Z)]


# Side panel front edge: the hood curve, SIDE_GAP behind it, stopping just
# under the hood top.
def side_edge(back=0.0):
  return [(y, min(z, TOP_Z - 0.010)) for y, z in hood_curve(back=SIDE_GAP + back)
          if z <= TOP_Z - 0.004]


SIDE_EDGE = side_edge()
SIDE_POLY = SIDE_EDGE + [(hy, TOP_Z - 0.010), (hy, BASE_H)]
# Hood side wing: the strip between the hood curve and the side panel edge.
WING_POLY = HOOD_CURVE + [(SIDE_EDGE[-1][0], TOP_Z)] + list(reversed(SIDE_EDGE))

beam_z0 = BED_TOP + GANTRY_CLEARANCE
SPX, SPY = -0.150, 0.075               # parked spindle axis (rear left)
bx0, bx1 = BED_CX - BED_X / 2 - 0.008, BED_CX + BED_X / 2 + 0.008
by0, by1 = BED_CY - BED_Y / 2 - 0.008, BED_CY + BED_Y / 2 + 0.008

BODY = [
    # Plinth with a front lip, rounded edges, on four feet.
    box("plinth", -hx, hx, -hy, hy, FOOT_H, BASE_H, SILVER, bevel=0.012),
    *[cyl(f"foot_{i}", sx * (hx - 0.05), sy * (hy - 0.05), 0.0, FOOT_H, 0.018, DARK,
          collide=False) for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)])],
    # Side panels and back.
    prism("side_left", -hx, -hx + PANEL_T, SIDE_POLY, SILVER, bevel=0.005),
    prism("side_right", hx - PANEL_T, hx, SIDE_POLY, SILVER, bevel=0.005),
    box("back", -hx + PANEL_T, hx - PANEL_T, hy - 0.020, hy, BASE_H, TOP_Z - 0.010, SILVER),
    # Dark interior back with the Y-axis cover band.
    box("inner_back", -hx + PANEL_T, hx - PANEL_T, hy - 0.030, hy - 0.020, BASE_H, TOP_Z - 0.03,
        DARK, collide=False),
    box("back_strip", -hx + PANEL_T, hx - PANEL_T, hy - 0.0305, hy - 0.030, BASE_H + 0.20,
        BASE_H + 0.215, SILVER, collide=False),
    box("cable_chain", -hx + PANEL_T + 0.02, SPX + 0.04, 0.115, 0.140, beam_z0 + 0.15, beam_z0 + 0.175,
        DARK, collide=False),
    box("y_cover", -hx + PANEL_T, hx - PANEL_T, 0.140, hy - 0.030, BASE_H, BASE_H + 0.09, DARK),
    # Bed: carriage + aluminium plate with a darker spoil board.
    box("bed_carriage", bx0 + 0.02, bx1 - 0.02, by0 + 0.01, by1 - 0.01, BASE_H, BED_TOP - BED_T, GREY),
    box("bed", bx0, bx1, by0, by1, BED_TOP - BED_T, BED_TOP, ALU, bevel=0.0015),
    # Origin anchor: the L-shaped block in the bed's front-left corner that
    # Carvera jobs zero against (anodised dark grey).
    box("anchor_x", bx0 + 0.004, bx0 + 0.064, by0 + 0.004, by0 + 0.016, BED_TOP, BED_TOP + 0.008,
        GREY, bevel=0.0008),
    box("anchor_y", bx0 + 0.004, bx0 + 0.016, by0 + 0.016, by0 + 0.064, BED_TOP, BED_TOP + 0.008,
        GREY, bevel=0.0008),
    # Gantry beam at the rear, spindle head parked rear-left (Y home, X left)
    # so the vise and the ATC are clear for the robot.
    box("gantry_beam", -hx + PANEL_T, hx - PANEL_T, 0.110, 0.140, beam_z0 + 0.03, beam_z0 + 0.15, DARK),
    box("x_carriage", SPX - 0.045, SPX + 0.045, 0.085, 0.110, beam_z0 + 0.02, beam_z0 + 0.20, ALU),
    box("spindle_mount", SPX - 0.038, SPX + 0.038, SPY - 0.035, 0.085, beam_z0 + 0.03, beam_z0 + 0.13,
        ALU, bevel=0.004),
    cyl("spindle", SPX, SPY, beam_z0 + 0.005, beam_z0 + 0.13, 0.026, STEEL),
    cyl("collet_nut", SPX, SPY, beam_z0 - 0.015, beam_z0 + 0.005, 0.011, BRASS),
    box("fan_box", SPX - 0.038, SPX + 0.038, SPY - 0.035, 0.085, beam_z0 + 0.13, beam_z0 + 0.20,
        STEEL, bevel=0.003),
    box("fan_grille", SPX - 0.030, SPX + 0.030, SPY - 0.0355, SPY - 0.035, beam_z0 + 0.137,
        beam_z0 + 0.193, DARK, collide=False),
    cyl("fan_hub", SPX, SPY - 0.036, beam_z0 + 0.155, beam_z0 + 0.175, 0.010, STEEL,
        collide=False, axis="y"),
    # ATC rack along the right of the bed, tool length sensor between T0 and T1.
    box("atc_rack", RACK_CX - RACK_W / 2, RACK_CX + RACK_W / 2,
        MCS_Y0 + SLOT_MCS_Y[6] - 0.020, MCS_Y0 + SLOT_MCS_Y[0] + 0.020, BASE_H, RACK_TOP, GREY,
        bevel=0.003),
    cyl("tool_length_sensor", RACK_CX, MCS_Y0 - 0.05448, RACK_TOP, RACK_TOP + 0.018, 0.009, STEEL),
    # Front of the plinth: power button with LED ring, side USB/status slot.
    cyl("power_ring", 0.215, FRONT_Y - BASE_FRONT_LIP - 0.001, 0.040, 0.054, 0.009, LED,
        collide=False, axis="y"),
    cyl("power_button", 0.215, FRONT_Y - BASE_FRONT_LIP - 0.002, 0.040, 0.054, 0.0065, WHITE,
        collide=False, axis="y"),
    *[prism(f"groove_{side}", sx * hx - (0.0008 if sx < 0 else 0), sx * hx + (0.0008 if sx > 0 else 0),
            side_edge(back=0.022) + list(reversed(side_edge(back=0.027))), GREY, collide=False)
      for side, sx in (("left", -1), ("right", 1))],
    # Rounded aluminium rail along the plinth's front edge.
    box("front_rail", -hx + 0.004, hx - 0.004, -hy, -hy + 0.022, BASE_H - 0.010, BASE_H + 0.008,
        SILVER, collide=False, bevel=0.008),
]
# Tapped-hole grid in the bed (estimated M6 on a 25 mm pitch), as dark discs.
HOLE_PITCH = 0.025
for i in range(int((bx1 - bx0) / HOLE_PITCH)):
  for j in range(int((by1 - by0) / HOLE_PITCH)):
    hxp = bx0 + HOLE_PITCH * (i + 0.5) + ((bx1 - bx0) % HOLE_PITCH) / 2
    hyp = by0 + HOLE_PITCH * (j + 0.5) + ((by1 - by0) % HOLE_PITCH) / 2
    if hxp < bx0 + 0.07 and hyp < by0 + 0.07:
      continue   # under the anchor
    BODY.append(dict(kind="cyl", name=f"hole_{i}_{j}", c=(hxp, hyp),
                     z=(BED_TOP - 0.0006, BED_TOP + 0.0003), r=0.0034, color=DARK,
                     collide=False, axis="z", verts=12))
for n, my in SLOT_MCS_Y.items():
  BODY.append(cyl(f"slot_ring_{n}", RACK_CX, MCS_Y0 + my, RACK_TOP, RACK_TOP + 0.0008,
                  0.0135, DARK, collide=False))

HOLDER = [
    cyl("bit", 0, 0, -0.045, -0.022, 0.0016, STEEL, collide=False),
    cyl("collet_nut", 0, 0, -0.022, 0.0, 0.0095, STEEL, collide=False),
    cyl("collar", 0, 0, 0.0005, 0.008, 0.0125, DARK),
    cyl("grip", 0, 0, 0.008, 0.030, 0.0100, STEEL),
    cyl("pull_taper", 0, 0, 0.030, 0.045, 0.0080, STEEL),
]


# ---------------------------------------------------------------- lid angle
def rot_about_hinge(p, th):
  """Lid rotation (front lifts) about the hinge line, axis -X."""
  dy, dz = p[0] - HINGE_Y, p[1] - TOP_Z
  return (HINGE_Y + dy * math.cos(th) + dz * math.sin(th),
          TOP_Z - dy * math.sin(th) + dz * math.cos(th))


def lid_max_z(th):
  return max(rot_about_hinge(p, th)[1] for p in HOOD_LINE + WING_POLY)


lo, hi = 0.0, math.pi / 2
for _ in range(60):
  mid = (lo + hi) / 2
  lo, hi = (mid, hi) if lid_max_z(mid) < H_LID_OPEN else (lo, mid)
LID_OPEN = lo


# ---------------------------------------------------------------- collision
def clip_band(poly, z0, z1):
  def clip(pts, keep, inter):
    out = []
    for i, cur in enumerate(pts):
      prev = pts[i - 1]
      if keep(cur):
        if not keep(prev):
          out.append(inter(prev, cur))
        out.append(cur)
      elif keep(prev):
        out.append(inter(prev, cur))
    return out

  def at_z(z):
    return lambda a, b: (a[0] + (b[0] - a[0]) * (z - a[1]) / (b[1] - a[1]), z)

  pts = clip(poly, lambda p: p[1] >= z0, at_z(z0))
  return clip(pts, lambda p: p[1] <= z1, at_z(z1)) if pts else []


def collision_shapes(parts):
  """Yields (name, center, rpy, geometry_xml)."""
  for p in parts:
    if not p["collide"]:
      continue
    if p["kind"] == "box":
      lo_, hi_ = p["lo"], p["hi"]
      yield (p["name"], [(a + b) / 2 for a, b in zip(lo_, hi_)], (0, 0, 0),
             "<box><size>%s</size></box>" % f3([b - a for a, b in zip(lo_, hi_)]))
    elif p["kind"] == "cyl":
      (cx, cy), (z0, z1) = p["c"], p["z"]
      yield (p["name"], (cx, cy, (z0 + z1) / 2), (0, 0, 0),
             "<cylinder><radius>%.5f</radius><length>%.5f</length></cylinder>"
             % (p["r"], z1 - z0))
    elif p["kind"] == "prism":
      x0, x1 = p["x"]
      zs = [q[1] for q in p["poly"]]
      zmin, zmax = min(zs), max(zs)
      n = p["bands"]
      for i in range(n):
        z0 = zmin + (zmax - zmin) * i / n
        z1 = zmin + (zmax - zmin) * (i + 1) / n
        seg = clip_band(p["poly"], z0, z1)
        if not seg:
          continue
        y0, y1 = min(q[0] for q in seg), max(q[0] for q in seg)
        yield (f'{p["name"]}_{i}', ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
               (0, 0, 0), "<box><size>%s</size></box>" % f3((x1 - x0, y1 - y0, z1 - z0)))
    elif p["kind"] == "shell":
      for i in range(len(p["line"]) - 1):
        (ya, za), (yb, zb) = p["line"][i], p["line"][i + 1]
        length = math.hypot(yb - ya, zb - za)
        roll = math.atan2(zb - za, yb - ya)
        yield (f'{p["name"]}_{i}', (0.0, (ya + yb) / 2, (za + zb) / 2), (roll, 0, 0),
               "<box><size>%s</size></box>" % f3((W, length, SHELL_T)))


def f3(v):
  return " ".join("%.5f" % x for x in v)


# Lid parts: the shell (visual built specially), wings as prisms.
LID = [
    dict(kind="shell", name="hood", line=HOOD_LINE, color=HOOD, collide=True),
    prism("wing_left", -hx, -hx + SHELL_T, WING_POLY, HOOD, bands=4),
    prism("wing_right", hx - SHELL_T, hx, WING_POLY, HOOD, bands=4),
    box("hinge_bar", -hx + 0.01, hx - 0.01, HINGE_Y - 0.010, hy, TOP_Z - 0.004, TOP_Z + 0.004,
        SILVER, collide=False),
]


# ---------------------------------------------------------------- blender
def material(color):
  key = "m_%02x%02x%02x%02x" % tuple(int(c * 255) for c in color)
  m = bpy.data.materials.get(key)
  if m:
    return m
  m = bpy.data.materials.new(key)
  m.use_nodes = True
  m.use_backface_culling = False
  bsdf = m.node_tree.nodes["Principled BSDF"]
  bsdf.inputs["Base Color"].default_value = color
  bsdf.inputs["Roughness"].default_value = 0.35 if color[3] < 1 else 0.55
  if color in (ALU, STEEL, BRASS, SILVER):
    bsdf.inputs["Metallic"].default_value = 0.8
  if color[3] < 1.0:
    bsdf.inputs["Alpha"].default_value = color[3]
    m.blend_method = "BLEND"
  return m


def link_mesh(name, bm):
  me = bpy.data.meshes.new(name)
  bm.to_mesh(me)
  bm.free()
  o = bpy.data.objects.new(name, me)
  bpy.context.scene.collection.objects.link(o)
  return o


def add_bevel(o, width):
  if width > 0:
    mod = o.modifiers.new("bevel", "BEVEL")
    mod.width = width
    mod.segments = 4
    mod.limit_method = "ANGLE"


def make_prism(p):
  bm = bmesh.new()
  x0, x1 = p["x"]
  a = [bm.verts.new((x0, y, z)) for y, z in p["poly"]]
  b = [bm.verts.new((x1, y, z)) for y, z in p["poly"]]
  bm.faces.new(list(reversed(a)))
  bm.faces.new(b)
  n = len(a)
  for i in range(n):
    bm.faces.new([a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]])
  bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
  o = link_mesh(p["name"], bm)
  add_bevel(o, p["bevel"])
  return o


def make_shell(p):
  bm = bmesh.new()
  x0, x1 = -hx, hx
  a = [bm.verts.new((x0, y, z)) for y, z in p["line"]]
  b = [bm.verts.new((x1, y, z)) for y, z in p["line"]]
  for i in range(len(a) - 1):
    bm.faces.new([a[i], a[i + 1], b[i + 1], b[i]])
  o = link_mesh(p["name"], bm)
  mod = o.modifiers.new("solid", "SOLIDIFY")
  mod.thickness = SHELL_T
  mod.offset = -1.0
  for f in o.data.polygons:
    f.use_smooth = True
  return o


def build(parts):
  for p in parts:
    if p["kind"] == "box":
      lo_, hi_ = p["lo"], p["hi"]
      bpy.ops.mesh.primitive_cube_add(size=1, location=[(a + b) / 2 for a, b in zip(lo_, hi_)])
      o = bpy.context.object
      o.scale = [b - a for a, b in zip(lo_, hi_)]
      bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
      add_bevel(o, p["bevel"])
    elif p["kind"] == "cyl":
      (cx, cy), (z0, z1) = p["c"], p["z"]
      if p["axis"] == "y":   # disc on a front face: z0..z1 is its diameter band
        bpy.ops.mesh.primitive_cylinder_add(vertices=40, radius=p["r"], depth=0.002,
                                            location=(cx, cy, (z0 + z1) / 2),
                                            rotation=(math.pi / 2, 0, 0))
      else:
        bpy.ops.mesh.primitive_cylinder_add(vertices=p.get("verts", 48), radius=p["r"], depth=z1 - z0,
                                            location=(cx, cy, (z0 + z1) / 2))
      o = bpy.context.object
      for f in o.data.polygons:
        f.use_smooth = len(f.vertices) == 4
    elif p["kind"] == "prism":
      o = make_prism(p)
    elif p["kind"] == "shell":
      o = make_shell(p)
    o.name = p["name"]
    o.data.materials.append(material(p["color"]))


def add_logo():
  bpy.ops.object.text_add(location=(-0.215, FRONT_Y - BASE_FRONT_LIP - 0.0005, 0.040),
                          rotation=(math.pi / 2, 0, 0))
  t = bpy.context.object
  t.data.body = "CARVERA"
  t.data.size = 0.020
  t.data.space_character = 1.35
  t.data.extrude = 0.0004
  bpy.ops.object.convert(target="MESH")
  t.name = "logo"
  t.data.materials.append(material(GREY))


def add_hose():
  """Vacuum / coolant hose from the spindle head up and back to the gantry."""
  cu = bpy.data.curves.new("hose", "CURVE")
  cu.dimensions = "3D"
  cu.bevel_depth = 0.012
  cu.bevel_resolution = 4
  sp = cu.splines.new("BEZIER")
  pts = [(SPX + 0.040, SPY - 0.02, beam_z0 + 0.04), (SPX + 0.085, SPY - 0.01, beam_z0 + 0.20),
         (SPX + 0.03, SPY + 0.02, beam_z0 + 0.27), (SPX - 0.08, 0.140, beam_z0 + 0.22),
         (-hx + PANEL_T + 0.01, 0.170, beam_z0 + 0.20)]
  sp.bezier_points.add(len(pts) - 1)
  for bp, co in zip(sp.bezier_points, pts):
    bp.co = co
    bp.handle_left_type = bp.handle_right_type = "AUTO"
  o = bpy.data.objects.new("hose", cu)
  bpy.context.scene.collection.objects.link(o)
  bpy.context.view_layer.objects.active = o
  o.select_set(True)
  bpy.ops.object.convert(target="MESH")
  o.data.materials.append(material(HOSE))


def export(parts, path, extras=()):
  bpy.ops.wm.read_factory_settings(use_empty=True)
  build(parts)
  for e in extras:
    e()
  bpy.ops.export_scene.gltf(filepath=path, export_format="GLB",
                            export_yup=False, export_apply=True)


# ---------------------------------------------------------------- sdf
LICENSE = """<!--
Copyright 2026 Intrinsic Innovation LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Generated by build_carvera.py - edit that script, not this file.
-->"""


def collisions(parts, indent="      "):
  return "\n".join(
      f'{indent}<collision name="{n}_collision">\n'
      f'{indent}  <pose>{f3(c)} {f3(rpy)}</pose>\n'
      f'{indent}  <geometry>{g}</geometry>\n'
      f'{indent}</collision>' for n, c, rpy, g in collision_shapes(parts))


def visual(name, uri, indent="      "):
  return (f'{indent}<visual name="{name}">\n'
          f'{indent}  <geometry><mesh><uri>{uri}</uri></mesh></geometry>\n'
          f'{indent}</visual>')


def carvera_sdf():
  frames = []
  for n, my in SLOT_MCS_Y.items():
    frames.append(
        f'    <frame name="atc_slot_{n}" attached_to="base_link" intrinsic:create_entity="true">\n'
        f'      <pose>{f3((RACK_CX, MCS_Y0 + my, RACK_TOP))} 0 0 0</pose>\n'
        f'    </frame>')
  frames.append(
      f'    <frame name="bed_center" attached_to="base_link" intrinsic:create_entity="true">\n'
      f'      <pose>{f3((BED_CX, BED_CY, BED_TOP))} 0 0 0</pose>\n'
      f'    </frame>')
  return f"""<?xml version="1.0" ?>
{LICENSE}
<!-- Makera Carvera desktop CNC. Hood joint lid_joint: 0 = closed,
     {LID_OPEN:.4f} rad = fully open (hood top at the published 840 mm). -->
<sdf xmlns:intrinsic="https://intrinsic.ai/" version="1.9">
  <model name="makera_carvera">
    <static>true</static>
    <link name="base_link">
{visual("body_visual", "carvera_body.glb")}
{collisions(BODY)}
    </link>
    <link name="lid_link">
{visual("lid_visual", "carvera_lid.glb")}
{collisions(LID)}
    </link>
    <joint name="lid_joint" type="revolute">
      <pose relative_to="base_link">0 {HINGE_Y:.5f} {TOP_Z:.5f} 0 0 0</pose>
      <parent>base_link</parent>
      <child>lid_link</child>
      <axis>
        <xyz>-1 0 0</xyz>
        <limit>
          <lower>0.0</lower>
          <upper>{LID_OPEN:.4f}</upper>
        </limit>
      </axis>
    </joint>
{chr(10).join(frames)}
  </model>
</sdf>
"""


def holder_sdf():
  return f"""<?xml version="1.0" ?>
{LICENSE}
<!-- Makera Carvera ATC tool holder with collet nut and bit (estimated shape).
     Origin on the rack seat (atc_slot_N frame); collision only above the seat.
     grip_frame is the centre of the grip band. -->
<sdf xmlns:intrinsic="https://intrinsic.ai/" version="1.9">
  <model name="carvera_tool_holder">
    <link name="base_link">
      <inertial><mass>0.06</mass></inertial>
{visual("holder_visual", "carvera_tool_holder.glb")}
{collisions(HOLDER)}
    </link>
    <frame name="grip_frame" attached_to="base_link" intrinsic:create_entity="true">
      <pose>0 0 0.019 0 0 0</pose>
    </frame>
  </model>
</sdf>
"""


def main():
  out = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "out"
  cdir = os.path.join(out, "makera_carvera")
  hdir = os.path.join(out, "carvera_tool_holder")
  os.makedirs(cdir, exist_ok=True)
  os.makedirs(hdir, exist_ok=True)
  export(BODY, os.path.abspath(os.path.join(cdir, "carvera_body.glb")), (add_logo, add_hose))
  export(LID, os.path.abspath(os.path.join(cdir, "carvera_lid.glb")))
  export(HOLDER, os.path.abspath(os.path.join(hdir, "carvera_tool_holder.glb")))
  with open(os.path.join(cdir, "makera_carvera.sdf"), "w") as fh:
    fh.write(carvera_sdf())
  with open(os.path.join(hdir, "carvera_tool_holder.sdf"), "w") as fh:
    fh.write(holder_sdf())
  print("LID_OPEN_RAD", round(LID_OPEN, 4))
  print("BED_TOP", BED_TOP)
  for n, my in SLOT_MCS_Y.items():
    print("SLOT", n, round(RACK_CX, 4), round(MCS_Y0 + my, 4), round(RACK_TOP, 4))


main()
