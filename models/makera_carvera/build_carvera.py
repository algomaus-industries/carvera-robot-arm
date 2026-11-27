"""Generates the Makera Carvera scene objects for OMTS (GLB visuals + SDF).

Run with Blender 3.4+ (headless):

    blender -b --python build_carvera.py -- <models_dir>

Writes <models_dir>/makera_carvera/{makera_carvera.sdf,carvera_body.glb,carvera_lid.glb}
and <models_dir>/bit_carousel/{bit_carousel.sdf,carousel_base.glb,carousel_platter.glb}.

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
gantry / spindle shape, rack size) is ESTIMATED from photos:
check against the real machine and adjust the constants below.
"""

import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix

# ---------------------------------------------------------------- parameters
# Published.
W, D, H = 0.580, 0.520, 0.541  # footprint, lid closed
H_LID_OPEN_PUBLISHED = 0.840  # max height, lid open (does not match the photos)
BED_X, BED_Y = 0.360, 0.240  # work area X/Y
GANTRY_CLEARANCE = 0.120
SLOT_MCS_Y = {
  0: -0.02448,
  1: -0.08448,
  2: -0.11448,
  3: -0.14448,
  4: -0.17448,
  5: -0.20448,
  6: -0.23448,
}

# Estimated from photos.
FOOT_H = 0.012  # rubber feet
BASE_H = 0.085  # top of the white plinth
BASE_FRONT_LIP = 0.025  # plinth sticks out in front of the hood
PANEL_T = 0.035  # side panel thickness (manual front view)
SHELL_T = 0.005  # hood acrylic
HOOD_BULGE = 0.015  # hood front: convex curve, this far proud of a
HOOD_ROLL = 0.200  # straight line; rolls into the flat top this far
# behind the front (Makera / workshop photos)
SIDE_KNEE_Z = 0.270  # side panel front edge: vertical up to here,
SIDE_EDGE_TOP_Y = 0.130  # then folding straight back to this far behind
# the hood front at the top (Makera side photo)
SIDE_FACET = 0.035  # chamfer facet along the panel's front edge
PANEL_TOP = H - 0.050  # side panels stop under the hood's top band
HINGE_Y = 0.100  # hood hinge line, ~1/3 back from the front: the manual's
# front view shows the open hood's front panel lying
# flat at ~0.86 m, i.e. ~90 deg about a hinge this far
# back; a fixed smoked panel covers the rear of the top
H_LID_OPEN = 0.850  # open hood height: ~0.86 m measured on the manual's
# front view, published 0.84 m
BED_TOP = 0.120
BED_T = 0.016
BED_CX, BED_CY = -0.010, -0.010  # bed centre (manual front view: centred)
RACK_CX = 0.205  # ATC rack centre X (manual front view)
RACK_W, RACK_TOP = 0.034, 0.148  # bar top ~28 mm above the bed (ATC photo)
MCS_Y0 = 0.115  # model Y of machine Y=0 (rear home)

hx, hy = W / 2, D / 2
FRONT_Y = -hy + BASE_FRONT_LIP  # hood front face
TOP_Z = H

SILVER = (0.74, 0.75, 0.77, 1.0)  # brushed aluminium shell
WHITE = (0.92, 0.92, 0.92, 1.0)
DARK = (0.06, 0.06, 0.07, 1.0)
GREY = (0.30, 0.31, 0.33, 1.0)
ALU = (0.70, 0.71, 0.73, 1.0)
HOOD = (0.20, 0.22, 0.25, 0.45)  # smoked acrylic
STEEL = (0.62, 0.63, 0.65, 1.0)
BRASS = (0.80, 0.65, 0.30, 1.0)
HOSE = (0.55, 0.56, 0.58, 1.0)
BED_PLATE = (0.24, 0.24, 0.25, 1.0)  # dark drilled bed plate
LED = (0.55, 0.85, 1.00, 1.0)


# ---------------------------------------------------------------- geometry
def box(name, x0, x1, y0, y1, z0, z1, color, collide=True, bevel=0.0):
  return dict(
    kind="box",
    name=name,
    lo=(x0, y0, z0),
    hi=(x1, y1, z1),
    color=color,
    collide=collide,
    bevel=bevel,
  )


def cyl(name, cx, cy, z0, z1, r, color, collide=True, axis="z"):
  return dict(
    kind="cyl",
    name=name,
    c=(cx, cy),
    z=(z0, z1),
    r=r,
    color=color,
    collide=collide,
    axis=axis,
  )


def prism(name, x0, x1, poly, color, collide=True, bevel=0.0, bands=5):
  """Polygon in the YZ plane, extruded along X from x0 to x1."""
  return dict(
    kind="prism",
    name=name,
    x=(x0, x1),
    poly=poly,
    color=color,
    collide=collide,
    bevel=bevel,
    bands=bands,
  )


def arc(cy, cz, r, a0, a1, n):
  return [
    (
      cy + r * math.cos(a0 + (a1 - a0) * i / n),
      cz + r * math.sin(a0 + (a1 - a0) * i / n),
    )
    for i in range(n + 1)
  ]


# Hood front: one smooth convex curve from the plinth lip up and back into
# the flat top (cubic Bezier), then flat back to the hinge at the rear edge.
def hood_curve(n=16, back=0.0):
  p0 = (FRONT_Y + 0.012, BASE_H + 0.004)
  p1 = (FRONT_Y - HOOD_BULGE, BASE_H + 0.70 * (TOP_Z - BASE_H))
  p2 = (FRONT_Y + 0.02, TOP_Z)
  p3 = (FRONT_Y + HOOD_ROLL, TOP_Z)
  pts = []
  for i in range(n + 1):
    t = i / n
    c = [(1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t**3]
    pts.append(
      (
        sum(w * p[0] for w, p in zip(c, (p0, p1, p2, p3), strict=False)) + back,
        sum(w * p[1] for w, p in zip(c, (p0, p1, p2, p3), strict=False)),
      )
    )
  return pts


HOOD_CURVE = hood_curve()
HOOD_LINE = HOOD_CURVE + [(HINGE_Y, TOP_Z)]


# Side panel front edge: vertical from the plinth to the knee, then a
# straight fold back to the top. The outer face starts SIDE_FACET behind
# that edge, so a chamfer facet runs along it (the crease in the photos).
def side_edge(back=0.0):
  y0 = FRONT_Y + 0.020 + back
  y1 = FRONT_Y + SIDE_EDGE_TOP_Y + back
  return [(y0, BASE_H), (y0, SIDE_KNEE_Z + back * 0.5), (y1, PANEL_TOP)]


SIDE_EDGE = side_edge()
SIDE_POLY = SIDE_EDGE + [(hy, PANEL_TOP), (hy, BASE_H)]
SIDE_POLY_OUTER = side_edge(SIDE_FACET) + [(hy, PANEL_TOP), (hy, BASE_H)]
# Hood side wing: between the hood curve and the side panel's folded edge.
# The wing also runs as a band above the panel top back to the hinge.
WING_POLY = (
  HOOD_CURVE
  + [(HINGE_Y, TOP_Z), (HINGE_Y, PANEL_TOP)]
  + list(reversed(SIDE_EDGE))
)

beam_z0 = BED_TOP + GANTRY_CLEARANCE
SPX, SPY = -0.150, 0.075  # parked spindle axis (rear left)
bx0, bx1 = BED_CX - BED_X / 2 - 0.008, BED_CX + BED_X / 2 + 0.008
by0, by1 = BED_CY - BED_Y / 2 - 0.008, BED_CY + BED_Y / 2 + 0.008

BODY = [
  # Plinth with a front lip, rounded edges, on four feet.
  box("plinth", -hx, hx, -hy, hy, FOOT_H, BASE_H, SILVER, bevel=0.012),
  *[
    cyl(
      f"foot_{i}",
      sx * (hx - 0.05),
      sy * (hy - 0.05),
      0.0,
      FOOT_H,
      0.018,
      DARK,
      collide=False,
    )
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)])
  ],
  # Side panels and back.
  dict(
    prism("side_left", -hx, -hx + PANEL_T, SIDE_POLY_OUTER, SILVER),
    poly1=SIDE_POLY,
    collide_poly=SIDE_POLY,
  ),
  dict(
    prism("side_right", hx - PANEL_T, hx, SIDE_POLY, SILVER),
    poly1=SIDE_POLY_OUTER,
    collide_poly=SIDE_POLY,
  ),
  box(
    "back",
    -hx + PANEL_T,
    hx - PANEL_T,
    hy - 0.020,
    hy,
    BASE_H,
    TOP_Z - 0.010,
    SILVER,
  ),
  # Dark interior back with the Y-axis cover band.
  # Fixed smoked top and side wings behind the hood hinge.
  box("top_rear", -hx, hx, HINGE_Y + 0.002, hy, TOP_Z - SHELL_T, TOP_Z, HOOD),
  prism(
    "rear_wing_left",
    -hx,
    -hx + SHELL_T,
    [
      (HINGE_Y + 0.002, PANEL_TOP),
      (HINGE_Y + 0.002, TOP_Z),
      (hy, TOP_Z),
      (hy, PANEL_TOP),
    ],
    HOOD,
    bands=1,
  ),
  prism(
    "rear_wing_right",
    hx - SHELL_T,
    hx,
    [
      (HINGE_Y + 0.002, PANEL_TOP),
      (HINGE_Y + 0.002, TOP_Z),
      (hy, TOP_Z),
      (hy, PANEL_TOP),
    ],
    HOOD,
    bands=1,
  ),
  box(
    "inner_back",
    -hx + PANEL_T,
    hx - PANEL_T,
    hy - 0.030,
    hy - 0.020,
    BASE_H,
    TOP_Z - 0.03,
    SILVER,
    collide=False,
  ),
  # Clear dust bin at the rear left (manual: "Dust Bin").
  box(
    "dust_bin",
    -hx + PANEL_T + 0.010,
    -hx + PANEL_T + 0.085,
    0.120,
    0.200,
    BASE_H,
    BASE_H + 0.200,
    (0.80, 0.85, 0.90, 0.35),
  ),
  box(
    "back_strip",
    -hx + PANEL_T,
    hx - PANEL_T,
    hy - 0.0305,
    hy - 0.030,
    BASE_H + 0.20,
    BASE_H + 0.215,
    SILVER,
    collide=False,
  ),
  box(
    "cable_chain",
    -hx + PANEL_T + 0.02,
    SPX + 0.04,
    0.115,
    0.140,
    beam_z0 + 0.15,
    beam_z0 + 0.175,
    DARK,
    collide=False,
  ),
  # Bed: carriage + aluminium plate with a darker spoil board.
  box(
    "bed_carriage",
    bx0 + 0.02,
    bx1 - 0.02,
    by0 + 0.01,
    by1 - 0.01,
    BASE_H,
    BED_TOP - BED_T,
    GREY,
  ),
  box(
    "bed_frame",
    bx0 - 0.006,
    bx1 + 0.006,
    by0 - 0.006,
    by1 + 0.006,
    BED_TOP - BED_T - 0.008,
    BED_TOP - BED_T + 0.002,
    SILVER,
    collide=False,
    bevel=0.002,
  ),
  box(
    "bed", bx0, bx1, by0, by1, BED_TOP - BED_T, BED_TOP, BED_PLATE, bevel=0.0015
  ),
  # Origin anchor: the L-shaped block in the bed's front-left corner that
  # Carvera jobs zero against (anodised dark grey).
  box(
    "anchor_x",
    bx0 + 0.004,
    bx0 + 0.064,
    by0 + 0.004,
    by0 + 0.016,
    BED_TOP,
    BED_TOP + 0.008,
    GREY,
    bevel=0.0008,
  ),
  box(
    "anchor_y",
    bx0 + 0.004,
    bx0 + 0.016,
    by0 + 0.016,
    by0 + 0.064,
    BED_TOP,
    BED_TOP + 0.008,
    GREY,
    bevel=0.0008,
  ),
  # Gantry beam at the rear, spindle head parked rear-left (Y home, X left)
  # so the vise and the ATC are clear for the robot.
  box(
    "gantry_beam",
    -hx + PANEL_T,
    hx - PANEL_T,
    0.110,
    0.140,
    beam_z0 + 0.03,
    beam_z0 + 0.15,
    DARK,
  ),
  box(
    "x_carriage",
    SPX - 0.045,
    SPX + 0.045,
    0.085,
    0.110,
    beam_z0 + 0.02,
    beam_z0 + 0.20,
    ALU,
  ),
  box(
    "spindle_mount",
    SPX - 0.038,
    SPX + 0.038,
    SPY - 0.035,
    0.085,
    beam_z0 + 0.03,
    beam_z0 + 0.13,
    ALU,
    bevel=0.004,
  ),
  cyl("spindle", SPX, SPY, beam_z0 + 0.005, beam_z0 + 0.13, 0.026, STEEL),
  cyl("collet_nut", SPX, SPY, beam_z0 - 0.015, beam_z0 + 0.005, 0.011, BRASS),
  box(
    "fan_box",
    SPX - 0.038,
    SPX + 0.038,
    SPY - 0.035,
    0.085,
    beam_z0 + 0.13,
    beam_z0 + 0.20,
    STEEL,
    bevel=0.003,
  ),
  box(
    "fan_grille",
    SPX + 0.038,
    SPX + 0.0385,
    SPY - 0.028,
    0.080,
    beam_z0 + 0.137,
    beam_z0 + 0.193,
    DARK,
    collide=False,
  ),
  # ATC rack along the right of the bed, tool length sensor between T0 and T1.
  box(
    "atc_rack",
    RACK_CX - RACK_W / 2,
    RACK_CX + RACK_W / 2,
    MCS_Y0 + SLOT_MCS_Y[6] - 0.020,
    MCS_Y0 + SLOT_MCS_Y[0] + 0.020,
    BASE_H,
    RACK_TOP,
    SILVER,
    bevel=0.003,
  ),
  # Wireless 3D probe parked in slot T0.
  cyl(
    "probe_body",
    RACK_CX,
    MCS_Y0 + SLOT_MCS_Y[0],
    RACK_TOP + 0.002,
    RACK_TOP + 0.030,
    0.0085,
    BRASS,
  ),
  cyl(
    "probe_shank",
    RACK_CX,
    MCS_Y0 + SLOT_MCS_Y[0],
    RACK_TOP + 0.030,
    RACK_TOP + 0.045,
    0.003,
    STEEL,
    collide=False,
  ),
  cyl(
    "tool_length_sensor",
    RACK_CX,
    MCS_Y0 - 0.05448,
    RACK_TOP,
    RACK_TOP + 0.012,
    0.0065,
    STEEL,
  ),
  # Front of the plinth: power button with LED ring, side USB/status slot.
  cyl(
    "power_ring",
    0.215,
    FRONT_Y - BASE_FRONT_LIP - 0.001,
    0.040,
    0.054,
    0.009,
    LED,
    collide=False,
    axis="y",
  ),
  cyl(
    "power_button",
    0.215,
    FRONT_Y - BASE_FRONT_LIP - 0.002,
    0.040,
    0.054,
    0.0065,
    WHITE,
    collide=False,
    axis="y",
  ),
  # Rounded aluminium rail along the plinth's front edge.
  box(
    "front_rail",
    -hx + 0.004,
    hx - 0.004,
    -hy,
    -hy + 0.022,
    BASE_H - 0.010,
    BASE_H + 0.008,
    SILVER,
    collide=False,
    bevel=0.008,
  ),
]
# Tapped-hole grid in the bed (estimated M6 on a 25 mm pitch), as dark discs.
HOLE_PITCH = 0.025
for i in range(int((bx1 - bx0) / HOLE_PITCH)):
  for j in range(int((by1 - by0) / HOLE_PITCH)):
    hxp = bx0 + HOLE_PITCH * (i + 0.5) + ((bx1 - bx0) % HOLE_PITCH) / 2
    hyp = by0 + HOLE_PITCH * (j + 0.5) + ((by1 - by0) % HOLE_PITCH) / 2
    if hxp < bx0 + 0.07 and hyp < by0 + 0.07:
      continue  # under the anchor
    BODY.append(
      dict(
        kind="cyl",
        name=f"hole_{i}_{j}",
        c=(hxp, hyp),
        z=(BED_TOP - 0.0006, BED_TOP + 0.0003),
        r=0.0034,
        color=DARK,
        collide=False,
        axis="z",
        verts=12,
      )
    )
for n, my in SLOT_MCS_Y.items():
  BODY.append(
    cyl(
      f"seat_{n}",
      RACK_CX,
      MCS_Y0 + my,
      RACK_TOP,
      RACK_TOP + 0.002,
      0.011,
      DARK,
      collide=False,
    )
  )


# ---------------------------------------------------------------- lid angle
def rot_about_hinge(p, th):
  """Lid rotation (front lifts) about the hinge line, axis -X."""
  dy, dz = p[0] - HINGE_Y, p[1] - TOP_Z
  return (
    HINGE_Y + dy * math.cos(th) + dz * math.sin(th),
    TOP_Z - dy * math.sin(th) + dz * math.cos(th),
  )


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
      yield (
        p["name"],
        [(a + b) / 2 for a, b in zip(lo_, hi_, strict=False)],
        (0, 0, 0),
        f"<box><size>{f3([b - a for a, b in zip(lo_, hi_, strict=False)])}</size></box>",
      )
    elif p["kind"] == "cyl":
      (cx, cy), (z0, z1) = p["c"], p["z"]
      yield (
        p["name"],
        (cx, cy, (z0 + z1) / 2),
        (0, 0, 0),
        f"<cylinder><radius>{p['r']:.5f}</radius>"
        f"<length>{z1 - z0:.5f}</length></cylinder>",
      )
    elif p["kind"] == "prism":
      x0, x1 = p["x"]
      poly = p.get("collide_poly", p["poly"])
      zs = [q[1] for q in poly]
      zmin, zmax = min(zs), max(zs)
      n = p["bands"]
      for i in range(n):
        z0 = zmin + (zmax - zmin) * i / n
        z1 = zmin + (zmax - zmin) * (i + 1) / n
        seg = clip_band(poly, z0, z1)
        if not seg:
          continue
        y0, y1 = min(q[0] for q in seg), max(q[0] for q in seg)
        yield (
          f"{p['name']}_{i}",
          ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
          (0, 0, 0),
          f"<box><size>{f3((x1 - x0, y1 - y0, z1 - z0))}</size></box>",
        )
    elif p["kind"] == "shell":
      for i in range(len(p["line"]) - 1):
        (ya, za), (yb, zb) = p["line"][i], p["line"][i + 1]
        length = math.hypot(yb - ya, zb - za)
        roll = math.atan2(zb - za, yb - ya)
        yield (
          f"{p['name']}_{i}",
          (0.0, (ya + yb) / 2, (za + zb) / 2),
          (roll, 0, 0),
          f"<box><size>{f3((W, length, SHELL_T))}</size></box>",
        )


def f3(v):
  return " ".join(f"{x:.5f}" for x in v)


# Lid parts: the shell (visual built specially), wings as prisms.
LID = [
  dict(kind="shell", name="hood", line=HOOD_LINE, color=HOOD, collide=True),
  prism("wing_left", -hx, -hx + SHELL_T, WING_POLY, HOOD, bands=4),
  prism("wing_right", hx - SHELL_T, hx, WING_POLY, HOOD, bands=4),
  # Silver trim along the hood's front (bottom) edge, as in the manual render.
  box(
    "front_trim",
    -hx,
    hx,
    FRONT_Y - 0.004,
    FRONT_Y + 0.020,
    BASE_H + 0.004,
    BASE_H + 0.020,
    SILVER,
    collide=False,
    bevel=0.003,
  ),
  box(
    "hinge_bar",
    -hx + 0.01,
    hx - 0.01,
    HINGE_Y - 0.010,
    hy,
    TOP_Z - 0.004,
    TOP_Z + 0.004,
    SILVER,
    collide=False,
  ),
]


# ---------------------------------------------------------------- blender
def material(color):
  key = "m_" + "".join(f"{int(c * 255):02x}" for c in color)
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
  b = [bm.verts.new((x1, y, z)) for y, z in p.get("poly1", p["poly"])]
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
      bpy.ops.mesh.primitive_cube_add(
        size=1, location=[(a + b) / 2 for a, b in zip(lo_, hi_, strict=False)]
      )
      o = bpy.context.object
      o.scale = [b - a for a, b in zip(lo_, hi_, strict=False)]
      bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
      add_bevel(o, p["bevel"])
      if p.get("yaw"):  # turned about the origin's z axis (visual only)
        o.location = Matrix.Rotation(p["yaw"], 3, "Z") @ o.location
        o.rotation_euler = (0, 0, p["yaw"])
    elif p["kind"] == "cyl":
      (cx, cy), (z0, z1) = p["c"], p["z"]
      if p["axis"] == "y":  # disc on a front face: z0..z1 is its diameter band
        bpy.ops.mesh.primitive_cylinder_add(
          vertices=40,
          radius=p["r"],
          depth=0.002,
          location=(cx, cy, (z0 + z1) / 2),
          rotation=(math.pi / 2, 0, 0),
        )
      else:
        bpy.ops.mesh.primitive_cylinder_add(
          vertices=p.get("verts", 48),
          radius=p["r"],
          depth=z1 - z0,
          location=(cx, cy, (z0 + z1) / 2),
        )
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
  bpy.ops.object.text_add(
    location=(-0.215, FRONT_Y - BASE_FRONT_LIP - 0.0005, 0.040),
    rotation=(math.pi / 2, 0, 0),
  )
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
  pts = [
    (SPX + 0.040, SPY - 0.02, beam_z0 + 0.04),
    (SPX + 0.085, SPY - 0.01, beam_z0 + 0.20),
    (SPX + 0.03, SPY + 0.02, beam_z0 + 0.27),
    (SPX - 0.08, 0.140, beam_z0 + 0.22),
    (-hx + PANEL_T + 0.01, 0.170, beam_z0 + 0.20),
  ]
  sp.bezier_points.add(len(pts) - 1)
  for bp, co in zip(sp.bezier_points, pts, strict=False):
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
  bpy.ops.export_scene.gltf(
    filepath=path, export_format="GLB", export_yup=False, export_apply=True
  )


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
    f"{indent}  <pose>{f3(c)} {f3(rpy)}</pose>\n"
    f"{indent}  <geometry>{g}</geometry>\n"
    f"{indent}</collision>"
    for n, c, rpy, g in collision_shapes(parts)
  )


def visual(name, uri, indent="      "):
  return (
    f'{indent}<visual name="{name}">\n'
    f"{indent}  <geometry><mesh><uri>{uri}</uri></mesh></geometry>\n"
    f"{indent}</visual>"
  )


def carvera_sdf():
  frames = []
  for n, my in SLOT_MCS_Y.items():
    frames.append(
      f'    <frame name="atc_slot_{n}" attached_to="base_link" intrinsic:create_entity="true">\n'
      f"      <pose>{f3((RACK_CX, MCS_Y0 + my, RACK_TOP + 0.002))} 0 0 0</pose>\n"
      f"    </frame>"
    )
  frames.append(
    f'    <frame name="bed_center" attached_to="base_link" intrinsic:create_entity="true">\n'
    f"      <pose>{f3((BED_CX, BED_CY, BED_TOP))} 0 0 0</pose>\n"
    f"    </frame>"
  )
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


# Bit carousel for the SO-101 cell (models/bit_carousel): a turning seat plate
# with every Makera bit, on a base on the cart; layout in carousel_layout.py,
# fabrication files in fab/carousel/. platter_link turns about the
# origin (carousel_joint, z up); frames cell_R_C sit on the seat plate's top.
sys.path.insert(
  0,
  os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "bit_carousel"
  ),
)
import carousel_layout as cl  # noqa: E402  pylint: disable=wrong-import-position

PLY = (0.78, 0.64, 0.45, 1.0)  # birch plywood
PRINT = (0.18, 0.42, 0.75, 1.0)  # printed PETG parts
SERVO = (0.06, 0.06, 0.07, 1.0)  # Feetech STS3215
CAROUSEL_BASE_T, CAROUSEL_BASE_R = cl.BASE_T, cl.BASE_R
RIM_R, RIM_Z0, ROLLER_R, HUB_R = cl.RIM_R, cl.RIM_Z0, cl.ROLLER_R, cl.HUB_R


def carousel_base_parts():
  z0 = CAROUSEL_BASE_T
  parts = [cyl("base_plate", 0, 0, 0.0, z0, CAROUSEL_BASE_R, PLY, collide=True)]
  # STS3215 upright, output shaft on the turntable axis, in its printed cradle
  sw, sl, sh = cl.SERVO_BODY
  y0 = -cl.SERVO_SHAFT_FROM_END
  parts.append(
    box(
      "servo_cradle",
      -sw / 2 - 0.003,
      sw / 2 + 0.003,
      y0 - 0.003,
      y0 + sl + 0.003,
      z0,
      z0 + 0.015,
      PRINT,
      collide=False,
    )
  )
  parts.append(
    box(
      "servo",
      -sw / 2,
      sw / 2,
      y0,
      y0 + sl,
      z0 + 0.0015,
      z0 + 0.0015 + sh,
      SERVO,
      collide=False,
    )
  )
  parts.append(
    cyl(
      "horn", 0, 0, z0 + 0.0015 + sh, cl.HORN_TOP, 0.010, WHITE, collide=False
    )
  )
  # 608 rollers on radial axles in printed brackets; the rim rides on them
  rc = RIM_Z0 - cl.BEARING_OD / 2
  for k in range(cl.ROLLERS):
    a = math.pi / 4 + k * 2 * math.pi / cl.ROLLERS
    for side in (-1, 1):
      parts.append(
        dict(
          kind="box",
          name=f"roller_cheek_{k}_{side}",
          lo=(ROLLER_R - 0.014, side * 0.0065 - 0.002, z0),
          hi=(ROLLER_R + 0.014, side * 0.0065 + 0.002, rc + 0.007),
          color=PRINT,
          collide=False,
          bevel=0.0,
          yaw=a,
        )
      )
    parts.append(
      dict(
        kind="box",
        name=f"roller_{k}",
        lo=(
          ROLLER_R - cl.BEARING_OD / 2,
          -cl.BEARING_W / 2,
          rc - cl.BEARING_OD / 2,
        ),
        hi=(
          ROLLER_R + cl.BEARING_OD / 2,
          cl.BEARING_W / 2,
          rc + cl.BEARING_OD / 2,
        ),
        color=STEEL,
        collide=False,
        bevel=0.004,
        yaw=a,
      )
    )
  return parts


def carousel_platter_parts():
  top = cl.SEAT_Z
  plate0 = top - cl.PLATE_T
  parts = [
    cyl("seat_plate", 0, 0, plate0, top, cl.PLATE_R, PLY, collide=True),
    cyl("hub", 0, 0, cl.HORN_TOP, plate0, HUB_R, PRINT, collide=False),
  ]
  n = 48
  for k in range(n):
    a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
    am = (a0 + a1) / 2
    w = RIM_R[1] * (a1 - a0) / 2 + 0.0005
    # rim segment: a box tangent to the rim circle, rotated into place below
    parts.append(
      dict(
        kind="box",
        name=f"rim_{k}",
        lo=(RIM_R[0], -w, RIM_Z0),
        hi=(RIM_R[1], w, plate0),
        color=PRINT,
        collide=False,
        bevel=0.0,
        yaw=am,
      )
    )
  for r, c, x, y, _ in cl.cells():
    parts.append(
      cyl(
        f"seat_{r}_{c}",
        x,
        y,
        top - 0.0002,
        top + 0.0004,
        0.0065,
        DARK,
        collide=False,
      )
    )
  return parts


# Visuals: the fabrication models themselves (fab/gen_fab.py exports them from
# fab/carousel/carousel.scad, mm, carousel axis at the origin), one STL per
# material; the collision boxes stay the simple parts above.
FAB = os.path.join(
  os.path.dirname(os.path.abspath(__file__)), "..", "..", "fab"
)


def export_fab(stls, path):
  bpy.ops.wm.read_factory_settings(use_empty=True)
  for part, color in stls:
    f = os.path.join(FAB, "carousel", "out", part + ".stl")
    if not os.path.exists(f):
      raise SystemExit(f"{f} is missing: run python3 fab/gen_fab.py first")
    bpy.ops.import_mesh.stl(filepath=f, global_scale=0.001)
    o = bpy.context.object
    o.name = part
    o.data.materials.append(material(color))
  bpy.ops.export_scene.gltf(
    filepath=path, export_format="GLB", export_yup=False, export_apply=True
  )


def carousel_sdf():
  frames = "\n".join(
    f'    <frame name="cell_{r}_{c}" attached_to="platter_link" intrinsic:create_entity="true">\n'
    f"      <pose>{x:.5f} {y:.5f} {cl.SEAT_Z:.5f} 0 0 {th:.6f}</pose>\n"
    f"    </frame>"
    for r, c, x, y, th in cl.cells()
  )
  counts = ", ".join(str(cl.ring_seats(r)) for r in range(len(cl.RINGS)))
  return f"""<?xml version="1.0" ?>
{LICENSE}
<!-- Bit carousel for the SO-101 cell: {len(cl.cells())} seats ({counts} per ring) on a
     turning seat plate. carousel_joint turns platter_link about the origin (z up,
     0..2 pi, an STS3215 in position mode on the real one); the seat on ring R
     at index C reaches ring R's pick position at platter angle
     carousel_layout.platter_angle_for(R, C). Frames cell_R_C sit on the seat
     plate's top, x radial. -->
<sdf xmlns:intrinsic="https://intrinsic.ai/" version="1.9">
  <model name="bit_carousel">
    <static>true</static>
    <link name="base_link">
{visual("base_visual", "carousel_base.glb")}
{collisions(carousel_base_parts())}
    </link>
    <link name="platter_link">
{visual("platter_visual", "carousel_platter.glb")}
{collisions(carousel_platter_parts())}
    </link>
    <joint name="carousel_joint" type="revolute">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
      <parent>base_link</parent>
      <child>platter_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>0.0</lower>
          <upper>{2 * math.pi:.4f}</upper>
        </limit>
      </axis>
    </joint>
{frames}
  </model>
</sdf>
"""


def main():
  out = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "out"
  cdir = os.path.join(out, "makera_carvera")
  os.makedirs(cdir, exist_ok=True)
  export(
    BODY,
    os.path.abspath(os.path.join(cdir, "carvera_body.glb")),
    (add_logo, add_hose),
  )
  export(LID, os.path.abspath(os.path.join(cdir, "carvera_lid.glb")))
  with open(os.path.join(cdir, "makera_carvera.sdf"), "w") as fh:
    fh.write(carvera_sdf())
  kdir = os.path.join(out, "bit_carousel")
  os.makedirs(kdir, exist_ok=True)
  export_fab(
    (
      ("sim_base_ply", PLY),
      ("sim_base_print", PRINT),
      ("sim_base_servo", SERVO),
      ("sim_base_steel", STEEL),
    ),
    os.path.abspath(os.path.join(kdir, "carousel_base.glb")),
  )
  export_fab(
    (("sim_platter_ply", PLY), ("sim_platter_print", PRINT)),
    os.path.abspath(os.path.join(kdir, "carousel_platter.glb")),
  )
  with open(os.path.join(kdir, "bit_carousel.sdf"), "w") as fh:
    fh.write(carousel_sdf())
  print("CAROUSEL seats", len(cl.cells()))
  print("LID_OPEN_RAD", round(LID_OPEN, 4))
  print("BED_TOP", BED_TOP)
  for n, my in SLOT_MCS_Y.items():
    print(
      "SLOT",
      n,
      round(RACK_CX, 4),
      round(MCS_Y0 + my, 4),
      round(RACK_TOP + 0.002, 4),
    )


main()
