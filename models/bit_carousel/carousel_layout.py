"""Layout of bit_carousel, the SO-101 cell's turntable bit rack.

The SO-101 reaches only a ~95 deg fan of free cart top (between the Carvera
and its shoulder_pan limit) with the gripper vertical, which holds ~100
seats at the spacing the open jaw needs. The carousel holds every Makera bit
on a turning seat plate instead: an STS3215 servo under its centre (position
mode, one full turn) turns the wanted seat to its ring's pick position, which
lies inside the arm's reach.

Coordinates: the carousel frame has its origin on the cart top under the
turntable axis, z up. Seat frames cell_R_C sit on the seat plate's top face,
x radial (out from the turntable axis). The pick position of ring R is at
angle PICK_ANGLE[R] in the carousel's fixed frame (platter angle 0); the
platter turns by PICK_ANGLE[R] - seat angle to bring a seat there.

Shared by the sim model (models/makera_carvera/build_carvera.py), the cell
layout (configs/carvera_so101/gen_cell_configs.py), move_bit (via the seat
frames) and the fabrication files (models/bit_carousel/fab/).
"""
import math

# ---------------------------------------------------------------- seats
SEAT_PITCH = 0.021        # min centre distance along a ring (21 mm)
RING_PITCH = 0.025        # between rings: the open jaw needs >= ~12 mm to a
                          # neighbouring bit sideways, see tools/atc/move_bit.py
RINGS = (0.045, 0.070, 0.095, 0.120, 0.145)   # ring radii
SEAT_Z = 0.075            # seat plate top above the cart top
PLATE_T = 0.006           # seat plate thickness
PLATE_R = 0.168           # seat plate radius
PUCK_R = 0.0105           # seat boss radius in the sim model

# ---------------------------------------------------------------- placement
# Turntable axis in the arm's pan frame (origin on the SO-101's pan axis,
# axes along the world's). On the cart (pan frame x 0.025..0.38,
# y -0.345..0.09) and clear of the arm pedestal.
CENTER_IN_PAN = (0.198, -0.170)
# Pick positions sit this far from the arm's pan axis (inside the 0.08..0.27 m
# band the arm reaches with the gripper vertical at seat height), on the side
# of each ring facing the ATC.
PICK_REACH = (0.225, 0.205, 0.190, 0.175, 0.160)


def ring_seats(ring: int) -> int:
  return int(2 * math.pi * RINGS[ring] / SEAT_PITCH)


def cells():
  """[(ring, index, x, y, angle)] in the carousel frame at platter angle 0."""
  out = []
  for r, rr in enumerate(RINGS):
    n = ring_seats(r)
    for i in range(n):
      th = 2 * math.pi * i / n
      out.append((r, i, rr * math.cos(th), rr * math.sin(th), th))
  return out


def _pick_angle(ring: int) -> float:
  """Angle (carousel frame) of ring `ring`'s pick position: where the ring
  crosses the circle of radius PICK_REACH[ring] around the pan axis, on the
  side nearer the ATC (world angle 180 deg from the pan axis)."""
  cx, cy = CENTER_IN_PAN
  d = math.hypot(cx, cy)
  rho, reach = RINGS[ring], PICK_REACH[ring]
  # angle at the turntable axis between "towards the pan axis" and the point
  cos_a = (d * d + rho * rho - reach * reach) / (2 * d * rho)
  a = math.acos(max(-1.0, min(1.0, cos_a)))
  toward_pan = math.atan2(-cy, -cx)
  best = None
  for th in (toward_pan + a, toward_pan - a):
    px, py = cx + rho * math.cos(th), cy + rho * math.sin(th)
    # nearer the ATC = larger angular distance from 0 deg (world +x)
    score = abs(math.atan2(py, px))
    if best is None or score > best[0]:
      best = (score, th)
  return best[1]


PICK_ANGLE = tuple(_pick_angle(r) for r in range(len(RINGS)))


def platter_angle_for(ring: int, index: int) -> float:
  """Platter angle (rad, 0..2pi) that brings seat (ring, index) to its pick
  position."""
  th = 2 * math.pi * index / ring_seats(ring)
  return (PICK_ANGLE[ring] - th) % (2 * math.pi)


if __name__ == "__main__":
  cs = cells()
  print(len(cs), "seats;", [ring_seats(r) for r in range(len(RINGS))])
  cx, cy = CENTER_IN_PAN
  for r in range(len(RINGS)):
    a = PICK_ANGLE[r]
    px, py = cx + RINGS[r] * math.cos(a), cy + RINGS[r] * math.sin(a)
    print(f"ring {r}: pick at pan-frame ({px:.3f}, {py:.3f}), "
          f"{math.hypot(px, py):.3f} m, {math.degrees(math.atan2(py, px)) % 360:.0f} deg")
