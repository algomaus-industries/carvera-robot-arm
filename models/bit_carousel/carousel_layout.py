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
SEAT_PITCH = 0.022  # min centre distance along a ring
RING_PITCH = 0.028  # between rings
# The open jaw needs >= ~12 mm sideways to a neighbouring bit (see
# tools/atc/move_bit.py); at these pitches every seat has a jaw direction with
# >= 13.3 mm (fab/check_layout.py). Tighter rings (21 x 25 mm) left the
# innermost ring only 5.5 mm, and the jaw hit its neighbours.
RINGS = tuple(round(0.045 + k * RING_PITCH, 4) for k in range(5))  # 45..157 mm
SEAT_Z = 0.075  # seat plate top above the cart top
PLATE_T = 0.006  # seat plate thickness
PLATE_R = 0.176  # seat plate radius
PUCK_R = 0.0105  # seat boss radius in the sim model

# ---------------------------------------------------------------- mechanics
# Shared by the sim model and the fabrication files (fab/). Heights above
# the cart top.
BASE_T = 0.006  # base plate (plywood, screwed to the cart top)
BASE_R = 0.178
RIM_R = (0.162, 0.170)  # printed rim under the seat plate's edge
RIM_Z0 = 0.032  # rim bottom, riding on the rollers
ROLLER_R = 0.166  # 608 roller circle (rim centre line)
ROLLERS = 4
BEARING_OD, BEARING_W = 0.022, 0.007  # 608 skate bearing
HUB_R = 0.022  # printed hub: servo horn to seat plate
SERVO_BODY = (0.0247, 0.0452, 0.035)  # STS3215 W x L x H (datasheet)
SERVO_SHAFT_FROM_END = 0.0115  # output shaft from the near end (CHECK on yours)
HORN_TOP = BASE_T + 0.0015 + SERVO_BODY[2] + 0.0045  # cradle floor, body, horn
HOLE_D = 0.0066  # seat hole: passes any Makera bit's cutter and
# shank (<= 6 mm), the collar (>= shank + 6 mm) rests
# on the plate

# ---------------------------------------------------------------- placement
# Turntable axis in the arm's pan frame (origin on the SO-101's pan axis,
# axes along the world's). On the cart (pan frame x 0.025..0.38,
# y -0.345..0.09) and clear of the arm pedestal.
CENTER_IN_PAN = (0.203, -0.167)
# Pick positions sit this far from the arm's pan axis (inside the 0.08..0.27 m
# band the arm reaches with the gripper vertical at seat height), on the side
# of each ring facing the ATC.
PICK_REACH = (0.225, 0.205, 0.190, 0.175, 0.160)


def ring_seats(ring: int) -> int:
  """Seats on a ring: as many as fit at SEAT_PITCH, rounded down to even, so
  the seat plate's seam (along x, the plate is cut in two halves) passes
  midway between two seats on both sides."""
  n = int(2 * math.pi * RINGS[ring] / SEAT_PITCH)
  return n - n % 2


def seat_angle(ring: int, index: int) -> float:
  """Seat angle at platter angle 0, half a step off the seam (x axis)."""
  return 2 * math.pi * (index + 0.5) / ring_seats(ring)


def cells():
  """[(ring, index, x, y, angle)] in the carousel frame at platter angle 0."""
  out = []
  for r, rr in enumerate(RINGS):
    n = ring_seats(r)
    for i in range(n):
      th = seat_angle(r, i)
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
  return (PICK_ANGLE[ring] - seat_angle(ring, index)) % (2 * math.pi)


if __name__ == "__main__":
  cs = cells()
  print(len(cs), "seats;", [ring_seats(r) for r in range(len(RINGS))])
  cx, cy = CENTER_IN_PAN
  for r in range(len(RINGS)):
    a = PICK_ANGLE[r]
    px, py = cx + RINGS[r] * math.cos(a), cy + RINGS[r] * math.sin(a)
    print(
      f"ring {r}: pick at pan-frame ({px:.3f}, {py:.3f}), "
      f"{math.hypot(px, py):.3f} m, {math.degrees(math.atan2(py, px)) % 360:.0f} deg"
    )
