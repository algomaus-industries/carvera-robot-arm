"""Checks the bit carousel's seat layout for the SO-101 jaw.

For every seat, finds the jaw direction that leaves the most room to the
neighbouring seats (the same rule tools/atc/move_bit.py uses) and reports the
tightest seat. The open jaw reaches ~20 mm either side of the shank along its
axis and needs >= ~12 mm sideways to a neighbouring bit: below that, the sim
showed the jaw hitting neighbours.

  python3 fab/check_layout.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "bit_carousel"))
import carousel_layout as cl  # noqa: E402

JAW_REACH = 0.020
NEED = 0.012


def seg_dist(px, py, dx, dy, half):
  t = max(-half, min(half, px * dx + py * dy))
  return math.hypot(px - t * dx, py - t * dy)


def main():
  seats = [(r, c, x, y) for r, c, x, y, _ in cl.cells()]
  worst = None
  for r, c, x, y in seats:
    near = [(qx - x, qy - y) for rr, cc, qx, qy in seats
            if (rr, cc) != (r, c) and math.hypot(qx - x, qy - y) < 0.04]
    best = 0.0
    for k in range(90):
      a = math.pi * k / 90
      d = min(seg_dist(px, py, math.cos(a), math.sin(a), JAW_REACH) for px, py in near)
      best = max(best, d)
    if worst is None or best < worst[0]:
      worst = (best, r, c)
  print(f"{len(seats)} seats; tightest seat cell_{worst[1]}_{worst[2]}: "
        f"{worst[0] * 1000:.1f} mm jaw clearance (need >= {NEED * 1000:.0f})")
  sys.exit(0 if worst[0] >= NEED else 1)


if __name__ == "__main__":
  main()
