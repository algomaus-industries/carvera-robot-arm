"""Grip-force check for the V-groove jaws: does the bit stay put in the hand?

The simulator does not model the grip (move_bit sets the jaw through the
world and attaches the bit), so this works it out by hand, as statics:

- Clamp force F: the gripper servo's torque on the jaw, through the lever
  from the hinge to the pad's rib (the hinge's height above the grip).
- Contact: the rib presses the shank into the 90 deg V, so each V face pushes
  back with F / (2 sin 45) = 0.71 F.
- Axial hold (the bit sliding along the jaw): friction on the rib and both
  V faces, mu_tpu F + mu_petg 1.41 F. Against the bit's weight, and the
  seat's grip on the bit when it is lifted out.
- Tipping: a side force P at the cutter's tip, L below the rib, is held by
  the V's line contact (z = z0 .. z1 along the shank in the jaw); it lets go
  when the V's reaction can no longer balance P L about the rib. In the
  jaw's plane: P <= F z1 / (L + z1). Across it (along the hinge) only the
  V faces' sideways share of their preload, 0.5 F each, holds it:
  P <= 0.5 F z1 / (L + z1).

  python3 fab/jaws/check_grip.py

Numbers marked ASSUMED are textbook values, not measured. Exits non-zero
if the weakest bit's axial hold is under 20x its weight at the suggested
torque limit.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
import gen_fab as g  # noqa: E402

J = g.JAW
# Gripper servo: the follower arm's STS3215, 12 V 1:345 (Seeed ST3215-C047,
# 30 kg.cm stall at 12 V; the SO-101 Pro kit's follower).
STALL_NM = 30 * 9.81 / 100
TORQUE_LIMITS = (0.3, 0.5, 1.0)
SUGGESTED = 0.5
MU_TPU = 0.6        # ASSUMED: TPU 95A on carbide/steel, dry
MU_PETG = 0.3       # ASSUMED: PETG on carbide/steel, dry
CARBIDE_KG_M3 = 14500.0
SIDE_LOAD_N = 1.0   # ASSUMED: a seat's chamfer or a neighbour nudging the cutter


def bits():
  with open(os.path.join(REPO, "models", "makera_bits", "makera_bits.json")) as fh:
    data = json.load(fh)
  return data["bits"], data["collar_h"] * 1000, data["stick_up"] * 1000


def main():
  tool = g._inv(g._xform(*g.TIP))
  joint = g._mul(tool, g._xform(*g.JAW_ORIGIN))
  lever = -joint[2][3]                       # mm, hinge to the rib at z = 0
  s = math.sin(math.radians(J["V_ANGLE"]) / 2)
  catalogue, collar_h, stick_up = bits()
  shank_top = g.GRASP_Z - collar_h - stick_up     # tool z (z grows towards the tip)
  z1 = J["TIP_Z"]                            # the V's contact runs to the fixed tip
  z0 = max(J["BLOCK_Z0"], shank_top)
  print(f"lever hinge -> rib {lever:.1f} mm; the shank lies in the V from z = {z0:.1f} "
        f"to {z1:.1f} mm (rib at 0); servo stall {STALL_NM:.2f} N m")
  # the heaviest bit and the longest reach below the rib
  def mass(b):
    return CARBIDE_KG_M3 * math.pi * (b["shank"] / 2) ** 2 * b["overall"]
  heavy = max(catalogue, key=mass)
  far = max(catalogue, key=lambda b: b["overall"])
  reach = g.GRASP_Z + far["overall"] * 1000 - collar_h - stick_up   # rib -> cutter tip
  weight = mass(heavy) * 9.81
  print(f"heaviest bit {heavy['id']}: {mass(heavy) * 1000:.0f} g; longest {far['id']}: "
        f"cutter tip {reach:.0f} mm below the rib")
  print(f"{'limit':>6} {'clamp F':>8} {'axial hold':>11} {'x weight':>9} "
        f"{'tip side load in plane / across':>32}")
  ok = True
  for lim in TORQUE_LIMITS:
    f = STALL_NM * lim / (lever / 1000)
    hold = MU_TPU * f + MU_PETG * f / s
    p_in = f * z1 / (reach + z1)
    p_across = 0.5 * f * z1 / (reach + z1)
    print(f"{lim * 100:5.0f}% {f:7.1f} N {hold:9.1f} N {hold / weight:8.0f}x "
          f"{p_in:14.2f} N / {p_across:.2f} N")
    if lim == SUGGESTED:
      ok = hold >= 20 * weight
      weakest = p_across
  print(f"At the suggested {SUGGESTED * 100:.0f}% torque limit the longest bit tips over "
        f"at {weakest:.2f} N sideways at its cutter (assumed nudges: ~{SIDE_LOAD_N:.0f} N):",
        "OK" if weakest >= SIDE_LOAD_N else "marginal, keep seats chamfered and bits aligned")
  sys.exit(0 if ok else 1)


if __name__ == "__main__":
  main()
