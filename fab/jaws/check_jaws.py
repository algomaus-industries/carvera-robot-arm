"""Checks the V-groove jaws' numbers that are easy to get wrong by eye.

- How far the moving fingertip stays above the bit's collar when the jaw is
  open for each shank size: it swings towards the collar as it opens.
- How high the moving jaw's new tip stands above the bed, printed as
  TheRobotStudio's plate lays the part (inner finger face up): the height
  its support has to bridge.
- The closing angle and the open angle for each shank, from the same gap
  table move_bit uses (tools/atc/so101_jaws.py, generated).

  python3 fab/jaws/check_jaws.py   (after python3 fab/gen_fab.py)

Exits non-zero if the open moving tip comes within 1.5 mm of a collar.
"""
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gen_fab as g  # noqa: E402

J = g.JAW
SHANKS = (3.175, 4.0, 6.0)
NEED = 1.5


def apply(m, p):
  return [sum(m[i][k] * p[k] for k in range(3)) + m[i][3] for i in range(3)]


def rot_z(a):
  c, s = math.cos(a), math.sin(a)
  return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def trans(x, y, z):
  return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def stl_vertices(path):
  with open(path, "rb") as fh:
    data = fh.read()
  n = struct.unpack_from("<I", data, 80)[0]
  for i in range(n):
    v = struct.unpack_from("<12f", data, 84 + 50 * i)
    for k in (3, 6, 9):
      yield v[k:k + 3]


def main():
  tool = g._inv(g._xform(*g.TIP))
  joint = g._mul(tool, g._xform(*g.JAW_ORIGIN))
  joint_inv = g._inv(joint)
  gap0, per_rad = g.jaw_gap_table(joint)
  s45 = math.sin(math.radians(J["V_ANGLE"]) / 2)
  xc = J["V_DEPTH"] - J["SHANK_REF"] / 2 / s45 - J["SHANK_REF"] / 2
  xf = xc - J["RIB_H"]
  collar_top = g.GRASP_Z - g.COLLAR_H
  ok = True
  print(f"fixed fingertip: {J['COLLAR_CLEAR']:.1f} mm above the collar")
  for d in SHANKS:
    a_close = (d - gap0) / per_rad
    a_open = (d + g.OPEN_EXTRA - gap0) / per_rad
    turn = g._mul(joint, g._mul(rot_z(a_open - J["A_REF"]), joint_inv))
    low = max(apply(turn, (x, 0.0, J["TIP_Z_MOVING"]))[2] for x in (xf - J["PAD_T"], xc))
    clear = collar_top - low
    ok &= clear >= NEED
    print(f"{d:5.3f} mm shank: closes at {a_close:+.3f} rad, opens to {a_open:+.3f} rad; "
          f"open moving tip {clear:.2f} mm above the collar")
  # print height: the exported part (official mesh frame) lies on the stock
  # part's flat back, the mesh's -x up (the inner finger face, tool +x, is
  # 0.08 rad off it); its new tip is everything past BLOCK_Z0 in the tool frame
  mesh_to_tool = g._mul(joint, g._mul(rot_z(J["A_REF"]), trans(0, 0, g.JAW_VISUAL_Z * g.MM)))
  vs = list(stl_vertices(os.path.join(HERE, "out", "moving_jaw_vjaw.stl")))
  bed = min(-v[0] for v in vs)
  tip = min(-v[0] for v in vs if apply(mesh_to_tool, v)[2] > J["BLOCK_Z0"])
  print(f"moving jaw printed inner face up: its new tip's underside is "
        f"{tip - bed:.1f} mm above the bed (support it)")
  sys.exit(0 if ok else 1)


if __name__ == "__main__":
  main()
