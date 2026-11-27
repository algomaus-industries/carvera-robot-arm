"""Writes RViz configs for the demo video: one per camera, tiled on screen.

Starts from configs/omts/omts.rviz (its displays, minus the TF axes), hides the side
docks and sets each window's place on a 1920 x 1080 screen and its orbit
camera:

  overview   left, 1280 x 1080: the Carvera, the cart, the carousel and the arm
  carousel   right, top 640 x 360: the carousel's pick positions, from above
  atc        right, middle 640 x 360: the ATC rack in the open Carvera
  (the ATC picker, tools/atc/atc_picker.py, takes the bottom right 640 x 360)

  python3 tools/demo/rviz_views.py --out=/tmp/demo_rviz
  # then: rviz2 -d /tmp/demo_rviz/overview.rviz (and the others)

World coordinates come from the SO-101 cell (configs/carvera_so101).
"""

import argparse
import os
import re

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC = os.path.join(REPO, "configs", "omts", "omts.rviz")

# name: (x, y, w, h), focal point, distance, pitch, yaw (RViz orbit: the camera
# sits at focal + distance * (cos p cos y, cos p sin y, sin p))
VIEWS = {
  "overview": ((0, 0, 1280, 1080), (-0.20, -0.17, 0.98), 1.55, 0.42, -0.75),
  "carousel": ((1280, 0, 640, 360), (-0.10, -0.20, 0.99), 0.40, 1.05, -0.35),
  "atc": ((1280, 360, 640, 360), (-0.415, -0.03, 1.00), 0.36, 0.55, -0.55),
}


def view_config(text: str, geom, focal, dist, pitch, yaw) -> str:
  x, y, w, h = geom
  cur = text.index("  Views:\n    Current:")
  end = text.index("    Saved:", cur)
  block = text[cur:end]
  block = re.sub(r"Distance: [-\d.e]+", f"Distance: {dist}", block, count=1)
  block = re.sub(r"Pitch: [-\d.e]+", f"Pitch: {pitch}", block, count=1)
  block = re.sub(r"Yaw: [-\d.e]+", f"Yaw: {yaw}", block, count=1)
  block = re.sub(
    r"Focal Point:\n\s+X: [-\d.e]+\n\s+Y: [-\d.e]+\n\s+Z: [-\d.e]+",
    f"Focal Point:\n        X: {focal[0]}\n        Y: {focal[1]}\n        Z: {focal[2]}",
    block,
  )
  text = text[:cur] + block + text[end:]
  geo = text.index("Window Geometry:")
  tail = text[geo:]
  for key, val in (
    ("Height", h),
    ("Width", w),
    ("X", x),
    ("Y", y),
    ("Hide Left Dock", "true"),
    ("Hide Right Dock", "true"),
  ):
    tail = re.sub(rf"(\n  {key}: )[^\n]+", rf"\g<1>{val}", tail, count=1)
  tail = re.sub(r"(\n  Image:\n    collapsed: )\w+", r"\g<1>true", tail)
  # Only the 3D view: no side panels (the saved Qt window state would bring
  # them back, so it goes), and no camera image display.
  tail = re.sub(r"\n  QMainWindow State: [^\n]+", "", tail)
  tail = re.sub(r"(\n  \w+:\n    collapsed: )\w+", r"\g<1>true", tail)
  head = text[:geo]
  head = re.sub(r"^Panels:\n(  [^\n]*\n|  - [^\n]*\n)+", "Panels: []\n", head)
  head = re.sub(
    r"    - Class: rviz_default_plugins/Image\n(      [^\n]*\n)+", "", head
  )
  # No TF axes or frame-editor handles in the video: with 140 seat frames and
  # every bit's frame they hide the arm and the bits. The meshes are the
  # Workcell Markers display.
  for cls in ("TF", "InteractiveMarkers"):
    head = re.sub(
      rf"(- Class: rviz_default_plugins/{cls}\n(?:\s+[A-Z][^\n]*\n)*?\s+Enabled: )true",
      r"\g<1>false",
      head,
    )
  return head + tail


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("--out", required=True)
  args = ap.parse_args()
  os.makedirs(args.out, exist_ok=True)
  src = open(SRC).read()
  for name, (geom, focal, dist, pitch, yaw) in VIEWS.items():
    path = os.path.join(args.out, name + ".rviz")
    with open(path, "w") as fh:
      fh.write(view_config(src, geom, focal, dist, pitch, yaw))
    print(path)


if __name__ == "__main__":
  main()
