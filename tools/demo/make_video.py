"""Turns a tools/demo/record_demo.sh recording into a captioned MP4.

  python3 tools/demo/make_video.py --rec=/path/to/demo --out=demo.mp4 [--speed=4]

Each frame is shown for its real capture interval divided by --speed, so the
moves keep their pace relative to each other; frames where nothing on screen
changes (the arm waiting for the planner) go by --idle_speed times faster. Captions come from steps.txt,
and the three camera tiles get labels. Needs ffmpeg with drawtext.
"""
import argparse
import os
import subprocess
import tempfile

import numpy as np
from PIL import Image

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
TILES = (("Overview", 20, 20), ("Carousel: pick position", 1300, 20), ("Carvera ATC", 1300, 380))


def esc(text: str) -> str:
  return text.replace("\\", "\\\\").replace(":", "\\:").replace("'", "’").replace(",", "\\,")


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("--rec", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--speed", type=float, default=4.0)
  ap.add_argument("--idle_speed", type=float, default=20.0,
                  help="speed-up for frames that barely differ from the one before")
  ap.add_argument("--idle_diff", type=float, default=0.15,
                  help="mean grey-level change (0-255) below which a frame counts as idle")
  ap.add_argument("--idle_min", type=float, default=1.0,
                  help="real seconds a still stretch must last to be sped up (no flicker)")
  ap.add_argument("--title", default="SO-101 changes the Makera Carvera's tools (simulation)")
  args = ap.parse_args()
  frames = [l.split() for l in open(os.path.join(args.rec, "frames", "frames.txt")) if l.strip()]
  steps = [l.rstrip("\n").split(" ", 1) for l in open(os.path.join(args.rec, "steps.txt")) if l.strip()]
  # Still frames: barely different from the one before.
  still, prev = [], None
  for name, _ in frames:
    img = np.asarray(Image.open(os.path.join(args.rec, "frames", name)).convert("L").reduce(4), float)
    still.append(prev is not None and np.abs(img - prev).mean() < args.idle_diff)
    prev = img
  # Idle: runs of still frames lasting at least idle_min, so the speed (and
  # its badge) does not flicker.
  idle = [False] * len(frames)
  i = 0
  while i < len(frames):
    j = i
    while j < len(frames) and still[j]:
      j += 1
    if j > i and float(frames[j - 1][1]) - float(frames[i][1]) >= args.idle_min:
      idle[i:j] = [True] * (j - i)
    i = max(j, i + 1)
  # Video time of every frame: real intervals / speed, idle ones / idle_speed.
  starts, durs, t_video = [], [], 0.0
  for k, ((name, t), nxt) in enumerate(zip(frames, frames[1:] + [None])):
    dur = ((float(nxt[1]) if nxt else float(t) + 0.5) - float(t)) / (
      args.idle_speed if idle[k] else args.speed)
    starts.append(t_video)
    durs.append(dur)
    t_video += dur
  real = np.array([float(t) for _, t in frames])

  def at(t_real: float) -> float:
    """Video time of real time t_real."""
    return float(np.interp(t_real, real, starts))

  print(f"{len(frames)} frames, {real[-1] - real[0]:.0f} s real -> {t_video:.0f} s video, "
        f"{sum(idle)} idle")
  with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as lst:
    for (name, _), dur in zip(frames, durs):
      lst.write(f"file '{os.path.join(args.rec, 'frames', name)}'\nduration {dur:.4f}\n")
    lst.write(f"file '{os.path.join(args.rec, 'frames', frames[-1][0])}'\n")
  draw = [f"drawtext=fontfile={FONT}:text='{esc(args.title)}':x=20:y=h-120:fontsize=30:"
          "fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=10"]
  for label, x, y in TILES:
    draw.append(f"drawtext=fontfile={FONT}:text='{esc(label)}':x={x}:y={y}:fontsize=24:"
                "fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=8")
  for (ts, cap), nxt in zip(steps, steps[1:] + [None]):
    if cap == "end":
      continue
    a = at(float(ts))
    b = at(float(nxt[0]) if nxt else float(frames[-1][1]))
    draw.append(f"drawtext=fontfile={FONT}:text='{esc(cap)}':x=20:y=h-65:fontsize=30:"
                f"fontcolor=yellow:box=1:boxcolor=black@0.55:boxborderw=10:enable='between(t,{a:.2f},{b:.2f})'")
  # Speed badge, bottom right of the overview tile (the picker is right of
  # it): the normal speed-up, or the idle one (in a
  # different colour) while the arm waits.
  runs, k = [], 0
  while k < len(frames):
    if idle[k]:
      j = k
      while j < len(frames) and idle[j]:
        j += 1
      runs.append((starts[k], starts[j - 1] + durs[j - 1]))
      k = j
    else:
      k += 1
  fast = "+".join(f"between(t,{a:.3f},{b:.3f})" for a, b in runs) or "0"
  badge = "fontfile={f}:x=1280-tw-20:y=h-65:fontsize=30:box=1:boxcolor=black@0.55:boxborderw=10".format(f=FONT)
  draw.append(f"drawtext={badge}:text='{args.speed:g}x':fontcolor=white:enable='not({fast})'")
  draw.append(f"drawtext={badge}:text='{args.idle_speed:g}x  waiting':fontcolor=0x7fd7ff:enable='{fast}'")
  cmd = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-f", "concat", "-safe", "0",
         "-i", lst.name, "-vf", ",".join(draw + ["fps=30", "format=yuv420p"]),
         "-c:v", "libx264", "-preset", "slow", "-crf", "24", "-movflags", "+faststart", args.out]
  subprocess.run(cmd, check=True)
  os.unlink(lst.name)
  print(args.out)


if __name__ == "__main__":
  main()
