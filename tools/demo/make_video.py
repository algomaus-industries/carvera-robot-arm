"""Turns a tools/demo/record_demo.sh recording into a captioned MP4.

  python3 tools/demo/make_video.py --rec=/path/to/demo --out=demo.mp4 [--speed=4]

Each frame is shown for its real capture interval divided by --speed, so the
moves keep their pace relative to each other. Captions come from steps.txt,
and the three camera tiles get labels. Needs ffmpeg with drawtext.
"""
import argparse
import os
import subprocess
import tempfile

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
TILES = (("Overview", 20, 20), ("Carousel: pick position", 1300, 20), ("Carvera ATC", 1300, 380))


def esc(text: str) -> str:
  return text.replace("\\", "\\\\").replace(":", "\\:").replace("'", "’").replace(",", "\\,")


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("--rec", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--speed", type=float, default=4.0)
  ap.add_argument("--title", default="SO-101 changes the Makera Carvera's tools (simulation)")
  args = ap.parse_args()
  frames = [l.split() for l in open(os.path.join(args.rec, "frames", "frames.txt")) if l.strip()]
  t0 = float(frames[0][1])
  steps = [l.rstrip("\n").split(" ", 1) for l in open(os.path.join(args.rec, "steps.txt")) if l.strip()]
  with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as lst:
    for (name, t), nxt in zip(frames, frames[1:] + [None]):
      dur = ((float(nxt[1]) if nxt else float(t) + 0.5) - float(t)) / args.speed
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
    a = (float(ts) - t0) / args.speed
    b = ((float(nxt[0]) if nxt else float(frames[-1][1])) - t0) / args.speed
    draw.append(f"drawtext=fontfile={FONT}:text='{esc(cap)}':x=20:y=h-65:fontsize=30:"
                f"fontcolor=yellow:box=1:boxcolor=black@0.55:boxborderw=10:enable='between(t,{a:.2f},{b:.2f})'")
  if args.speed != 1:
    draw.append(f"drawtext=fontfile={FONT}:text='{args.speed:g}x':x=w-90:y=h-65:fontsize=30:"
                "fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=10")
  cmd = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-f", "concat", "-safe", "0",
         "-i", lst.name, "-vf", ",".join(draw + ["fps=30", "format=yuv420p"]),
         "-c:v", "libx264", "-preset", "slow", "-crf", "24", "-movflags", "+faststart", args.out]
  subprocess.run(cmd, check=True)
  os.unlink(lst.name)
  print(args.out)


if __name__ == "__main__":
  main()
