"""Grabs an X display into JPEG frames, for screen recordings without ffmpeg.

Uses xwd (X11 window dump) and PIL, which the omts container has:

  python3 tools/demo/grab_frames.py --display=:1 --fps=6 --out=/tmp/frames
  # stops when --stop_file appears (default <out>/STOP), or after --seconds

Frames are <out>/000001.jpg ...; frames.txt lists each frame's capture time (Unix),
so ffmpeg can play them back at their real pace (or faster).
"""
import argparse
import os
import struct
import subprocess
import time

import numpy as np
from PIL import Image


def xwd_to_image(data: bytes) -> Image.Image:
  """Decodes a 24/32-bit ZPixmap XWD dump (what `xwd -root` writes)."""
  (header_size, _version, _fmt, depth, width, height, _xoff, byte_order, _unit, _bit_order,
   _pad, bpp, bytes_per_line, _visual, red_mask, green_mask, blue_mask, _bits_rgb,
   _cmap_entries, ncolors, _ww, _wh, _wx, _wy, _bw) = struct.unpack(">25I", data[:100])
  start = header_size + ncolors * 12
  if bpp == 32:
    px = np.frombuffer(data, dtype="<u4" if byte_order == 0 else ">u4", count=height * bytes_per_line // 4,
                       offset=start).reshape(height, bytes_per_line // 4)[:, :width]
  elif bpp == 24:   # packed 3 bytes per pixel (the VNC display in the omts container)
    b = np.frombuffer(data, dtype=np.uint8, count=height * bytes_per_line, offset=start)
    b = b.reshape(height, bytes_per_line)[:, :width * 3].reshape(height, width, 3).astype(np.uint32)
    if byte_order != 0:
      b = b[:, :, ::-1]
    px = b[:, :, 0] | (b[:, :, 1] << 8) | (b[:, :, 2] << 16)
  else:
    raise ValueError(f"only 24 or 32 bits per pixel supported, got {bpp} (depth {depth})")

  def chan(mask):
    shift = (mask & -mask).bit_length() - 1
    return ((px & mask) >> shift).astype(np.uint8)
  return Image.fromarray(np.dstack([chan(red_mask), chan(green_mask), chan(blue_mask)]), "RGB")


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("--display", default=":1")
  ap.add_argument("--fps", type=float, default=6.0)
  ap.add_argument("--out", required=True)
  ap.add_argument("--stop_file", default="")
  ap.add_argument("--seconds", type=float, default=3600.0)
  ap.add_argument("--quality", type=int, default=88)
  args = ap.parse_args()
  os.makedirs(args.out, exist_ok=True)
  stop = args.stop_file or os.path.join(args.out, "STOP")
  log = open(os.path.join(args.out, "frames.txt"), "w")
  t0, n = time.time(), 0
  while not os.path.exists(stop) and time.time() - t0 < args.seconds:
    t = time.time()
    dump = subprocess.run(["xwd", "-root", "-silent", "-display", args.display],
                          capture_output=True, check=True).stdout
    n += 1
    xwd_to_image(dump).save(os.path.join(args.out, f"{n:06d}.jpg"), quality=args.quality)
    log.write(f"{n:06d}.jpg {t:.3f}\n")
    log.flush()
    time.sleep(max(0.0, 1.0 / args.fps - (time.time() - t)))
  print(f"{n} frames in {time.time() - t0:.0f} s")


if __name__ == "__main__":
  main()
