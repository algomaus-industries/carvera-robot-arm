# Demo video (`tools/demo/`)

Records the SO-101 cell in RViz and turns it into a captioned MP4
([`docs/carvera_so101_demo.mp4`](../../docs/carvera_so101_demo.mp4)).

The screen (1920 × 1080, the container's VNC display) is tiled: a large
overview on the left; on the right a camera over the carousel's pick
positions, one on the Carvera's ATC rack, and the ATC picker. The cameras are
RViz configs made from `configs/omts/omts.rviz` by
[`rviz_views.py`](rviz_views.py).

1. Inside the omts container, with the SO-101 cell deployed and laid out:

   ```bash
   OUT=/tmp/demo tools/demo/record_demo.sh
   ```

   It parks the cart, opens the three RViz windows, grabs frames with
   [`grab_frames.py`](grab_frames.py) (`xwd`, 6 frames/s), and runs: dock,
   the picker filling T1–T6 from all five carousel rings, the picker
   swapping two slots, park. `SET1` / `SET2` override the picks.
2. Copy `$OUT` out of the container and, where ffmpeg (with drawtext) is:

   ```bash
   python3 tools/demo/make_video.py --rec=demo --out=docs/carvera_so101_demo.mp4 --speed=4
   ```

   Frames keep their real spacing divided by `--speed`; captions come from
   the steps' start times.
