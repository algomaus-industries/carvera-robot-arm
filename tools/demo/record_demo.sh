#!/bin/bash
# Records the SO-101 cell demo in RViz: three cameras tiled on the VNC display
# (tools/demo/rviz_views.py) and the ATC picker (tools/atc/atc_picker.py),
# frames grabbed with tools/demo/grab_frames.py, the steps below run with
# move_bit and the picker. Run inside the omts container from the
# repo root, with the SO-101 cell deployed and laid out (README: Running it).
# Writes $OUT/frames/*.jpg, frames.txt and steps.txt (Unix start time and
# caption of each step); tools/demo/make_video.sh turns them into an MP4.
#
#   OUT=/tmp/demo tools/demo/record_demo.sh
set -u
OUT=${OUT:-/tmp/demo}
DISPLAY_NUM=${DISPLAY_NUM:-:1}
MB="bazel run -c opt //tools/atc:move_bit -- --config=configs/carvera_so101/app_config.yaml"
mkdir -p "$OUT"
rm -rf "$OUT/frames" "$OUT/steps.txt"

step() {  # step "caption" command...
  local caption=$1; shift
  echo "$(date +%s.%N) $caption" >> "$OUT/steps.txt"
  echo "== $caption"
  "$@" 2>&1 | grep -E "✓|\[!\]|turning|Rolling|Opening|Closing" || true
}

# Build first, so the recording does not wait for Bazel (the picker runs
# bazel-bin/tools/atc/move_bit).
bazel build -c opt //tools/atc:move_bit 2>&1 | tail -1
MB="bazel-bin/tools/atc/move_bit --config=configs/carvera_so101/app_config.yaml"

# Start parked: cart away, hood shut (not recorded).
$MB --park 2>&1 | grep -E "✓|\[!\]"

# A SIGTERMed rviz2 can hang in its shutdown while its Zenoh subscriptions
# keep buffering the marker stream (it grew to 18 GB once), so: TERM, wait,
# then KILL.
stop_rviz() {
  pkill -f "rviz2 -d $OUT/rviz" || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do pgrep -f "rviz2 -d $OUT/rviz" > /dev/null || return 0; sleep 0.5; done
  pkill -9 -f "rviz2 -d $OUT/rviz" || true
}
# Also if this script dies part-way: no RViz left behind.
trap 'touch "$OUT/frames/STOP" 2> /dev/null; stop_rviz' EXIT

# Three RViz windows; the ROS bridge restart makes new RViz windows see the
# whole world, not only the latest updates.
python3 tools/demo/rviz_views.py --out="$OUT/rviz" > /dev/null
pkill -f "rviz2 .*omts.rviz" || true
kubectl delete pod -n app-resources rs-flowstate-ros-bridge-0 > /dev/null
kubectl wait --for=condition=Ready pod/rs-flowstate-ros-bridge-0 -n app-resources --timeout=180s > /dev/null
set +u; source /opt/ros/lyrical/setup.bash; set -u   # the ROS setup reads unset variables
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:17447"]'
for v in overview carousel atc; do
  DISPLAY=$DISPLAY_NUM nohup rviz2 -d "$OUT/rviz/$v.rviz" > "$OUT/rviz_$v.log" 2>&1 &
done
sleep 25

# Hide the XFCE panels (taskbar, dock) while recording; restored at the end.
export DBUS_SESSION_BUS_ADDRESS=$(grep -z DBUS_SESSION_BUS_ADDRESS \
  /proc/$(pgrep -u "$USER" -x xfdesktop | head -1)/environ | tr -d '\0' | cut -d= -f2-)
PANEL_STATE=""
for p in $(xfconf-query -c xfce4-panel -p /panels 2>/dev/null | grep -E '^[0-9]+$'); do
  PANEL_STATE+="$p=$(xfconf-query -c xfce4-panel -p /panels/panel-$p/autohide-behavior 2>/dev/null || echo 0) "
  xfconf-query -c xfce4-panel -p /panels/panel-$p/autohide-behavior -n -t uint -s 2
done
sleep 2

python3 tools/demo/grab_frames.py --display=$DISPLAY_NUM --fps=${FPS:-6} --out="$OUT/frames" &
GRAB=$!
# Watchdog: if the container runs low on memory or one RViz window grows past
# MAX_RVIZ_MB, stop the recording and close its RViz windows rather than let
# the kernel kill the whole desktop.
( while sleep 5; do
    avail=$(free -m | awk '/Mem/ {print $7}')
    big=$(ps -o rss= -p "$(pgrep -d, -f "rviz2 -d $OUT/rviz")" 2>/dev/null | sort -n | tail -1)
    if [ "$avail" -lt "${MIN_FREE_MB:-5000}" ] || [ "${big:-0}" -gt $(( ${MAX_RVIZ_MB:-3000} * 1024 )) ]; then
      echo "[!] ${avail} MB free, largest RViz ${big:-0} kB: stopping the recording" >&2
      touch "$OUT/frames/STOP"; stop_rviz; break
    fi
  done ) &
WATCH=$!
sleep 3

PICK="python3 tools/atc/atc_picker.py --geometry=640x360+1280+720 --exit_when_done"
step "Hood opens, the cart rolls up to the Carvera" $MB --dock
step "ATC picker: one bit per slot, from all five carousel rings, then Apply" \
  env DISPLAY=$DISPLAY_NUM $PICK --demo="${SET1:-1=oflm4_12,2=bn18_1x4,3=eng18_30x0p2,4=drill18_1p6,5=corn18_1,6=thread18_m3}"
step "Change of plan: two slots swapped (old bits back to their seats)" \
  env DISPLAY=$DISPLAY_NUM $PICK --demo="${SET2:-2=ofl6_22,5=chamfer90_4}"
step "Cart rolls away, hood shuts" $MB --park
echo "$(date +%s.%N) end" >> "$OUT/steps.txt"

sleep 3
touch "$OUT/frames/STOP"
wait $GRAB
kill $WATCH 2>/dev/null
for kv in $PANEL_STATE; do
  xfconf-query -c xfce4-panel -p /panels/panel-${kv%%=*}/autohide-behavior -n -t uint -s "${kv#*=}"
done
stop_rviz
# Give the usual single RViz back.
[ -x ~/.local/bin/omts-rviz ] && DISPLAY=$DISPLAY_NUM nohup xfce4-terminal -e ~/.local/bin/omts-rviz > /dev/null 2>&1 &
echo "frames: $(ls "$OUT/frames" | grep -c jpg)"
