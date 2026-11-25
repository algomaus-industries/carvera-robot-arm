#!/bin/bash
# Fetches TheRobotStudio's official SO-101 meshes (Apache-2.0, ~16 MB) that
# build_so101.py turns into the so101_*.glb files. Not checked in.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p official_meshes
base=https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main
for f in base_so101_v2 base_motor_holder_so101_v1 waveshare_mounting_plate_so101_v2 \
    sts3215_03a_v1 sts3215_03a_no_horn_v1 motor_holder_so101_base_v1 rotation_pitch_so101_v1 \
    upper_arm_so101_v1 under_arm_so101_v1 motor_holder_so101_wrist_v1 wrist_roll_pitch_so101_v2 \
    wrist_roll_follower_so101_v1 moving_jaw_so101_v1; do
  curl -sSfL -o "official_meshes/$f.stl" "$base/Simulation/SO101/assets/$f.stl"
done
curl -sSfL -o official_meshes/LICENSE "$base/LICENSE"
