#!/bin/bash
# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0
# Record one full mission with the demo cameras and the robot camera's
# detection view, then cut it into an MP4 and a GIF.
#
# Usage: tools/record_demo.sh [OUT_DIR=/tmp/mobile_robot_demo]
# Result: OUT_DIR/video/mobile_manipulation_demo.{mp4,gif}
# Needs ffmpeg. Build the workspace first.

OUT=${1:-/tmp/mobile_robot_demo}
source "$(dirname "$0")/sim_common.sh"
setup_env
REC="$OUT/recording"
rm -rf "$REC"; mkdir -p "$REC" "$OUT/video"
trap 'stop_sim; exit 130' INT TERM

if ! start_sim "$OUT/bringup.log" record:=true record_dir:="$REC"; then
  echo "Nav2 did not come up, see $OUT/bringup.log"
  stop_sim; exit 1
fi

# Cube 3 cm right of centre and turned 9 deg, so the camera has to find it
place_cube -0.03 0.16
setsid python3 tools/record_detector_view.py "$REC" > "$OUT/detector_view.log" 2>&1 < /dev/null &

# Prefix every mission log line with the overview frame count, so the
# video can be cut by mission phase
ros2 run mobile_robot_bringup mobile_manipulation_mission.py 2>&1 \
  | while IFS= read -r line; do
      echo "$(ls "$REC/overview" 2>/dev/null | wc -l)|$line"
    done > "$OUT/mission_events.log"

res=$(grep -o -E 'MISSION COMPLETE|MISSION ABORTED' "$OUT/mission_events.log" | tail -1)
placed=$(cube_result)
stop_sim
echo "${res:-NO RESULT}, $placed"

if [ "$placed" = "CUBE ON DROP PEDESTAL" ]; then
  python3 tools/make_demo_video.py "$REC" "$OUT/mission_events.log" "$OUT/video"
else
  echo "Not making a video of a failed run."
  exit 1
fi
