#!/bin/bash
# Repeated full missions, each in a freshly started headless simulation,
# with the cube put at a random place on the pickup pedestal before each.
# A run only counts as a success if Gazebo shows the cube on the drop
# pedestal afterwards.
#
# Usage: tools/run_trials.sh [N=5] [OUT_DIR=/tmp/mobile_robot_trials]
# Logs per trial in OUT_DIR, one summary line per trial on stdout and in
# OUT_DIR/summary.log. Build the workspace first.

N=${1:-5}
OUT=${2:-/tmp/mobile_robot_trials}
source "$(dirname "$0")/sim_common.sh"
setup_env
mkdir -p "$OUT"
: > "$OUT/summary.log"
trap 'stop_sim; exit 130' INT TERM

ok=0
for i in $(seq 1 "$N"); do
  if start_sim "$OUT/bringup_$i.log"; then
    read -r dy yaw < <(place_cube_randomly)
    timeout 1500 ros2 run mobile_robot_bringup mobile_manipulation_mission.py \
      > "$OUT/mission_$i.log" 2>&1
    res=$(grep -o -E 'MISSION COMPLETE|MISSION ABORTED' "$OUT/mission_$i.log" | tail -1)
    placed=$(cube_result)
    seen=$(grep -m1 -o -E 'Cube seen by the camera at \([^)]*\)' "$OUT/mission_$i.log")
    err=$(grep -m1 -E '\[ERROR\]' "$OUT/mission_$i.log" | sed -E 's/.*\]: //' | cut -c1-120)
    line="TRIAL $i: ${res:-NO RESULT}, $placed | cube offset $dy m, yaw $yaw rad | $seen${err:+ | $err}"
    [ "$placed" = "CUBE ON DROP PEDESTAL" ] && ok=$((ok + 1))
  else
    line="TRIAL $i: Nav2 did not come up"
  fi
  echo "$line" | tee -a "$OUT/summary.log"
  stop_sim
done
echo "SUCCESS: $ok of $N" | tee -a "$OUT/summary.log"
