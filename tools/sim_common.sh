# Shared helpers for run_trials.sh and record_demo.sh (sourced, not run).

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

setup_env() {
  # setup.bash scripts read unset variables, so no 'set -u' around them
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
  cd "$WS"
}

# Start the full bringup headless; extra launch arguments are passed on.
# Waits until both Nav2 lifecycle managers report active (or ~4 min).
# Returns 0 when Nav2 is up.
start_sim() {
  local log=$1; shift
  ros2 daemon stop > /dev/null 2>&1
  setsid ros2 launch mobile_robot_bringup mobile_manipulation.launch.py \
    gui:=false "$@" > "$log" 2>&1 < /dev/null &
  local n=0
  for _ in $(seq 1 48); do
    sleep 5
    n=$(grep -c 'Managed nodes are active' "$log")
    [ "$n" -ge 2 ] && return 0
    grep -q 'Failed to bring up' "$log" && return 1
  done
  return 1
}

# Stop the simulation and every node it started. ROS 2 Python nodes show
# up as both "/usr/bin/python3 ..." and "python3 ...", so match both.
stop_sim() {
  local nodes='^(gzserver|gzclient|/opt/ros/humble/lib/|(/usr/bin/)?python3 (/opt/ros|'"$WS"'/install|tools/))'
  local p
  p=$(pgrep -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch mobile_robot_bringup')
  [ -n "$p" ] && kill -INT $p
  sleep 10
  p=$(pgrep -f "$nodes")
  [ -n "$p" ] && kill $p
  sleep 3
  p=$(pgrep -f "$nodes")
  [ -n "$p" ] && kill -9 $p
  return 0
}

# Put the cube at a random place on the pickup pedestal
# (sideways +-5 cm, turned +-15 deg). Prints "<offset_m> <yaw_rad>".
place_cube_randomly() {
  local dy yaw
  read -r dy yaw < <(LC_ALL=C python3 -c \
    "import random; print(f'{random.uniform(-0.05,0.05):.3f} {random.uniform(-0.26,0.26):.3f}')")
  place_cube "$dy" "$yaw"
  echo "$dy $yaw"
}

# Put the cube <offset_m> sideways and turned <yaw_rad> on the pickup pedestal.
place_cube() {
  local cy
  cy=$(LC_ALL=C python3 -c "print(f'{-0.30 + $1:.3f}')")
  gz model -m pickup_cube -x 2.205 -y "$cy" -z 0.20 -R 0 -P 0 -Y "$2"
  sleep 2
}

# "CUBE ON DROP PEDESTAL" if Gazebo shows the cube upright on it.
cube_result() {
  timeout 10 gz model -m pickup_cube -p 2>/dev/null | head -1 | LC_ALL=C awk '{
    if ($1 > -2.425 && $1 < -2.175 && $2 > -0.125 && $2 < 0.125 && $3 > 0.19 && $3 < 0.21)
      print "CUBE ON DROP PEDESTAL"; else print "CUBE NOT ON DROP PEDESTAL" }'
}
