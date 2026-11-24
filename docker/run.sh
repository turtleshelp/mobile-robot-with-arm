#!/usr/bin/env bash
# Run this workspace in a ROS 2 Humble container with the Gazebo/RViz
# windows on your desktop.
#
#   docker/run.sh image     build the Docker image (once, ~10-20 min)
#   docker/run.sh build     colcon build the workspace inside the container
#   docker/run.sh sim       terminal 1: Gazebo, Nav2, MoveIt (extra args go to the launch)
#   docker/run.sh mission   terminal 2: the pick-and-place mission
#   docker/run.sh shell     an interactive shell in the container
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=mobile-robot-with-arm:humble
NAME=mobile-robot-with-arm

image() {
  docker build -t "$IMAGE" -f "$WS/docker/Dockerfile" \
    --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" "$WS"
}

# Start the long-running container if it is not running yet.
ensure_container() {
  if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then return; fi
  docker rm -f "$NAME" > /dev/null 2>&1 || true
  docker image inspect "$IMAGE" > /dev/null 2>&1 || image

  local xauth_args=()
  if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
    xauth_args=(-v "$XAUTHORITY:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
  fi
  local gpu_args=()
  if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
    # NVIDIA GPU via nvidia-container-toolkit. The PRIME variables make
    # GLX use the NVIDIA GPU on hybrid laptops where Intel drives the screen.
    echo "Rendering on the NVIDIA GPU" >&2
    gpu_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all
              -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)
  else
    # Ubuntu 22.04's Mesa (23.2) can't drive newer Intel GPUs such as
    # Arrow Lake, and trying makes gzclient hang before it opens a window.
    # Render on the CPU instead (slow). Set GPU=intel to try the iGPU.
    echo "No NVIDIA container runtime: rendering on the CPU (slow)" >&2
    if [ "${GPU:-}" = intel ] && [ -d /dev/dri ]; then
      gpu_args=(--device /dev/dri)
      for g in video render; do
        local gid; gid=$(getent group "$g" | cut -d: -f3 || true)
        [ -n "$gid" ] && gpu_args+=(--group-add "$gid")
      done
    else
      gpu_args=(-e LIBGL_ALWAYS_SOFTWARE=1)
    fi
  fi

  docker run -d --name "$NAME" \
    --net=host --ipc=host \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${xauth_args[@]}" "${gpu_args[@]}" \
    -v "$WS:/ws" \
    "$IMAGE" sleep infinity > /dev/null
}

in_container() {
  local tty=(-i); [ -t 0 ] && tty=(-it)
  docker exec "${tty[@]}" "$NAME" bash -ic "$*"
}

cmd=${1:-shell}; shift || true
case "$cmd" in
  image)   image ;;
  build)   ensure_container; in_container "colcon build --symlink-install $*" ;;
  sim)     ensure_container; in_container "ros2 launch mobile_robot_bringup mobile_manipulation.launch.py $*" ;;
  mission) ensure_container; in_container "ros2 run mobile_robot_bringup mobile_manipulation_mission.py $*" ;;
  shell)   ensure_container; in_container "bash" ;;
  stop)    docker rm -f "$NAME" > /dev/null && echo "container stopped" ;;
  *) echo "usage: $0 {image|build|sim|mission|shell|stop}"; exit 1 ;;
esac
