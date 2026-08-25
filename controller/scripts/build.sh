#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/noetic/setup.bash
LOCAL_ACKERMANN_PREFIX="$(dirname "$ROOT")/f1tenth_ws/local_ros_noetic/opt/ros/noetic"
if [[ -d "$LOCAL_ACKERMANN_PREFIX/share/ackermann_msgs" ]]; then
  export CMAKE_PREFIX_PATH="$LOCAL_ACKERMANN_PREFIX:${CMAKE_PREFIX_PATH:-}"
fi
RESIDUAL_DYNAMICS_WS="${RESIDUAL_DYNAMICS_WS:-$(dirname "$ROOT")/f1tenth_residual_ws}"
source "$RESIDUAL_DYNAMICS_WS/devel/setup.bash"
set -u
catkin_make -C "$ROOT" --force-cmake
