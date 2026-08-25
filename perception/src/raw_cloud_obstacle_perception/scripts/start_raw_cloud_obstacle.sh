#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/tianbot/raw_cloud_obstacle_ws"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
# The car has both 192.168.1.x and 192.168.43.x interfaces.  Advertising the
# bare hostname can make a remote RViz resolve the wrong interface even though
# the topics are visible through the master.
export ROS_IP="${RAW_CLOUD_ROS_IP:-192.168.43.59}"
unset ROS_HOSTNAME 2>/dev/null || true
if rosnode list 2>/dev/null | grep -qx '/raw_cloud_obstacle_perception'; then
  echo "[raw_cloud_perception] already running"
  exit 0
fi
exec roslaunch raw_cloud_obstacle_perception raw_cloud_obstacle.launch "$@"
