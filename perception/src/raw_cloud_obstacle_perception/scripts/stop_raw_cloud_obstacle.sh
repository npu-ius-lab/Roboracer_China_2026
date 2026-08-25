#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${RAW_CLOUD_ROS_IP:-192.168.43.59}"
unset ROS_HOSTNAME 2>/dev/null || true

if rosnode list 2>/dev/null | grep -qx '/raw_cloud_obstacle_perception'; then
  rosnode kill /raw_cloud_obstacle_perception
else
  echo "[raw_cloud_perception] not running"
fi
