#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/ros_env_remote_master.sh"

if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] cannot contact TianRacer ROS master at $ROS_MASTER_URI" >&2
  exit 1
fi

echo "[OK] ROS master: $ROS_MASTER_URI"
echo "[OK] local ROS IP: $ROS_IP"
exec rviz -d "$ROOT/src/f1tenth_dynamic_mpcc/rviz/nav.rviz"

