#!/usr/bin/env bash
# play_bag.sh - 在隔离 roscore(11313)上回放 bag 给检测器/RViz 看。
#
# 用法(机载电脑上,先运行 detector_rviz.sh):
#   ./scripts/play_bag.sh                  # 默认 bag
#   ./scripts/play_bag.sh /path/to.bag     # 指定 bag
#
# 前台运行,显示回放进度;Ctrl-C 停止;可反复运行同一个/不同 bag。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-/home/tianbot/bag/2026-08-17-18-13-44.bag}"
PORT=11313

if [[ ! -f "$BAG" ]]; then
  echo "[ERROR] bag 不存在: $BAG" >&2
  exit 2
fi

set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"
source /home/tianbot/tianbot_ws/devel/setup.bash
set -u
export ROS_PACKAGE_PATH="$ROOT/src:/home/tianbot/f1tenth_residual_ws/src:/home/tianbot/tianbot_ws/src:$ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$ROOT/devel/lib:/home/tianbot/f1tenth_residual_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/home/tianbot/tianbot_ws/devel/lib:$LD_LIBRARY_PATH"
export ROS_MASTER_URI="http://127.0.0.1:$PORT"

if ! timeout 2 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
  echo "[ERROR] 隔离 roscore($PORT)未运行,先执行: ./scripts/detector_rviz.sh" >&2
  exit 3
fi

echo "[PLAY] 回放: $BAG (只回放 /livox/lidar + /localization/odom + /livox/imu)"
echo "[PLAY] Ctrl-C 停止"
rosbag play "$BAG" --topics /livox/lidar /localization/odom /livox/imu
