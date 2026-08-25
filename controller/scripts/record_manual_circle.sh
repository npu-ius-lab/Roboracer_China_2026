#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

RUN_ID="${1:-manual_circle_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$ROOT/lateral_identification/data"
PREFIX="$OUT_DIR/$RUN_ID"
mkdir -p "$OUT_DIR"

if [[ -e "$PREFIX.bag" || -e "$PREFIX.active" ]]; then
  echo "[FAIL] output already exists: $PREFIX" >&2
  exit 2
fi
if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] cannot contact ROS master: $ROS_MASTER_URI" >&2
  exit 2
fi
if ! timeout 4 rostopic echo -n 1 /localization/vehicle_odom >/dev/null 2>&1; then
  echo "[FAIL] /localization/vehicle_odom is not live" >&2
  exit 2
fi

touch "$PREFIX.active"
monitor_pid=""
bag_pid=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  if [[ -n "$monitor_pid" ]] && kill -0 "$monitor_pid" 2>/dev/null; then
    kill -INT "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
  fi
  rm -f "$PREFIX.active"
  echo "[STOPPED] $PREFIX.bag"
}
trap cleanup INT TERM EXIT

python3 "$ROOT/lateral_identification/manual_circle_monitor.py" &
monitor_pid=$!
sleep 0.5
rosbag record --buffsize=256 --chunksize=768 -O "$PREFIX" \
  /f1tenth_identification/manual_circle_metrics \
  /localization/vehicle_odom /localization/odom /localization/status \
  /livox/imu /tianracer/odom /tianracer/imu \
  /tianracer/ackermann_cmd /tianracer/debug_result \
  /diagnostics /tf /tf_static &
bag_pid=$!

echo "[RECORDING] $PREFIX.bag"
echo "[DRIVE] Use the physical RC smoothly; include stationary start/end and both directions."
echo "[STOP] Press Ctrl-C once."
wait "$bag_pid"
