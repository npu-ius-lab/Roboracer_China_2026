#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 BAG_PATH OUTPUT_JSON [MASTER_PORT=11319] [PLAY_RATE=3.0]" >&2
  exit 2
fi

BAG_PATH="$1"
OUTPUT_JSON="$2"
MASTER_PORT="${3:-11319}"
PLAY_RATE="${4:-3.0}"
AVOIDANCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MPCC_ROOT="$(cd "$AVOIDANCE_DIR/.." && pwd)"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
RACELINE="${AVOIDANCE_RACELINE:-$MPCC_ROOT/src/f1tenth_dynamic_mpcc/data/tracks/racelinev3/raceline.csv}"

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[FAIL] bag does not exist: $BAG_PATH" >&2
  exit 1
fi

RESTORE_NOUNSET=false
case "$-" in
  *u*) RESTORE_NOUNSET=true ;;
esac
set +u
source /opt/ros/noetic/setup.bash
if [[ "$RESTORE_NOUNSET" == "true" ]]; then
  set -u
fi
unset RESTORE_NOUNSET
export PYTHONPATH="$AVOIDANCE_DIR/python:$PERCEPTION_WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_MASTER_URI="http://127.0.0.1:$MASTER_PORT"
export ROS_IP="${AVOIDANCE_TEST_ROS_IP:-127.0.0.1}"
unset ROS_HOSTNAME 2>/dev/null || true

PIDS=()
cleanup() {
  local index pid
  # Stop consumers/launch files first and the isolated roscore last.
  for ((index=${#PIDS[@]}-1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

roscore -p "$MASTER_PORT" >/tmp/avoidance_test_roscore_${MASTER_PORT}.log 2>&1 &
PIDS+=("$!")
for _ in {1..50}; do
  rosnode list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

rosparam load "$AVOIDANCE_DIR/config/offline_scan_adapter.yaml" /offline_scan_target_adapter
rosparam set /offline_scan_target_adapter/raceline_csv "$RACELINE"
rosparam set /offline_scan_target_adapter/topics/odom /localization/vehicle_odom
rosparam load "$AVOIDANCE_DIR/config/overtake_hardware.yaml" /overtake_shadow
rosparam set /overtake_shadow/raceline_csv "$RACELINE"
rosparam set /overtake_shadow/topics/odom /localization/vehicle_odom
rosparam set /overtake_shadow/input/odom_timeout_s 1.0
rosparam set /overtake_shadow/input/target_timeout_s 1.0
rosparam load "$AVOIDANCE_DIR/config/state_visualizer.yaml" /avoidance_state_visualizer
rosparam set /avoidance_state_visualizer/raceline_csv "$RACELINE"
rosparam set /avoidance_state_visualizer/topics/odom /localization/vehicle_odom
rosparam set /avoidance_state_visualizer/input/odom_timeout_s 1.0
rosparam set /avoidance_state_visualizer/input/target_timeout_s 1.0

python3 "$AVOIDANCE_DIR/scripts/offline_scan_target_adapter.py" __name:=offline_scan_target_adapter &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/overtake_shadow_node.py" __name:=overtake_shadow &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/avoidance_state_visualizer.py" __name:=avoidance_state_visualizer &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/bag_test_observer.py" \
  __name:=avoidance_bag_test_observer \
  _output_path:="$OUTPUT_JSON" _bag_path:="$BAG_PATH" &
OBSERVER_PID="$!"
PIDS+=("$OBSERVER_PID")

sleep 1
echo "[bag-test] bag=$BAG_PATH rate=$PLAY_RATE master=$ROS_MASTER_URI"
rosbag play "$BAG_PATH" --clock --rate "$PLAY_RATE" \
  --topics /scan /localization/vehicle_odom /tf /tf_static
sleep 1
kill -INT "$OBSERVER_PID" 2>/dev/null || true
wait "$OBSERVER_PID" 2>/dev/null || true
echo "[bag-test] result=$OUTPUT_JSON"
