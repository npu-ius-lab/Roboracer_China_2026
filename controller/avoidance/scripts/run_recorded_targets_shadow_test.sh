#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 BAG_PATH OUTPUT_JSON [MASTER_PORT=11321] [PLAY_RATE=3.0]" >&2
  exit 2
fi

BAG_PATH="$1"
OUTPUT_JSON="$2"
MASTER_PORT="${3:-11321}"
PLAY_RATE="${4:-3.0}"
AVOIDANCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MPCC_ROOT="$(cd "$AVOIDANCE_DIR/.." && pwd)"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
RACELINE="${AVOIDANCE_RACELINE:-$MPCC_ROOT/src/f1tenth_dynamic_mpcc/data/tracks/racelinev3/raceline.csv}"
ROS_LOG_DIR="${AVOIDANCE_TEST_LOG_DIR:-/tmp/avoidance_recorded_targets_${MASTER_PORT}}"

for path in "$BAG_PATH" "$RACELINE"; do
  if [[ ! -f "$path" ]]; then
    echo "[FAIL] required file does not exist: $path" >&2
    exit 1
  fi
done

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
export ROS_LOG_DIR
unset ROS_HOSTNAME 2>/dev/null || true
mkdir -p "$ROS_LOG_DIR" "$(dirname "$OUTPUT_JSON")"

PIDS=()
cleanup() {
  local index pid
  for ((index=${#PIDS[@]}-1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

roscore -p "$MASTER_PORT" >"$ROS_LOG_DIR/roscore.log" 2>&1 &
PIDS+=("$!")
for _ in {1..100}; do
  rosnode list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

rosparam load "$AVOIDANCE_DIR/config/overtake_hardware.yaml" /overtake_shadow
rosparam set /overtake_shadow/raceline_csv "$RACELINE"
rosparam load "$AVOIDANCE_DIR/config/state_visualizer.yaml" /avoidance_state_visualizer
rosparam set /avoidance_state_visualizer/raceline_csv "$RACELINE"

python3 "$AVOIDANCE_DIR/scripts/overtake_shadow_node.py" \
  __name:=overtake_shadow >"$ROS_LOG_DIR/overtake_shadow.log" 2>&1 &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/avoidance_state_visualizer.py" \
  __name:=avoidance_state_visualizer >"$ROS_LOG_DIR/visualizer.log" 2>&1 &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/bag_test_observer.py" \
  __name:=avoidance_bag_test_observer \
  _output_path:="$OUTPUT_JSON" _bag_path:="$BAG_PATH" &
OBSERVER_PID="$!"
PIDS+=("$OBSERVER_PID")

sleep 1
echo "[recorded-target-test] bag=$BAG_PATH rate=$PLAY_RATE master=$ROS_MASTER_URI"
rosbag play "$BAG_PATH" --quiet --clock --rate "$PLAY_RATE" \
  --topics /localization/odom /perception/targets_detailed /tf /tf_static
sleep 1
kill -TERM "$OBSERVER_PID" 2>/dev/null || true
wait "$OBSERVER_PID" 2>/dev/null || true
echo "[recorded-target-test] result=$OUTPUT_JSON logs=$ROS_LOG_DIR"
