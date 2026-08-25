#!/usr/bin/env bash
# Offline V5 Chaoche perception/planning regression. No controller is started.
set -eo pipefail

if [[ $# -lt 4 || $# -gt 5 ]]; then
  echo "usage: $0 BAG START_SECONDS DURATION_SECONDS OUTPUT_BAG [RATE]" >&2
  exit 2
fi

INPUT_BAG="$1"
START_SECONDS="$2"
DURATION_SECONDS="$3"
OUTPUT_BAG="$4"
RATE="${5:-1.0}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
ROS_PORT="${V5_CHAOCHE_REPLAY_ROS_PORT:-11321}"
RACELINE="$WS/src/f1tenth_dynamic_mpcc/data/tracks/racelinev3_stable_v5_fast5_std32/raceline.csv"

if [[ ! -f "$INPUT_BAG" || -e "$OUTPUT_BAG" || -e "$OUTPUT_BAG.active" ]]; then
  echo "[FAIL] input missing or output already exists" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "$PERCEPTION_WS/devel/setup.bash"
source "$WS/devel/setup.bash" --extend
set -u
export PYTHONPATH="$WS/avoidance/python:$PERCEPTION_WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"
export ROS_IP=127.0.0.1

run_tag="v5c_replay_$$"
pids=()
recorder_pid=""
cleanup() {
  if [[ -n "$recorder_pid" ]]; then
    kill -INT "$recorder_pid" 2>/dev/null || true
  fi
  local pid
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

roscore -p "$ROS_PORT" >"/tmp/${run_tag}_roscore.log" 2>&1 &
pids+=("$!")
for unused in $(seq 1 50); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

rosparam load "$WS/avoidance/v5_chaoche/config/static_target_fusion.yaml" /v5_chaoche_static_target_fusion
rosparam set /v5_chaoche_static_target_fusion/raceline_csv "$RACELINE"
python3 "$WS/avoidance/v5_chaoche/scripts/static_target_fusion.py" \
  __name:=v5_chaoche_static_target_fusion >"/tmp/${run_tag}_fusion.log" 2>&1 &
pids+=("$!")

rosparam load "$WS/avoidance/v5_chaoche/config/overtake.yaml" /v5_chaoche_overtake_shadow
rosparam set /v5_chaoche_overtake_shadow/raceline_csv "$RACELINE"
rosparam set /v5_chaoche_overtake_shadow/topics/targets /v5_chaoche/perception/targets_detailed
rosparam set /v5_chaoche_overtake_shadow/topics/left_path /v5_chaoche/overtake/candidate_left
rosparam set /v5_chaoche_overtake_shadow/topics/right_path /v5_chaoche/overtake/candidate_right
rosparam set /v5_chaoche_overtake_shadow/topics/selected_path /v5_chaoche/overtake/selected_path
rosparam set /v5_chaoche_overtake_shadow/topics/markers /v5_chaoche/overtake/markers
rosparam set /v5_chaoche_overtake_shadow/topics/diagnostics /v5_chaoche/overtake/diagnostics
rosparam set /v5_chaoche_overtake_shadow/topics/valid_area /v5_chaoche/overtake/valid_area
python3 "$WS/avoidance/v5_chaoche/scripts/overtake_planner.py" \
  __name:=v5_chaoche_overtake_shadow >"/tmp/${run_tag}_planner.log" 2>&1 &
pids+=("$!")

rosparam load "$WS/avoidance/config/state_visualizer.yaml" /v5_chaoche_avoidance_state
rosparam set /v5_chaoche_avoidance_state/raceline_csv "$RACELINE"
rosparam set /v5_chaoche_avoidance_state/topics/targets /v5_chaoche/perception/targets_detailed
rosparam set /v5_chaoche_avoidance_state/topics/selected_path /v5_chaoche/overtake/selected_path
rosparam set /v5_chaoche_avoidance_state/topics/diagnostics /v5_chaoche/overtake/diagnostics
rosparam set /v5_chaoche_avoidance_state/topics/status /v5_chaoche/avoidance/status
rosparam set /v5_chaoche_avoidance_state/topics/decision /v5_chaoche/avoidance/decision
python3 "$WS/avoidance/scripts/avoidance_state_visualizer.py" \
  __name:=v5_chaoche_avoidance_state >"/tmp/${run_tag}_state.log" 2>&1 &
pids+=("$!")

rosparam load "$WS/avoidance/v5_chaoche/config/reference_manager.yaml" /v5_chaoche_reference_manager
rosparam set /v5_chaoche_reference_manager/raceline_csv "$RACELINE"
python3 "$WS/avoidance/v5_chaoche/scripts/overtake_reference_manager.py" \
  __name:=v5_chaoche_reference_manager >"/tmp/${run_tag}_manager.log" 2>&1 &
pids+=("$!")

for unused in $(seq 1 50); do
  [[ "$(rosnode list 2>/dev/null | grep -c v5_chaoche || true)" -ge 4 ]] && break
  sleep 0.1
done

rosbag record --lz4 -O "$OUTPUT_BAG" \
  /v5_chaoche/perception/targets_detailed \
  /v5_chaoche/perception/fusion_status \
  /v5_chaoche/perception/static_obstacle_markers \
  /v5_chaoche/overtake/diagnostics \
  /v5_chaoche/overtake/selected_path \
  /v5_chaoche/avoidance/status \
  /v5_chaoche/local_reference \
  /v5_chaoche/local_speed_cap \
  /v5_chaoche/state \
  /v5_chaoche/reference_status >"/tmp/${run_tag}_record.log" 2>&1 &
recorder_pid="$!"
sleep 0.5

rosbag play --quiet --clock --rate "$RATE" -s "$START_SECONDS" -u "$DURATION_SECONDS" \
  "$INPUT_BAG" --topics \
  /localization/odom \
  /v5_chaoche/perception/targets_detailed \
  /v5_chaoche/perception/target_markers \
  /automatic_relaunch_supervisor_v5_chaoche/state \
  /v5_chaoche/perception/targets_detailed:=/v5_chaoche/perception/targets_dynamic

sleep 0.3
kill -INT "$recorder_pid" 2>/dev/null || true
for unused in $(seq 1 30); do
  kill -0 "$recorder_pid" 2>/dev/null || break
  sleep 0.1
done
kill -TERM "$recorder_pid" 2>/dev/null || true
wait "$recorder_pid" 2>/dev/null || true
recorder_pid=""
echo "[OK] offline planning regression: $OUTPUT_BAG"
