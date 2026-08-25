#!/usr/bin/env bash
set -euo pipefail

V5C_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AVOIDANCE_DIR="$(cd "$V5C_DIR/.." && pwd)"
MPCC_ROOT="$(cd "$AVOIDANCE_DIR/.." && pwd)"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
RACELINE="${V5_CHAOCHE_RACELINE:-$MPCC_ROOT/src/f1tenth_dynamic_mpcc/data/tracks/racelinev3_stable_v5_fast5_std32/raceline.csv}"
BASE_PERCEPTION="$PERCEPTION_WS/src/opponent_perception/config/perception.yaml"
ASSOCIATION_OVERRIDES="$V5C_DIR/config/perception_association_overrides.yaml"
REFERENCE_CONFIG="$V5C_DIR/config/reference_manager.yaml"
REFERENCE_MANAGER="$V5C_DIR/scripts/overtake_reference_manager.py"
OVERTAKE_CONFIG="$V5C_DIR/config/overtake.yaml"
OVERTAKE_PLANNER="$V5C_DIR/scripts/overtake_planner.py"
FUSION_CONFIG="$V5C_DIR/config/static_target_fusion.yaml"
FUSION_NODE="$V5C_DIR/scripts/static_target_fusion.py"
MODE="${1:---foreground}"

restore_nounset=false
case "$-" in
  *u*) restore_nounset=true ;;
esac
set +u
source /opt/ros/noetic/setup.bash
source "$PERCEPTION_WS/devel/setup.bash"
if [[ -f "$MPCC_ROOT/devel/setup.bash" ]]; then
  source "$MPCC_ROOT/devel/setup.bash" --extend
fi
if [[ "$restore_nounset" == true ]]; then
  set -u
fi
unset restore_nounset
export PYTHONPATH="$AVOIDANCE_DIR/python:$PERCEPTION_WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

check_files() {
  local path
  for path in "$RACELINE" "$BASE_PERCEPTION" "$ASSOCIATION_OVERRIDES" \
      "$REFERENCE_CONFIG" "$REFERENCE_MANAGER" \
      "$OVERTAKE_CONFIG" "$OVERTAKE_PLANNER" \
      "$FUSION_CONFIG" "$FUSION_NODE" \
      "$AVOIDANCE_DIR/scripts/avoidance_state_visualizer.py" \
      "$AVOIDANCE_DIR/config/state_visualizer.yaml"; do
    if [[ ! -f "$path" ]]; then
      echo "[FAIL] missing V5 Chaoche avoidance file: $path" >&2
      return 1
    fi
  done
  python3 -m py_compile "$REFERENCE_MANAGER" "$OVERTAKE_PLANNER" "$FUSION_NODE" \
    "$V5C_DIR/scripts/follow_speed_controller.py"
  if grep -nE 'AckermannDrive|ackermann_cmd|/tianracer/ackermann_cmd' \
      "$REFERENCE_MANAGER" "$OVERTAKE_PLANNER" "$FUSION_NODE"; then
    echo "[FAIL] V5 Chaoche avoidance contains a command interface" >&2
    return 1
  fi
  python3 -c 'from opponent_perception.msg import TargetArray; from f1tenth_overtake.track import PeriodicTrack; print("[OK] V5 Chaoche perception and track interfaces import")'
  echo "[OK] V5 Chaoche avoidance is reference-only; command_authority=false"
}

node_is_alive() {
  local node="$1"
  if ! rosnode list 2>/dev/null | grep -Fqx "$node"; then
    return 1
  fi
  if rosnode ping -c 1 "$node" >/dev/null 2>&1; then
    return 0
  fi

  # rosnode list can retain a dead node after its parent launcher is
  # interrupted. Remove only this confirmed-unreachable registration; never
  # run the broad interactive `rosnode cleanup` against unrelated nodes.
  echo "[WARN] removing stale ROS registration: $node" >&2
  if python3 - "$node" <<'PY'
import sys

import rosgraph
import rosnode

master = rosgraph.Master("/v5_chaoche_startup_cleanup")
rosnode.cleanup_master_blacklist(master, [sys.argv[1]])
PY
  then
    return 1
  fi

  echo "[FAIL] could not remove stale ROS registration: $node" >&2
  return 0
}

if [[ "$MODE" == "--check-only" ]]; then
  check_files
  exit 0
fi
if [[ "$MODE" != "--foreground" ]]; then
  echo "usage: $0 [--check-only|--foreground]" >&2
  exit 2
fi

check_files
if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
fi
for node in /opponent_perception_v5_chaoche /v5_chaoche_static_target_fusion \
    /v5_chaoche_overtake_shadow \
    /v5_chaoche_avoidance_state /v5_chaoche_reference_manager; do
  if node_is_alive "$node"; then
    echo "[FAIL] V5 Chaoche node is already running: $node" >&2
    exit 1
  fi
done
if node_is_alive '/opponent_perception'; then
  echo "[FAIL] original /opponent_perception is already running" >&2
  echo "       V5 Chaoche will not start a second MID360 perception process or reuse untuned parameters" >&2
  exit 1
fi

pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rosparam load "$BASE_PERCEPTION" /opponent_perception_v5_chaoche
rosparam load "$ASSOCIATION_OVERRIDES" /opponent_perception_v5_chaoche
rosrun opponent_perception perception_node \
  __name:=opponent_perception_v5_chaoche \
  /perception/obstacle_cloud:=/v5_chaoche/perception/obstacle_cloud \
  /perception/targets:=/v5_chaoche/perception/targets \
  /perception/targets_detailed:=/v5_chaoche/perception/targets_dynamic \
  /perception/target_markers:=/v5_chaoche/perception/target_markers \
  /perception/ground_cloud:=/v5_chaoche/perception/ground_cloud \
  /perception/roi_cloud:=/v5_chaoche/perception/roi_cloud &
pids+=("$!")
echo "[V5_CHAOCHE] isolated MID360 perception started with bag-validated association gate=0.50"

rosparam load "$FUSION_CONFIG" /v5_chaoche_static_target_fusion
rosparam set /v5_chaoche_static_target_fusion/raceline_csv "$RACELINE"
python3 "$FUSION_NODE" __name:=v5_chaoche_static_target_fusion &
pids+=("$!")
echo "[V5_CHAOCHE] dynamic targets + confirmed compact static obstacles -> fused targets"

rosparam load "$OVERTAKE_CONFIG" /v5_chaoche_overtake_shadow
rosparam set /v5_chaoche_overtake_shadow/raceline_csv "$RACELINE"
rosparam set /v5_chaoche_overtake_shadow/topics/targets /v5_chaoche/perception/targets_detailed
rosparam set /v5_chaoche_overtake_shadow/topics/left_path /v5_chaoche/overtake/candidate_left
rosparam set /v5_chaoche_overtake_shadow/topics/right_path /v5_chaoche/overtake/candidate_right
rosparam set /v5_chaoche_overtake_shadow/topics/selected_path /v5_chaoche/overtake/selected_path
rosparam set /v5_chaoche_overtake_shadow/topics/markers /v5_chaoche/overtake/markers
rosparam set /v5_chaoche_overtake_shadow/topics/diagnostics /v5_chaoche/overtake/diagnostics
rosparam set /v5_chaoche_overtake_shadow/topics/valid_area /v5_chaoche/overtake/valid_area
python3 "$OVERTAKE_PLANNER" \
  __name:=v5_chaoche_overtake_shadow &
pids+=("$!")

rosparam load "$AVOIDANCE_DIR/config/state_visualizer.yaml" /v5_chaoche_avoidance_state
rosparam set /v5_chaoche_avoidance_state/raceline_csv "$RACELINE"
rosparam set /v5_chaoche_avoidance_state/topics/targets /v5_chaoche/perception/targets_detailed
rosparam set /v5_chaoche_avoidance_state/topics/selected_path /v5_chaoche/overtake/selected_path
rosparam set /v5_chaoche_avoidance_state/topics/diagnostics /v5_chaoche/overtake/diagnostics
rosparam set /v5_chaoche_avoidance_state/topics/markers /v5_chaoche/avoidance/markers
rosparam set /v5_chaoche_avoidance_state/topics/track_markers /v5_chaoche/avoidance/track_markers
rosparam set /v5_chaoche_avoidance_state/topics/red_local_plan /v5_chaoche/avoidance/local_plan_red_shadow
rosparam set /v5_chaoche_avoidance_state/topics/status /v5_chaoche/avoidance/status
rosparam set /v5_chaoche_avoidance_state/topics/decision /v5_chaoche/avoidance/decision
python3 "$AVOIDANCE_DIR/scripts/avoidance_state_visualizer.py" \
  __name:=v5_chaoche_avoidance_state &
pids+=("$!")

rosparam load "$REFERENCE_CONFIG" /v5_chaoche_reference_manager
rosparam set /v5_chaoche_reference_manager/raceline_csv "$RACELINE"
python3 "$REFERENCE_MANAGER" __name:=v5_chaoche_reference_manager &
pids+=("$!")

echo "[V5_CHAOCHE] adaptive FOLLOW -> authorized local planner -> locked red MPCC reference"
echo "[V5_CHAOCHE] static compact obstacles are fused after persistence + track-boundary rejection"
echo "[V5_CHAOCHE] return to global requires path end + |ey|<=0.12m + obstacle clear 0.60s"
echo "[V5_CHAOCHE] outputs: /v5_chaoche/local_reference /v5_chaoche/local_plan_red /v5_chaoche/state"
echo "[V5_CHAOCHE] no command publisher is present in this avoidance launcher"

wait -n "${pids[@]}"
echo "[FAIL] one V5 Chaoche avoidance process exited" >&2
exit 1
