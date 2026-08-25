#!/usr/bin/env bash
set -euo pipefail

V3C_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AVOIDANCE_DIR="$(cd "$V3C_DIR/.." && pwd)"
ROOT="$(cd "$AVOIDANCE_DIR/.." && pwd)"
V5C_DIR="$AVOIDANCE_DIR/v5_chaoche"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
RACELINE="${V3MAX_CHAOCHE_RACELINE:-$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/raceline_smooth/raceline.csv}"
BASE_PERCEPTION="$PERCEPTION_WS/src/opponent_perception/config/perception.yaml"
ASSOCIATION_OVERRIDES="$V5C_DIR/config/perception_association_overrides.yaml"
FUSION_CONFIG="$V5C_DIR/config/static_target_fusion.yaml"
OVERTAKE_CONFIG="$V5C_DIR/config/overtake.yaml"
REFERENCE_CONFIG="$V5C_DIR/config/reference_manager.yaml"
FUSION_NODE="$V3C_DIR/scripts/static_target_fusion.py"
OVERTAKE_NODE="$V5C_DIR/scripts/overtake_planner.py"
STATE_NODE="$AVOIDANCE_DIR/scripts/avoidance_state_visualizer.py"
REFERENCE_NODE="$V3C_DIR/scripts/overtake_reference_manager.py"
MODE="${1:---foreground}"

restore_nounset=false
case "$-" in *u*) restore_nounset=true ;; esac
set +u
source /opt/ros/noetic/setup.bash
source "$PERCEPTION_WS/devel/setup.bash"
if [[ -f "$ROOT/devel/setup.bash" ]]; then
  source "$ROOT/devel/setup.bash" --extend
fi
if [[ "$restore_nounset" == true ]]; then set -u; fi
unset restore_nounset
export PYTHONPATH="$AVOIDANCE_DIR/python:$PERCEPTION_WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

check_files() {
  local path
  for path in "$RACELINE" "$BASE_PERCEPTION" "$ASSOCIATION_OVERRIDES" \
      "$FUSION_CONFIG" "$OVERTAKE_CONFIG" "$REFERENCE_CONFIG" \
      "$FUSION_NODE" "$OVERTAKE_NODE" "$STATE_NODE" "$REFERENCE_NODE" \
      "$AVOIDANCE_DIR/config/state_visualizer.yaml"; do
    if [[ ! -f "$path" ]]; then
      echo "[FAIL] missing V3 Pro Max Chaoche dependency: $path" >&2
      return 1
    fi
  done
  python3 -m py_compile "$FUSION_NODE" "$OVERTAKE_NODE" "$STATE_NODE" \
    "$REFERENCE_NODE" "$V3C_DIR/scripts/follow_speed_controller.py"
  if grep -nE 'AckermannDrive|ackermann_cmd|/tianracer/ackermann_cmd' \
      "$FUSION_NODE" "$OVERTAKE_NODE" "$STATE_NODE" "$REFERENCE_NODE"; then
    echo "[FAIL] avoidance layer contains an actuator command interface" >&2
    return 1
  fi
  python3 -c 'from opponent_perception.msg import TargetArray; from f1tenth_overtake.track import PeriodicTrack'
  echo "[OK] V3 Pro Max Chaoche avoidance is reference-only"
}

node_is_alive() {
  local node="$1"
  rosnode list 2>/dev/null | grep -Fqx "$node" || return 1
  rosnode ping -c 1 "$node" >/dev/null 2>&1 && return 0
  python3 - "$node" <<'PY'
import sys
import rosgraph
import rosnode
rosnode.cleanup_master_blacklist(
    rosgraph.Master("/roboracer_chaoche_startup_cleanup"), [sys.argv[1]])
PY
  return 1
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
rosnode list >/dev/null 2>&1 || {
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
}

nodes=(
  /opponent_perception_roboracer_chaoche
  /roboracer_chaoche_static_target_fusion
  /roboracer_chaoche_overtake_shadow
  /roboracer_chaoche_avoidance_state
  /roboracer_chaoche_reference_manager
)
for node in "${nodes[@]}"; do
  if node_is_alive "$node"; then
    echo "[FAIL] node is already running: $node" >&2
    exit 1
  fi
done
for node in /opponent_perception /opponent_perception_v5_chaoche; do
  if node_is_alive "$node"; then
    echo "[FAIL] another opponent perception node is already running: $node" >&2
    exit 1
  fi
done

pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rosparam load "$BASE_PERCEPTION" /opponent_perception_roboracer_chaoche
rosparam load "$ASSOCIATION_OVERRIDES" /opponent_perception_roboracer_chaoche
rosrun opponent_perception perception_node \
  __name:=opponent_perception_roboracer_chaoche \
  /perception/obstacle_cloud:=/roboracer_chaoche/perception/obstacle_cloud \
  /perception/targets:=/roboracer_chaoche/perception/targets \
  /perception/targets_detailed:=/roboracer_chaoche/perception/targets_dynamic \
  /perception/target_markers:=/roboracer_chaoche/perception/target_markers \
  /perception/ground_cloud:=/roboracer_chaoche/perception/ground_cloud \
  /perception/roi_cloud:=/roboracer_chaoche/perception/roi_cloud &
pids+=("$!")

rosparam load "$FUSION_CONFIG" /roboracer_chaoche_static_target_fusion
rosparam set /roboracer_chaoche_static_target_fusion/raceline_csv "$RACELINE"
rosparam set /roboracer_chaoche_static_target_fusion/slow_target_id_base 500000000
rosparam set /roboracer_chaoche_static_target_fusion/motion/minimum_static_observations 6
rosparam set /roboracer_chaoche_static_target_fusion/motion/maximum_static_speed_mps 0.08
rosparam set /roboracer_chaoche_static_target_fusion/motion/maximum_candidate_speed_mps 3.0
rosparam set /roboracer_chaoche_static_target_fusion/motion/velocity_alpha 0.30
rosparam set /roboracer_chaoche_static_target_fusion/topics/dynamic_targets /roboracer_chaoche/perception/targets_dynamic
rosparam set /roboracer_chaoche_static_target_fusion/topics/perception_markers /roboracer_chaoche/perception/target_markers
rosparam set /roboracer_chaoche_static_target_fusion/topics/output /roboracer_chaoche/perception/targets_detailed
rosparam set /roboracer_chaoche_static_target_fusion/topics/static_markers /roboracer_chaoche/perception/static_obstacle_markers
rosparam set /roboracer_chaoche_static_target_fusion/topics/status /roboracer_chaoche/perception/fusion_status
python3 "$FUSION_NODE" __name:=roboracer_chaoche_static_target_fusion &
pids+=("$!")

rosparam load "$OVERTAKE_CONFIG" /roboracer_chaoche_overtake_shadow
rosparam set /roboracer_chaoche_overtake_shadow/raceline_csv "$RACELINE"
rosparam set /roboracer_chaoche_overtake_shadow/topics/targets /roboracer_chaoche/perception/targets_detailed
rosparam set /roboracer_chaoche_overtake_shadow/topics/left_path /roboracer_chaoche/overtake/candidate_left
rosparam set /roboracer_chaoche_overtake_shadow/topics/right_path /roboracer_chaoche/overtake/candidate_right
rosparam set /roboracer_chaoche_overtake_shadow/topics/selected_path /roboracer_chaoche/overtake/selected_path
rosparam set /roboracer_chaoche_overtake_shadow/topics/markers /roboracer_chaoche/overtake/markers
rosparam set /roboracer_chaoche_overtake_shadow/topics/diagnostics /roboracer_chaoche/overtake/diagnostics
rosparam set /roboracer_chaoche_overtake_shadow/topics/valid_area /roboracer_chaoche/overtake/valid_area
python3 "$OVERTAKE_NODE" __name:=roboracer_chaoche_overtake_shadow &
pids+=("$!")

rosparam load "$AVOIDANCE_DIR/config/state_visualizer.yaml" /roboracer_chaoche_avoidance_state
rosparam set /roboracer_chaoche_avoidance_state/raceline_csv "$RACELINE"
rosparam set /roboracer_chaoche_avoidance_state/topics/targets /roboracer_chaoche/perception/targets_detailed
rosparam set /roboracer_chaoche_avoidance_state/topics/selected_path /roboracer_chaoche/overtake/selected_path
rosparam set /roboracer_chaoche_avoidance_state/topics/diagnostics /roboracer_chaoche/overtake/diagnostics
rosparam set /roboracer_chaoche_avoidance_state/topics/markers /roboracer_chaoche/avoidance/markers
rosparam set /roboracer_chaoche_avoidance_state/topics/track_markers /roboracer_chaoche/avoidance/track_markers
rosparam set /roboracer_chaoche_avoidance_state/topics/red_local_plan /roboracer_chaoche/avoidance/local_plan_red_shadow
rosparam set /roboracer_chaoche_avoidance_state/topics/status /roboracer_chaoche/avoidance/status
rosparam set /roboracer_chaoche_avoidance_state/topics/decision /roboracer_chaoche/avoidance/decision
python3 "$STATE_NODE" __name:=roboracer_chaoche_avoidance_state &
pids+=("$!")

rosparam load "$REFERENCE_CONFIG" /roboracer_chaoche_reference_manager
rosparam set /roboracer_chaoche_reference_manager/raceline_csv "$RACELINE"
rosparam set /roboracer_chaoche_reference_manager/topics/candidate /roboracer_chaoche/overtake/selected_path
rosparam set /roboracer_chaoche_reference_manager/topics/upstream_status /roboracer_chaoche/avoidance/status
rosparam set /roboracer_chaoche_reference_manager/topics/planner_diagnostics /roboracer_chaoche/overtake/diagnostics
rosparam set /roboracer_chaoche_reference_manager/topics/fusion_status /roboracer_chaoche/perception/fusion_status
rosparam set /roboracer_chaoche_reference_manager/topics/supervisor_state /automatic_relaunch_supervisor_roboracer_chaoche/state
rosparam set /roboracer_chaoche_reference_manager/topics/reference /roboracer_chaoche/local_reference
rosparam set /roboracer_chaoche_reference_manager/topics/red_path /roboracer_chaoche/local_plan_red
rosparam set /roboracer_chaoche_reference_manager/topics/speed_cap /roboracer_chaoche/local_speed_cap
rosparam set /roboracer_chaoche_reference_manager/topics/state /roboracer_chaoche/state
rosparam set /roboracer_chaoche_reference_manager/topics/active /roboracer_chaoche/reference_active
rosparam set /roboracer_chaoche_reference_manager/topics/status /roboracer_chaoche/reference_status
python3 "$REFERENCE_NODE" __name:=roboracer_chaoche_reference_manager &
pids+=("$!")

echo "[V3MAX_CHAOCHE] isolated perception -> adaptive FOLLOW -> local MPCC reference"
echo "[V3MAX_CHAOCHE] command_authority=false in the avoidance layer"
echo "[V3MAX_CHAOCHE] red path: /roboracer_chaoche/local_plan_red"
wait -n "${pids[@]}"
echo "[FAIL] one V3 Pro Max Chaoche avoidance process exited" >&2
exit 1
