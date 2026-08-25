#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_START="$ROOT/scripts/start_mpcc_hardware_stable_roboracer.sh"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_h2h_safe_start_fast2_escape_candidate.launch"
RUNTIME_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_h2h_faststart/cpp_runtime.yaml"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor_stable_roboracer_race_start_only_candidate.py"
POLICY="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/safe_race_start.py"
FOLLOW_GATE="$ROOT/src/f1tenth_dynamic_mpcc/src/roboracer_h2h_follow_gate.cpp"
ESCAPE_GATE="$ROOT/src/f1tenth_dynamic_mpcc/src/roboracer_contact_escape_gate.cpp"
RECORDER="$ROOT/scripts/record_roboracer_h2h_safe_start_fast2_escape_bag.sh"
PERCEPTION_HELPER="$ROOT/scripts/ensure_raw_cloud_perception.sh"
RVIZ_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/rviz/stable_roboracer.rviz"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"

ACTION="${1:-mpcc}"
case "$ACTION" in
  mpcc|--check-only|--prepare-only) ;;
  *) echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2; exit 2 ;;
esac

for path in "$BASE_START" "$LAUNCH" "$RUNTIME_CONFIG" "$SUPERVISOR" \
    "$POLICY" "$FOLLOW_GATE" "$ESCAPE_GATE" "$RECORDER" "$PERCEPTION_HELPER"; do
  [[ -f "$path" ]] || { echo "[FAIL] missing H2H fast-start dependency: $path" >&2; exit 1; }
done

"$BASE_START" --check-only
grep -Fq 'type="v5_jubu_local_planner_cpp"' "$LAUNCH"
grep -Fq 'type="mpcc_node_auto_relaunch_jubu_cpp"' "$LAUNCH"
grep -Fq 'type="automatic_relaunch_supervisor_stable_roboracer_race_start_only_candidate.py"' "$LAUNCH"
grep -Fq 'type="roboracer_h2h_follow_gate_cpp"' "$LAUNCH"
grep -Fq 'type="roboracer_contact_escape_gate_cpp"' "$LAUNCH"
grep -Fq 'minimum_follow_speed_mps" value="1.0"' "$LAUNCH"
grep -Fq 'integration_guard_enabled" value="true"' "$LAUNCH"
python3 - "$LAUNCH" <<'PY'
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1]
root = ET.parse(path).getroot()
supervisor = next(
    node for node in root.findall("node")
    if node.get("name") == "automatic_relaunch_supervisor_roboracer_h2h_faststart"
)
params = {
    param.get("name"): param.get("value")
    for param in supervisor.findall("param")
}
probe = float(params["probe_speed_mps"])
recovery_minimum = float(params["recovery_minimum_speed_mps"])
race_speed = float(params["race_start_speed_mps"])
fixed_handoff = float(params["race_start_handoff_speed_mps"])
if recovery_minimum > probe:
    raise SystemExit(
        "[FAIL] H2H escape recovery_minimum_speed_mps exceeds probe_speed_mps: "
        f"{recovery_minimum} > {probe}"
    )
if race_speed < probe:
    raise SystemExit(
        "[FAIL] H2H escape race_start_speed_mps is below probe_speed_mps: "
        f"{race_speed} < {probe}"
    )
if fixed_handoff > race_speed:
    raise SystemExit(
        "[FAIL] H2H escape race_start_handoff_speed_mps exceeds "
        f"race_start_speed_mps: {fixed_handoff} > {race_speed}"
    )
print(
    "[OK] H2H escape supervisor constructor constraints: "
    f"recovery_min={recovery_minimum:.2f} probe={probe:.2f} "
    f"race={race_speed:.2f} fixed_handoff={fixed_handoff:.2f} m/s"
)
PY
if grep -R -nE 'overtake_planner\.py|state_visualizer\.py|reference_manager\.py|target_tracker\.py' \
    "$LAUNCH" "$RUNTIME_CONFIG"; then
  echo "[FAIL] Python overtaking core referenced by the candidate" >&2
  exit 1
fi

RACE_START_ENABLED="${ROBORACER_SAFE_RACE_START_ENABLED:-true}"
case "$RACE_START_ENABLED" in
  true|false) ;;
  *) echo "[FAIL] ROBORACER_SAFE_RACE_START_ENABLED must be true or false" >&2; exit 2 ;;
esac

if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] isolated RoboRacer H2H safe-start-fast2 ESCAPE candidate"
  echo "[OK] launch/carry/global handoff=safe-start-fast2; overtaking core=C++17"
  echo "[OK] FOLLOW floor=strict; contact escape=guarded brake/reverse/forward recovery"
  echo "[OK] no node or hardware publisher started"
  exit 0
fi

source "$ROOT/scripts/ros_env.sh"

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer_h2h_faststart}"
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="raceline_smooth"
export MPCC_CONTROLLER_CONFIG="candidates/stable_roboracer/controller.yaml"
export MPCC_VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
export MPCC_COST_CONTOUR_OVERRIDE="45.0"
export MPCC_COST_HEADING_RACE_OVERRIDE="1.0"
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="0.60"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$ACTION" == "--prepare-only" ]]; then
  echo "[OK] RoboRacer H2H safe-start-fast2 solver is ready"
  exit 0
fi

LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
if [[ ! -f "$LOCALIZATION_WS/devel/setup.bash" ]]; then
  echo "[FAIL] localization workspace setup is missing: $LOCALIZATION_WS/devel/setup.bash" >&2
  exit 1
fi
RESTORE_NOUNSET=false
case "$-" in *u*) RESTORE_NOUNSET=true ;; esac
set +u
source "$LOCALIZATION_WS/devel/setup.bash" --extend
if [[ "$RESTORE_NOUNSET" == true ]]; then set -u; fi
unset RESTORE_NOUNSET
export PYTHONPATH="$ROOT/src/f1tenth_dynamic_mpcc/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'from point_lio_sam_lighterbev.msg import LocalizationStatus; from f1tenth_dynamic_mpcc.safe_race_start import SafeRaceStartPolicy'

if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
fi

topic_has_publisher() {
  rostopic info "$1" 2>/dev/null \
    | sed -n '/^Publishers:/,/^Subscribers:/p' \
    | grep -q '^ \* '
}

require_live_topic() {
  local topic="$1" expected_type="$2" wait_s="$3" actual_type
  actual_type="$(rostopic type "$topic" 2>/dev/null || true)"
  if [[ "$actual_type" != "$expected_type" ]] || ! topic_has_publisher "$topic"; then
    echo "[FAIL] missing live $topic ($expected_type); actual=$actual_type" >&2
    return 1
  fi
  timeout "$wait_s" rostopic echo -n 1 --noarr "$topic" >/dev/null 2>&1 || {
    echo "[FAIL] $topic produced no fresh message within ${wait_s}s" >&2
    return 1
  }
  echo "[OK] live input $topic ($expected_type)"
}

require_live_topic /localization/odom nav_msgs/Odometry 4
require_live_topic /localization/vehicle_odom nav_msgs/Odometry 4
require_live_topic /localization/status point_lio_sam_lighterbev/LocalizationStatus 4
"$PERCEPTION_HELPER"
require_live_topic /raw_cloud_perception/obstacle_cloud sensor_msgs/PointCloud2 6

for node in /f1tenth_dynamic_mpcc /f1tenth_dynamic_mpcc_roboracer_chaoche \
    /f1tenth_dynamic_mpcc_v5_jubu /f1tenth_dynamic_mpcc_roboracer_eth_h2h \
    /f1tenth_dynamic_mpcc_roboracer_h2h_faststart \
    /automatic_relaunch_supervisor /automatic_relaunch_supervisor_roboracer_h2h_faststart \
    /roboracer_contact_escape_gate_cpp \
    /roboracer_h2h_faststart_contact_escape_gate_cpp; do
  if rosnode list 2>/dev/null | grep -Fqx "$node"; then
    echo "[FAIL] another MPCC/supervisor is registered: $node" >&2
    exit 1
  fi
done

SPEED_CAP="${H2H_SPEED_CAP_MPS:-${RESIDUAL_MPCC_SPEED_CAP:-4.0}}"
if ! awk -v value="$SPEED_CAP" 'BEGIN { exit !(value > 0.0 && value <= 4.0) }'; then
  echo "[FAIL] speed cap must be in (0, 4.0]" >&2
  exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROOT/log" "$ROOT/bags"
pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/roboracer_h2h_safe_start_fast2_escape_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  pids+=("$!")
  echo "[RECORD] H2H safe-start-fast2 ESCAPE bag -> $BAG_PATH"
fi

echo "============================================================"
echo "[ROBORACER_H2H_FASTSTART_ESCAPE] isolated reverse-enabled candidate; cap=$SPEED_CAP m/s"
echo "[ROBORACER_H2H_FASTSTART] race-start=$RACE_START_ENABLED; carry recovery=frozen RoboRacer"
echo "[ROBORACER_H2H_FASTSTART] launch PP<=2.0m/s, continuous handoff, no MPCC start notch reaches hardware"
echo "[ROBORACER_H2H_FASTSTART] overtaking=current C++ parameters; static target is pass-eligible"
echo "[ROBORACER_H2H_FASTSTART] 1.0m/s floor only for confirmed healthy FOLLOW"
echo "[ROBORACER_H2H_FASTSTART] guarded contact escape: brake -> -0.55m/s reverse -> forward recovery"
echo "[ROBORACER_H2H_FASTSTART] escape cannot arm during launch/handoff/carry or normal obstacle FOLLOW"
echo "[ROBORACER_H2H_FASTSTART] all zero/risk/recovery/PREPARE/PASS/RETURN commands pass exactly"
echo "[WARNING] Hold physical E-stop until supervisor reports WAIT_RELEASE."
echo "============================================================"

roslaunch f1tenth_dynamic_mpcc \
  hardware_mpcc_stable_roboracer_h2h_safe_start_fast2_escape_candidate.launch \
  track:=raceline_smooth \
  controller_config:=candidates/stable_roboracer/controller.yaml \
  vehicle_config:=vehicle_stable_v3_racelineV3.yaml \
  runtime_config:="$RUNTIME_CONFIG" \
  formulation:=mpcc \
  allow_real_hardware:=true \
  race_start_enabled:="$RACE_START_ENABLED" \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  rviz_config:="$RVIZ_CONFIG" \
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}" \
  lap_log_path:="$ROOT/log/laps_roboracer_h2h_safe_start_fast2_escape_${STAMP}.csv" \
  runtime_speed_cap_mps:="$SPEED_CAP" \
  generated_dir:="$GENERATED_DIR" \
  residual_model_path:="$RESIDUAL" \
  residual_feature_set:=markov_v1
