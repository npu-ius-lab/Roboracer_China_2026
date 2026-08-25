#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_LAUNCHER="$ROOT/scripts/start_mpcc_hardware_stable_v3_5mps_auto_relaunch_brake4_asym_60hz_candidate.sh"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v3_5mps_auto_relaunch_brake4_asym_60hz_pp_debug_candidate.launch"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_pure_pursuit_debug_supervisor.py"
RECORDER="$ROOT/scripts/record_auto_relaunch_bag.sh"
TRACK="racelinev3_5mps_arc_straight_std3_candidate"
CONTROLLER_CONFIG="candidates/stable_v3_5mps_auto_relaunch_brake4_asym_60hz_candidate/controller.yaml"
VEHICLE_CONFIG="candidates/stable_v3_5mps_arc_straight_std3_candidate/vehicle.yaml"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_v3_5mps_auto_relaunch_brake4_asym_60hz}"

if [[ ! -x "$BASE_LAUNCHER" ]]; then
  echo "[FAIL] accepted 60 Hz candidate launcher is missing: $BASE_LAUNCHER" >&2
  exit 1
fi
if [[ ! -f "$LAUNCH" || ! -f "$SUPERVISOR" ]]; then
  echo "[FAIL] isolated Pure Pursuit debug candidate files are incomplete" >&2
  exit 1
fi

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] Pure Pursuit debug candidate file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}
check_hash "5f6eaf0a4ef50dde95ca603dcb3138421c64a769286dac193de6c46046618ed9" "$SUPERVISOR"
check_hash "cede243bf5a613194237af1b4862d2fa8a588cc7976a69f2353ea30b4d54e29c" "$LAUNCH"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  "$BASE_LAUNCHER" --check-only
  python3 -m py_compile "$SUPERVISOR"
  python3 -c 'import xml.etree.ElementTree as ET,sys; ET.parse(sys.argv[1])' "$LAUNCH"
  echo "[OK] isolated continuous Pure Pursuit debug candidate is complete"
  exit 0
fi
if [[ "$ACTION" == "--prepare-only" ]]; then
  "$BASE_LAUNCHER" --prepare-only
  echo "[OK] Pure Pursuit debug candidate uses the prepared 60 Hz comparison solver"
  exit 0
fi
if [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

# Preserve all accepted solver/configuration preflight checks and prepare the
# identical 60 Hz MPCC that runs only as a telemetry reference in this mode.
"$BASE_LAUNCHER" --prepare-only

if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
source "$ROOT/scripts/ros_env.sh"
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
if [[ ! -f "$LOCALIZATION_WS/devel/setup.bash" ]]; then
  echo "[FAIL] localization workspace setup is missing: $LOCALIZATION_WS/devel/setup.bash" >&2
  exit 1
fi
RESTORE_NOUNSET=false
case "$-" in
  *u*) RESTORE_NOUNSET=true ;;
esac
set +u
source "$LOCALIZATION_WS/devel/setup.bash" --extend
if [[ "$RESTORE_NOUNSET" == true ]]; then
  set -u
fi
unset RESTORE_NOUNSET
export PYTHONPATH="$ROOT/src/f1tenth_dynamic_mpcc/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'from point_lio_sam_lighterbev.msg import LocalizationStatus' || {
  echo "[FAIL] point_lio_sam_lighterbev/LocalizationStatus is not importable" >&2
  exit 1
}

"$ROOT/scripts/check_mpcc_topics.sh"

require_localization_rate_param() {
  local name="$1"
  local expected="$2"
  local value
  value="$(rosparam get "$name" 2>/dev/null || true)"
  if ! awk -v value="$value" -v expected="$expected" \
    'BEGIN { exit !(value != "" && (value-expected < 1e-6) && (expected-value < 1e-6)) }'; then
    echo "[FAIL] localization frequency profile mismatch: $name=$value expected=$expected" >&2
    exit 1
  fi
}
require_localization_rate_param "/laserMapping/odometry/max_publish_hz" "60.0"
require_localization_rate_param "/pointlio_robust_localizer/control_output_hz" "0.0"
require_localization_rate_param "/pointlio_robust_localizer/backend_hz" "5.0"

PP_SPEED="${PURE_PURSUIT_DEBUG_SPEED_CAP:-3.0}"
PP_MIN_SPEED="${PURE_PURSUIT_DEBUG_MIN_SPEED:-1.20}"
PP_LOOKAHEAD="${PURE_PURSUIT_DEBUG_LOOKAHEAD:-0.90}"
PP_LOOKAHEAD_GAIN="${PURE_PURSUIT_DEBUG_LOOKAHEAD_SPEED_GAIN:-0.25}"
PP_LOOKAHEAD_MAX="${PURE_PURSUIT_DEBUG_LOOKAHEAD_MAX:-1.20}"
PP_SPEED_PREVIEW="${PURE_PURSUIT_DEBUG_SPEED_PREVIEW:-3.0}"
PP_LATERAL_LIMIT="${PURE_PURSUIT_DEBUG_LATERAL_ACCEL_LIMIT:-2.0}"
RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-5.0}"

if ! awk -v v="$PP_SPEED" 'BEGIN { exit !(v >= 1.2 && v <= 3.0) }'; then
  echo "[FAIL] PURE_PURSUIT_DEBUG_SPEED_CAP must be in [1.2, 3.0] m/s" >&2
  exit 2
fi
if ! awk -v lo="$PP_MIN_SPEED" -v hi="$PP_SPEED" \
  'BEGIN { exit !(lo > 0.0 && lo <= hi) }'; then
  echo "[FAIL] PURE_PURSUIT_DEBUG_MIN_SPEED must be positive and <= speed cap" >&2
  exit 2
fi
if ! awk -v lo="$PP_LOOKAHEAD" -v hi="$PP_LOOKAHEAD_MAX" \
  'BEGIN { exit !(lo >= 0.45 && hi >= lo && hi <= 2.5) }'; then
  echo "[FAIL] PP lookahead must satisfy 0.45 <= base <= max <= 2.5 m" >&2
  exit 2
fi
if ! awk -v v="$PP_LOOKAHEAD_GAIN" 'BEGIN { exit !(v >= 0.0 && v <= 1.0) }'; then
  echo "[FAIL] PURE_PURSUIT_DEBUG_LOOKAHEAD_SPEED_GAIN must be in [0, 1] s" >&2
  exit 2
fi
if ! awk -v p="$PP_SPEED_PREVIEW" -v a="$PP_LATERAL_LIMIT" \
  'BEGIN { exit !(p > 0.0 && a > 0.0) }'; then
  echo "[FAIL] PP speed preview and lateral acceleration limit must be positive" >&2
  exit 2
fi
if ! awk -v v="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(v > 0.0 && v <= 5.0) }'; then
  echo "[FAIL] RESIDUAL_MPCC_SPEED_CAP must be in (0, 5.0]" >&2
  exit 2
fi

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_v3_60hz_pp_debug_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] continuous PP debug bag -> $BAG_PATH"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[PP_DEBUG] isolated candidate; stableV3 and the accepted 60 Hz candidate are unchanged"
echo "[PP_DEBUG] hardware owner=continuous adaptive Pure Pursuit at 60 Hz"
echo "[PP_DEBUG] MPCC remains enabled only for comparison telemetry; it never reaches hardware"
echo "[PP_DEBUG] speed cap/min=${PP_SPEED}/${PP_MIN_SPEED} m/s"
echo "[PP_DEBUG] lookahead=${PP_LOOKAHEAD}+${PP_LOOKAHEAD_GAIN}*(v-vmin), max=${PP_LOOKAHEAD_MAX} m"
echo "[PP_DEBUG] speed preview=${PP_SPEED_PREVIEW} m; lateral acceleration limit=${PP_LATERAL_LIMIT} m/s^2"
echo "[PP_DEBUG] accepted carry, ground stability, global-S, race-start and relocation logic retained"
echo "[WARNING] Hold the physical E-stop before launch. Continuous PP does not hand off to MPCC."

ARGS=(
  track:="$TRACK"
  controller_config:="$CONTROLLER_CONFIG"
  vehicle_config:="$VEHICLE_CONFIG"
  formulation:=mpcc
  allow_real_hardware:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  telemetry_path:="$ROOT/log/hardware_stable_v3_60hz_pp_debug_${STAMP}.jsonl"
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}"
  lap_log_path:="$ROOT/log/laps_stable_v3_60hz_pp_debug_${STAMP}.csv"
  runtime_speed_cap_mps:="$RESIDUAL_MPCC_SPEED_CAP"
  generated_dir:="$GENERATED_DIR"
  residual_model_path:="$RESIDUAL"
  residual_feature_set:=markov_v1
  hardware_command_topic:="${AUTO_RELAUNCH_HARDWARE_COMMAND_TOPIC:-/tianracer/ackermann_cmd}"
  debug_pp_speed_mps:="$PP_SPEED"
  debug_pp_minimum_speed_mps:="$PP_MIN_SPEED"
  debug_pp_lookahead_m:="$PP_LOOKAHEAD"
  debug_pp_lookahead_speed_gain_s:="$PP_LOOKAHEAD_GAIN"
  debug_pp_lookahead_maximum_m:="$PP_LOOKAHEAD_MAX"
  debug_pp_speed_preview_m:="$PP_SPEED_PREVIEW"
  debug_pp_lateral_acceleration_limit_mps2:="$PP_LATERAL_LIMIT"
)

if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc \
    hardware_mpcc_stable_v3_5mps_auto_relaunch_brake4_asym_60hz_pp_debug_candidate.launch \
    "${ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc \
    hardware_mpcc_stable_v3_5mps_auto_relaunch_brake4_asym_60hz_pp_debug_candidate.launch \
    "${ARGS[@]}"
fi
