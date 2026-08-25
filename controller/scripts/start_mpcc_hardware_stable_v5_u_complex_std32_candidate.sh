#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="stable_v5_u_complex_std32_candidate"
TRACK="racelinev6_u_complex_std32_stable_v5_candidate"
CONTROLLER_CONFIG="candidates/stable_v5/controller.yaml"
VEHICLE_CONFIG="candidates/stable_v5/vehicle.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v5.launch"
TRACK_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
SPEED_ZONES="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/speed_zones.csv"
MPCC_NODE="$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor.py"
MOTION_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] Stable V5 U-complex candidate file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# Candidate track is isolated; every behavior-defining dependency remains the
# frozen Stable V5 release file and is checked against the car's release hash.
check_hash "350b532ca033aba4c7fd03a63cb7170cfd34989673705767752ab95491c685cb" "$CONTROLLER"
check_hash "fd5f9d91bc1fe967d7429cb12f42d982e8527d0d8e559f41dbefd8f2cf24366d" "$VEHICLE"
check_hash "4a8209690e9443b59cba3c23069e6abc4ee22b634069933b7b8657fcb777c989" "$TRACK_CSV"
check_hash "910cbe2cfdea9c65f0ebbbffa34c1fd2162f45ff8eeb80acdfb3959da450c7e5" "$SPEED_ZONES"
check_hash "604c80574833135f460b7057bf16be0887a5f8391151621e79475f0a96b63d45" "$LAUNCH"
check_hash "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" "$RESIDUAL"
check_hash "344bc7003ff491ff082351a2fb6979a608b9ead2df0238a0cee6c0d1b911bde0" "$MPCC_NODE"
check_hash "f9996adfb6c59f66a2ccaf7522f40825e2c74a9e3370c7f8ffc5e4fa1a22545a" "$SUPERVISOR"
check_hash "43760fcbb6a965da5c7d8e68b7683a2a520d2129da30dcc8d4eff8db2521a8d6" "$MOTION_LOGIC"
check_hash "21217b67871accf841c1bef76f9e6c7f874526cfd1bd99c497706b989cb7b178" "$PLACEMENT_LOGIC"
check_hash "1ebd7a7a5b4b5531058206b3a64377b5bb17ff03643d97bbb5dc294d695302ec" "$RECORDER"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] Stable V5 frozen dependencies and isolated U-complex track match"
  echo "[OK] Stable V5 itself was not modified"
  echo "[OK] track=$TRACK; ordinary=3.2 m/s; fast/global=5.0 m/s; profile decel=4.0 m/s^2"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
elif [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_v5_u_complex_std32_candidate}"
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

export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="$TRACK"
export MPCC_CONTROLLER_CONFIG="$CONTROLLER_CONFIG"
export MPCC_VEHICLE_CONFIG="$VEHICLE_CONFIG"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
export MPCC_COST_CONTOUR_OVERRIDE="45.0"
export MPCC_COST_HEADING_RACE_OVERRIDE="1.0"
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="0.60"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "[OK] Stable V5 U-complex candidate solvers are ready; hardware was not started"
  exit 0
fi

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

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-5.0}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 5.0) }'; then
  echo "[FAIL] RESIDUAL_MPCC_SPEED_CAP must be in (0, 5.0]" >&2
  exit 2
fi

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/${CANDIDATE}_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] Stable V5 U-complex candidate bag -> $BAG_PATH"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[CANDIDATE] Stable V5 60 Hz release + hardware-tested V6 compound U geometry"
echo "[CANDIDATE] ordinary=3.2 m/s; fast/global=${RESIDUAL_MPCC_SPEED_CAP} m/s; profile/publisher decel=4.0 m/s^2"
echo "[CANDIDATE] contour=45; heading=1.0; steering-command-rate=0.60; Stable V3 H3 residual"
echo "[CANDIDATE] Stable V5 and every previous track remain unchanged"
echo "[WARNING] Hold the physical E-stop before launch."

LAUNCH_ARGS=(
  track:="$TRACK"
  controller_config:="$CONTROLLER_CONFIG"
  vehicle_config:="$VEHICLE_CONFIG"
  formulation:=mpcc
  allow_real_hardware:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  runtime_speed_cap_mps:="$RESIDUAL_MPCC_SPEED_CAP"
  generated_dir:="$GENERATED_DIR"
  residual_model_path:="$RESIDUAL"
  residual_feature_set:=markov_v1
  hardware_command_topic:="${AUTO_RELAUNCH_HARDWARE_COMMAND_TOPIC:-/tianracer/ackermann_cmd}"
  telemetry_path:="$ROOT/log/hardware_${CANDIDATE}_${STAMP}.jsonl"
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}"
  lap_log_path:="$ROOT/log/laps_${CANDIDATE}_${STAMP}.csv"
)

if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v5.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v5.launch "${LAUNCH_ARGS[@]}"
fi
