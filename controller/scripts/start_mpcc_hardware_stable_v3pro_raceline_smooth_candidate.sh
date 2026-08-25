#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="stable_v3pro"
TRACK="raceline_smooth"
CONTROLLER_CONFIG="candidates/$CANDIDATE/controller.yaml"
VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v3pro.launch"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MPCC_NODE="$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor.py"
MOTION_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] V3pro file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

check_hash_any() {
  local path="$1"
  shift
  local actual expected
  actual="$(sha256sum "$path" | awk '{print $1}')"
  for expected in "$@"; do
    [[ "$actual" == "$expected" ]] && return 0
  done
  echo "[FAIL] unsupported shared auto-relaunch node: $path" >&2
  echo "       actual=$actual" >&2
  exit 1
}

# V3pro is additive. Verify that the frozen Stable V3 inputs remain intact.
"$ROOT/scripts/start_mpcc_hardware_stable_v3.sh" --check-only
check_hash "df8f558111e09db665a70bafaf4baacf238bbc37095e116e6fc992e88498bfe2" "$CONTROLLER"
check_hash "ce8a796f28cd120d48cb3bd9a9719ad2564b909f4ad10ba11f1a6199af8b9b47" "$LAUNCH"
check_hash "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6" "$VEHICLE"
check_hash "1f7b00d996f67c19181094799621dcd4b684a0bf7a1e7be4f22c779ade060476" "$RACELINE"
check_hash "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" "$RESIDUAL"
# Accept the frozen car-side V5 node and the compatible local high-speed-gate
# extension. V3pro explicitly disables that extension in its own config.
check_hash_any "$MPCC_NODE" \
  "344bc7003ff491ff082351a2fb6979a608b9ead2df0238a0cee6c0d1b911bde0" \
  "5f173d9bee60421af3c102b7fb98745422186ade796029adff07dd05c5701440"
check_hash "f9996adfb6c59f66a2ccaf7522f40825e2c74a9e3370c7f8ffc5e4fa1a22545a" "$SUPERVISOR"
check_hash "43760fcbb6a965da5c7d8e68b7683a2a520d2129da30dcc8d4eff8db2521a8d6" "$MOTION_LOGIC"
check_hash "21217b67871accf841c1bef76f9e6c7f874526cfd1bd99c497706b989cb7b178" "$PLACEMENT_LOGIC"
check_hash "1ebd7a7a5b4b5531058206b3a64377b5bb17ff03643d97bbb5dc294d695302ec" "$RECORDER"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] V3pro raceline_smooth candidate inputs match"
  echo "[OK] V3pro behavior + conservative raceline_smooth boundaries"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
elif [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_v3pro_raceline_smooth_candidate}"
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
source "$ROOT/scripts/ros_env.sh"

# Automatic relaunch consumes LocalizationStatus from the localization build.
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
if [[ ! -f "$LOCALIZATION_WS/devel/setup.bash" ]]; then
  echo "[FAIL] localization workspace setup is missing: $LOCALIZATION_WS/devel/setup.bash" >&2
  exit 1
fi
MPCC_RESTORE_NOUNSET=false
case "$-" in
  *u*) MPCC_RESTORE_NOUNSET=true ;;
esac
set +u
source "$LOCALIZATION_WS/devel/setup.bash" --extend
if [[ "$MPCC_RESTORE_NOUNSET" == true ]]; then
  set -u
fi
unset MPCC_RESTORE_NOUNSET
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
  echo "[OK] V3pro solver is ready; hardware was not started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-4.0}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 4.0) }'; then
  echo "[FAIL] V3pro RESIDUAL_MPCC_SPEED_CAP must be in (0, 4.0]" >&2
  exit 2
fi

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_v3pro_raceline_smooth_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] V3pro automatic-relaunch bag -> $BAG_PATH"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[V3PRO_SMOOTH_CANDIDATE] Stable V3 centerline/residual/speed/braking/safety preserved"
echo "[V3PRO_SMOOTH_CANDIDATE] MPCC solve=40 Hz; command publisher/supervisor=50 Hz"
echo "[V3PRO_SMOOTH_CANDIDATE] contour/heading/rate-cost=45.0/1.0/0.60; steering rate=3.4 rad/s"
echo "[V3PRO_SMOOTH_CANDIDATE] profile decel=3.0; publisher accel/decel=1.5/2.0 m/s^2"
echo "[V3PRO_SMOOTH_CANDIDATE] runtime cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; track=raceline_smooth"
echo "[V3PRO] carry detection, stable-ground gate and global-S reprojection enabled"
echo "[V3PRO] arbitrary-position PP recovery=2.0 m/s; smooth PP-to-MPCC handoff"
echo "[V3PRO] dual-grid race start=4.0 m/s within 2.0 m of start"
echo "[V3PRO_SMOOTH_CANDIDATE] Stable V3, V3pro and Stable V5 remain unchanged"
echo "[WARNING] Hold the physical E-stop before launch or carrying the car."

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
  telemetry_path:="$ROOT/log/hardware_stable_v3pro_raceline_smooth_${STAMP}.jsonl"
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}"
  lap_log_path:="$ROOT/log/laps_stable_v3pro_raceline_smooth_${STAMP}.csv"
)

if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v3pro.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v3pro.launch "${LAUNCH_ARGS[@]}"
fi

