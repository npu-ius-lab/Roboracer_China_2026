#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/controller.yaml"
RESIDUAL="${STABLEV2_DEV_MODEL_PATH:-$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stablev2_dev_racelinev3_h3_scale030.yaml}"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/vehicle_new_surface_step1_candidate.yaml"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_track_h3_candidate.launch"
TRACK="racelinev3"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
VEHICLE_CONFIG="vehicle_new_surface_step1_candidate.yaml"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] stablev2_dev file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# This development profile is intentionally independent of stableV2.
check_hash "a78197b6c064c9f5f02bf6ed5d08dae0bbf231958d917a69a0ec6593d46e4f54" "$CONTROLLER"
check_hash "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" "$RESIDUAL"
check_hash "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6" "$VEHICLE"
check_hash "43eee5af91365e2f9c1e832fe86ef1649b04f351053c3ac99391398ea701d089" "$LAUNCH"
check_hash "e44fe674e4c8bbc976adc980067ee9b9806dec352516d66a32761184e79b6635" "$RACELINE"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] stablev2_dev hashes match; stableV2 is untouched"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
  FORMULATION=mpcc
elif [[ "$ACTION" == "baseline" || "$ACTION" == "mpcc" ]]; then
  FORMULATION="$ACTION"
else
  echo "usage: $0 [baseline|mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stablev2_dev}"
source "$ROOT/scripts/ros_env.sh"
export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="$TRACK"
export MPCC_VEHICLE_CONFIG="$VEHICLE_CONFIG"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "[OK] stablev2_dev solvers are ready; no ROS node or command gate was started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-2.0}"
mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stablev2_dev_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH (localization+tf+control+telemetry+imu)"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[STABLEV2_DEV] racelineV3 Markov16 residual, recursive H3 joint fit"
echo "[STABLEV2_DEV] 64 healthy formal-track segments; 11138 H3 windows"
echo "[STABLEV2_DEV] residual output scales: vx=0.0, vy=0.30, yaw=0.30"
echo "[STABLEV2_DEV] runtime speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; vehicle ceiling=4.5 m/s"
echo "[STABLEV2_DEV] development profile only; stableV2 remains unchanged"
echo "[WARNING] This run starts the real command gate enabled. Keep emergency stop ready."

LAUNCH_ARGS=(
  track:="$TRACK"
  formulation:="$FORMULATION"
  allow_real_hardware:=true
  start_enabled:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP}"
  generated_dir:="$GENERATED_DIR"
  residual_model_path:="$RESIDUAL"
  residual_feature_set:=markov_v1
  telemetry_path:="$ROOT/log/hardware_stablev2_dev_${FORMULATION}_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_stablev2_dev_${FORMULATION}_${STAMP}.csv"
)
if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_track_h3_candidate.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_track_h3_candidate.launch "${LAUNCH_ARGS[@]}"
fi
