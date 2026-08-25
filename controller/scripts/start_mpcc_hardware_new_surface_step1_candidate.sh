#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/controller.yaml"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_new_surface_step1_candidate.yaml"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/vehicle_new_surface_step1_candidate.yaml"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_new_surface_step1_candidate.launch"
TRACK="${MPCC_TRACK:-racelinev1}"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
VEHICLE_CONFIG="vehicle_new_surface_step1_candidate.yaml"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] new-surface step-1 candidate file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# Pin the candidate inputs so a later accidental edit cannot silently change
# the generated solver/runtime pair.
check_hash "a78197b6c064c9f5f02bf6ed5d08dae0bbf231958d917a69a0ec6593d46e4f54" "$CONTROLLER"
check_hash "f526e695706749f2c2bf765ba85e56684aad999ddd09c51d7b22c87be5d1794a" "$RESIDUAL"
check_hash "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6" "$VEHICLE"
check_hash "c8c698ade3c31035d3ade284fec735f8882578ef8f94083718c15ad65ff5fd23" "$LAUNCH"
check_hash "90cbbd3bba23d23cdccd57f88cb2ba97b69ca3a33064bde2f80e2cc57dd706b1" "$RACELINE"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] new-surface step-1 candidate hashes match"
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

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_new_surface_step1_candidate}"
source "$ROOT/scripts/ros_env.sh"
export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="$TRACK"
export MPCC_VEHICLE_CONFIG="$VEHICLE_CONFIG"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "[OK] candidate solvers are ready; no ROS node or command gate was started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-3.0}"
mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/new_surface_step1_candidate_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH (localization+tf+control+telemetry+imu)"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[CANDIDATE] 2026-08-20 surface, one-step robust-ridge residual from 12 R=2.5 m bags"
echo "[CANDIDATE] residual output scales: vx=0.0, vy=0.25, yaw=0.25"
echo "[CANDIDATE] nominal: Cf=73.84, Cr=103.67, K=0.965830, bias=-0.011769 rad, tau=0.062672 s, Td=0.105 s"
echo "[CANDIDATE] runtime speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; vehicle ceiling=4.5 m/s"
echo "[CANDIDATE] track=$TRACK; this version is NOT stableV3"
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
  telemetry_path:="$ROOT/log/hardware_new_surface_step1_candidate_${FORMULATION}_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_new_surface_step1_candidate_${FORMULATION}_${STAMP}.csv"
)
if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_new_surface_step1_candidate.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_new_surface_step1_candidate.launch "${LAUNCH_ARGS[@]}"
fi
