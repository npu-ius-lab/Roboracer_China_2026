#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_LAUNCHER="$ROOT/scripts/start_mpcc_hardware_stable_v3.sh"
CONTROLLER_CONFIG="controller_stable_v3_racelineV3.yaml"
VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
TRACK="racelinev5_candidate_min_curvature"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
SUMMARY="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/CANDIDATE_SUMMARY.json"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] racelineV5 candidate file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# First prove that the independently frozen stableV3 controller/residual/
# vehicle/racelineV3 files are unchanged, then pin only this candidate track.
"$STABLE_LAUNCHER" --check-only
check_hash "31285098a7ed40a2321d469a244cede5008d63a7b7cf185845cb1a3c255dfb9f" "$RACELINE"
check_hash "8011e1337b14355d50df98e866293d6a86d06d89b1e612b5a35a404bb6fe3336" "$SUMMARY"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] racelineV5 candidate and frozen stableV3 dependencies match"
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

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_v3_racelinev5_candidate}"
# The car keeps the historical sibling residual workspace; this local checkout
# carries the package in the same workspace. Preserve either valid layout.
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
source "$ROOT/scripts/ros_env.sh"
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
  echo "[OK] racelineV5 candidate solvers are ready; hardware was not started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

# The geometry is offline-valid but has not completed a closed-loop run.
# Start at 2 m/s unless the operator explicitly raises the runtime cap.
RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-2.0}"
mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_v3_racelinev5_candidate_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[CANDIDATE] stableV3 controller/residual + racelineV5 minimum-curvature line"
echo "[CANDIDATE] contour/heading=45.0/1.0; steering_command_rate=0.60"
echo "[CANDIDATE] profile_max_decel=3.0 m/s^2; runtime speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s"
echo "[CANDIDATE] stableV2 and stableV3 remain unchanged"
echo "[WARNING] Offline-valid candidate. Begin at 2.0 m/s and keep emergency stop ready."

LAUNCH_ARGS=(
  track:="$TRACK"
  controller_config:="$CONTROLLER_CONFIG"
  vehicle_config:="$VEHICLE_CONFIG"
  formulation:="$FORMULATION"
  allow_real_hardware:=true
  start_enabled:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP}"
  generated_dir:="$GENERATED_DIR"
  residual_model_path:="$RESIDUAL"
  residual_feature_set:=markov_v1
  telemetry_path:="$ROOT/log/hardware_stable_v3_racelinev5_candidate_${FORMULATION}_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_stable_v3_racelinev5_candidate_${FORMULATION}_${STAMP}.csv"
)
if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v3.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v3.launch "${LAUNCH_ARGS[@]}"
fi
