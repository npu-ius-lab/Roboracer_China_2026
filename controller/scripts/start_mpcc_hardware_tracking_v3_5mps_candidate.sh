#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="tracking_v3_5mps_candidate"
TRACK="racelinev3_5mps_candidate"
TRACK_TAG="racelinev3_5mps_std35"
STEERING_RATE_COST="0.60"
EXPERIMENT_TAG="${TRACK_TAG}_steer060"

CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/$CANDIDATE/controller.yaml"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/$CANDIDATE/vehicle.yaml"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/$CANDIDATE/residual_h3_scale030.yaml"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
SPEED_ZONES="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/speed_zones.csv"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_track_h3_tracking_5mps_candidate.launch"
CONTROLLER_CONFIG="candidates/$CANDIDATE/controller.yaml"
VEHICLE_CONFIG="candidates/$CANDIDATE/vehicle.yaml"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] 5mps candidate file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# Candidate inputs are pinned independently. The current 4 m/s tracking
# launcher, controller, vehicle file and racelineV3 are not modified here.
check_hash "7e370f54e9d63e4f7ef82703a10f17e7d54dcba83cbcd3a89a918631faed58a8" "$CONTROLLER"
check_hash "c642b824cd41d4210137a944a0c4d10c10428135c9f2056a78e07e31b56548bc" "$VEHICLE"
check_hash "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" "$RESIDUAL"
check_hash "751aec7cfb57cde9528f9eeaa849eaf60ca167562d8bad917c7f5b905e54a1c1" "$RACELINE"
check_hash "2b6dacdadb6985b736ea22b8f9f6bf087eeb0e08354a5b41543d7f0778bc6c21" "$SPEED_ZONES"
check_hash "eaad7d61793e9713f6113f8d6dc1125a8e1174c0918d27b54d33c399f893d979" "$LAUNCH"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] isolated V3 5.0/3.5 m/s candidate hashes match"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
elif [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_tracking_v3_5mps_std35_steer060}"
# This local checkout contains the residual package in the same catkin
# workspace. The car may still provide the historical separate workspace.
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
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="$STEERING_RATE_COST"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "[OK] V3 5.0/3.5 candidate solvers are ready; no ROS node or command gate was started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-5.0}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" 'BEGIN { exit !(value > 0.0 && value <= 5.0) }'; then
  echo "[FAIL] RESIDUAL_MPCC_SPEED_CAP must be in (0, 5.0] for this candidate" >&2
  exit 2
fi

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/${EXPERIMENT_TAG}_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[TRACKING_5MPS_CANDIDATE] track=$TRACK; global=5.0; standard/tight=3.5; A3-A6=4.0; straight<=5.0"
echo "[TRACKING_5MPS_CANDIDATE] contour=45.0; heading_race=1.0; steering_command_rate=$STEERING_RATE_COST"
echo "[TRACKING_5MPS_CANDIDATE] PP recovery remains 1.20..1.50 m/s and PP hard max remains 4.0 m/s"
echo "[TRACKING_5MPS_CANDIDATE] checkpoint restart recovery is enabled with a 1.60 m/s cap"
echo "[TRACKING_5MPS_CANDIDATE] runtime speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s"
echo "[WARNING] Experimental 5 m/s ceiling is outside validated residual-model speed coverage. Keep emergency stop ready."

LAUNCH_ARGS=(
  track:="$TRACK"
  formulation:=mpcc
  allow_real_hardware:=true
  start_enabled:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  runtime_speed_cap_mps:="$RESIDUAL_MPCC_SPEED_CAP"
  generated_dir:="$GENERATED_DIR"
  residual_model_path:="$RESIDUAL"
  residual_feature_set:=markov_v1
  cost_contour_override:=45.0
  cost_heading_race_override:=1.0
  cost_steering_command_rate_override:="$STEERING_RATE_COST"
  telemetry_path:="$ROOT/log/hardware_${EXPERIMENT_TAG}_mpcc_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_${EXPERIMENT_TAG}_mpcc_${STAMP}.csv"
)

if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_track_h3_tracking_5mps_candidate.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_track_h3_tracking_5mps_candidate.launch "${LAUNCH_ARGS[@]}"
fi
