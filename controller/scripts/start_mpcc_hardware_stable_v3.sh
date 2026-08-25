#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER_CONFIG="controller_stable_v3_racelineV3.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
MANIFEST="$ROOT/src/f1tenth_dynamic_mpcc/config/STABLE_V3_MANIFEST.json"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v3.launch"
MPCC_NODE_SOURCE="$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node.cpp"
TRACK="racelinev3"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] stable V3 profile file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# Stable V3 is an independent, byte-for-byte freeze of the base inputs used by
# start_mpcc_hardware_stablev2_dev_tracking.sh, accepted at a 4.0 m/s runtime
# cap on 2026-08-21.  The launch below reproduces that run's explicit overrides:
# contour/heading=45/1, steering-command-rate cost=0.60, profile decel=3 m/s^2,
# and startup/publisher/PP steering rate=3.4 rad/s.
# Authoritative dev fingerprints on the car were: launcher d99240ca..., launch
# b5a032c9..., controller a78197b6..., vehicle 0188d1ea..., residual 95a13797....
check_hash "a78197b6c064c9f5f02bf6ed5d08dae0bbf231958d917a69a0ec6593d46e4f54" "$CONTROLLER"
check_hash "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" "$RESIDUAL"
check_hash "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6" "$VEHICLE"
check_hash "cca9287c62fd96b90e1c97a16809fc38fe1ef31cd16b7fac42a6cb170664665d" "$MANIFEST"
check_hash "2a8ec34a2b503b8ca8eb6ce3851350ff89211f89ce343e6f7e03c522bbda6791" "$LAUNCH"
check_hash "3bc7c472ed534331c0798960520294e346a9c931a5aa1adbaaea068a753371f9" "$RACELINE"
check_hash "ee3e752f6321a1cb0ad74c7f47a71d63227bd9a646941a4ea39f32a3a9004b3d" "$MPCC_NODE_SOURCE"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] stable V3 profile hashes match; stable V2 is untouched"
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

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_v3_racelinev3}"
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
  echo "[OK] stable V3 solvers are ready; no ROS node or command gate was started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-4.0}"
mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_v3_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH (localization+tf+control+telemetry+imu)"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi

echo "[STABLE_V3] track=racelinev3; residual=H3/0.30"
echo "[STABLE_V3] contour/heading=45.0/1.0; steering_command_rate=0.60"
echo "[STABLE_V3] profile_max_decel=3.0 m/s^2; runtime speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s"
echo "[STABLE_V3] stable V2 remains unchanged"
echo "[WARNING] This profile starts the real command gate enabled. Keep emergency stop ready."

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
  telemetry_path:="$ROOT/log/hardware_stable_v3_${FORMULATION}_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_stable_v3_${FORMULATION}_${STAMP}.csv"
)
if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v3.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v3.launch "${LAUNCH_ARGS[@]}"
fi
