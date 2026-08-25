#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER_CONFIG="controller_stable_v2_racelineV2.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_markov_v1.yaml"
VEHICLE_CONFIG="vehicle_stable_v2_racelineV2.yaml"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
TRACK="racelineV2_smooth"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] stable V2 profile file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# Stable V2 dynamics from the verified 2026-08-17 candidate run:
# unified steering model + 3.4 rad/s + OOB PP recovery + tire Cf=70/Cr=110,
# racelineV2 changes only the speed ceilings and consumes per-node limits from
# the pinned competition raceline (two bends at 80%, named fast zones at 6 m/s).
check_hash "2ee25e57d9428f05075be653e52a42ef863b486a66741201d30cfd18bdb3def0" "$CONTROLLER"
check_hash "16a18c94442f90415cf8bd0cea81b7c7887e0624adfa1f3e80a26e44246fe3fc" "$RESIDUAL"
check_hash "7a191cc917cdd689a68ed76edfa7ba7e7ce4860329b7d709efafc4cac6f270bf" "$VEHICLE"
check_hash "1c50f60a40f4d214dee4acb899f450729ce0b6d53d9f9db8241cf83d1ea33915" "$RACELINE"

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] Stable V2 raceline-smooth candidate hashes match; original Stable V2 is untouched"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
  FORMULATION="mpcc"
else
  FORMULATION="$ACTION"
fi
if [[ "$FORMULATION" != "baseline" && "$FORMULATION" != "mpcc" ]]; then
  echo "usage: $0 [baseline|mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_v2_racelineV2_smooth_candidate}"
source "$ROOT/scripts/ros_env.sh"
export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="$TRACK"
export MPCC_CONTROLLER_CONFIG="$CONTROLLER_CONFIG"
export MPCC_VEHICLE_CONFIG="$VEHICLE_CONFIG"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
"$ROOT/scripts/ensure_acados_solvers.sh"
if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "[OK] Stable V2 raceline-smooth candidate solvers are ready; hardware was not started"
  exit 0
fi
"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-6.0}"
mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_v2_raceline_smooth_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH (localization+tf+control+telemetry+imu)"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi
echo "[STABLE_V2] Unified steering model: K=0.974299, bias=-0.011499 rad, tau=0.062672 s, Td=0.105 s"
echo "[STABLE_V2] tire Cf=70.0 N/rad Cr=110.0 N/rad (conservative identification, Kus≈+0.0002)"
echo "[STABLE_V2] steering rate=3.4 rad/s (startup/publisher/PP), out-of-bounds PP recovery=enabled"
echo "[STABLE_V2] speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s"
echo "[STABLE_V2] track=$TRACK"
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
  residual_feature_set:="markov_v1"
  telemetry_path:="$ROOT/log/hardware_stable_v2_raceline_smooth_${FORMULATION}_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_stable_v2_raceline_smooth_${FORMULATION}_${STAMP}.csv"
)
if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v2.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v2.launch "${LAUNCH_ARGS[@]}"
fi
