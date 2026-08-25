#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_circle_unified}"
source "$ROOT/scripts/ros_env.sh"
export MPCC_GENERATED_DIR="$GENERATED_DIR"
"$ROOT/scripts/ensure_acados_solvers.sh"
"$ROOT/scripts/check_mpcc_topics.sh"

FORMULATION="${1:-mpcc}"
if [[ "$FORMULATION" != "baseline" && "$FORMULATION" != "mpcc" ]]; then
  echo "usage: $0 [baseline|mpcc]" >&2
  exit 2
fi
RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-3.0}"

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/circle_unified_${STAMP}.bag}"
  "$ROOT/scripts/record_analysis_bag.sh" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] analysis bag -> $BAG_PATH (localization+tf+control+telemetry+imu)"
  trap 'kill -INT "$RECORD_PID" 2>/dev/null || true' EXIT
fi
echo "[EXPERIMENT] Unified circular-PRBS steering candidate"
echo "[EXPERIMENT] K=0.974299, bias=-0.011499 rad, tau=0.062672 s, Td=0.105 s"
echo "[EXPERIMENT] steering rate=3.4 rad/s (startup/publisher/PP), out-of-bounds PP recovery=enabled"
echo "[EXPERIMENT] speed cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; stable config is untouched"
echo "[WARNING] This profile starts the real command gate enabled. Keep emergency stop ready."

LAUNCH_ARGS=(
  formulation:="$FORMULATION"
  allow_real_hardware:=true
  start_enabled:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP}"
  generated_dir:="$GENERATED_DIR"
  telemetry_path:="$ROOT/log/hardware_circle_unified_${FORMULATION}_${STAMP}.jsonl"
  lap_timer:=true
  lap_log_path:="$ROOT/log/laps_circle_unified_${FORMULATION}_${STAMP}.csv"
)
if [[ -n "$RECORD_PID" ]]; then
  roslaunch f1tenth_dynamic_mpcc hardware_mpcc_circle_unified_candidate.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_circle_unified_candidate.launch "${LAUNCH_ARGS[@]}"
fi
