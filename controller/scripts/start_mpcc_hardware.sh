#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FORMULATION="${1:-mpcc}"
if [[ "$FORMULATION" != "baseline" && "$FORMULATION" != "mpcc" ]]; then
  echo "usage: $0 [baseline|mpcc]" >&2
  exit 2
fi
GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados}"
source "$ROOT/scripts/ros_env.sh"
export MPCC_GENERATED_DIR="$GENERATED_DIR"
"$ROOT/scripts/ensure_acados_solvers.sh"
"$ROOT/scripts/check_mpcc_topics.sh"

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RESIDUAL_MODEL_PATH="${MPCC_RESIDUAL_MODEL_PATH:-}"
RESIDUAL_FEATURE_SET="${MPCC_RESIDUAL_FEATURE_SET:-}"
echo "WARNING: residual MPCC preflight passed; controller and hardware gate will START ENABLED."
if [[ -n "$RESIDUAL_MODEL_PATH" ]]; then
  echo "Residual model=$RESIDUAL_MODEL_PATH feature_set=$RESIDUAL_FEATURE_SET"
else
  echo "Residual model is selected by config/controller.yaml"
fi
echo "First-test speed cap=${RESIDUAL_MPCC_SPEED_CAP:-2.0} m/s."
echo "Race cost overrides: contour=${MPCC_COST_CONTOUR_OVERRIDE:-config}, heading=${MPCC_COST_HEADING_RACE_OVERRIDE:-config}, steering_rate=${MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE:-config}."
echo "Risk policy: tiered=${MPCC_RISK_TIERING_ENABLED:-false}, near=${MPCC_NEAR_TRACK_HORIZON_OVERRIDE:-config}s, speed hold=${MPCC_PREDICTIVE_SPEED_HOLD_OVERRIDE:-config}s, recovery=${MPCC_PREDICTIVE_SPEED_RECOVERY_OVERRIDE:-config}m/s^2."
echo "Controller trial: solve=40 Hz, command publish=50 Hz, steering pure delay Td=0.0 s."
echo "Lap timer=enabled; results=$ROOT/log/laps_${FORMULATION}_${STAMP}.csv"
echo "The vehicle may move immediately. Keep the remote stop ready."
exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc.launch \
  formulation:="$FORMULATION" \
  allow_real_hardware:=true \
  start_enabled:=true \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP:-2.0}" \
  generated_dir:="$GENERATED_DIR" \
  residual_model_path:="$RESIDUAL_MODEL_PATH" \
  residual_feature_set:="$RESIDUAL_FEATURE_SET" \
  cost_contour_override:="${MPCC_COST_CONTOUR_OVERRIDE:--1.0}" \
  cost_heading_race_override:="${MPCC_COST_HEADING_RACE_OVERRIDE:--1.0}" \
  cost_steering_command_rate_override:="${MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE:--1.0}" \
  risk_tiering_enabled:="${MPCC_RISK_TIERING_ENABLED:-false}" \
  near_track_horizon_override:="${MPCC_NEAR_TRACK_HORIZON_OVERRIDE:--1.0}" \
  predictive_speed_hold_override:="${MPCC_PREDICTIVE_SPEED_HOLD_OVERRIDE:--1.0}" \
  predictive_speed_recovery_override:="${MPCC_PREDICTIVE_SPEED_RECOVERY_OVERRIDE:--1.0}" \
  pp_current_margin_override:="${MPCC_PP_CURRENT_MARGIN_OVERRIDE:--1.0}" \
  pp_confirmation_cycles_override:="${MPCC_PP_CONFIRMATION_CYCLES_OVERRIDE:--1}" \
  telemetry_path:="$ROOT/log/hardware_${FORMULATION}_${STAMP}.jsonl" \
  lap_timer:=true \
  lap_log_path:="$ROOT/log/laps_${FORMULATION}_${STAMP}.csv"
