#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
"$ROOT/scripts/ensure_acados_solvers.sh"
"$ROOT/scripts/check_mpcc_topics.sh"
mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
echo "[SHADOW] Residual MPCC is solving but is NOT connected to /tianracer/ackermann_cmd."
exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc.launch \
  formulation:=mpcc \
  allow_real_hardware:=false \
  start_enabled:=true \
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP:-2.0}" \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  telemetry_path:="$ROOT/log/shadow_residual_${STAMP}.jsonl"
