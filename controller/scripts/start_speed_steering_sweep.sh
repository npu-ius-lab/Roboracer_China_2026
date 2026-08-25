#!/usr/bin/env bash
# One safe, track-guided lap per requested speed cap.  The runner records a
# separate bag per lap and always sends zero command before advancing.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

RUN_PREFIX="${1:-speed_steer_sweep_$(date +%Y%m%d_%H%M%S)}"
# This is the steering perturbation about the raceline-following command. It
# may be overridden without editing code: STEER_EXCITATION_RAD=0.06 <script>.
STEER_EXCITATION_RAD="${STEER_EXCITATION_RAD:-0.040}"
SPEEDS=(0.5 1.0 1.5 2.0 2.5 3.0 3.5)

echo "[WARNING] This publishes REAL commands to /tianracer/ackermann_cmd."
echo "[PLAN] Track-guided one-lap bags, speed caps: ${SPEEDS[*]} m/s"
echo "[PLAN] Steering excitation: +/-${STEER_EXCITATION_RAD} rad, only on safe straight sections."
echo "[SAFETY] Curvature preview, body-margin prediction and stop-on-abort remain enabled."

for speed in "${SPEEDS[@]}"; do
  speed_tag="${speed/./p}"
  run_id="${RUN_PREFIX}_v${speed_tag}"
  echo "[RUN] ${run_id}: cap=${speed} m/s"
  python3 "$ROOT/lateral_identification/run_experiment.py" track_tire_high "$run_id" \
    --speed-cap "$speed" --excitation-rad "$STEER_EXCITATION_RAD"
  echo "[DONE] ${run_id}; waiting 4 s while the vehicle remains at zero command."
  sleep 4
done

echo "[COMPLETE] Bags are in $ROOT/lateral_identification/data"
