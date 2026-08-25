#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_PREFIX="${1:-prbs_highspeed_$(date +%Y%m%d_%H%M%S)}"
shift || true

PLAN_ONLY=false
NO_ANALYZE=false
for option in "$@"; do
  case "$option" in
    --plan) PLAN_ONLY=true ;;
    --no-analyze) NO_ANALYZE=true ;;
    *)
      echo "usage: $0 [run_prefix] [--plan|--no-analyze]" >&2
      exit 2
      ;;
  esac
done

profiles=(track_prbs_150 track_prbs_200 track_prbs_250 track_prbs_300)
speeds=(1.5 2.0 2.5 3.0)

echo "[REAL VEHICLE] Four-speed steering PRBS sweep"
echo "[REAL VEHICLE] Commands go to /tianracer/ackermann_cmd"
echo "[RECORD] Each stage creates a separate bag under lateral_identification/data"
echo "[SAFETY] Keep the emergency stop ready; stop the sweep if the vehicle leaves the safe corridor."
echo "[PLAN] ${speeds[*]} m/s; steering excitation decreases with speed"

for index in "${!profiles[@]}"; do
  profile="${profiles[$index]}"
  speed="${speeds[$index]}"
  run_id="${RUN_PREFIX}_${speed/./p}"
  echo "[STAGE $((index + 1))/4] ${speed} m/s -> ${run_id}"
  args=("$profile" "$run_id")
  if "$PLAN_ONLY"; then
    args+=(--plan)
  fi
  if "$NO_ANALYZE"; then
    args+=(--no-analyze)
  fi
  "$ROOT/scripts/start_steering_prbs_collection.sh" "${args[@]}"
  echo "[STAGE $((index + 1))/4] completed; verify the vehicle is stationary before continuing."
done

echo "[DONE] Four-speed steering PRBS sweep completed."
