#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="track_static_060"
if [[ $# -gt 0 && "${1}" != --* ]]; then
  PROFILE="$1"
  shift
fi

case "$PROFILE" in
  track_static_060|track_prbs_060|track_prbs_100|track_prbs_150|track_prbs_200|track_prbs_250|track_prbs_300) ;;
  *)
    echo "usage: $0 [track_static_060|track_prbs_060|track_prbs_100|track_prbs_150|track_prbs_200|track_prbs_250|track_prbs_300] [run_id] [--plan|--no-analyze]" >&2
    exit 2
    ;;
esac

RUN_ID=""
if [[ $# -gt 0 && "${1}" != --* ]]; then
  RUN_ID="$1"
  shift
fi

source "$ROOT/scripts/ros_env.sh"
"$ROOT/scripts/start_mpcc_hardware_stable.sh" --check-only

echo "[REAL VEHICLE] Publishes commands to /tianracer/ackermann_cmd and records a bag."
echo "[SAFETY] Keep the emergency stop ready; place the car near the raceline facing forward."
echo "[PROFILE] $PROFILE"

if [[ -n "$RUN_ID" ]]; then
  exec python3 "$ROOT/lateral_identification/run_experiment.py" \
    "$PROFILE" "$RUN_ID" "$@"
else
  exec python3 "$ROOT/lateral_identification/run_experiment.py" \
    "$PROFILE" "$@"
fi
