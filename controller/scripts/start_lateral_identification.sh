#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-track_actuator}"
if [[ $# -gt 0 ]]; then shift; fi
RUN_ID="${1:-}"
if [[ $# -gt 0 ]]; then shift; fi

case "$PROFILE" in
  actuator|tire_low|tire_mid|tire_high|track_actuator|track_tire_low|track_tire_mid|track_tire_high|track_static_060|track_prbs_060|track_prbs_100|track_prbs_150|track_prbs_200|track_prbs_250|track_prbs_300) ;;
  *)
    echo "usage: $0 [track_static_060|track_prbs_060|track_prbs_100|track_prbs_150|track_prbs_200|track_prbs_250|track_prbs_300|legacy-track-profile] [run_id] [--plan|--no-analyze]" >&2
    exit 2
    ;;
esac

source "$ROOT/scripts/ros_env.sh"
echo "[WARNING] This script publishes REAL commands to /tianracer/ackermann_cmd."
echo "[WARNING] Place the car near the raceline, facing forward, and keep the emergency stop ready."
echo "[SAFETY] Legacy open-loop profiles are disabled; track_* profiles stop at their configured lap limit."
if [[ -n "$RUN_ID" ]]; then
  exec python3 "$ROOT/lateral_identification/run_experiment.py" "$PROFILE" "$RUN_ID" "$@"
else
  exec python3 "$ROOT/lateral_identification/run_experiment.py" "$PROFILE" "$@"
fi
