#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-circle_r2p5_$(date +%Y%m%d_%H%M%S)}"
shift || true
source "$ROOT/scripts/ros_env.sh"

echo "[WARNING] REAL commands will be published to /tianracer/ackermann_cmd."
echo "[WARNING] Stop stable MPCC/planner first; keep the emergency stop ready."
echo "[PROFILE] fixed-radius circle R=2.0 m by default; base effective angle=9.09 deg."

exec /usr/bin/python3 "$ROOT/lateral_identification/run_circle_prbs.py" "$RUN_ID" "$@"
