#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-fixed_circle_$(date +%Y%m%d_%H%M%S)}"
shift || true
source "$ROOT/scripts/ros_env.sh"
echo "[WARNING] This test publishes 0.50 m/s and 30 degrees to /tianracer/ackermann_cmd."
echo "[WARNING] Confirm the open area is clear and keep the RC emergency stop ready."
exec python3 "$ROOT/lateral_identification/run_fixed_circle.py" "$RUN_ID" "$@"
