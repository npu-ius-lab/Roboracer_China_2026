#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FORMULATION="${1:-baseline}"
if [[ "$FORMULATION" != "baseline" && "$FORMULATION" != "mpcc" ]]; then
  echo "usage: $0 [baseline|mpcc]" >&2
  exit 2
fi

source "$ROOT/scripts/ros_env.sh"
"$ROOT/scripts/check_mpcc_topics.sh"

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
echo "WARNING: Python controller and Python hardware gate will START ENABLED."
echo "The vehicle may move immediately. Keep the remote stop ready."
exec roslaunch f1tenth_dynamic_mpcc hardware_mpcc_python.launch \
  formulation:="$FORMULATION" \
  allow_real_hardware:=true \
  start_enabled:=true \
  rviz:=true \
  telemetry_path:="$ROOT/log/hardware_python_${FORMULATION}_${STAMP}.jsonl"
