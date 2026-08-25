#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FORMULATION="${1:-mpcc}"
if [[ "$FORMULATION" != "baseline" && "$FORMULATION" != "mpcc" ]]; then
  echo "usage: $0 [baseline|mpcc]" >&2
  exit 2
fi
source "$ROOT/scripts/ros_env.sh"
"$ROOT/scripts/check_mpcc_topics.sh"

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
echo "Starting disabled DEBUG controller; it cannot publish to the chassis."
exec roslaunch f1tenth_dynamic_mpcc debug_mpcc.launch \
  formulation:="$FORMULATION" \
  start_enabled:=false \
  telemetry_path:="$ROOT/log/debug_${FORMULATION}_${STAMP}.jsonl"

