#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo "usage: $0 BAG [BAG ...] [--actuator-result REPORT.json] [--output REPORT.json]" >&2
  exit 2
fi
source "$ROOT/scripts/ros_env.sh"
exec python3 "$ROOT/lateral_identification/analyze_bag.py" "$@"
