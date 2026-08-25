#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 2 ]]; then
  echo "usage: $0 BAG1 BAG2 [BAG ...] [--output REPORT.json] [--bootstrap N]" >&2
  echo "At least two complete bags are required for grouped validation." >&2
  exit 2
fi

source "$ROOT/scripts/ros_env.sh"
exec python3 "$ROOT/lateral_identification/analyze_joint.py" "$@"
