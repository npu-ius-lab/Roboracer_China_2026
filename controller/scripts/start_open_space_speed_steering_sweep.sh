#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
exec python3 "$ROOT/lateral_identification/run_open_space_sweep.py" "$@"
