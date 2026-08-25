#!/usr/bin/env bash
set -euo pipefail

RELEASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER_WS="${CONTROLLER_WS:-$RELEASE_ROOT/controller}"
ENTRY="$CONTROLLER_WS/scripts/start_mpcc_hardware_stable_roboracer.sh"

[[ -x "$ENTRY" ]] || { echo "[FAIL] missing launcher: $ENTRY" >&2; exit 1; }
export RESIDUAL_MPCC_SPEED_CAP=4.0
if (( $# == 0 )); then set -- mpcc; fi
exec "$ENTRY" "$@"
