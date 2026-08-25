#!/usr/bin/env bash
set -euo pipefail

RELEASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER_WS="${CONTROLLER_WS:-$RELEASE_ROOT/controller}"
ENTRY="$CONTROLLER_WS/scripts/start_mpcc_hardware_stable_roboracer_oldmap_c11_outer_boundary_expanded_react_candidate.sh"

[[ -x "$ENTRY" ]] || { echo "[FAIL] missing launcher: $ENTRY" >&2; exit 1; }
export RESIDUAL_MPCC_SPEED_CAP=4.0
export RAW_CLOUD_OBSTACLE_WS="${RAW_CLOUD_OBSTACLE_WS:-$RELEASE_ROOT/perception}"
export AUTO_RELAUNCH_LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-$RELEASE_ROOT/localization}"
if (( $# == 0 )); then set -- --hardware; fi
exec "$ENTRY" "$@"
