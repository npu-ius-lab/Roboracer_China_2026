#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBORACER_ARC_STRAIGHT_VARIANT="4p5"
exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_arc_straight_candidate.sh" "$@"
