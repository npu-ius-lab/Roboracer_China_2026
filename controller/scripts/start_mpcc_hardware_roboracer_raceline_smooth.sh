#!/usr/bin/env bash
set -euo pipefail

# Canonical entry point for the validated RoboRacer Raceline Smooth package.
# Keep this as a thin alias so Stable RoboRacer and the frozen candidate remain
# separate, hash-gated implementations.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/start_mpcc_hardware_stable_roboracer_raceline_smooth_p1p2_local4cm_candidate.sh" "$@"
