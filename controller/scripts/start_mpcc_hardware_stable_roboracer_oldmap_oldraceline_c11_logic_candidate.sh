#!/usr/bin/env bash
set -euo pipefail

# Additive A/B candidate:
#   localization map: frozen old competition_map
#   raceline: frozen Stable RoboRacer raceline_smooth
#   control/recovery: C11-stabilized launch and publisher-matched profile
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export TWO_MAP_CANDIDATE_NAME="stable_roboracer_oldmap_oldraceline_c11_logic"
export TWO_MAP_TRACK="raceline_smooth"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
export TWO_MAP_LOCALIZATION_ID="old"
export TWO_MAP_TRACK_SHA256="1f7b00d996f67c19181094799621dcd4b684a0bf7a1e7be4f22c779ade060476"
export TWO_MAP_CONTROLLER_SHA256="c6c90509cfbc81dd15bb20ad7f33e102c0f10ed4f512722dad1669cb90ac4f3f"
export TWO_MAP_PROFILE_ACCEL_MPS2="1.5"
export TWO_MAP_PROFILE_DECEL_MPS2="2.0"
export TWO_MAP_RACE_START_CENTER_X="0.0"
export TWO_MAP_RACE_START_CENTER_Y="0.0"

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_c11_stabilized_candidate.sh" "$@"
