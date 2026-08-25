#!/usr/bin/env bash
set -euo pipefail

# Additive new-map candidate: only the longitudinal planning/publisher envelope
# changes from +1.5/-2.0 to +2.0/-2.5 m/s^2.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TWO_MAP_CANDIDATE_NAME="stable_roboracer_newmap_20260822_accel20_decel25"
export TWO_MAP_TRACK="raceline_newmap_20260822_c11_c12_smooth_candidate"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_newmap_20260822_accel20_decel25/controller.yaml"
export TWO_MAP_LOCALIZATION_ID="new"
export TWO_MAP_TRACK_SHA256="8d6442845ca159fe56fff59734cbd4895b1e04e480fe8fc9cb449ff50b3d1f73"
export TWO_MAP_CONTROLLER_SHA256="d459448ba820f1e6a7e1e918b19c7c305af8e4fb3ffc8d9cbd0e301455239a78"
export TWO_MAP_PROFILE_ACCEL_MPS2="2.0"
export TWO_MAP_PROFILE_DECEL_MPS2="2.5"
export TWO_MAP_RACE_START_CENTER_X="1.227476887"
export TWO_MAP_RACE_START_CENTER_Y="-0.015194987"
exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_c11_stabilized_candidate.sh" "$@"
