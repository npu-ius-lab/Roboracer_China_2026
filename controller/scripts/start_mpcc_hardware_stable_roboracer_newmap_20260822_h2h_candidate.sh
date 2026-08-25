#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TWO_MAP_TRACK="raceline_newmap_20260822_c11_c12_smooth_candidate"
export TWO_MAP_H2H_CANDIDATE_NAME="stable_roboracer_newmap_20260822_h2h_guarded"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_newmap_20260822/controller.yaml"
export TWO_MAP_RUNTIME_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_newmap_h2h_guarded/cpp_runtime.yaml"
export TWO_MAP_LOCALIZATION_ID="new"
export TWO_MAP_TRACK_SHA256="8d6442845ca159fe56fff59734cbd4895b1e04e480fe8fc9cb449ff50b3d1f73"
export TWO_MAP_CONTROLLER_SHA256="c6c90509cfbc81dd15bb20ad7f33e102c0f10ed4f512722dad1669cb90ac4f3f"
export TWO_MAP_RACE_START_CENTER_X="1.227476887"
export TWO_MAP_RACE_START_CENTER_Y="-0.015194987"
exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_c11_stabilized_candidate.sh" "$@"
