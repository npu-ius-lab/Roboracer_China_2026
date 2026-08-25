#!/usr/bin/env bash
set -euo pipefail

# Independent H2H candidate:
# - old localization map
# - C11 physical outer boundary expansion (raceline center/speed unchanged)
# - perception and MPCC consume the exact same track CSV
# - perception keeps obstacles only inside the yellow MPCC center corridor
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACK="raceline_smooth_c11_outer_boundary_expanded_candidate"
TRACK_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"

export TWO_MAP_H2H_CANDIDATE_NAME="stable_roboracer_oldmap_c11_outer_boundary_expanded_h2h_yellow_corridor"
export TWO_MAP_TRACK="$TRACK"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
export TWO_MAP_RUNTIME_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_oldmap_h2h_guarded/cpp_runtime.yaml"
export TWO_MAP_LOCALIZATION_ID="old"
export TWO_MAP_TRACK_SHA256="cae8c6833492457bf2a884eb4c4760960b5b49bc7eac5fa16e1cd2a98194d354"
export TWO_MAP_CONTROLLER_SHA256="c6c90509cfbc81dd15bb20ad7f33e102c0f10ed4f512722dad1669cb90ac4f3f"
export TWO_MAP_PROFILE_ACCEL_MPS2="1.5"
export TWO_MAP_PROFILE_DECEL_MPS2="2.0"
export TWO_MAP_RACE_START_CENTER_X="0.0"
export TWO_MAP_RACE_START_CENTER_Y="0.0"

# The MPCC yellow vehicle-center corridor is inset from each stored physical
# boundary by ego_width/2 + safety.track_control_margin_m + 0.05:
# 0.24/2 + 0.05 + 0.05 = 0.22 m.
export ROBORACER_OBSTACLE_RACELINE_CSV="$TRACK_CSV"
export ROBORACER_OBSTACLE_BOUNDARY_INSET_M="0.22"
export ROBORACER_OBSTACLE_WAIT_S="${ROBORACER_OBSTACLE_WAIT_S:-20}"
export ROBORACER_OBSTACLE_TOPIC_WAIT_S="${ROBORACER_OBSTACLE_TOPIC_WAIT_S:-12}"
export TWO_MAP_PERCEPTION_START_RETRIES="${TWO_MAP_PERCEPTION_START_RETRIES:-3}"
export TWO_MAP_PERCEPTION_RETRY_DELAY_S="${TWO_MAP_PERCEPTION_RETRY_DELAY_S:-1}"

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_c11_stabilized_candidate.sh" "$@"
