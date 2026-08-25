#!/usr/bin/env bash
set -euo pipefail

# Additive geometry-only candidate based on the currently used old-map RoboRacer:
#   localization map: frozen old competition_map
#   raceline/control logic: unchanged from oldmap + C11 stabilized candidate
#   only C11 rear/outer physical boundary (w_tr_left_m) expands smoothly by 0.35 m
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export TWO_MAP_CANDIDATE_NAME="stable_roboracer_oldmap_c11_outer_boundary_expanded"
export TWO_MAP_TRACK="raceline_smooth_c11_outer_boundary_expanded_candidate"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
export TWO_MAP_LOCALIZATION_ID="old"
export TWO_MAP_TRACK_SHA256="cae8c6833492457bf2a884eb4c4760960b5b49bc7eac5fa16e1cd2a98194d354"
export TWO_MAP_CONTROLLER_SHA256="c6c90509cfbc81dd15bb20ad7f33e102c0f10ed4f512722dad1669cb90ac4f3f"
export TWO_MAP_PROFILE_ACCEL_MPS2="1.5"
export TWO_MAP_PROFILE_DECEL_MPS2="2.0"
export TWO_MAP_RACE_START_CENTER_X="0.0"
export TWO_MAP_RACE_START_CENTER_Y="0.0"

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_c11_stabilized_candidate.sh" "$@"
