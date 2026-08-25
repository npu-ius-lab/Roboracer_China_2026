#!/usr/bin/env bash
set -euo pipefail

# Independent H2H candidate. Control keeps the expanded yellow corridor;
# perception alone uses the green hybrid corridor (old widths from C10-C12).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACK="raceline_smooth_c11_outer_boundary_expanded_candidate"
GREEN_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/perception_corridors/roboracer_green_c10_c12.csv"

export TWO_MAP_H2H_CANDIDATE_NAME="stable_roboracer_oldmap_c11_outer_boundary_expanded_h2h_green_c10_c12"
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
export TWO_MAP_RVIZ_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/rviz/stable_roboracer_green_corridor_candidate.rviz"

export ROBORACER_OBSTACLE_RACELINE_CSV="$GREEN_CSV"
export ROBORACER_OBSTACLE_BOUNDARY_INSET_M="0.22"
export ROBORACER_OBSTACLE_MARKER_REQUIRED="true"
export ROBORACER_OBSTACLE_WAIT_S="${ROBORACER_OBSTACLE_WAIT_S:-20}"
export ROBORACER_OBSTACLE_TOPIC_WAIT_S="${ROBORACER_OBSTACLE_TOPIC_WAIT_S:-12}"
export TWO_MAP_PERCEPTION_START_RETRIES="${TWO_MAP_PERCEPTION_START_RETRIES:-3}"
export TWO_MAP_PERCEPTION_RETRY_DELAY_S="${TWO_MAP_PERCEPTION_RETRY_DELAY_S:-1}"

[[ -f "$GREEN_CSV" ]] || {
  echo "[FAIL] green C10-C12 perception corridor is missing: $GREEN_CSV" >&2
  exit 1
}
[[ "$(sha256sum "$GREEN_CSV" | awk '{print $1}')" == \
   "ff10c67cf5390cb3151aa087bd4cb8739a04db2e54201e885a5c44b1b2c01765" ]] || {
  echo "[FAIL] green C10-C12 perception corridor hash mismatch" >&2
  exit 1
}

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_c11_stabilized_candidate.sh" "$@"
