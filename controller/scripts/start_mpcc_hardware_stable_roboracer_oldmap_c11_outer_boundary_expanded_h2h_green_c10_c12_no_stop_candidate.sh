#!/usr/bin/env bash
set -euo pipefail

# Race candidate: expanded MPCC corridor + C10-C12 green perception corridor
# + graceful PASS target-loss recovery. No other controller/overtaking tuning
# differs from the guarded RoboRacer H2H candidate.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACK="raceline_smooth_c11_outer_boundary_expanded_candidate"
GREEN_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/perception_corridors/roboracer_green_c10_c12.csv"
RUNTIME="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_oldmap_h2h_green_no_stop/cpp_runtime.yaml"

export TWO_MAP_H2H_CANDIDATE_NAME="stable_roboracer_oldmap_c11_outer_boundary_expanded_h2h_green_c10_c12_no_stop"
export TWO_MAP_TRACK="$TRACK"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
export TWO_MAP_RUNTIME_CONFIG="$RUNTIME"
export TWO_MAP_RUNTIME_SHA256="fc276076dfec304d2d18a3a8babf585796887377e3488f9f754ec8afae0033d8"
export TWO_MAP_REQUIRE_GRACEFUL_TARGET_LOSS="true"
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

[[ -f "$GREEN_CSV" && -f "$RUNTIME" ]] || {
  echo "[FAIL] green no-stop candidate data/config is missing" >&2
  exit 1
}
[[ "$(sha256sum "$GREEN_CSV" | awk '{print $1}')" == \
   "ff10c67cf5390cb3151aa087bd4cb8739a04db2e54201e885a5c44b1b2c01765" ]] || {
  echo "[FAIL] green C10-C12 perception corridor hash mismatch" >&2
  exit 1
}

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_c11_stabilized_candidate.sh" "$@"
