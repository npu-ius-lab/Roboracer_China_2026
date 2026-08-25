#!/usr/bin/env bash
set -euo pipefail

# Isolated 4.5 m/s race candidate: expanded MPCC corridor + C10-C12 green perception corridor
# + graceful PASS target-loss recovery. No other controller/overtaking tuning
# differs from the guarded RoboRacer H2H candidate.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACK="raceline_smooth_c11_outer_boundary_expanded_arc_straight_4p5_candidate"
GREEN_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/perception_corridors/roboracer_green_c10_c12.csv"
RUNTIME="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_oldmap_h2h_green_no_stop_4p5/cpp_runtime.yaml"

export TWO_MAP_H2H_CANDIDATE_NAME="stable_roboracer_oldmap_c11_outer_boundary_expanded_h2h_green_c10_c12_no_stop_4p5"
export TWO_MAP_TRACK="$TRACK"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_outer_expanded_arc_straight_4p5/controller.yaml"
export TWO_MAP_RUNTIME_CONFIG="$RUNTIME"
export TWO_MAP_RUNTIME_SHA256="533f5baf42ecca7831fdf89f65fced2747b1389a425ebe3f0340f27caf717b48"
export TWO_MAP_REQUIRE_GRACEFUL_TARGET_LOSS="true"
export TWO_MAP_LOCALIZATION_ID="old"
export TWO_MAP_TRACK_SHA256="f42badd7a7d22b7371d8b6b0afcb6d96c192e6fc2e12c6a23af4c8d0fbffe906"
export TWO_MAP_CONTROLLER_SHA256="14df4868242fdd905ad2e75f34954c915f57c06134c96ddcddcf08794dba2e77"
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

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_c11_stabilized_4p5_candidate.sh" "$@"
