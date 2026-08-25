#!/usr/bin/env bash
set -euo pipefail

# H2H companion to the old-map/old-raceline C11-logic A/B candidate.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export TWO_MAP_H2H_CANDIDATE_NAME="stable_roboracer_oldmap_oldraceline_c11_logic_h2h_guarded"
export TWO_MAP_TRACK="raceline_smooth"
export TWO_MAP_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
export TWO_MAP_RUNTIME_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_oldmap_h2h_guarded/cpp_runtime.yaml"
export TWO_MAP_LOCALIZATION_ID="old"
export TWO_MAP_TRACK_SHA256="1f7b00d996f67c19181094799621dcd4b684a0bf7a1e7be4f22c779ade060476"
export TWO_MAP_CONTROLLER_SHA256="c6c90509cfbc81dd15bb20ad7f33e102c0f10ed4f512722dad1669cb90ac4f3f"
export TWO_MAP_PROFILE_ACCEL_MPS2="1.5"
export TWO_MAP_PROFILE_DECEL_MPS2="2.0"
export TWO_MAP_RACE_START_CENTER_X="0.0"
export TWO_MAP_RACE_START_CENTER_Y="0.0"
# Keep this legacy A/B candidate reproducible after the yellow-corridor
# perception candidate has run.  The perception node is intentionally
# process-group isolated and may still be alive between MPCC launches.
export ROBORACER_OBSTACLE_RACELINE_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/raceline_smooth/raceline.csv"
export ROBORACER_OBSTACLE_BOUNDARY_INSET_M="0.08"
# This H2H candidate requires raw-cloud ROI/obstacle outputs.  A cold ROS
# process can register its node before both PointCloud2 publishers are ready,
# so allow a longer first start and retry the complete readiness check.
export ROBORACER_OBSTACLE_WAIT_S="${ROBORACER_OBSTACLE_WAIT_S:-20}"
export ROBORACER_OBSTACLE_TOPIC_WAIT_S="${ROBORACER_OBSTACLE_TOPIC_WAIT_S:-12}"
export TWO_MAP_PERCEPTION_START_RETRIES="${TWO_MAP_PERCEPTION_START_RETRIES:-3}"
export TWO_MAP_PERCEPTION_RETRY_DELAY_S="${TWO_MAP_PERCEPTION_RETRY_DELAY_S:-1}"

exec "$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_c11_stabilized_candidate.sh" "$@"
