#!/usr/bin/env bash
set -euo pipefail

# Start a rosbag recording of every analysis-relevant topic: localization,
# transforms, control, opponent detections and shadow-only overtake planning.
# Large point clouds stay opt-in so recording does not disturb real-time MPCC.
#
# Usage: record_analysis_bag.sh <output.bag>
# The recorder runs in the foreground of the caller's background subshell;
# send SIGINT (Ctrl-C on the parent launch) to finalize the bag cleanly.

BAG_PATH="${1:?usage: record_analysis_bag.sh <output.bag>}"
mkdir -p "$(dirname "$BAG_PATH")"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=record_lightweight_overtake_topics.sh
source "$SCRIPT_DIR/record_lightweight_overtake_topics.sh"

TOPICS=(
  /aft_mapped_to_init
  /localization/odom
  /localization/vehicle_odom
  /localization/status
  /tf
  /tf_static
  "${LIGHTWEIGHT_OVERTAKE_TOPICS[@]}"
  /perception/targets
  /overtake/candidate_left
  /overtake/candidate_right
  /overtake/markers
  /avoidance/track_markers
  /f1tenth_mpcc/real/ackermann_cmd_stamped
  /f1tenth_mpcc/telemetry
  /tianracer/ackermann_cmd
  /tianracer/odom
  /tianracer/imu
  /livox/imu
)

# Enable only when perception debugging/replay needs point-level evidence:
#   MPCC_RECORD_OVERTAKE_RAW_CLOUDS=true ./scripts/...
# These topics can make a nine-minute bag very large.
if [[ "${MPCC_RECORD_OVERTAKE_RAW_CLOUDS:-false}" == "true" ]]; then
  TOPICS+=(
    /localization/aligned_scan
    /perception/obstacle_cloud
    /perception/ground_cloud
    /perception/roi_cloud
  )
fi

exec rosbag record -O "$BAG_PATH" "${TOPICS[@]}"
