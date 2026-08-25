#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 BAG_PATH" >&2
  exit 2
fi

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
  /f1tenth_mpcc/real/ackermann_cmd_stamped
  /f1tenth_mpcc/telemetry
  /automatic_relaunch_supervisor/state
  /tianracer/ackermann_cmd
  /tianracer/odom
  /tianracer/imu
  /livox/imu
  "${LIGHTWEIGHT_OVERTAKE_TOPICS[@]}"
)

exec rosbag record -O "$1" "${TOPICS[@]}"
