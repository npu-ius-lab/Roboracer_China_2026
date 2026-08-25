#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 BAG_PATH" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPIC_LIST="$SCRIPT_DIR/record_lightweight_overtake_topics.sh"
RECORD_SHADOW_TOPICS="${MPCC_RECORD_SHADOW_TOPICS:-false}"
if [[ "$RECORD_SHADOW_TOPICS" != "true" && "$RECORD_SHADOW_TOPICS" != "false" ]]; then
  echo "[FAIL] MPCC_RECORD_SHADOW_TOPICS must be true or false" >&2
  exit 2
fi

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
)

if [[ "$RECORD_SHADOW_TOPICS" == "true" ]]; then
  if [[ ! -f "$TOPIC_LIST" ]]; then
    echo "[FAIL] shadow topic list is missing: $TOPIC_LIST" >&2
    exit 1
  fi
  # shellcheck source=record_lightweight_overtake_topics.sh
  source "$TOPIC_LIST"
  TOPICS+=("${LIGHTWEIGHT_OVERTAKE_TOPICS[@]}")
  echo "[RECORD] base + shadow topics"
else
  echo "[RECORD] base topics only"
fi

exec rosbag record -O "$1" "${TOPICS[@]}"
