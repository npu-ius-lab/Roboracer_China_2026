#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG" >&2
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
  /lio/relocalization/lighterbev_match
  /scan
  /semantic_region/state
  /perception/targets_detailed
  /perception/target_markers
  /overtake/diagnostics
  /overtake/selected_path
  /overtake/valid_area
  /avoidance/status
  /avoidance/decision
  /avoidance/markers
  /avoidance/local_plan_red
)

if [[ "${MPCC_RECORD_OVERTAKE_ALIGNED_SCAN:-false}" == "true" ]]; then
  TOPICS+=(/localization/aligned_scan)
fi

exec rosbag record -O "$1" "${TOPICS[@]}"
