#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG" >&2
  exit 2
fi

topics=(
  /localization/aligned_scan
  /localization/odom
  /localization/vehicle_odom
  /localization/status
  /tf
  /tf_static
  /tianracer/odom
  /tianracer/imu
  /livox/imu
  /v5_chaoche/perception/targets_dynamic
  /v5_chaoche/perception/targets_detailed
  /v5_chaoche/perception/target_markers
  /v5_chaoche/perception/static_obstacle_markers
  /v5_chaoche/perception/fusion_status
  /v5_chaoche/overtake/diagnostics
  /v5_chaoche/overtake/selected_path
  /v5_chaoche/overtake/valid_area
  /v5_chaoche/avoidance/status
  /v5_chaoche/avoidance/decision
  /v5_chaoche/avoidance/markers
  /v5_chaoche/local_reference
  /v5_chaoche/local_plan_red
  /v5_chaoche/local_speed_cap
  /v5_chaoche/state
  /v5_chaoche/reference_active
  /v5_chaoche/reference_status
  /v5_chaoche/mpcc_reference_status
  /v5_chaoche/mpcc/ackermann_cmd_stamped
  /f1tenth_mpcc/telemetry
  /f1tenth_mpcc/prediction
  /automatic_relaunch_supervisor_v5_chaoche/state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
