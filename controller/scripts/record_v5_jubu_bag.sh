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
  /f1tenth_mpcc/v5_jubu/ackermann_cmd_stamped
  /f1tenth_mpcc/telemetry
  /v5_jubu/command_gate_state
  /tianracer/ackermann_cmd
  /tianracer/odom
  /tianracer/imu
  /livox/imu
  /lio/relocalization/lighterbev_match
  /perception/roi_cloud
  /v5_jubu/candidate_diagnostics
  /v5_jubu/selected_path
  /v5_jubu/valid_area
  /v5_jubu/status
  /v5_jubu/candidate_left
  /v5_jubu/candidate_right
  /v5_jubu/local_reference
  /v5_jubu/local_speed_cap
  /v5_jubu/state
  /v5_jubu/health
  /v5_jubu/mpcc_reference_status
)

exec rosbag record -O "$1" "${TOPICS[@]}"
