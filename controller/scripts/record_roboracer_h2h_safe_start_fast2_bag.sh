#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG" >&2
  exit 2
fi

topics=(
  /cloud_registered_body
  /raw_cloud_perception/roi_cloud
  /raw_cloud_perception/obstacle_cloud
  /localization/odom
  /localization/vehicle_odom
  /localization/status
  /tf
  /tf_static
  /tianracer/odom
  /tianracer/imu
  /livox/imu
  /roboracer_h2h_faststart/candidate_left
  /roboracer_h2h_faststart/candidate_right
  /roboracer_h2h_faststart/selected_path
  /roboracer_h2h_faststart/valid_area
  /roboracer_h2h_faststart/local_reference
  /roboracer_h2h_faststart/local_speed_cap
  /roboracer_h2h_faststart/state
  /roboracer_h2h_faststart/status
  /roboracer_h2h_faststart/health
  /roboracer_h2h_faststart/candidate_diagnostics
  /roboracer_h2h_faststart/mpcc_reference_status
  /roboracer_h2h_faststart/mpcc/ackermann_cmd_stamped
  /roboracer_h2h_faststart/mpcc/telemetry
  /roboracer_h2h_faststart/mpcc/prediction
  /roboracer_h2h_faststart/supervised_command
  /roboracer_h2h_faststart/follow_gate_state
  /automatic_relaunch_supervisor_roboracer_h2h_faststart/state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
