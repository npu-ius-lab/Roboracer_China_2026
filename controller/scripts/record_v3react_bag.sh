#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG" >&2
  exit 2
fi

topics=(
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
  /roboracer_react/health
  /roboracer_react/state
  /roboracer_react/status
  /roboracer_react/diagnostics
  /roboracer_react/occupancy_markers
  /roboracer_react/candidate_left
  /roboracer_react/candidate_right
  /roboracer_react/selected_path
  /roboracer_react/valid_area
  /roboracer_react/local_reference
  /roboracer_react/local_plan_red
  /roboracer_react/local_speed_cap
  /roboracer_react/mpcc_reference_status
  /roboracer_react/mpcc/raw_ackermann_cmd_stamped
  /roboracer_react/mpcc/ackermann_cmd_stamped
  /roboracer_react/min_speed_gate/status
  /roboracer_react/mpcc/telemetry
  /roboracer_react/mpcc/prediction
  /automatic_relaunch_supervisor_roboracer_react/state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
