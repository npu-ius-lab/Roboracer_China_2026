#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG" >&2
  exit 2
fi

topics=(
  /raw_cloud_perception/roi_cloud
  /raw_cloud_perception/obstacle_cloud
  /raw_cloud_perception/corridor_markers
  /localization/odom
  /localization/vehicle_odom
  /localization/status
  /tf
  /tf_static
  /tianracer/odom
  /tianracer/imu
  /livox/imu
  /roboracer_c11_react/health
  /roboracer_c11_react/state
  /roboracer_c11_react/status
  /roboracer_c11_react/diagnostics
  /roboracer_c11_react/occupancy_markers
  /roboracer_c11_react/candidate_left
  /roboracer_c11_react/candidate_right
  /roboracer_c11_react/selected_path
  /roboracer_c11_react/valid_area
  /roboracer_c11_react/local_reference
  /roboracer_c11_react/local_plan_red
  /roboracer_c11_react/local_speed_cap
  /roboracer_c11_react/mpcc_reference_status
  /roboracer_c11_react/mpcc/raw_ackermann_cmd_stamped
  /roboracer_c11_react/mpcc/ackermann_cmd_stamped
  /roboracer_c11_react/min_speed_gate/status
  /roboracer_c11_react/mpcc/telemetry
  /roboracer_c11_react/mpcc/prediction
  /automatic_relaunch_supervisor_roboracer_c11_react/state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
