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
  /roboracer_react_fast/health
  /roboracer_react_fast/state
  /roboracer_react_fast/status
  /roboracer_react_fast/diagnostics
  /roboracer_react_fast/occupancy_markers
  /roboracer_react_fast/candidate_left
  /roboracer_react_fast/candidate_right
  /roboracer_react_fast/selected_path
  /roboracer_react_fast/valid_area
  /roboracer_react_fast/local_reference
  /roboracer_react_fast/local_plan_red
  /roboracer_react_fast/local_speed_cap
  /roboracer_react_fast/mpcc_reference_status
  /roboracer_react_fast/mpcc/raw_ackermann_cmd_stamped
  /roboracer_react_fast/mpcc/ackermann_cmd_stamped
  /roboracer_react_fast/min_speed_gate/status
  /roboracer_react_fast/mpcc/telemetry
  /roboracer_react_fast/mpcc/prediction
  /automatic_relaunch_supervisor_roboracer_react_fast/state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
