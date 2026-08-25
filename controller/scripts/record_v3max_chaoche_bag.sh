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
  /roboracer_chaoche/perception/targets_dynamic
  /roboracer_chaoche/perception/targets_detailed
  /roboracer_chaoche/perception/target_markers
  /roboracer_chaoche/perception/static_obstacle_markers
  /roboracer_chaoche/perception/fusion_status
  /roboracer_chaoche/overtake/diagnostics
  /roboracer_chaoche/overtake/selected_path
  /roboracer_chaoche/overtake/valid_area
  /roboracer_chaoche/avoidance/status
  /roboracer_chaoche/avoidance/decision
  /roboracer_chaoche/avoidance/markers
  /roboracer_chaoche/local_reference
  /roboracer_chaoche/local_plan_red
  /roboracer_chaoche/local_speed_cap
  /roboracer_chaoche/state
  /roboracer_chaoche/reference_active
  /roboracer_chaoche/reference_status
  /roboracer_chaoche/mpcc_reference_status
  /roboracer_chaoche/mpcc/ackermann_cmd_stamped
  /roboracer_chaoche/mpcc/telemetry
  /roboracer_chaoche/mpcc/prediction
  /automatic_relaunch_supervisor_roboracer_chaoche/state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
