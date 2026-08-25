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
  /roboracer_eth_h2h/candidate_left
  /roboracer_eth_h2h/candidate_right
  /roboracer_eth_h2h/selected_path
  /roboracer_eth_h2h/valid_area
  /roboracer_eth_h2h/local_reference
  /roboracer_eth_h2h/local_speed_cap
  /roboracer_eth_h2h/state
  /roboracer_eth_h2h/status
  /roboracer_eth_h2h/health
  /roboracer_eth_h2h/candidate_diagnostics
  /roboracer_eth_h2h/mpcc_reference_status
  /roboracer_eth_h2h/mpcc/ackermann_cmd_stamped
  /roboracer_eth_h2h/mpcc/telemetry
  /roboracer_eth_h2h/mpcc/prediction
  /roboracer_eth_h2h/command_gate_state
  /tianracer/ackermann_cmd
)

exec rosbag record --lz4 -O "$1" "${topics[@]}"
