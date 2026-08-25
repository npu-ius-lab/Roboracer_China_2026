#!/usr/bin/env bash
set -euo pipefail

RELEASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME || true

set +u
source /opt/ros/noetic/setup.bash
source "$RELEASE_ROOT/dependencies_ws/devel/setup.bash" --extend
source "$RELEASE_ROOT/residual/devel/setup.bash" --extend
if [[ -f "$RELEASE_ROOT/localization/devel/setup.bash" ]]; then
  source "$RELEASE_ROOT/localization/devel/setup.bash" --extend
fi
source "$RELEASE_ROOT/perception/devel/setup.bash" --extend
source "$RELEASE_ROOT/controller/devel/setup.bash" --extend
source "$RELEASE_ROOT/simulation_ws/devel/setup.bash" --extend
set -u

export RESIDUAL_DYNAMICS_WS="$RELEASE_ROOT/residual"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-/home/ros/opt/acados-0.5.5}"
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:$RELEASE_ROOT/controller/src/f1tenth_dynamic_mpcc/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MPLBACKEND=Agg

prepare_solver() {
  local mode="$1"
  export MPCC_VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
  export MPCC_RESIDUAL_MODEL_PATH="$RELEASE_ROOT/controller/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
  export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
  export MPCC_COST_CONTOUR_OVERRIDE="45.0"
  export MPCC_COST_HEADING_RACE_OVERRIDE="1.0"
  export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="0.60"
  case "$mode" in
    no_overtake)
      export MPCC_TRACK="raceline_smooth"
      export MPCC_CONTROLLER_CONFIG="candidates/stable_roboracer/controller.yaml"
      export MPCC_GENERATED_DIR="$HOME/.cache/f1tenth_roboracer_sim/no_overtake"
      ;;
    reactive_green|h2h_green)
      export MPCC_TRACK="raceline_smooth_c11_outer_boundary_expanded_candidate"
      export MPCC_CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
      export MPCC_GENERATED_DIR="$HOME/.cache/f1tenth_roboracer_sim/$mode"
      ;;
    *) echo "unknown simulation mode: $mode" >&2; return 2 ;;
  esac
  "$RELEASE_ROOT/controller/scripts/ensure_acados_solvers.sh"
}

gazebo_gui_default() {
  [[ -n "${DISPLAY:-}" ]] && echo true || echo false
}
