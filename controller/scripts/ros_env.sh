#!/usr/bin/env bash
# Source this file from the other runtime scripts.

MPCC_ROOT="${MPCC_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
if [[ -z "${ACADOS_SOURCE_DIR:-}" ]]; then
  if [[ -d /home/tianbot/opt/acados-0.5.5 ]]; then
    ACADOS_SOURCE_DIR=/home/tianbot/opt/acados-0.5.5
  elif [[ -d /home/ros/opt/acados-0.5.5 ]]; then
    ACADOS_SOURCE_DIR=/home/ros/opt/acados-0.5.5
  else
    ACADOS_SOURCE_DIR="$HOME/opt/acados-0.5.5"
  fi
fi
if [[ -f "$MPCC_ROOT/scripts/configure_remote_ros_master.sh" ]]; then
  source "$MPCC_ROOT/scripts/configure_remote_ros_master.sh"
else
  ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
fi

MPCC_RESTORE_NOUNSET=false
case "$-" in
  *u*) MPCC_RESTORE_NOUNSET=true ;;
esac
set +u
source /opt/ros/noetic/setup.bash
LOCAL_ACKERMANN_PREFIX="$(dirname "$MPCC_ROOT")/f1tenth_ws/local_ros_noetic/opt/ros/noetic"
if [[ -d "$LOCAL_ACKERMANN_PREFIX/share/ackermann_msgs" ]]; then
  export CMAKE_PREFIX_PATH="$LOCAL_ACKERMANN_PREFIX:${CMAKE_PREFIX_PATH:-}"
  export PYTHONPATH="$LOCAL_ACKERMANN_PREFIX/lib/python3/dist-packages:${PYTHONPATH:-}"
fi
RESIDUAL_DYNAMICS_WS="${RESIDUAL_DYNAMICS_WS:-$(dirname "$MPCC_ROOT")/f1tenth_residual_ws}"
source "$RESIDUAL_DYNAMICS_WS/devel/setup.bash"
source "$MPCC_ROOT/devel/setup.bash"
if [[ "$MPCC_RESTORE_NOUNSET" == true ]]; then
  set -u
fi
unset MPCC_RESTORE_NOUNSET
export MPCC_ROOT ACADOS_SOURCE_DIR ROS_MASTER_URI RESIDUAL_DYNAMICS_WS
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MPLBACKEND=Agg
