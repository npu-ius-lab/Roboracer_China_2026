#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.bash"
prepare_solver reactive_green
exec roslaunch roboracer_gazebo reactive_green_4mps.launch gui:="${GAZEBO_GUI:-$(gazebo_gui_default)}" "$@"
