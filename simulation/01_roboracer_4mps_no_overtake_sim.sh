#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.bash"
prepare_solver no_overtake
exec roslaunch roboracer_gazebo no_overtake_4mps.launch gui:="${GAZEBO_GUI:-$(gazebo_gui_default)}" "$@"
