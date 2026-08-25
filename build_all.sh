#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
catkin_make -C "$ROOT/dependencies_ws" --source "$ROOT/third_party" -DCMAKE_BUILD_TYPE=Release
set +u; source "$ROOT/dependencies_ws/devel/setup.bash" --extend; set -u
LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/opt/roboracer-libtorch-cxx11/libtorch}"
if [[ ! -f "$LIBTORCH_ROOT/share/cmake/Torch/TorchConfig.cmake" ]]; then
  echo "Missing C++11-ABI LibTorch at $LIBTORCH_ROOT" >&2
  echo "Run: $ROOT/tools/install_libtorch_cpu.sh" >&2
  exit 2
fi
catkin_make -C "$ROOT/localization" -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='' \
  -DTorch_DIR="$LIBTORCH_ROOT/share/cmake/Torch" \
  -DCaffe2_DIR="$LIBTORCH_ROOT/share/cmake/Caffe2"
set +u; source "$ROOT/localization/devel/setup.bash" --extend; set -u
catkin_make -C "$ROOT/residual" -DCMAKE_BUILD_TYPE=Release
set +u; source "$ROOT/residual/devel/setup.bash" --extend; set -u
catkin_make -C "$ROOT/perception" -DCMAKE_BUILD_TYPE=Release
set +u; source "$ROOT/perception/devel/setup.bash" --extend; set -u
catkin_make -C "$ROOT/controller" -DCMAKE_BUILD_TYPE=Release -DRESIDUAL_STANDALONE_ROOT="$ROOT/residual"
set +u; source "$ROOT/controller/devel/setup.bash" --extend; set -u
if [[ -d "$ROOT/simulation_ws/src" ]]; then
  catkin_make -C "$ROOT/simulation_ws" -DCMAKE_BUILD_TYPE=Release
fi
echo "[OK] complete RoboRacer stack built"
