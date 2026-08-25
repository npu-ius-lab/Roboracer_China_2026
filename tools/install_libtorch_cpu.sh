#!/usr/bin/env bash
set -euo pipefail

VERSION="2.4.1"
INSTALL_ROOT="${LIBTORCH_INSTALL_ROOT:-$HOME/opt/roboracer-libtorch-cxx11}"
CONFIG="$INSTALL_ROOT/libtorch/share/cmake/Torch/TorchConfig.cmake"
ARCHIVE="$INSTALL_ROOT/libtorch-cxx11-abi-${VERSION}.zip"
URL="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${VERSION}%2Bcpu.zip"

if [[ -f "$CONFIG" ]]; then
  echo "[OK] LibTorch already installed: $INSTALL_ROOT/libtorch"
  exit 0
fi

mkdir -p "$INSTALL_ROOT"
curl -fL --retry 3 -o "$ARCHIVE" "$URL"
unzip -q -o "$ARCHIVE" -d "$INSTALL_ROOT"
test -f "$CONFIG"
echo "[OK] installed C++11-ABI CPU LibTorch: $INSTALL_ROOT/libtorch"
