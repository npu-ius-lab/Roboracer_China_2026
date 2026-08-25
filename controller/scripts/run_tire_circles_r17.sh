#!/usr/bin/env bash
set -euo pipefail

# R=1.7 m constant-radius tire-identification circles with small steering PRBS.
# Runs 2.0 and 2.5 m/s, each left and right, one at a time so the car can be
# repositioned between runs. Every run auto-records its bag and ends with zero.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

RADIUS=1.7
EXCITATION_DEG=2
DURATION=30

run_one() {
  local speed="$1"
  local direction="$2"
  local tag
  tag="tire_r17_v${speed/./p}_${direction:0:1}"
  echo
  echo "[NEXT] R=${RADIUS}m v=${speed} m/s ${direction}"
  echo "       请把车放到空旷区中央，确认静止，遥控急停就绪。"
  read -r -p "       按回车开始（Ctrl-C 可中断本档）: "
  python3 "$ROOT/lateral_identification/run_circle_prbs.py" \
    "$tag" \
    --radius "$RADIUS" \
    --speed "$speed" \
    --direction "$direction" \
    --excitation-deg "$EXCITATION_DEG" \
    --duration "$DURATION" \
    --max-lateral-accel 6.0
  sleep 3
}

echo "[PLAN] 四档：R=1.7m × v={2.0, 2.5} × {left, right}"
echo "[PLAN] ay ≈ 2.35 m/s² (v=2.0) 与 3.68 m/s² (v=2.5)"
echo "[PLAN] bag 输出目录: lateral_identification/data/circle_prbs/"

run_one 2.0 left
run_one 2.0 right
run_one 2.5 left
run_one 2.5 right

echo
echo "[DONE] 四档全部完成。"
