#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_DATA_DIR="$ROOT/lateral_identification/data/remote_20260816_steering_actuator"
if [[ $# -ge 1 ]]; then
  DATA_DIR="$1"
elif compgen -G "$DEFAULT_DATA_DIR/*.bag" >/dev/null; then
  DATA_DIR="$DEFAULT_DATA_DIR"
else
  # The car stores the same campaign bags directly under data/.
  DATA_DIR="$ROOT/lateral_identification/data"
fi
OUTPUT="${2:-$DEFAULT_DATA_DIR/offline_steering_experiments_v1.json}"
mkdir -p "$(dirname "$OUTPUT")"

EXTRA_ARGS=()
LEGACY_STEERING_DIR="${LEGACY_STEERING_DIR:-/home/ros/steering_actuator_identification_archive/legacy_20260814/data}"
if [[ ! -f "$LEGACY_STEERING_DIR/actuator_run01.bag" && -d "$HOME/f1tenth_mpcc/lateral_identification/data" ]]; then
  LEGACY_STEERING_DIR="$HOME/f1tenth_mpcc/lateral_identification/data"
fi
for bag in "$LEGACY_STEERING_DIR"/actuator_run01.bag "$LEGACY_STEERING_DIR"/actuator_run02.bag; do
  if [[ -f "$bag" ]]; then
    EXTRA_ARGS+=(--external-bag "$bag")
  fi
done

for name in lateral_mpcc_20260814_185646_0.bag mpcc_40hz_td0_20260814_193817.bag; do
  for bag in \
    "/home/ros/analysis/live_lateral_20260814/$name" \
    "$ROOT/analysis/bags/$name" \
    "$ROOT/bags/live_lateral/$name" \
    "$ROOT/bags/$name"; do
    if [[ -f "$bag" ]]; then
      EXTRA_ARGS+=(--external-bag "$bag")
      break
    fi
  done
done

CIRCLE_VALIDATION_DIR="${CIRCLE_VALIDATION_DIR:-/home/ros/f1tenth_ws/f1tenth_mpcc/src/f1tenth_dynamic_mpcc/speed_identification/data/validation}"
if ! compgen -G "$CIRCLE_VALIDATION_DIR/circle_*.bag" >/dev/null && [[ -d "$HOME/f1tenth_mpcc/src/f1tenth_dynamic_mpcc/speed_identification/data/validation" ]]; then
  CIRCLE_VALIDATION_DIR="$HOME/f1tenth_mpcc/src/f1tenth_dynamic_mpcc/speed_identification/data/validation"
fi
for bag in "$CIRCLE_VALIDATION_DIR"/circle_*.bag; do
  if [[ -f "$bag" ]]; then
    EXTRA_ARGS+=(--circle-bag "$bag")
  fi
done

export PYTHONPATH="$ROOT:$ROOT/lateral_identification:$ROOT/src/f1tenth_dynamic_mpcc/python:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 "$ROOT/lateral_identification/offline_experiments.py" \
  --data-dir "$DATA_DIR" \
  --output "$OUTPUT" \
  "${EXTRA_ARGS[@]}"
