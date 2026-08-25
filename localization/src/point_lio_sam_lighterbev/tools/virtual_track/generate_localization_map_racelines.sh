#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MAP_DIR="${1:-$PACKAGE_ROOT/point_lio/localization_map}"
VEHICLE_CONFIG="${2:-${F1TENTH_VEHICLE_CONFIG:-}}"
MAP_YAML="$MAP_DIR/point_lio_map_2d.yaml"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "[ERROR] Map YAML not found: $MAP_YAML" >&2
  exit 2
fi

COMMON_ARGS=(
  --map-yaml "$MAP_YAML"
)

if [[ -n "$VEHICLE_CONFIG" ]]; then
  if [[ ! -f "$VEHICLE_CONFIG" ]]; then
    echo "[ERROR] Vehicle config not found: $VEHICLE_CONFIG" >&2
    exit 2
  fi
  COMMON_ARGS+=(--vehicle-config "$VEHICLE_CONFIG")
else
  COMMON_ARGS+=(
    --raceline-points 260
    --wheelbase-m 0.265
    --vehicle-width-m 0.22
    --track-margin-m 0.08
    --min-speed-mps 0.20
    --max-speed-mps 1.85
    --max-steer-rad 0.55
    --max-steer-rate-radps 5.5
    --max-accel-mps2 2.4
    --max-decel-mps2 4.0
    --lateral-accel-limit-mps2 3.2
    --min-tracker-boundary-margin-m 0.25
  )
fi

generate_one() {
  local track_name="$1"
  local control_points="$2"
  local track_dir="$MAP_DIR/$track_name"
  local corridor="$track_dir/virtual_track_corridor.csv"

  if [[ ! -f "$corridor" ]]; then
    echo "[ERROR] Existing paired-boundary corridor not found: $corridor" >&2
    exit 2
  fi

  echo "[INFO] Generating $track_name from existing paired boundaries"
  python3 "$SCRIPT_DIR/generate_centerline_raceline.py" \
    --corridor-csv "$corridor" \
    --output-dir "$track_dir" \
    --optimizer-control-points "$control_points" \
    "${COMMON_ARGS[@]}"
}

# This command intentionally leaves all boundary and corridor files untouched.
generate_one virtual_track 24
generate_one virtual_track_u_turn 24
generate_one virtual_track_s_u 16

python3 "$SCRIPT_DIR/validate_localization_map_racelines.py" \
  --map-dir "$MAP_DIR"
python3 "$SCRIPT_DIR/generate_three_track_overview.py" \
  --map-dir "$MAP_DIR"

echo "[OK] Three localization-map racelines regenerated and validated."
