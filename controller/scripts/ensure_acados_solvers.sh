#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados}"
RESIDUAL_MODEL_PATH="${MPCC_RESIDUAL_MODEL_PATH:-}"
RESIDUAL_FEATURE_SET="${MPCC_RESIDUAL_FEATURE_SET:-}"
COST_CONTOUR_OVERRIDE="${MPCC_COST_CONTOUR_OVERRIDE:-}"
COST_HEADING_RACE_OVERRIDE="${MPCC_COST_HEADING_RACE_OVERRIDE:-}"
COST_STEERING_RATE_OVERRIDE="${MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE:-}"
if [[ -n "$RESIDUAL_MODEL_PATH" || -n "$RESIDUAL_FEATURE_SET" ]]; then
  if [[ -z "$RESIDUAL_MODEL_PATH" || -z "$RESIDUAL_FEATURE_SET" ]]; then
    echo "MPCC_RESIDUAL_MODEL_PATH and MPCC_RESIDUAL_FEATURE_SET must be set together" >&2
    exit 2
  fi
fi
EXPECTED_NP=14
PACKAGE_ROOT="$ROOT/src/f1tenth_dynamic_mpcc"
TRACK="${MPCC_TRACK:-virtual_track}"
TRACK_CSV="$PACKAGE_ROOT/data/tracks/$TRACK/raceline.csv"
CONTROLLER_CONFIG="${MPCC_CONTROLLER_CONFIG:-controller.yaml}"
CONTROLLER_CONFIG_PATH="$PACKAGE_ROOT/config/$CONTROLLER_CONFIG"
VEHICLE_CONFIG="${MPCC_VEHICLE_CONFIG:-vehicle.yaml}"
VEHICLE_CONFIG_PATH="$PACKAGE_ROOT/config/$VEHICLE_CONFIG"
if [[ ! -f "$TRACK_CSV" ]]; then
  echo "Track not found: $TRACK_CSV" >&2
  exit 2
fi
if [[ ! -f "$CONTROLLER_CONFIG_PATH" ]]; then
  echo "Controller config not found: $CONTROLLER_CONFIG_PATH" >&2
  exit 2
fi
if [[ ! -f "$VEHICLE_CONFIG_PATH" ]]; then
  echo "Vehicle config not found: $VEHICLE_CONFIG_PATH" >&2
  exit 2
fi
# Solver generation must also work before this overlay has been rebuilt or
# sourced. Import the package directly from its source tree.
export PYTHONPATH="$PACKAGE_ROOT/python:${PYTHONPATH:-}"
INPUT_HASH="$({
  find "$PACKAGE_ROOT/python/f1tenth_dynamic_mpcc" \
       "$PACKAGE_ROOT/config" \
       "$TRACK_CSV" \
       "$PACKAGE_ROOT/scripts/generate_acados_solvers.py" \
       -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.csv' \) \
       -print0 | sort -z | xargs -0 sha256sum
  printf '%s\n%s\n%s\n%s\n%s\n%s\n%s\n' \
    "$RESIDUAL_MODEL_PATH" "$RESIDUAL_FEATURE_SET" \
    "$COST_CONTOUR_OVERRIDE" "$COST_HEADING_RACE_OVERRIDE" \
    "$COST_STEERING_RATE_OVERRIDE" \
    "$CONTROLLER_CONFIG" "$VEHICLE_CONFIG"
} | sha256sum | awk '{print $1}')"
STORED_HASH="$(cat "$GENERATED_DIR/solver_inputs.sha256" 2>/dev/null || true)"
SOLVER_BASELINE="$GENERATED_DIR/c_generated_code_baseline/libacados_ocp_solver_f1tenth_dynamic_baseline.so"
SOLVER_MPCC="$GENERATED_DIR/c_generated_code_mpcc/libacados_ocp_solver_f1tenth_dynamic_mpcc.so"
STORED_ARTIFACT_HASH="$(cat "$GENERATED_DIR/solver_artifacts.sha256" 2>/dev/null || true)"
COMPUTED_ARTIFACT_HASH="$({
  for solver_path in "$SOLVER_BASELINE" "$SOLVER_MPCC"; do
    if [[ -f "$solver_path" ]]; then
      sha256sum "$solver_path"
    else
      echo "MISSING $solver_path"
    fi
  done
} | sha256sum | awk '{print $1}')"

if [[ -f "$GENERATED_DIR/interface_np.txt" ]] \
  && [[ "$(tr -d '[:space:]' < "$GENERATED_DIR/interface_np.txt")" == "$EXPECTED_NP" ]] \
  && [[ "$STORED_HASH" == "$INPUT_HASH" ]] \
  && [[ -n "$STORED_ARTIFACT_HASH" ]] \
  && [[ "$STORED_ARTIFACT_HASH" == "$COMPUTED_ARTIFACT_HASH" ]]; then
  exit 0
fi

echo "Generated acados interface is missing or stale; rebuilding np=$EXPECTED_NP solvers."
GENERATOR_ARGS=(--generated-dir "$GENERATED_DIR")
GENERATOR_ARGS+=(--track "$TRACK")
GENERATOR_ARGS+=(--controller-config "$CONTROLLER_CONFIG")
GENERATOR_ARGS+=(--vehicle-config "$VEHICLE_CONFIG")
if [[ -n "$RESIDUAL_MODEL_PATH" ]]; then
  GENERATOR_ARGS+=(--residual-model "$RESIDUAL_MODEL_PATH"
                   --residual-feature-set "$RESIDUAL_FEATURE_SET")
fi
if [[ -n "$COST_CONTOUR_OVERRIDE" ]]; then
  GENERATOR_ARGS+=(--cost-contour "$COST_CONTOUR_OVERRIDE")
fi
if [[ -n "$COST_HEADING_RACE_OVERRIDE" ]]; then
  GENERATOR_ARGS+=(--cost-heading-race "$COST_HEADING_RACE_OVERRIDE")
fi
if [[ -n "$COST_STEERING_RATE_OVERRIDE" ]]; then
  GENERATOR_ARGS+=(--cost-steering-command-rate "$COST_STEERING_RATE_OVERRIDE")
fi
python3 "$ROOT/src/f1tenth_dynamic_mpcc/scripts/generate_acados_solvers.py" "${GENERATOR_ARGS[@]}"
printf '%s\n' "$INPUT_HASH" > "$GENERATED_DIR/solver_inputs.sha256"
COMPUTED_ARTIFACT_HASH="$({
  for solver_path in "$SOLVER_BASELINE" "$SOLVER_MPCC"; do
    if [[ -f "$solver_path" ]]; then
      sha256sum "$solver_path"
    else
      echo "MISSING $solver_path"
    fi
  done
} | sha256sum | awk '{print $1}')"
printf '%s\n' "$COMPUTED_ARTIFACT_HASH" > "$GENERATED_DIR/solver_artifacts.sha256"
