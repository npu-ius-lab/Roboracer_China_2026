#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
"$ROOT/scripts/ensure_acados_solvers.sh"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
BIN="$TMP_DIR/cpp_delayed_closed_loop"
RESIDUAL_PACKAGE="$RESIDUAL_DYNAMICS_WS/src/f1tenth_residual_dynamics"
if [[ ! -f "$RESIDUAL_PACKAGE/include/f1tenth_residual_dynamics/residual_model.hpp" ]]; then
  echo "[FAIL] residual-dynamics source package unavailable: $RESIDUAL_PACKAGE" >&2
  exit 2
fi

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I "$ROOT/src/f1tenth_dynamic_mpcc/include" -I /usr/include/eigen3 \
  -I "$RESIDUAL_PACKAGE/include" \
  "$ROOT/src/f1tenth_dynamic_mpcc/src/core.cpp" \
  "$ROOT/src/f1tenth_dynamic_mpcc/src/acados_runtime.cpp" \
  "$ROOT/src/f1tenth_dynamic_mpcc/src/pure_pursuit_candidate.cpp" \
  "$RESIDUAL_PACKAGE/src/residual_model.cpp" \
  "$ROOT/src/f1tenth_dynamic_mpcc/tests/cpp_delayed_closed_loop.cpp" \
  -lyaml-cpp -ldl -pthread -o "$BIN"

run_matrix() {
  for variant in legacy simplified cap_guard full; do
    "$BIN" \
      "$ROOT/src/f1tenth_dynamic_mpcc/config/controller.yaml" \
      "$ROOT/src/f1tenth_dynamic_mpcc/config/vehicle.yaml" \
      "$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/virtual_track/raceline.csv" \
      "${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_dynamic_mpcc/acados}" \
      "$variant" || true
  done
}

RESULT_PATH="${1:-}"
if [[ -z "$RESULT_PATH" ]]; then
  run_matrix
else
  mkdir -p "$(dirname "$RESULT_PATH")"
  RAW_OUTPUT="$(mktemp)"
  run_matrix 2>&1 | tee "$RAW_OUTPUT"
  awk '/^\{.*\}$/' "$RAW_OUTPUT" | tee "$RESULT_PATH"
  rm -f "$RAW_OUTPUT"
  echo "[OK] machine-readable summaries: $RESULT_PATH"
fi
