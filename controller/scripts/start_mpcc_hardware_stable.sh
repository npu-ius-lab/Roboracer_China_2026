#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/controller.yaml"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_markov_v1.yaml"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/vehicle.yaml"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] stable profile file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

# These hashes identify the on-car configuration that produced the verified
# 1.5 rad/s run. Refuse to call an experimental configuration under the stable
# name.
check_hash "a78197b6c064c9f5f02bf6ed5d08dae0bbf231958d917a69a0ec6593d46e4f54" "$CONTROLLER"
check_hash "16a18c94442f90415cf8bd0cea81b7c7887e0624adfa1f3e80a26e44246fe3fc" "$RESIDUAL"
check_hash "2afe83c6a651a1a95a62d3a8f1d79145b85acfc46e2b63e4df715c0f9b62061e" "$VEHICLE"

if [[ "${1:-}" == "--check-only" ]]; then
  echo "[OK] stable profile hashes match"
  exit 0
fi

export RESIDUAL_MPCC_SPEED_CAP=3.0
export MPCC_COST_CONTOUR_OVERRIDE=45.0
export MPCC_COST_HEADING_RACE_OVERRIDE=1.0
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE=0.90
export MPCC_RISK_TIERING_ENABLED=true
export MPCC_NEAR_TRACK_HORIZON_OVERRIDE=0.40
export MPCC_PREDICTIVE_SPEED_HOLD_OVERRIDE=0.50
export MPCC_PREDICTIVE_SPEED_RECOVERY_OVERRIDE=1.0
export MPCC_PP_CURRENT_MARGIN_OVERRIDE=0.02
export MPCC_PP_CONFIRMATION_CYCLES_OVERRIDE=2

echo "[STABLE] Verified V1 residual MPCC, steering rate=1.5 rad/s, speed cap=3.0 m/s"
exec "$ROOT/scripts/start_mpcc_hardware_v1_tracking.sh" "${1:-mpcc}"
