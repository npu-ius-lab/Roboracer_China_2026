#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_v2p1_physics.yaml"

if [[ ! -f "$MODEL" ]]; then
  echo "[FAIL] V2.1 validation candidate is not installed: $MODEL" >&2
  echo "Use start_mpcc_hardware_v1.sh until the candidate passes validation." >&2
  exit 1
fi

export MPCC_RESIDUAL_MODEL_PATH="$MODEL"
export MPCC_RESIDUAL_FEATURE_SET="physics_v2"
export MPCC_GENERATED_DIR="${MPCC_V2_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/v2p1_physics}"
export RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-3.0}"

echo "[PROFILE] V2.1 physics residual candidate, speed cap=$RESIDUAL_MPCC_SPEED_CAP m/s (V1 remains the default)"
exec "$ROOT/scripts/start_mpcc_hardware.sh" "${1:-mpcc}"
