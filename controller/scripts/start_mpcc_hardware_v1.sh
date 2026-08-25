#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_markov_v1.yaml"

if [[ ! -f "$MODEL" ]]; then
  echo "[FAIL] V1 rollback model is missing: $MODEL" >&2
  exit 1
fi

export MPCC_RESIDUAL_MODEL_PATH="$MODEL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
export MPCC_GENERATED_DIR="${MPCC_V1_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/v1}"
export RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-3.0}"

echo "[PROFILE] V1 rollback-safe residual, speed cap=$RESIDUAL_MPCC_SPEED_CAP m/s"
exec "$ROOT/scripts/start_mpcc_hardware.sh" "${1:-mpcc}"
