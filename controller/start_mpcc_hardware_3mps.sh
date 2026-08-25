#!/usr/bin/env bash
set -euo pipefail

# Dedicated 3 m/s residual-MPCC hardware profile. The existing
# start_mpcc_hardware.sh remains unchanged at its 2 m/s default and can still
# be used as the conservative fallback.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-3.0}"

if (($# == 0)); then
  set -- mpcc
fi

exec "$ROOT/scripts/start_mpcc_hardware.sh" "$@"
