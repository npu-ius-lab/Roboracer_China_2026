#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START="$ROOT/scripts/start_mpcc_hardware_stable_roboracer_eth_h2h.sh"
PERCEPTION="$ROOT/scripts/ensure_raw_cloud_perception.sh"

[[ -x "$START" ]] || {
  echo "[FAIL] missing ROBORACER overtaking launcher: $START" >&2
  exit 1
}
[[ -x "$PERCEPTION" ]] || {
  echo "[FAIL] missing obstacle-perception helper: $PERCEPTION" >&2
  exit 1
}

export H2H_SPEED_CAP_MPS="${H2H_SPEED_CAP_MPS:-4.0}"
export H2H_AUTO_ARM="${H2H_AUTO_ARM:-true}"
export MPCC_RECORD_BAG="${MPCC_RECORD_BAG:-true}"

case "${1:-mpcc}" in
  --check-only|--prepare-only) ;;
  *) "$PERCEPTION" ;;
esac

echo "[stableroboracer_chaoche] cap=${H2H_SPEED_CAP_MPS}m/s auto_arm=${H2H_AUTO_ARM} record_bag=${MPCC_RECORD_BAG}"
exec "$START" "$@"
