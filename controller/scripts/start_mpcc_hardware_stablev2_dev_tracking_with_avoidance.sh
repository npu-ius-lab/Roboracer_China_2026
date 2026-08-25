#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORIGINAL="$ROOT/scripts/start_mpcc_hardware_stablev2_dev_tracking.sh"
AVOIDANCE="$ROOT/avoidance/scripts/start_avoidance.sh"
EXPECTED_ORIGINAL_SHA256="d99240ca56157a1f51a5a716eb67035a4426498236df547eb3640bf154854b8c"

actual="$(sha256sum "$ORIGINAL" | awk '{print $1}')"
if [[ "$actual" != "$EXPECTED_ORIGINAL_SHA256" ]]; then
  echo "[FAIL] original stablev2_dev_tracking launcher changed" >&2
  echo "       expected=$EXPECTED_ORIGINAL_SHA256" >&2
  echo "       actual=$actual" >&2
  exit 1
fi

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  "$ORIGINAL" --check-only
  "$AVOIDANCE" --check-only
  echo "[OK] original controller launcher is unchanged; avoidance wrapper is ready"
  exit 0
fi
if [[ "$ACTION" == "--prepare-only" ]]; then
  "$AVOIDANCE" --check-only
  exec "$ORIGINAL" --prepare-only
fi

AVOIDANCE_PID=""
cleanup() {
  if [[ -n "$AVOIDANCE_PID" ]]; then
    kill -INT "$AVOIDANCE_PID" 2>/dev/null || true
    wait "$AVOIDANCE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"$AVOIDANCE" --foreground &
AVOIDANCE_PID="$!"
for _ in {1..100}; do
  if rosnode list 2>/dev/null | grep -qx '/avoidance_state_visualizer'; then
    break
  fi
  if ! kill -0 "$AVOIDANCE_PID" 2>/dev/null; then
    echo "[FAIL] avoidance module exited during startup" >&2
    exit 1
  fi
  sleep 0.1
done

echo "[WRAPPER] avoidance visualization active; starting untouched stablev2_dev_tracking launcher"
"$ORIGINAL" "$@"
