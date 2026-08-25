#!/usr/bin/env bash
set -euo pipefail

# Race-ready wrapper.  Keep the frozen H2H candidate script and its parameters
# unchanged; reject launch if the separately-built JUBU executable is stale.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/scripts/start_mpcc_hardware_stable_roboracer_h2h_safe_start_fast2_candidate.sh"
JUBU="$ROOT/devel/lib/f1tenth_dynamic_mpcc/mpcc_node_auto_relaunch_jubu_cpp"

[[ -x "$BASE" ]] || { echo "[FAIL] missing base H2H script: $BASE" >&2; exit 1; }
[[ -x "$JUBU" ]] || { echo "[FAIL] missing JUBU executable: $JUBU" >&2; exit 1; }

if ldd -r "$JUBU" 2>&1 | grep -q 'undefined symbol.*CommandManager.*setSolution'; then
  echo "[FAIL] stale JUBU executable: CommandManager::setSolution ABI mismatch" >&2
  echo "[FAIL] do not release the car; rebuild the isolated JUBU target first" >&2
  exit 1
fi

echo "[OK] race-ready JUBU ABI check passed"
echo "[OK] H2H control parameters unchanged; raw-cloud perception preflight enabled"
exec "$BASE" "${1:-mpcc}"
