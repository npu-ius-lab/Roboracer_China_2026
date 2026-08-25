#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

if [[ $# -lt 2 || $# -gt 4 ]]; then
  cat >&2 <<'EOF'
Usage: ./scripts/record_constant_circle.sh SPEED_MPS STEERING_RAD [DURATION_S] [RUN_ID]

  SPEED_MPS     Constant raw Ackermann speed command, (0, 3.0].
  STEERING_RAD  Constant steering command. Positive=left, negative=right.
  DURATION_S    Measurement duration; 0 means run until Ctrl-C (default: 0).
  RUN_ID        Optional bag identifier (default: circle_<timestamp>).

Example:
  ./scripts/record_constant_circle.sh 1.0 0.25 0 circle_left_1mps
EOF
  exit 2
fi

speed="$1"
steering="$2"
duration="${3:-0}"
run_id="${4:-circle_$(date +%Y%m%d_%H%M%S)}"

python3 - "$speed" "$steering" "$duration" <<'PY'
import math
import sys

speed, steering, duration = map(float, sys.argv[1:])
if not math.isfinite(speed) or not 0.0 < speed <= 3.0:
    raise SystemExit("[FAIL] SPEED_MPS must be finite and in (0, 3.0]")
if not math.isfinite(steering) or not 0.05 <= abs(steering) <= 0.45:
    raise SystemExit("[FAIL] |STEERING_RAD| must be in [0.05, 0.45]")
if not math.isfinite(duration) or (duration != 0.0 and duration < 5.0):
    raise SystemExit("[FAIL] DURATION_S must be 0 or at least 5 seconds")

wheelbase = 0.320
radius = wheelbase / abs(math.tan(steering))
lateral_accel = speed * speed / radius
direction = "left/左转" if steering > 0.0 else "right/右转"
print(f"[PLAN] speed={speed:.3f} m/s steering={steering:.3f} rad ({direction})")
print(f"[PLAN] kinematic radius~{radius:.3f} m, lateral acceleration~{lateral_accel:.3f} m/s^2")
if lateral_accel > 3.2:
    raise SystemExit("[FAIL] estimated lateral acceleration exceeds 3.2 m/s^2")
PY

echo "[SAFETY] Stop MPCC/planner first; keep the remote emergency stop ready."
echo "[RECORD] Two seconds at zero, then one fixed speed/steering command."
echo "[RECORD] Press Ctrl-C once to send zero speed and finish the bag."

exec python3 "$ROOT/scripts/capture_fixed_speed.py" \
  validation "$run_id" "$speed" \
  --duration "$duration" \
  --steering "$steering" \
  --pre-zero 2.0 \
  --post-zero 3.0 \
  --execute
