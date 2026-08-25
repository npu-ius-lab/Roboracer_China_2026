#!/usr/bin/env bash
# Seven fixed-steering, approximately one-lap circles in the same open space.
# Each circle has a separate bag and returns close to its entry point before
# the next speed begins.  Direction: left by default, or right with DIR=-1.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

PREFIX="${1:-circle_sweep_$(date +%Y%m%d_%H%M%S)}"
DIR="${DIR:-1}"
if [[ "$DIR" != "1" && "$DIR" != "-1" ]]; then
  echo "DIR must be 1 (left) or -1 (right)" >&2
  exit 2
fi

# Low-speed measured command-to-equivalent-wheel-angle gain.  30 deg command
# gave 25.8 deg effective steering in the previous 0.5 m/s circle.
STEER_GAIN=0.859
WHEELBASE=0.320
LATERAL_ACCEL=0.80

echo "[WARNING] REAL fixed-circle tests on /tianracer/ackermann_cmd."
echo "[PLAN] Seven approximately one-lap ${DIR:+left/right} circles; max diameter is 30.6 m."
echo "[PLAN] a_y target <= ${LATERAL_ACCEL} m/s^2; command steering <= 30 deg."

for speed in 0.5 1.0 1.5 2.0 2.5 3.0 3.5; do
  read -r steering_deg duration <<< "$(python3 - "$speed" "$DIR" "$STEER_GAIN" "$WHEELBASE" "$LATERAL_ACCEL" <<'PY'
import math, sys
v, direction, gain, wheelbase, ay = map(float, sys.argv[1:])
effective = math.atan(wheelbase * ay / (v * v))
command = min(math.radians(30.0), effective / gain) * direction
effective_realistic = min(abs(command) * gain, math.radians(25.8))
radius = wheelbase / math.tan(effective_realistic)
duration = 2.0 * math.pi * radius / v
print(f"{math.degrees(command):.4f} {duration:.2f}")
PY
)"
  tag="${speed/./p}"
  echo "[RUN] v=${speed} m/s steer=${steering_deg} deg duration=${duration}s"
  python3 "$ROOT/lateral_identification/run_fixed_circle.py" "${PREFIX}_v${tag}" \
    --speed "$speed" --steering-deg "$steering_deg" --duration "$duration"
  echo "[DONE] v=${speed}; zero command sent. Waiting 4s."
  sleep 4
done

echo "[COMPLETE] bags: $ROOT/lateral_identification/data/${PREFIX}_v*.bag"
