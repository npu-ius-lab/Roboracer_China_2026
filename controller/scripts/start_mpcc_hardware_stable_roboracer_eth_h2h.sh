#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="$ROOT/scripts/start_mpcc_hardware_stable_roboracer_eth_h2h_candidate.sh"
STOPPER="$ROOT/scripts/stop_roboracer_eth_h2h_candidate.sh"

for path in "$CANDIDATE" "$STOPPER"; do
  [[ -x "$path" ]] || { echo "[FAIL] missing executable: $path" >&2; exit 1; }
done

# Validate the frozen ROBORACER dependency and the C++ runtime before changing
# the running candidate mode.
"$CANDIDATE" --check-only

# Replace only our own shadow candidate. Never stop the frozen ROBORACER or any
# upstream perception/localization node here.
set +u
source /opt/ros/noetic/setup.bash
set -u

PERCEPTION_NODE=/raw_cloud_obstacle_perception
PERCEPTION_WS=/home/tianbot/raw_cloud_obstacle_ws
PERCEPTION_LOG="$ROOT/log/roboracer_eth_h2h_perception.log"
mkdir -p "$ROOT/log"
if ! rosnode list 2>/dev/null | grep -Fqx "$PERCEPTION_NODE"; then
  if [[ "${H2H_AUTOSTART_PERCEPTION:-true}" != "true" ]]; then
    echo "[FAIL] $PERCEPTION_NODE is not running" >&2
    exit 1
  fi
  [[ -f "$PERCEPTION_WS/devel/setup.bash" ]] || {
    echo "[FAIL] perception workspace is unavailable: $PERCEPTION_WS" >&2
    exit 1
  }
  echo "[ROBORACER_ETH_H2H] starting unchanged raw-cloud perception"
  setsid -f bash -lc \
    "source /opt/ros/noetic/setup.bash; source $PERCEPTION_WS/devel/setup.bash; exec roslaunch raw_cloud_obstacle_perception raw_cloud_obstacle.launch" \
    >>"$PERCEPTION_LOG" 2>&1
  for _ in $(seq 1 80); do
    if rosnode list 2>/dev/null | grep -Fqx "$PERCEPTION_NODE"; then
      break
    fi
    sleep 0.1
  done
  if ! rosnode list 2>/dev/null | grep -Fqx "$PERCEPTION_NODE"; then
    echo "[FAIL] perception node did not start; see $PERCEPTION_LOG" >&2
    exit 1
  fi
fi

if rosnode list 2>/dev/null | grep -Fqx /roboracer_eth_h2h_planner_cpp; then
  "$STOPPER"
  for _ in $(seq 1 30); do
    if ! rosnode list 2>/dev/null | grep -Fqx /roboracer_eth_h2h_planner_cpp; then
      break
    fi
    sleep 0.1
  done
fi

echo "[ROBORACER_ETH_H2H] direct hardware deployment"
echo "[ROBORACER_ETH_H2H] core=C++17, default cap=${H2H_SPEED_CAP_MPS:-4.0}m/s"
export H2H_AUTO_ARM="${H2H_AUTO_ARM:-true}"
if [[ "$H2H_AUTO_ARM" == "true" ]]; then
  echo "[ROBORACER_ETH_H2H] auto-arm enabled; gate opens only after 50 consecutive healthy frames"
else
  echo "[ROBORACER_ETH_H2H] auto-arm disabled; command gate remains DISARMED"
fi

exec "$CANDIDATE" --hardware
