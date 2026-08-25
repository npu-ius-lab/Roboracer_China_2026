#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESIDUAL_WS="${RESIDUAL_DYNAMICS_WS:-$(dirname "$ROOT")/f1tenth_residual_ws}"
RUN_ID="${1:-residual_mpcc_3mps_$(date +%Y%m%d_%H%M%S)}"

if [[ ! "$RUN_ID" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "[FAIL] run id may contain only letters, numbers, dot, underscore, and dash" >&2
  exit 2
fi
if [[ ! -x "$ROOT/scripts/start_mpcc_hardware_3mps.sh" ]]; then
  echo "[FAIL] missing 3 m/s launcher: $ROOT/scripts/start_mpcc_hardware_3mps.sh" >&2
  exit 1
fi
if [[ ! -x "$RESIDUAL_WS/scripts/record_dataset.sh" ]]; then
  echo "[FAIL] missing residual recorder: $RESIDUAL_WS/scripts/record_dataset.sh" >&2
  exit 1
fi

source "$ROOT/scripts/ros_env.sh"
if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] ROS master unavailable at ${ROS_MASTER_URI:-unset}" >&2
  exit 1
fi
for node in /f1tenth_dynamic_mpcc /f1tenth_mpcc_command_gate /f1tenth_mpcc_telemetry_logger; do
  if rosnode list 2>/dev/null | grep -qx "$node"; then
    echo "[FAIL] $node is already running; stop the previous MPCC first." >&2
    exit 1
  fi
done

RUN_DIR="$RESIDUAL_WS/data/bags/$RUN_ID"
RECORDER_PID=""
CONTROLLER_PID=""

cleanup() {
  local status=$?
  trap - INT TERM EXIT
  echo "[STOP] Disabling controller and hardware command gate..."
  rosservice call /f1tenth_dynamic_mpcc/set_enabled "data: false" >/dev/null 2>&1 || true
  rosservice call /f1tenth_mpcc_command_gate/set_enabled "data: false" >/dev/null 2>&1 || true
  if [[ -n "$CONTROLLER_PID" ]] && kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    kill -INT "$CONTROLLER_PID" 2>/dev/null || true
  fi
  rosnode kill /f1tenth_dynamic_mpcc /f1tenth_mpcc_command_gate \
    /f1tenth_mpcc_telemetry_logger >/dev/null 2>&1 || true
  if [[ -n "$CONTROLLER_PID" ]]; then
    for _ in {1..50}; do
      kill -0 "$CONTROLLER_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$CONTROLLER_PID" 2>/dev/null; then
      kill -TERM "$CONTROLLER_PID" 2>/dev/null || true
    fi
    wait "$CONTROLLER_PID" 2>/dev/null || true
  fi
  if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
  fi
  if [[ -f "$RUN_DIR/$RUN_ID.bag" ]]; then
    echo "[SAVED] $RUN_DIR/$RUN_ID.bag"
  elif [[ -f "$RUN_DIR/$RUN_ID.bag.active" ]]; then
    echo "[WARN] bag is still active/incomplete: $RUN_DIR/$RUN_ID.bag.active" >&2
  fi
  exit "$status"
}
trap cleanup INT TERM EXIT

echo "[WARNING] This starts 3 m/s residual MPCC and publishes REAL vehicle commands."
echo "[WARNING] Keep the emergency stop ready. Press Ctrl-C once to stop and finalize the bag."
echo "[DATASET] $RUN_DIR/$RUN_ID.bag"

MPCC_ROOT="$ROOT" "$RESIDUAL_WS/scripts/record_dataset.sh" "$RUN_ID" &
RECORDER_PID=$!
for _ in {1..100}; do
  if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
    wait "$RECORDER_PID" || true
    echo "[FAIL] residual recorder exited during startup" >&2
    exit 1
  fi
  if [[ -f "$RUN_DIR/$RUN_ID.bag.active" ]]; then
    break
  fi
  sleep 0.1
done
if [[ ! -f "$RUN_DIR/$RUN_ID.bag.active" ]]; then
  echo "[FAIL] rosbag did not become ready within 10 seconds" >&2
  exit 1
fi

"$ROOT/scripts/start_mpcc_hardware_3mps.sh" &
CONTROLLER_PID=$!
for _ in {1..150}; do
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    wait "$CONTROLLER_PID" || true
    echo "[FAIL] MPCC launcher exited during startup" >&2
    exit 1
  fi
  if rosnode list 2>/dev/null | grep -qx /f1tenth_dynamic_mpcc; then
    break
  fi
  sleep 0.1
done
if ! rosnode list 2>/dev/null | grep -qx /f1tenth_dynamic_mpcc; then
  echo "[FAIL] MPCC node did not register within 15 seconds" >&2
  exit 1
fi

echo "[READY] 3 m/s MPCC and residual-dynamics bag recording are running."
echo "[READY] Drive several clean laps, then press Ctrl-C once."
while kill -0 "$CONTROLLER_PID" 2>/dev/null; do
  if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "[FAIL] rosbag recorder exited unexpectedly" >&2
    exit 1
  fi
  sleep 1
done
wait "$CONTROLLER_PID" || true
