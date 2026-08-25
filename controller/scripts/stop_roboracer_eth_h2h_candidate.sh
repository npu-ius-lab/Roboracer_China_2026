#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/noetic/setup.bash
set -u
rosservice call /roboracer_eth_h2h_command_gate_cpp/set_armed "data: false" \
  >/dev/null 2>&1 || true
for node in \
    /roboracer_eth_h2h_command_gate_cpp \
    /f1tenth_dynamic_mpcc_roboracer_eth_h2h \
    /roboracer_eth_h2h_planner_cpp \
    /f1tenth_mpcc_lap_timer_roboracer_eth_h2h; do
  if rosnode list 2>/dev/null | grep -Fqx "$node"; then
    # Killing the required command gate makes roslaunch concurrently remove
    # the remaining candidate nodes. Treat that race as an already-stopped
    # success so rollback stays idempotent.
    rosnode kill "$node" >/dev/null 2>&1 || true
  fi
done
echo "[OK] ROBORACER ETH H2H candidate stopped; frozen ROBORACER was not touched"
