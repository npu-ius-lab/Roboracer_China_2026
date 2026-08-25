#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RAW_WS="${RAW_CLOUD_OBSTACLE_WS:-/home/tianbot/raw_cloud_obstacle_ws}"
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
BASE_CHECK="$ROOT/scripts/start_mpcc_hardware_stable_roboracer.sh"
RECORDER="$ROOT/scripts/record_v3react_bag.sh"
PACKAGE_DIR="$ROOT/src/roboracer_react_overtake"
MPCC_BINARY="$ROOT/devel/lib/f1tenth_dynamic_mpcc/mpcc_node_auto_relaunch_chaoche_cpp"
MPCC_CORE_LIBRARY="$ROOT/devel/lib/libf1tenth_mpcc_core.so"
MPCC_CHAOCHE_LIBRARY="$ROOT/devel/lib/libf1tenth_mpcc_acados_runtime_chaoche.so"
ACTION="${1:---shadow}"

usage() {
  echo "usage: $0 [--shadow|--static-test|--hardware|--check-only]" >&2
}

case "$ACTION" in
  --shadow|--static-test|--hardware|--check-only) ;;
  *) usage; exit 2 ;;
esac

required=(
  "$RAW_WS/devel/setup.bash"
  "$RAW_WS/start_raw_cloud_obstacle.sh"
  "$LOCALIZATION_WS/devel/setup.bash"
  "$ROOT/devel/setup.bash"
  "$BASE_CHECK"
  "$RECORDER"
  "$PACKAGE_DIR/config/react.yaml"
  "$PACKAGE_DIR/launch/roboracer_react_control.launch"
  "$MPCC_BINARY"
  "$MPCC_CORE_LIBRARY"
  "$MPCC_CHAOCHE_LIBRARY"
  "$ROOT/devel/lib/roboracer_react_overtake/roboracer_react_overtake_node"
  "$ROOT/devel/lib/roboracer_react_overtake/roboracer_react_minimum_speed_gate_node"
)
for path in "${required[@]}"; do
  [[ -e "$path" ]] || { echo "[FAIL] missing V3 React dependency: $path" >&2; exit 1; }
done

"$BASE_CHECK" --check-only
if grep -RInE 'targets_detailed|TargetArray|static_target|dynamic_target' \
    "$PACKAGE_DIR/src" "$PACKAGE_DIR/include" >/tmp/v3react_forbidden.$$; then
  echo "[FAIL] V3 React source contains a tracked-target dependency:" >&2
  sed -n '1,20p' /tmp/v3react_forbidden.$$ >&2
  rm -f /tmp/v3react_forbidden.$$
  exit 1
fi
rm -f /tmp/v3react_forbidden.$$
echo "[OK] React input is ROI occupancy only; motion/static classification is absent"

restore_nounset=false
case "$-" in *u*) restore_nounset=true ;; esac
set +u
source /opt/ros/noetic/setup.bash
source "$RAW_WS/devel/setup.bash" --extend
source "$ROOT/devel/setup.bash" --extend
if [[ "$restore_nounset" == true ]]; then set -u; fi
unset restore_nounset
source "$ROOT/scripts/ros_env.sh"
restore_nounset=false
case "$-" in *u*) restore_nounset=true ;; esac
set +u
source "$LOCALIZATION_WS/devel/setup.bash" --extend
if [[ "$restore_nounset" == true ]]; then set -u; fi
unset restore_nounset

abi_report="$(ldd -r "$MPCC_BINARY" 2>&1 || true)"
if grep -Eq 'not found|undefined symbol' <<<"$abi_report"; then
  echo "[FAIL] Chaoche MPCC executable and its shared libraries are ABI-incompatible:" >&2
  grep -E 'not found|undefined symbol' <<<"$abi_report" >&2
  echo "[HINT] rebuild f1tenth_dynamic_mpcc before starting V3 React" >&2
  exit 1
fi
for expected_library in "$MPCC_CORE_LIBRARY" "$MPCC_CHAOCHE_LIBRARY"; do
  if ! grep -Fq "=> $expected_library " <<<"$abi_report"; then
    echo "[FAIL] Chaoche MPCC resolved a library outside this workspace: $expected_library" >&2
    exit 1
  fi
done
unset abi_report expected_library
echo "[OK] Chaoche MPCC executable/library ABI and workspace linkage are valid"

python3 - <<'PY'
from point_lio_sam_lighterbev.msg import LocalizationStatus
print("[OK] LocalizationStatus Python message is available")
PY

if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] no ROS node, recorder, controller, or hardware publisher was started"
  exit 0
fi

rosnode list >/dev/null 2>&1 || {
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
}

node_alive() {
  rosnode list 2>/dev/null | grep -Fqx "$1" && rosnode ping -c 1 "$1" >/dev/null 2>&1
}

mapfile -t conflicting_nodes < <(
  rosnode list 2>/dev/null | \
    grep -E '^/(f1tenth_dynamic_mpcc|automatic_relaunch_supervisor|.*(chaoche|overtake|roboracer_react))' || true
)
for node in "${conflicting_nodes[@]}"; do
  if node_alive "$node"; then
    echo "[FAIL] conflicting control/planner node is already running: $node" >&2
    exit 1
  fi
done

pids=()
raw_owned=false
cleanup_started=false
cleanup() {
  local status=$?
  local pid
  trap - EXIT INT TERM
  if [[ "$cleanup_started" == true ]]; then
    exit "$status"
  fi
  cleanup_started=true
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! node_alive /raw_cloud_obstacle_perception; then
  mkdir -p "$ROOT/log"
  "$RAW_WS/start_raw_cloud_obstacle.sh" >"$ROOT/log/v3react_raw_cloud.log" 2>&1 &
  pids+=("$!")
  raw_owned=true
  for _ in $(seq 1 30); do
    node_alive /raw_cloud_obstacle_perception && break
    sleep 0.1
  done
  node_alive /raw_cloud_obstacle_perception || {
    echo "[FAIL] raw ROI node did not become ready" >&2
    exit 1
  }
fi

start_controller=false
allow_hardware=false
lap_timer=false
telemetry_path=""
if [[ "$ACTION" == "--static-test" || "$ACTION" == "--hardware" ]]; then
  start_controller=true
  export MPCC_GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer}"
  export RESIDUAL_DYNAMICS_WS="${RESIDUAL_DYNAMICS_WS:-$ROOT}"
  "$ROOT/scripts/ensure_acados_solvers.sh"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROOT/log" "$ROOT/bags"
if [[ "$ACTION" == "--hardware" ]]; then
  allow_hardware=true
  lap_timer=true
  telemetry_path="$ROOT/log/hardware_v3react_${STAMP}.jsonl"
fi

if [[ "${V3REACT_RECORD_BAG:-YES}" == "YES" ]]; then
  "$RECORDER" "$ROOT/bags/v3react_${STAMP}.bag" &
  pids+=("$!")
fi

echo "[V3REACT] mode=$ACTION raw_roi=/raw_cloud_perception/roi_cloud"
echo "[V3REACT] no target ID, speed, or motion/static classification"
echo "[V3REACT] red local path=/roboracer_react/local_plan_red"
echo "[V3REACT] driving-state minimum speed=1.0 m/s (safety stops preserved)"
echo "[V3REACT] controller=$start_controller hardware=$allow_hardware raw_owned=$raw_owned"

roslaunch roboracer_react_overtake roboracer_react_control.launch \
  start_controller:="$start_controller" \
  allow_real_hardware:="$allow_hardware" \
  lap_timer:="$lap_timer" \
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP:-4.0}" \
  generated_dir:="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer}" \
  telemetry_path:="$telemetry_path" \
  lap_log_path:="$ROOT/log/laps_v3react_${STAMP}.csv" \
  rviz:=false &
launch_pid=$!
pids+=("$launch_pid")
wait "$launch_pid"
