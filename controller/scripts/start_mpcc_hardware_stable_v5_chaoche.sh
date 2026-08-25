#!/usr/bin/env bash
set -euo pipefail

MPCC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE_V5_CHECK="$MPCC_ROOT/scripts/start_mpcc_hardware_stable_v5.sh"
AVOIDANCE_LAUNCHER="$MPCC_ROOT/avoidance/v5_chaoche/scripts/start_v5_chaoche_avoidance.sh"
RECORDER="$MPCC_ROOT/scripts/record_v5_chaoche_bag.sh"
STATIC_CONTROL_CHECK="$MPCC_ROOT/scripts/check_v5_chaoche_hardware_control.py"
LAUNCH_FILE="$MPCC_ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v5_chaoche.launch"
CHAOCHE_NODE="$MPCC_ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch_chaoche.cpp"
CHAOCHE_SOLVER="$MPCC_ROOT/src/f1tenth_dynamic_mpcc/src/acados_runtime_chaoche.cpp"
TRACK="racelinev3_stable_v5_fast5_std32"
CONTROLLER_CONFIG="candidates/stable_v5/controller.yaml"
VEHICLE_CONFIG="candidates/stable_v5/vehicle.yaml"
RESIDUAL="$MPCC_ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
ACTION="${1:---hardware}"

# Keep the old internal spelling usable, but expose one unambiguous real-car
# entry point to operators.
if [[ "$ACTION" == "mpcc" ]]; then
  ACTION="--hardware"
fi

check_candidate() {
  "$STABLE_V5_CHECK" --check-only
  "$AVOIDANCE_LAUNCHER" --check-only
  local path
  for path in "$RECORDER" "$STATIC_CONTROL_CHECK" "$LAUNCH_FILE" "$CHAOCHE_NODE" "$CHAOCHE_SOLVER"; do
    if [[ ! -f "$path" ]]; then
      echo "[FAIL] missing V5 Chaoche file: $path" >&2
      return 1
    fi
  done
  python3 "$STATIC_CONTROL_CHECK" --workspace "$MPCC_ROOT"
  python3 - <<'PY' "$LAUNCH_FILE" "$CHAOCHE_NODE"
import pathlib
import sys
import xml.etree.ElementTree as ET

launch = pathlib.Path(sys.argv[1])
node = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8")
ET.parse(str(launch))
launch_text = launch.read_text(encoding="utf-8")
assert "mpcc_node_auto_relaunch_chaoche_cpp" in launch_text
assert "/v5_chaoche/mpcc/ackermann_cmd_stamped" in launch_text
assert "localReferenceCallback" in node
assert "no Pure Pursuit fallback" in node
print("[OK] V5 Chaoche launch XML and MPCC-local-reference contract")
PY
  echo "[OK] Stable V5 is hash-verified and remains unchanged"
}

if [[ "$ACTION" == "--check-only" || "$ACTION" == "--static-test" ]]; then
  check_candidate
  echo "[OK] static test only; no ROS node started and no hardware command published"
  exit 0
fi
if [[ "$ACTION" == "--prepare-only" ]]; then
  check_candidate
  "$STABLE_V5_CHECK" --prepare-only
  echo "[OK] V5 Chaoche generated solver is ready; no ROS node was started"
  exit 0
fi
if [[ "$ACTION" != "--shadow" && "$ACTION" != "--hardware" ]]; then
  echo "usage: $0 [--check-only|--static-test|--prepare-only|--shadow|--hardware]" >&2
  exit 2
fi

allow_hardware=false
if [[ "$ACTION" == "--hardware" ]]; then
  if [[ "${V5_CHAOCHE_ALLOW_REAL_HARDWARE:-YES}" != "YES" ]]; then
    echo "[FAIL] real hardware was explicitly disabled by V5_CHAOCHE_ALLOW_REAL_HARDWARE" >&2
    exit 1
  fi
  allow_hardware=true
fi

check_candidate
if [[ ! -x "$MPCC_ROOT/devel/lib/f1tenth_dynamic_mpcc/mpcc_node_auto_relaunch_chaoche_cpp" ]]; then
  echo "[FAIL] V5 Chaoche executable is not built; run $0 --prepare-only after catkin_make" >&2
  exit 1
fi

restore_nounset=false
case "$-" in
  *u*) restore_nounset=true ;;
esac
set +u
source "$MPCC_ROOT/scripts/ros_env.sh"
source /home/tianbot/localization_main/devel/setup.bash --extend
if [[ "$restore_nounset" == true ]]; then
  set -u
fi
unset restore_nounset
export PYTHONPATH="$MPCC_ROOT/avoidance/python:/home/tianbot/perception/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

generated_dir="${MPCC_GENERATED_DIR:-/home/tianbot/.cache/f1tenth_residual_mpcc/acados_stable_v5}"
speed_cap="${RESIDUAL_MPCC_SPEED_CAP:-5.0}"
if ! awk -v value="$speed_cap" 'BEGIN { exit !(value > 0.0 && value <= 5.0) }'; then
  echo "[FAIL] RESIDUAL_MPCC_SPEED_CAP must be in (0, 5.0]" >&2
  exit 2
fi

mkdir -p "$MPCC_ROOT/bags" "$MPCC_ROOT/log"
stamp="$(date +%Y%m%d_%H%M%S)"
pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$AVOIDANCE_LAUNCHER" --foreground &
pids+=("$!")
for _ in {1..50}; do
  if rosnode list 2>/dev/null | grep -qx '/v5_chaoche_reference_manager'; then
    break
  fi
  sleep 0.1
done
if ! rosnode list 2>/dev/null | grep -qx '/v5_chaoche_reference_manager'; then
  echo "[FAIL] V5 Chaoche reference manager did not start" >&2
  exit 1
fi

if [[ "${V5_CHAOCHE_RECORD_BAG:-true}" == "true" ]]; then
  bag_path="${V5_CHAOCHE_BAG_PATH:-$MPCC_ROOT/bags/v5_chaoche_${stamp}.bag}"
  "$RECORDER" "$bag_path" &
  pids+=("$!")
  echo "[RECORD] V5 Chaoche analysis bag -> $bag_path"
fi

echo "[V5_CHAOCHE] Stable V5 files: frozen and hash-verified"
echo "[V5_CHAOCHE] local avoidance path is locked through PREPARE/PASS/RETURN/ABORT"
echo "[V5_CHAOCHE] active maneuver controller=MPCC; Pure Pursuit candidate is disabled"
echo "[V5_CHAOCHE] global MPCC resumes only after centered + no-obstacle confirmation"
if [[ "$allow_hardware" == false ]]; then
  echo "[SHADOW] hardware supervisor is disabled; no /tianracer/ackermann_cmd publisher is added"
else
  echo "[HARDWARE] explicit hardware gate accepted; physical E-stop must remain available"
fi

roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v5_chaoche.launch \
  track:="$TRACK" \
  controller_config:="$CONTROLLER_CONFIG" \
  vehicle_config:="$VEHICLE_CONFIG" \
  formulation:=mpcc \
  allow_real_hardware:="$allow_hardware" \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  lap_timer:="$allow_hardware" \
  runtime_speed_cap_mps:="$speed_cap" \
  generated_dir:="$generated_dir" \
  residual_model_path:="$RESIDUAL" \
  residual_feature_set:=markov_v1 \
  telemetry_path:="$MPCC_ROOT/log/v5_chaoche_${stamp}.jsonl" \
  lap_log_path:="$MPCC_ROOT/log/laps_v5_chaoche_${stamp}.csv"
