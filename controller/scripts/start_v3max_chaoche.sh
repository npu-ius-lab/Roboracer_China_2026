#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_START="$ROOT/scripts/start_mpcc_hardware_stable_roboracer.sh"
AVOIDANCE_START="$ROOT/avoidance/roboracer_chaoche/scripts/start_roboracer_chaoche_avoidance.sh"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_chaoche.launch"
RECORDER="$ROOT/scripts/record_v3max_chaoche_bag.sh"
# This dedicated real-car entry point defaults to hardware mode.  Offline
# checks remain explicit so an operator can still validate the stack without
# creating a chassis-command publisher.
ACTION="${1:---hardware}"

"$BASE_START" --check-only
"$AVOIDANCE_START" --check-only

for path in "$LAUNCH" "$RECORDER"; do
  [[ -f "$path" ]] || { echo "[FAIL] missing V3 Pro Max Chaoche file: $path" >&2; exit 1; }
done
python3 - "$LAUNCH" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
args = {x.attrib["name"]: x.attrib.get("default", "") for x in root.findall("arg")}
assert args.get("allow_real_hardware") == "false"
nodes = root.findall("node")
controller = [n for n in nodes if n.attrib.get("type") == "mpcc_node_auto_relaunch_chaoche_cpp"]
assert len(controller) == 1
assert controller[0].attrib.get("name") == "f1tenth_dynamic_mpcc_roboracer_chaoche"
supervisors = [n for n in nodes if n.attrib.get("type") == "automatic_relaunch_supervisor_stable_roboracer.py"]
assert len(supervisors) == 1 and supervisors[0].attrib.get("if") == "$(arg allow_real_hardware)"
print("[OK] additive launch: V3 Pro Max inputs + isolated local-reference controller")
PY

if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] frozen V3 Pro Max files were verified by their original launcher"
  echo "[OK] no ROS node or publisher was started"
  exit 0
fi
if [[ "$ACTION" != "--static-test" && "$ACTION" != "--hardware" ]]; then
  echo "usage: $0 [--hardware|--static-test|--check-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer}"
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
source "$ROOT/scripts/ros_env.sh"
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
restore_nounset=false
case "$-" in *u*) restore_nounset=true ;; esac
set +u
source "$LOCALIZATION_WS/devel/setup.bash" --extend
if [[ "$restore_nounset" == true ]]; then set -u; fi
unset restore_nounset
export PYTHONPATH="$ROOT/src/f1tenth_dynamic_mpcc/python${PYTHONPATH:+:$PYTHONPATH}"

export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="raceline_smooth"
export MPCC_CONTROLLER_CONFIG="candidates/stable_roboracer/controller.yaml"
export MPCC_VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
export MPCC_RESIDUAL_MODEL_PATH="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
export MPCC_COST_CONTOUR_OVERRIDE="45.0"
export MPCC_COST_HEADING_RACE_OVERRIDE="1.0"
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="0.60"
"$ROOT/scripts/ensure_acados_solvers.sh"

if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
fi
for node in /f1tenth_dynamic_mpcc /f1tenth_dynamic_mpcc_v5_chaoche \
    /f1tenth_dynamic_mpcc_roboracer_chaoche; do
  if rosnode list 2>/dev/null | grep -Fqx "$node"; then
    echo "[FAIL] another MPCC controller is already registered: $node" >&2
    exit 1
  fi
done

STAMP="$(date +%Y%m%d_%H%M%S)"
pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$AVOIDANCE_START" --foreground &
pids+=("$!")

allow_hardware=false
lap_timer=false
telemetry_path=""
if [[ "$ACTION" == "--hardware" ]]; then
  allow_hardware=true
  lap_timer=true
  mkdir -p "$ROOT/log" "$ROOT/bags"
  telemetry_path="$ROOT/log/hardware_roboracer_chaoche_${STAMP}.jsonl"
  "$RECORDER" "$ROOT/bags/roboracer_chaoche_${STAMP}.bag" &
  pids+=("$!")
fi

echo "[V3MAX_CHAOCHE] mode=$ACTION base=raceline_smooth Stable V3 Pro Max"
echo "[V3MAX_CHAOCHE] avoidance red path=/roboracer_chaoche/local_plan_red"
echo "[V3MAX_CHAOCHE] allow_real_hardware=$allow_hardware"
roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_roboracer_chaoche.launch \
  track:=raceline_smooth \
  controller_config:=candidates/stable_roboracer/controller.yaml \
  vehicle_config:=vehicle_stable_v3_racelineV3.yaml \
  formulation:=mpcc \
  allow_real_hardware:="$allow_hardware" \
  rviz:=false \
  lap_timer:="$lap_timer" \
  runtime_speed_cap_mps:="${RESIDUAL_MPCC_SPEED_CAP:-4.0}" \
  generated_dir:="$GENERATED_DIR" \
  residual_model_path:="$MPCC_RESIDUAL_MODEL_PATH" \
  residual_feature_set:=markov_v1 \
  telemetry_path:="$telemetry_path" \
  lap_log_path:="$ROOT/log/laps_roboracer_chaoche_${STAMP}.csv"
