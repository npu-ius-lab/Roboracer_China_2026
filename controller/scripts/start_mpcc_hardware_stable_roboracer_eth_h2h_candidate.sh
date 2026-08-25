#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_START="$ROOT/scripts/start_mpcc_hardware_stable_roboracer.sh"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_eth_h2h_candidate.launch"
RUNTIME_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_roboracer_eth_h2h/cpp_runtime.yaml"
RECORDER="$ROOT/scripts/record_roboracer_eth_h2h_bag.sh"
ACTION="${1:---shadow}"

case "$ACTION" in
  --check-only|--shadow|--static-test|--hardware) ;;
  *) echo "usage: $0 [--check-only|--shadow|--static-test|--hardware]" >&2; exit 2 ;;
esac

for path in "$BASE_START" "$LAUNCH" "$RUNTIME_CONFIG" "$RECORDER"; do
  [[ -f "$path" ]] || { echo "[FAIL] missing ETH H2H dependency: $path" >&2; exit 1; }
done

# Prove that this candidate references, rather than changes, the frozen base.
"$BASE_START" --check-only
grep -Fq 'type="v5_jubu_local_planner_cpp"' "$LAUNCH"
grep -Fq 'type="mpcc_node_auto_relaunch_jubu_cpp"' "$LAUNCH"
grep -Fq 'type="v5_jubu_command_gate_cpp"' "$LAUNCH"
if grep -R -nE 'overtake_planner\.py|state_visualizer\.py|reference_manager\.py|target_tracker\.py' \
    "$LAUNCH" "$RUNTIME_CONFIG"; then
  echo "[FAIL] Python overtaking core referenced by the C++ candidate" >&2
  exit 1
fi

if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] additive ROBORACER ETH H2H candidate"
  echo "[OK] overtaking core=C++17; no node or command publisher started"
  exit 0
fi

source "$ROOT/scripts/ros_env.sh"
if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
fi

topic_has_publisher() {
  rostopic info "$1" 2>/dev/null \
    | sed -n '/^Publishers:/,/^Subscribers:/p' \
    | grep -q '^ \* '
}

require_live_topic() {
  local topic="$1" expected_type="$2" wait_s="$3"
  local actual_type
  actual_type="$(rostopic type "$topic" 2>/dev/null || true)"
  if [[ "$actual_type" != "$expected_type" ]]; then
    echo "[FAIL] $topic type=$actual_type; expected $expected_type" >&2
    return 1
  fi
  if ! topic_has_publisher "$topic"; then
    echo "[FAIL] $topic has no publisher" >&2
    return 1
  fi
  if ! timeout "$wait_s" rostopic echo -n 1 --noarr "$topic" \
      >/dev/null 2>&1; then
    echo "[FAIL] $topic produced no fresh message within ${wait_s}s" >&2
    return 1
  fi
  echo "[OK] live input $topic ($expected_type)"
}

require_live_topic /localization/odom nav_msgs/Odometry 4
require_live_topic /raw_cloud_perception/obstacle_cloud sensor_msgs/PointCloud2 6

start_controller=false
controller_start_enabled=false
allow_hardware=false
lap_timer=false
case "$ACTION" in
  --shadow)
    ;;
  --static-test)
    start_controller=true
    controller_start_enabled=true
    ;;
  --hardware)
    start_controller=true
    allow_hardware=true
    lap_timer=true
    ;;
esac

if [[ "$start_controller" == true ]]; then
  for node in /f1tenth_dynamic_mpcc /f1tenth_dynamic_mpcc_roboracer_chaoche \
      /f1tenth_dynamic_mpcc_v5_jubu /f1tenth_dynamic_mpcc_roboracer_eth_h2h; do
    if rosnode list 2>/dev/null | grep -Fqx "$node"; then
      echo "[FAIL] another MPCC controller is registered: $node" >&2
      exit 1
    fi
  done

  GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer_eth_h2h}"
  if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
     && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
    export RESIDUAL_DYNAMICS_WS="$ROOT"
  fi
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
else
  GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer_eth_h2h}"
  MPCC_RESIDUAL_MODEL_PATH="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

auto_arm_when_healthy() {
  local service=/roboracer_eth_h2h_command_gate_cpp/set_armed
  local sample true_count false_count response
  for _ in $(seq 1 300); do
    if rosservice list 2>/dev/null | grep -Fqx "$service"; then
      break
    fi
    sleep 0.1
  done
  if ! rosservice list 2>/dev/null | grep -Fqx "$service"; then
    echo "[AUTO_ARM][FAIL] command-gate service did not appear" >&2
    return 1
  fi
  for _ in $(seq 1 20); do
    sample="$(timeout 5 rostopic echo -n 50 /roboracer_eth_h2h/health \
      2>/dev/null || true)"
    true_count="$(grep -c '^data: True' <<<"$sample" || true)"
    false_count="$(grep -c '^data: False' <<<"$sample" || true)"
    if [[ "$true_count" -eq 50 && "$false_count" -eq 0 ]]; then
      response="$(rosservice call "$service" "data: true" 2>&1 || true)"
      if grep -Fq 'success: True' <<<"$response"; then
        echo "[AUTO_ARM][OK] 50 consecutive healthy frames; C++ gate armed"
        return 0
      fi
      echo "[AUTO_ARM][WARN] gate rejected arm request: $response" >&2
    fi
    sleep 0.5
  done
  echo "[AUTO_ARM][FAIL] planner health was not continuously true; gate remains DISARMED" >&2
  return 1
}

telemetry_path=""
lap_log_path=""
if [[ "$ACTION" == "--hardware" ]]; then
  mkdir -p "$ROOT/log" "$ROOT/bags"
  telemetry_path="$ROOT/log/hardware_roboracer_eth_h2h_${STAMP}.jsonl"
  lap_log_path="$ROOT/log/laps_roboracer_eth_h2h_${STAMP}.csv"
  if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
    "$RECORDER" "$ROOT/bags/roboracer_eth_h2h_${STAMP}.bag" &
    pids+=("$!")
  fi
  if [[ "${H2H_AUTO_ARM:-false}" == "true" ]]; then
    auto_arm_when_healthy &
    pids+=("$!")
  fi
fi

echo "[ROBORACER_ETH_H2H] mode=$ACTION core=C++17 cap=${H2H_SPEED_CAP_MPS:-4.0}m/s"
echo "[ROBORACER_ETH_H2H] upstream perception/localization unchanged"
if [[ "$ACTION" == "--hardware" ]]; then
  echo "[ROBORACER_ETH_H2H] hardware gate starts DISARMED"
  if [[ "${H2H_AUTO_ARM:-false}" == "true" ]]; then
    echo "[ROBORACER_ETH_H2H] auto-arm waits for 50 consecutive healthy frames"
  else
    echo "[ROBORACER_ETH_H2H] arm only after health=true:"
    echo "  rosservice call /roboracer_eth_h2h_command_gate_cpp/set_armed 'data: true'"
  fi
fi

roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_roboracer_eth_h2h_candidate.launch \
  track:=raceline_smooth \
  controller_config:=candidates/stable_roboracer/controller.yaml \
  vehicle_config:=vehicle_stable_v3_racelineV3.yaml \
  runtime_config:="$RUNTIME_CONFIG" \
  formulation:=mpcc \
  start_planner:=true \
  start_controller:="$start_controller" \
  controller_start_enabled:="$controller_start_enabled" \
  allow_real_hardware:="$allow_hardware" \
  rviz:=false \
  lap_timer:="$lap_timer" \
  runtime_speed_cap_mps:="${H2H_SPEED_CAP_MPS:-4.0}" \
  generated_dir:="$GENERATED_DIR" \
  residual_model_path:="$MPCC_RESIDUAL_MODEL_PATH" \
  residual_feature_set:=markov_v1 \
  telemetry_path:="$telemetry_path" \
  lap_log_path:="$lap_log_path"
