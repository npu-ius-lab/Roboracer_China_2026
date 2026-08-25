#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="v5_jubu"
TRACK="racelinev3_stable_v5_fast5_std32"
CONTROLLER_CONFIG="candidates/$CANDIDATE/controller.yaml"
VEHICLE_CONFIG="candidates/$CANDIDATE/vehicle.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v5_jubu.launch"
TRACK_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
SPEED_ZONES="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/speed_zones.csv"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MANIFEST="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/$CANDIDATE/MANIFEST.sha256"
RECORDER="$ROOT/scripts/record_v5_jubu_bag.sh"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"

check_hash() {
  local label="$1"
  local expected="$2"
  local path="$3"
  local actual
  if [[ ! -f "$path" ]]; then
    echo "[FAIL] missing $label: $path" >&2
    exit 1
  fi
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] $label changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

check_release_files() {
  # Prove that the frozen V5 release remains byte-for-byte unchanged.
  check_hash "Stable V5 controller" \
    "40b89b20179510642a3c1f7c649deb92d8ae487e6d3f3241a0f158466611da41" \
    "$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_v5/controller.yaml"
  check_hash "Stable V5 vehicle" \
    "fd5f9d91bc1fe967d7429cb12f42d982e8527d0d8e559f41dbefd8f2cf24366d" \
    "$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/stable_v5/vehicle.yaml"
  check_hash "Stable V5 raceline" \
    "88ca2acf725207a68d0ea20aa9e26d8af93dcff027afd235b9ce2045f866b82a" \
    "$TRACK_CSV"
  check_hash "Stable V5 speed zones" \
    "3ace10062380356525c99d02e07d61f125c971b476594bdd4c26c2da6f250132" \
    "$SPEED_ZONES"
  check_hash "Stable V5 launch" \
    "a258e650d0a66192a0eae5256267159f1786e14bffe55cd745098e88bbaa68ca" \
    "$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v5.launch"
  check_hash "Stable V5 MPCC source" \
    "344bc7003ff491ff082351a2fb6979a608b9ead2df0238a0cee6c0d1b911bde0" \
    "$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch.cpp"
  check_hash "Stable V5 supervisor" \
    "f9996adfb6c59f66a2ccaf7522f40825e2c74a9e3370c7f8ffc5e4fa1a22545a" \
    "$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor.py"
  check_hash "Stable V5 residual" \
    "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" \
    "$RESIDUAL"

  # The pre-existing remote overtake implementation is a read-only reference.
  check_hash "existing overtake node" \
    "9f0c405612d0204525fc83c44f681dfe297f7e1a50474a4c375fef1103d8875c" \
    "$ROOT/avoidance/scripts/overtake_shadow_node.py"
  check_hash "existing overtake core" \
    "42589a38318a39cca8d75a5a00d919a69ea51f1031ed89ba1ef4c0f10f581d80" \
    "$ROOT/avoidance/python/f1tenth_overtake/core.py"
  check_hash "existing overtake config" \
    "a3b46b93f7bc2551b3804c926a459f95b24c24aa8265ac30cc1673700a860254" \
    "$ROOT/avoidance/config/overtake_hardware.yaml"

  check_hash "V5 JUBU manifest" \
    "f148f21d1cf56f54670abed8d29909e8a657158ae0a2ae6b795d07dd93f35bd5" \
    "$MANIFEST"
  (cd "$ROOT" && sha256sum --check "$MANIFEST")

  if grep -R -nE 'AckermannDrive|/tianracer/ackermann_cmd|ackermann_cmd_stamped' \
      "$ROOT/src/f1tenth_dynamic_mpcc/src/v5_jubu_local_planner.cpp" \
      "$ROOT/src/f1tenth_dynamic_mpcc/src/v5_jubu_roi.cpp" \
      "$ROOT/src/f1tenth_dynamic_mpcc/include/f1tenth_dynamic_mpcc/v5_jubu_roi.hpp" \
      >/tmp/v5_jubu_forbidden_command_interfaces.txt; then
    echo "[FAIL] V5 JUBU planner contains a chassis-command interface" >&2
    sed -n '1,20p' /tmp/v5_jubu_forbidden_command_interfaces.txt >&2
    exit 1
  fi
}

source_extend() {
  local setup="$1"
  local restore_nounset=false
  case "$-" in
    *u*) restore_nounset=true ;;
  esac
  set +u
  source "$setup" --extend
  if [[ "$restore_nounset" == true ]]; then
    set -u
  fi
}

setup_environment() {
  if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
     && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
    export RESIDUAL_DYNAMICS_WS="$ROOT"
  fi
  source "$ROOT/scripts/ros_env.sh"
  [[ -f "$LOCALIZATION_WS/devel/setup.bash" ]] || {
    echo "[FAIL] localization setup missing: $LOCALIZATION_WS/devel/setup.bash" >&2
    exit 1
  }
  [[ -f "$PERCEPTION_WS/devel/setup.bash" ]] || {
    echo "[FAIL] perception setup missing: $PERCEPTION_WS/devel/setup.bash" >&2
    exit 1
  }
  source_extend "$LOCALIZATION_WS/devel/setup.bash"
  source_extend "$PERCEPTION_WS/devel/setup.bash"
}

check_runtime_interfaces() {
  local executable
  for executable in \
      mpcc_node_auto_relaunch_jubu_cpp \
      v5_jubu_local_planner_cpp \
      v5_jubu_command_gate_cpp \
      v5_jubu_roi_test; do
    [[ -x "$ROOT/devel/lib/f1tenth_dynamic_mpcc/$executable" ]] || {
      echo "[FAIL] V5 JUBU C++ executable is not built: $executable" >&2
      exit 1
    }
  done
  "$ROOT/devel/lib/f1tenth_dynamic_mpcc/v5_jubu_roi_test"
  if grep -nE 'type="[^"]*\.py"' "$LAUNCH"; then
    echo "[FAIL] V5 JUBU live launch graph contains a Python node" >&2
    exit 1
  fi
  roslaunch --nodes f1tenth_dynamic_mpcc hardware_mpcc_stable_v5_jubu.launch \
    start_planner:=true start_controller:=true allow_real_hardware:=false \
    >/dev/null
  echo "[OK] V5 JUBU C++ test, launch graph and executables are ready"
}

ACTION="${1:-mpcc}"
case "$ACTION" in
  mpcc|--check-only|--prepare-only|--planner-only) ;;
  *)
    echo "usage: $0 [mpcc|--check-only|--prepare-only|--planner-only]" >&2
    exit 2
    ;;
esac

check_release_files
setup_environment
check_runtime_interfaces

if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] Stable V5 and the existing overtake version are unchanged"
  echo "[OK] V5 JUBU is isolated and fail-closed on planner/perception loss"
  exit 0
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_v5_jubu}"
export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="$TRACK"
export MPCC_CONTROLLER_CONFIG="$CONTROLLER_CONFIG"
export MPCC_VEHICLE_CONFIG="$VEHICLE_CONFIG"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
export MPCC_COST_CONTOUR_OVERRIDE="45.0"
export MPCC_COST_HEADING_RACE_OVERRIDE="1.0"
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="0.60"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$ACTION" == "--prepare-only" ]]; then
  echo "[OK] V5 JUBU solver is ready; hardware was not started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

require_localization_rate_param() {
  local name="$1"
  local expected="$2"
  local value
  value="$(rosparam get "$name" 2>/dev/null || true)"
  if ! awk -v value="$value" -v expected="$expected" \
    'BEGIN { exit !(value != "" && (value-expected < 1e-6) && (expected-value < 1e-6)) }'; then
    echo "[FAIL] localization frequency profile mismatch: $name=$value expected=$expected" >&2
    exit 1
  fi
}

require_localization_rate_param "/laserMapping/odometry/max_publish_hz" "60.0"
require_localization_rate_param "/pointlio_robust_localizer/control_output_hz" "0.0"
require_localization_rate_param "/pointlio_robust_localizer/backend_hz" "5.0"

V5_JUBU_SPEED_CAP="${V5_JUBU_SPEED_CAP:-3.0}"
if ! awk -v value="$V5_JUBU_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 3.0) }'; then
  echo "[FAIL] V5_JUBU_SPEED_CAP must be in (0, 3.0]" >&2
  exit 2
fi

if rosnode list 2>/dev/null | grep -Eq \
  '^/(f1tenth_dynamic_mpcc[^/]*|automatic_relaunch_supervisor[^/]*|v5_jubu_command_gate_cpp)$'; then
  echo "[FAIL] another MPCC or hardware supervisor is already running" >&2
  exit 1
fi

HARDWARE_COMMAND_TOPIC="${AUTO_RELAUNCH_HARDWARE_COMMAND_TOPIC:-/tianracer/ackermann_cmd}"
if rostopic info "$HARDWARE_COMMAND_TOPIC" 2>/dev/null \
    | sed -n '/Publishers:/,/Subscribers:/p' \
    | grep -Eq '^ \* /'; then
  echo "[FAIL] $HARDWARE_COMMAND_TOPIC already has a publisher" >&2
  echo "       Stop the active controller before starting V5 JUBU." >&2
  exit 1
fi

mkdir -p "$ROOT/log" "$ROOT/bags"
STAMP="$(date +%Y%m%d_%H%M%S)"
PERCEPTION_PID=""
PLANNER_PID=""
CONTROL_PID=""
RECORD_PID=""

cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "$CONTROL_PID" "$PLANNER_PID" "$RECORD_PID" "$PERCEPTION_PID"; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$CONTROL_PID" "$PLANNER_PID" "$RECORD_PID" "$PERCEPTION_PID"; do
    if [[ "$pid" =~ ^[0-9]+$ ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

roi_cloud_has_publisher() {
  rostopic info /perception/roi_cloud 2>/dev/null \
    | sed -n '/Publishers:/,/Subscribers:/p' \
    | grep -Eq '^ \* /'
}

if roi_cloud_has_publisher; then
  echo "[V5_JUBU] reusing existing /perception/roi_cloud publisher"
else
  "$PERCEPTION_WS/start_perception_car.sh" --master http://127.0.0.1:11311 \
    >"$ROOT/log/v5_jubu_perception_${STAMP}.log" 2>&1 &
  PERCEPTION_PID=$!
  echo "[V5_JUBU] started perception pid=$PERCEPTION_PID"
fi

for _ in {1..100}; do
  if roi_cloud_has_publisher; then
    break
  fi
  sleep 0.2
done
if ! roi_cloud_has_publisher; then
  echo "[FAIL] /perception/roi_cloud has no opponent_perception publisher" >&2
  exit 1
fi

roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v5_jubu.launch \
  track:="$TRACK" controller_config:="$CONTROLLER_CONFIG" \
  vehicle_config:="$VEHICLE_CONFIG" start_planner:=true \
  start_controller:=false allow_real_hardware:=false rviz:=false \
  >"$ROOT/log/v5_jubu_planner_${STAMP}.log" 2>&1 &
PLANNER_PID=$!

HEALTH_SAMPLE=""
for _ in {1..100}; do
  # On this ROS Noetic target, `rostopic echo -n 1` can print the latched
  # Bool and still remain alive until timeout.  Capture its output and ignore
  # that timeout status; with pipefail enabled, piping it directly to grep
  # otherwise turns a healthy True sample into a failed pipeline.
  HEALTH_SAMPLE="$(
    timeout 1 rostopic echo -n 1 /v5_jubu/health 2>/dev/null || true
  )"
  if grep -q 'data: True' <<<"$HEALTH_SAMPLE"; then
    break
  fi
  if ! kill -0 "$PLANNER_PID" 2>/dev/null; then
    echo "[FAIL] V5 JUBU planner exited; see log/v5_jubu_planner_${STAMP}.log" >&2
    exit 1
  fi
  sleep 0.1
done
if ! grep -q 'data: True' <<<"$HEALTH_SAMPLE"; then
  echo "[FAIL] V5 JUBU planner health did not become true; hardware remains untouched" >&2
  exit 1
fi
echo "[OK] V5 JUBU C++ planner, odometry and ROI-cloud streams are healthy"

if [[ "$ACTION" == "--planner-only" ]]; then
  echo "[V5_JUBU] planner-only mode; no hardware command publisher was started"
  wait "$PLANNER_PID"
  exit $?
fi

if [[ "${MPCC_RECORD_BAG:-true}" == "true" ]]; then
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/v5_jubu_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" >"$ROOT/log/v5_jubu_bag_${STAMP}.log" 2>&1 &
  RECORD_PID=$!
  echo "[RECORD] V5 JUBU bag -> $BAG_PATH"
fi

echo "[V5_JUBU] isolated Stable V5 local-avoidance release"
echo "[V5_JUBU] global cap=$V5_JUBU_SPEED_CAP m/s; pass/return cap=2.20 m/s"
echo "[V5_JUBU] planner/perception/reference timeout or ABORT => hardware gate sends zero"
echo "[V5_JUBU] C++ command gate is the sole hardware publisher"
echo "[WARNING] Hold the physical E-stop before launch. Release only on-track with an operator ready."

roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_v5_jubu.launch \
  track:="$TRACK" controller_config:="$CONTROLLER_CONFIG" \
  vehicle_config:="$VEHICLE_CONFIG" formulation:=mpcc \
  start_planner:=false start_controller:=true allow_real_hardware:=true \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  runtime_speed_cap_mps:="$V5_JUBU_SPEED_CAP" \
  generated_dir:="$GENERATED_DIR" residual_model_path:="$RESIDUAL" \
  residual_feature_set:=markov_v1 \
  hardware_command_topic:="$HARDWARE_COMMAND_TOPIC" \
  telemetry_path:="$ROOT/log/hardware_v5_jubu_${STAMP}.jsonl" \
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}" \
  lap_log_path:="$ROOT/log/laps_v5_jubu_${STAMP}.csv" &
CONTROL_PID=$!

set +e
wait "$CONTROL_PID"
STATUS=$?
set -e
exit "$STATUS"
