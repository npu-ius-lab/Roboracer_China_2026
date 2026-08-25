#!/usr/bin/env bash
set -euo pipefail

# Additive React/local-reference MPCC variant of:
#   start_mpcc_hardware_stable_roboracer_oldmap_c11_outer_boundary_expanded_candidate.sh
# The baseline entry, controller, track, vehicle, and residual files are
# hash-checked and never written by this launcher.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# This file is a hardware launcher: no argument means real-car MPCC mode.
# Keep the historical `mpcc` spelling while also accepting the explicit
# `--hardware` spelling used by the other TianRacer launchers.
ACTION="${1:---hardware}"

BASELINE_ENTRY="$ROOT/scripts/start_mpcc_hardware_stable_roboracer_oldmap_c11_outer_boundary_expanded_candidate.sh"
BASELINE_ENTRY_SHA256="84336c341481c98f908cb306f79af28556d3c1c86a15a6be3d1df4a8abad6406"
PACKAGE_DIR="$ROOT/src/roboracer_react_fast_overtake"
REACT_CONFIG="$PACKAGE_DIR/config/react_c11_baseline.yaml"
REACT_CONFIG_SHA256="fde04bf67a0bc36ffd347f1b2893fadb624c509870ef4392d6b52f39f8e4313a"
REACT_LAUNCH="$PACKAGE_DIR/launch/roboracer_c11_react_control.launch"
REACT_LAUNCH_SHA256="a816249e9a68f51e75961b729afb2c1e371048cbbeb10c335350f6f24c0e2784"
RECORDER="$ROOT/scripts/record_roboracer_c11_react_bag.sh"
RECORDER_SHA256="c1e1396815cea32eecdd4f9b99b1197b1466ba4a6e9fec551d7be0170dabbd16"

TRACK_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/raceline_smooth_c11_outer_boundary_expanded_candidate/raceline.csv"
TRACK_SHA256="cae8c6833492457bf2a884eb4c4760960b5b49bc7eac5fa16e1cd2a98194d354"
GREEN_PERCEPTION_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/perception_corridors/roboracer_green_c10_c12.csv"
GREEN_PERCEPTION_CSV_SHA256="ff10c67cf5390cb3151aa087bd4cb8739a04db2e54201e885a5c44b1b2c01765"
GREEN_PERCEPTION_INSET_M="0.22"
CONTROLLER_CONFIG="candidates/stable_roboracer_c11_stabilized/controller.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
CONTROLLER_SHA256="c6c90509cfbc81dd15bb20ad7f33e102c0f10ed4f512722dad1669cb90ac4f3f"
VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MPCC_BINARY="$ROOT/devel/lib/f1tenth_dynamic_mpcc/mpcc_node_auto_relaunch_chaoche_cpp"
MPCC_CORE_LIBRARY="$ROOT/devel/lib/libf1tenth_mpcc_core.so"
MPCC_CHAOCHE_LIBRARY="$ROOT/devel/lib/libf1tenth_mpcc_acados_runtime_chaoche.so"
PLANNER_BINARY="$ROOT/devel/lib/roboracer_react_fast_overtake/roboracer_react_fast_overtake_node"
GATE_BINARY="$ROOT/devel/lib/roboracer_react_fast_overtake/roboracer_react_fast_minimum_speed_gate_node"

RAW_WS="${RAW_CLOUD_OBSTACLE_WS:-/home/tianbot/raw_cloud_obstacle_ws}"
RAW_START="$RAW_WS/start_raw_cloud_obstacle.sh"
PERCEPTION_HELPER="$ROOT/scripts/ensure_raw_cloud_perception.sh"
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
MAP_IDENTITY_CHECK="$ROOT/scripts/check_localization_map_identity.sh"

# This candidate deliberately separates the controller track from the obstacle
# filter.  MPCC follows the C11 outer-boundary-expanded track, while perception
# uses the green corridor: yellow-equivalent elsewhere and the non-expanded
# C10-C12 corridor where wall returns must be rejected.
export ROBORACER_OBSTACLE_WS="$RAW_WS"
export ROBORACER_OBSTACLE_RACELINE_CSV="$GREEN_PERCEPTION_CSV"
export ROBORACER_OBSTACLE_BOUNDARY_INSET_M="$GREEN_PERCEPTION_INSET_M"
export ROBORACER_OBSTACLE_MARKER_REQUIRED="true"

usage() {
  echo "usage: $0 [--hardware|mpcc|--check-only|--prepare-only]" >&2
  echo "       default: --hardware (real car; supervisor is sole actuator publisher)" >&2
}

case "$ACTION" in
  --hardware|mpcc) ACTION="mpcc" ;;
  --check-only|--prepare-only) ;;
  *) usage; exit 2 ;;
esac

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] frozen dependency changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

required=(
  "$BASELINE_ENTRY"
  "$REACT_CONFIG"
  "$REACT_LAUNCH"
  "$RECORDER"
  "$TRACK_CSV"
  "$GREEN_PERCEPTION_CSV"
  "$CONTROLLER"
  "$RESIDUAL"
  "$MPCC_BINARY"
  "$MPCC_CORE_LIBRARY"
  "$MPCC_CHAOCHE_LIBRARY"
  "$PLANNER_BINARY"
  "$GATE_BINARY"
  "$ROOT/devel/setup.bash"
  "$RAW_WS/devel/setup.bash"
  "$RAW_START"
  "$PERCEPTION_HELPER"
  "$LOCALIZATION_WS/devel/setup.bash"
  "$MAP_IDENTITY_CHECK"
)
for path in "${required[@]}"; do
  [[ -e "$path" ]] || { echo "[FAIL] missing C11 React dependency: $path" >&2; exit 1; }
done

check_hash "$BASELINE_ENTRY_SHA256" "$BASELINE_ENTRY"
"$BASELINE_ENTRY" --check-only
check_hash "$TRACK_SHA256" "$TRACK_CSV"
check_hash "$GREEN_PERCEPTION_CSV_SHA256" "$GREEN_PERCEPTION_CSV"
check_hash "$CONTROLLER_SHA256" "$CONTROLLER"
check_hash "$REACT_CONFIG_SHA256" "$REACT_CONFIG"
check_hash "$REACT_LAUNCH_SHA256" "$REACT_LAUNCH"
check_hash "$RECORDER_SHA256" "$RECORDER"

if grep -RInE 'targets_detailed|TargetArray|static_target|dynamic_target' \
    "$PACKAGE_DIR/src" "$PACKAGE_DIR/include" >/tmp/roboracer_c11_react_forbidden.$$; then
  echo "[FAIL] C11 React source contains a tracked-target dependency:" >&2
  sed -n '1,20p' /tmp/roboracer_c11_react_forbidden.$$ >&2
  rm -f /tmp/roboracer_c11_react_forbidden.$$
  exit 1
fi
rm -f /tmp/roboracer_c11_react_forbidden.$$

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

python3 -c 'from point_lio_sam_lighterbev.msg import LocalizationStatus' || {
  echo "[FAIL] point_lio_sam_lighterbev/LocalizationStatus is not importable" >&2
  exit 1
}

abi_report="$(ldd -r "$MPCC_BINARY" 2>&1 || true)"
if grep -Eq 'not found|undefined symbol' <<<"$abi_report"; then
  echo "[FAIL] local-reference MPCC executable/library ABI mismatch:" >&2
  grep -E 'not found|undefined symbol' <<<"$abi_report" >&2
  exit 1
fi
for expected_library in "$MPCC_CORE_LIBRARY" "$MPCC_CHAOCHE_LIBRARY"; do
  if ! grep -Fq "=> $expected_library " <<<"$abi_report"; then
    echo "[FAIL] MPCC resolved a library outside this workspace: $expected_library" >&2
    exit 1
  fi
done
unset abi_report expected_library

echo "[OK] source baseline hash and all frozen baseline dependencies match"
echo "[OK] old-map C11-expanded track shared by React, MPCC, supervisor, and lap timer"
echo "[OK] controller=C11 stabilized; accel/decel=1.5/2.0 m/s^2; cap<=4.0 m/s"
echo "[OK] perception is pinned to green C10-C12 corridor; inset=${GREEN_PERCEPTION_INSET_M}m"
echo "[OK] React trigger input=/raw_cloud_perception/obstacle_cloud (post green-corridor clustering)"
echo "[OK] empty obstacle cloud=GLOBAL; nonempty valid blockage=PASS/FOLLOW"
echo "[OK] local-reference MPCC ABI and workspace linkage are valid"

if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] no ROS node, recorder, controller, or hardware publisher was started"
  exit 0
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer_oldmap_c11_react}"
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
export MPCC_GENERATED_DIR="$GENERATED_DIR"
export MPCC_TRACK="raceline_smooth_c11_outer_boundary_expanded_candidate"
export MPCC_CONTROLLER_CONFIG="$CONTROLLER_CONFIG"
export MPCC_VEHICLE_CONFIG="$VEHICLE_CONFIG"
export MPCC_RESIDUAL_MODEL_PATH="$RESIDUAL"
export MPCC_RESIDUAL_FEATURE_SET="markov_v1"
export MPCC_COST_CONTOUR_OVERRIDE="45.0"
export MPCC_COST_HEADING_RACE_OVERRIDE="1.0"
export MPCC_COST_STEERING_COMMAND_RATE_OVERRIDE="0.60"
"$ROOT/scripts/ensure_acados_solvers.sh"

if [[ "$ACTION" == "--prepare-only" ]]; then
  echo "[OK] C11 React solver is ready; hardware was not started"
  exit 0
fi

rosnode list >/dev/null 2>&1 || {
  echo "[FAIL] ROS master is unavailable" >&2
  exit 1
}

mapfile -t conflicting_nodes < <(
  rosnode list 2>/dev/null | grep -E \
    '^/(f1tenth_dynamic_mpcc|automatic_relaunch_supervisor|roboracer_.*(react|h2h)|.*(chaoche|overtake))' || true
)
if (( ${#conflicting_nodes[@]} > 0 )); then
  echo "[FAIL] another controller/planner is already running:" >&2
  printf '       %s\n' "${conflicting_nodes[@]}" >&2
  exit 1
fi

"$ROOT/scripts/check_mpcc_topics.sh"
"$MAP_IDENTITY_CHECK" old

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-4.0}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 4.0) }'; then
  echo "[FAIL] C11 React RESIDUAL_MPCC_SPEED_CAP must be in (0, 4.0]" >&2
  exit 2
fi

pids=()
cleanup_started=false
cleanup() {
  local status=$?
  local pid
  trap - EXIT INT TERM
  if [[ "$cleanup_started" == true ]]; then exit "$status"; fi
  cleanup_started=true
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

"$PERCEPTION_HELPER"

STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROOT/log" "$ROOT/bags"
case "${MPCC_RECORD_BAG:-false}" in
  true|TRUE|yes|YES|1)
    BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/roboracer_oldmap_c11_react_${STAMP}.bag}"
    "$RECORDER" "$BAG_PATH" &
    pids+=("$!")
    echo "[RECORD] C11 React bag -> $BAG_PATH"
    ;;
esac

echo "[ROBORACER_C11_REACT] baseline=oldmap C11 outer-boundary-expanded candidate"
echo "[ROBORACER_C11_REACT] global controller=C11 stabilized residual MPCC"
echo "[ROBORACER_C11_REACT] React=GLOBAL/FOLLOW/PASS/RETURN/ABORT from green-corridor obstacle_cloud"
echo "[ROBORACER_C11_REACT] red local plan=/roboracer_c11_react/local_plan_red"
echo "[ROBORACER_C11_REACT] speed cap=$RESIDUAL_MPCC_SPEED_CAP m/s; running-state floor=1.0 m/s"
echo "[ROBORACER_C11_REACT] supervisor remains the sole /tianracer/ackermann_cmd publisher"
echo "[WARNING] Hold the physical E-stop before launch or carrying the car."

roslaunch roboracer_react_fast_overtake roboracer_c11_react_control.launch \
  track_csv:="$TRACK_CSV" \
  controller_config:="$CONTROLLER_CONFIG" \
  start_controller:=true \
  allow_real_hardware:=true \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  telemetry_path:="$ROOT/log/hardware_roboracer_oldmap_c11_react_${STAMP}.jsonl" \
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}" \
  lap_log_path:="$ROOT/log/laps_roboracer_oldmap_c11_react_${STAMP}.csv" \
  runtime_speed_cap_mps:="$RESIDUAL_MPCC_SPEED_CAP" \
  generated_dir:="$GENERATED_DIR" \
  residual_model_path:="$RESIDUAL" \
  residual_feature_set:=markov_v1 \
  hardware_command_topic:="${AUTO_RELAUNCH_HARDWARE_COMMAND_TOPIC:-/tianracer/ackermann_cmd}" \
  race_start_center_x_m:="${TWO_MAP_RACE_START_CENTER_X:-0.0}" \
  race_start_center_y_m:="${TWO_MAP_RACE_START_CENTER_Y:-0.0}"
