#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="stable_roboracer_oldmap_c11_outer_boundary_expanded_arc_straight_4p5"
TRACK="raceline_smooth_c11_outer_boundary_expanded_arc_straight_4p5_candidate"
CONTROLLER_CONFIG="candidates/stable_roboracer_c11_outer_expanded_arc_straight_4p5/controller.yaml"
MAP_ID="old"
RACE_START_CENTER_X="${TWO_MAP_RACE_START_CENTER_X:-0.0}"
RACE_START_CENTER_Y="${TWO_MAP_RACE_START_CENTER_Y:-0.0}"
VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_c11_stabilized_candidate.launch"
CORE="$ROOT/src/f1tenth_dynamic_mpcc/src/core.cpp"
RVIZ_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/rviz/stable_roboracer.rviz"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
SPEED_ZONES="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/speed_zones.csv"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MPCC_NODE="$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor_stable_roboracer.py"
MOTION_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"
OBSTACLE_WS="${ROBORACER_OBSTACLE_WS:-/home/tianbot/raw_cloud_obstacle_ws}"
OBSTACLE_START="$OBSTACLE_WS/start_raw_cloud_obstacle.sh"
OBSTACLE_NODE="/raw_cloud_obstacle_perception"
MAP_IDENTITY_CHECK="$ROOT/scripts/check_localization_map_identity.sh"
EXPECTED_TRACK_SHA256="f42badd7a7d22b7371d8b6b0afcb6d96c192e6fc2e12c6a23af4c8d0fbffe906"
EXPECTED_SPEED_ZONES_SHA256="72de9753f31e91ea0f4dc093b0506a5e1245e368deeddb988e0b087512789796"
EXPECTED_CONTROLLER_SHA256="14df4868242fdd905ad2e75f34954c915f57c06134c96ddcddcf08794dba2e77"
PROFILE_ACCEL_MPS2="1.5"
PROFILE_DECEL_MPS2="2.0"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] Stable RoboRacer frozen file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

check_hash_any() {
  local path="$1"
  shift
  local actual expected
  actual="$(sha256sum "$path" | awk '{print $1}')"
  for expected in "$@"; do
    [[ "$actual" == "$expected" ]] && return 0
  done
  echo "[FAIL] unsupported shared auto-relaunch node: $path" >&2
  echo "       actual=$actual" >&2
  exit 1
}

# The candidate is additive. Verify the exact C11-expanded baseline first.
"$ROOT/scripts/start_mpcc_hardware_stable_roboracer_oldmap_c11_outer_boundary_expanded_candidate.sh" --check-only
check_hash "$EXPECTED_CONTROLLER_SHA256" "$CONTROLLER"
check_hash "301dc5370a5841d059a9e2616e3f22c308ba11abb858f05173080e12dfb59781" "$LAUNCH"
check_hash "2d61cba865498e41edd06ade95491cef0bc87d0a642eceacb1e391428b53d476" "$CORE"
check_hash_any "$RVIZ_CONFIG" \
  "86d3cdd49e69ab2c6fa129bdda762016da9846b27f1cd40cbd3f7f78f2fb5dd2" \
  "461453216a2b459a811e11de73a68aef66ff29883050444ef8e8b82844605b60"
check_hash "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6" "$VEHICLE"
check_hash "$EXPECTED_TRACK_SHA256" "$RACELINE"
check_hash "$EXPECTED_SPEED_ZONES_SHA256" "$SPEED_ZONES"
check_hash "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44" "$RESIDUAL"
# Accept the frozen car-side V5 node and the compatible local high-speed-gate
# extension. V3pro explicitly disables that extension in its own config.
check_hash_any "$MPCC_NODE" \
  "344bc7003ff491ff082351a2fb6979a608b9ead2df0238a0cee6c0d1b911bde0" \
  "5f173d9bee60421af3c102b7fb98745422186ade796029adff07dd05c5701440"
check_hash "fb42de2f5a1bad9e97099b64bd98af40545984d2a2531fcdb9f70b354197d364" "$SUPERVISOR"
check_hash "43760fcbb6a965da5c7d8e68b7683a2a520d2129da30dcc8d4eff8db2521a8d6" "$MOTION_LOGIC"
check_hash "21217b67871accf841c1bef76f9e6c7f874526cfd1bd99c497706b989cb7b178" "$PLACEMENT_LOGIC"
check_hash "1ebd7a7a5b4b5531058206b3a64377b5bb17ff03643d97bbb5dc294d695302ec" "$RECORDER"

if [[ "${ROBORACER_OBSTACLE_PERCEPTION:-false}" == "true" ]]; then
  if [[ ! -f "$OBSTACLE_WS/devel/setup.bash" ]]; then
    echo "[FAIL] obstacle-perception workspace is not built: $OBSTACLE_WS" >&2
    exit 1
  fi
  if [[ ! -x "$OBSTACLE_START" ]]; then
    echo "[FAIL] obstacle-perception launcher is missing or not executable: $OBSTACLE_START" >&2
    exit 1
  fi
fi

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] isolated old-map C11-expanded arc/straight 4.5 m/s candidate hashes match"
  echo "[OK] C11-expanded geometry preserved; fast zones=4.5 m/s; standard/tight bends=3.0 m/s"
  echo "[OK] profile/publisher accel/decel=${PROFILE_ACCEL_MPS2}/${PROFILE_DECEL_MPS2} m/s^2"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
elif [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_${CANDIDATE}}"
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
source "$ROOT/scripts/ros_env.sh"

# Automatic relaunch consumes LocalizationStatus from the localization build.
LOCALIZATION_WS="${AUTO_RELAUNCH_LOCALIZATION_WS:-/home/tianbot/localization_main}"
if [[ ! -f "$LOCALIZATION_WS/devel/setup.bash" ]]; then
  echo "[FAIL] localization workspace setup is missing: $LOCALIZATION_WS/devel/setup.bash" >&2
  exit 1
fi
MPCC_RESTORE_NOUNSET=false
case "$-" in
  *u*) MPCC_RESTORE_NOUNSET=true ;;
esac
set +u
source "$LOCALIZATION_WS/devel/setup.bash" --extend
if [[ "$MPCC_RESTORE_NOUNSET" == true ]]; then
  set -u
fi
unset MPCC_RESTORE_NOUNSET
export PYTHONPATH="$ROOT/src/f1tenth_dynamic_mpcc/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'from point_lio_sam_lighterbev.msg import LocalizationStatus' || {
  echo "[FAIL] point_lio_sam_lighterbev/LocalizationStatus is not importable" >&2
  exit 1
}

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

if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "[OK] Stable RoboRacer solver is ready; hardware was not started"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"
"$MAP_IDENTITY_CHECK" "$MAP_ID"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-4.5}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 4.5) }'; then
  echo "[FAIL] candidate RESIDUAL_MPCC_SPEED_CAP must be in (0, 4.5]" >&2
  exit 2
fi

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
RECORD_PID=""
OBSTACLE_PID=""

cleanup_children() {
  if [[ -n "$RECORD_PID" ]]; then
    kill -INT "$RECORD_PID" 2>/dev/null || true
    wait "$RECORD_PID" 2>/dev/null || true
  fi
  if [[ -n "$OBSTACLE_PID" ]]; then
    kill -INT "$OBSTACLE_PID" 2>/dev/null || true
    wait "$OBSTACLE_PID" 2>/dev/null || true
  fi
}
trap cleanup_children EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_obstacle_perception() {
  if [[ "${ROBORACER_OBSTACLE_PERCEPTION:-false}" != "true" ]]; then
    echo "[PERCEPTION] RoboRacer obstacle perception disabled by default"
    return 0
  fi

  if rosnode list 2>/dev/null | grep -qx "$OBSTACLE_NODE"; then
    echo "[PERCEPTION] Reusing existing $OBSTACLE_NODE"
    return 0
  fi

  "$OBSTACLE_START" &
  OBSTACLE_PID=$!
  for _ in $(seq 1 30); do
    if rosnode list 2>/dev/null | grep -qx "$OBSTACLE_NODE"; then
      echo "[PERCEPTION] Obstacle cloud ready: /raw_cloud_perception/obstacle_cloud"
      return 0
    fi
    if ! kill -0 "$OBSTACLE_PID" 2>/dev/null; then
      wait "$OBSTACLE_PID" || true
      OBSTACLE_PID=""
      echo "[FAIL] obstacle-perception process exited during startup" >&2
      return 1
    fi
    sleep 0.1
  done

  echo "[FAIL] timed out waiting for $OBSTACLE_NODE" >&2
  return 1
}

start_obstacle_perception

if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/${CANDIDATE}_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] Stable RoboRacer bag -> $BAG_PATH"
fi

echo "[ROBORACER_C11_4P5] Isolated candidate based on the frozen old-map C11-expanded candidate"
echo "[STABLE_ROBORACER] MPCC solve=40 Hz; command publisher/supervisor=50 Hz"
echo "[STABLE_ROBORACER] contour/heading/rate-cost=45.0/1.0/0.60; steering rate=3.4 rad/s"
echo "[ROBORACER_C11_4P5] profile and publisher accel/decel=${PROFILE_ACCEL_MPS2}/${PROFILE_DECEL_MPS2} m/s^2"
echo "[ROBORACER_C11_4P5] runtime cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; track=$TRACK"
echo "[ROBORACER_C11_4P5] arc/round-arc/straight=4.5 m/s; standard/tight bends=3.0 m/s"
echo "[ROBORACER_C11_4P5] C11 outer-boundary expansion and all recovery/start logic are preserved"
echo "[STABLE_ROBORACER] RViz=/f1tenth_mpcc/stable_roboracer/* (isolated smooth-boundary display)"
echo "[STABLE_ROBORACER] carry detection, stable-ground gate and global-S reprojection enabled"
echo "[STABLE_ROBORACER] arbitrary-position PP recovery=2.0 m/s; smooth PP-to-MPCC handoff"
echo "[STABLE_ROBORACER] grid start PP lateral limit=3.2 m/s^2; speed-relative MPCC handoff"
echo "[STABLE_ROBORACER] grid-start extracted-boundary gate disabled; localization/heading/ground gates retained"
echo "[STABLE_ROBORACER] obstacle perception disabled by default; set ROBORACER_OBSTACLE_PERCEPTION=true to enable"
echo "[STABLE_ROBORACER] control debug: MODE / STATE / OWNER / TAKEOVER transitions are highlighted"
echo "[STABLE_ROBORACER] state topic: /automatic_relaunch_supervisor/state"
echo "[STABLE_ROBORACER] Stable V3, V3pro, source candidate and Stable V5 remain unchanged"
echo "[WARNING] Hold the physical E-stop before launch or carrying the car."

LAUNCH_ARGS=(
  track:="$TRACK"
  controller_config:="$CONTROLLER_CONFIG"
  vehicle_config:="$VEHICLE_CONFIG"
  formulation:=mpcc
  allow_real_hardware:=true
  rviz:="${MPCC_REMOTE_RVIZ:-false}"
  rviz_config:="$RVIZ_CONFIG"
  runtime_speed_cap_mps:="$RESIDUAL_MPCC_SPEED_CAP"
  generated_dir:="$GENERATED_DIR"
  residual_model_path:="$RESIDUAL"
  residual_feature_set:=markov_v1
  hardware_command_topic:="${AUTO_RELAUNCH_HARDWARE_COMMAND_TOPIC:-/tianracer/ackermann_cmd}"
  race_start_center_x_m:="$RACE_START_CENTER_X"
  race_start_center_y_m:="$RACE_START_CENTER_Y"
  telemetry_path:="$ROOT/log/hardware_${CANDIDATE}_${STAMP}.jsonl"
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}"
  lap_log_path:="$ROOT/log/laps_${CANDIDATE}_${STAMP}.csv"
)

roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_roboracer_c11_stabilized_candidate.launch "${LAUNCH_ARGS[@]}"
