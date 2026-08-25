#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATE="stable_roboracer"
TRACK="racelinev6_p1p2_candidate"
CONTROLLER_CONFIG="candidates/$CANDIDATE/controller.yaml"
VEHICLE_CONFIG="vehicle_stable_v3_racelineV3.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_racelinev6_p1p2_candidate.launch"
RVIZ_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/rviz/stable_roboracer_racelinev6_p1p2_candidate.rviz"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MPCC_NODE="$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor_stable_roboracer.py"
MOTION_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"
OBSTACLE_WS="${ROBORACER_OBSTACLE_WS:-/home/tianbot/raw_cloud_obstacle_ws}"
OBSTACLE_START="$OBSTACLE_WS/start_raw_cloud_obstacle.sh"
OBSTACLE_NODE="/raw_cloud_obstacle_perception"

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

# V3pro is additive. Verify that the frozen Stable V3 inputs remain intact.
"$ROOT/scripts/start_mpcc_hardware_stable_v3.sh" --check-only
check_hash "df8f558111e09db665a70bafaf4baacf238bbc37095e116e6fc992e88498bfe2" "$CONTROLLER"
check_hash "29a65c3e3c9f255c286f7efb56bf9eea2d27987b662e5f74d7d2354aba34c877" "$LAUNCH"
check_hash "7e285cd3e87d71aa342755c93a642e85633fa79c001aaaa57c71ed17cce71c55" "$RVIZ_CONFIG"
check_hash "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6" "$VEHICLE"
check_hash "72631efa6199d05e8fc0d6dad3a8c2d7ab04796301617e9b0e8674ed1d4c29d1" "$RACELINE"
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

if [[ "${ROBORACER_OBSTACLE_PERCEPTION:-true}" == "true" ]]; then
  if [[ ! -f "$OBSTACLE_WS/devel/setup.bash" ]]; then
    echo "[FAIL] obstacle-perception workspace is not built: $OBSTACLE_WS" >&2
    exit 1
  fi
  if [[ ! -x "$OBSTACLE_START" ]]; then
    echo "[FAIL] obstacle-perception launcher is missing or not executable: $OBSTACLE_START" >&2
    exit 1
  fi
fi

python3 - "$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/raceline_smooth/raceline.csv" "$RACELINE" <<'PY'
import csv
import sys

with open(sys.argv[1], newline="", encoding="utf-8") as stream:
    baseline = list(csv.DictReader(stream))
with open(sys.argv[2], newline="", encoding="utf-8") as stream:
    candidate = list(csv.DictReader(stream))
if len(baseline) != len(candidate):
    raise SystemExit("[FAIL] candidate raceline row count differs from RoboRacer")
width_fields = {"w_tr_left_m", "w_tr_right_m"}
changed = False
for before, after in zip(baseline, candidate):
    for field in before:
        if field in width_fields:
            changed |= before[field] != after[field]
        elif before[field] != after[field]:
            raise SystemExit(f"[FAIL] non-width raceline field changed: {field}")
if not changed:
    raise SystemExit("[FAIL] candidate has no boundary-width changes")
print("[OK] candidate contract: only left/right physical boundary widths differ")
PY

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] Stable RoboRacer P1/P2 raceline candidate hashes match"
  echo "[OK] only raceline boundary widths changed; current car-side ProMax behavior preserved"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
elif [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer_racelinev6_p1p2_candidate}"
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

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-4.0}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 4.0) }'; then
  echo "[FAIL] Stable RoboRacer RESIDUAL_MPCC_SPEED_CAP must be in (0, 4.0]" >&2
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
  if [[ "${ROBORACER_OBSTACLE_PERCEPTION:-true}" != "true" ]]; then
    echo "[PERCEPTION] RoboRacer obstacle perception disabled by environment"
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
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_roboracer_racelinev6_p1p2_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] Stable RoboRacer bag -> $BAG_PATH"
fi

echo "[ROBORACER_P1P2] Frozen from the validated V3pro smooth race-start-v2 candidate"
echo "[ROBORACER_P1P2] MPCC solve=40 Hz; command publisher/supervisor=50 Hz"
echo "[ROBORACER_P1P2] contour/heading/rate-cost=45.0/1.0/0.60; steering rate=3.4 rad/s"
echo "[ROBORACER_P1P2] profile decel=3.0; publisher accel/decel=1.5/2.0 m/s^2"
echo "[ROBORACER_P1P2] runtime cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; track=racelinev6_p1p2_candidate"
echo "[ROBORACER_P1P2] RViz=/f1tenth_mpcc/stable_roboracer_racelinev6_p1p2/* (isolated smooth-boundary display)"
echo "[ROBORACER_P1P2] carry detection, stable-ground gate and global-S reprojection enabled"
echo "[ROBORACER_P1P2] arbitrary-position PP recovery=2.0 m/s; smooth PP-to-MPCC handoff"
echo "[ROBORACER_P1P2] grid start PP lateral limit=3.2 m/s^2; speed-relative MPCC handoff"
echo "[ROBORACER_P1P2] grid-start extracted-boundary gate disabled; localization/heading/ground gates retained"
echo "[ROBORACER_P1P2] obstacle perception=/raw_cloud_perception/obstacle_cloud (enabled by default)"
echo "[ROBORACER_P1P2] Stable V3, V3pro, RoboRacer and Stable V5 remain unchanged"
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
  telemetry_path:="$ROOT/log/hardware_stable_roboracer_racelinev6_p1p2_${STAMP}.jsonl"
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}"
  lap_log_path:="$ROOT/log/laps_stable_roboracer_racelinev6_p1p2_${STAMP}.csv"
)

roslaunch f1tenth_dynamic_mpcc hardware_mpcc_stable_roboracer_racelinev6_p1p2_candidate.launch "${LAUNCH_ARGS[@]}"
