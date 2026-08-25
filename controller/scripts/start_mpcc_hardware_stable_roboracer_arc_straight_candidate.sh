#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VARIANT="${ROBORACER_ARC_STRAIGHT_VARIANT:-}"
case "$VARIANT" in
  4p5)
    CANDIDATE="stable_roboracer_arc_straight_4p5_candidate"
    TRACK="raceline_smooth_roboracer_arc_straight_4p5_candidate"
    CANDIDATE_CAP="4.5"
    ;;
  5p0)
    CANDIDATE="stable_roboracer_arc_straight_5p0_candidate"
    TRACK="raceline_smooth_roboracer_arc_straight_5p0_candidate"
    CANDIDATE_CAP="5.0"
    ;;
  *)
    echo "[FAIL] internal RoboRacer arc/straight variant must be 4p5 or 5p0" >&2
    exit 2
    ;;
esac

CONTROLLER_CONFIG="candidates/$CANDIDATE/controller.yaml"
VEHICLE_CONFIG="candidates/$CANDIDATE/vehicle.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
RACELINE="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
MANIFEST="$ROOT/src/f1tenth_dynamic_mpcc/config/candidates/$CANDIDATE/MANIFEST.json"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_arc_straight_dual_decel_candidate.launch"
RVIZ_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/rviz/stable_roboracer.rviz"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"
OBSTACLE_WS="${ROBORACER_OBSTACLE_WS:-/home/tianbot/raw_cloud_obstacle_ws}"
OBSTACLE_START="$OBSTACLE_WS/start_raw_cloud_obstacle.sh"
OBSTACLE_NODE="/raw_cloud_obstacle_perception"

# First prove that the complete accepted ProMax release is still frozen.
"$ROOT/scripts/start_mpcc_hardware_stable_roboracer.sh" --check-only

python3 - "$ROOT" "$MANIFEST" "$CANDIDATE_CAP" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
expected_cap = float(sys.argv[3])
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
if manifest.get("source_release") != "stable_roboracer":
    raise SystemExit("[FAIL] candidate is not derived from Stable RoboRacer")
policy = manifest.get("policy", {})
if float(policy.get("runtime_global_cap_mps", -1.0)) != expected_cap:
    raise SystemExit("[FAIL] candidate speed policy does not match launcher")
if not policy.get("all_other_roboracer_behavior_frozen", False):
    raise SystemExit("[FAIL] candidate does not preserve the ProMax behavior contract")
for relative, expected in manifest.get("base_hashes", {}).items():
    path = root / relative
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit(f"[FAIL] frozen base changed: {path}: {actual}")
for relative, expected in manifest.get("output_hashes", {}).items():
    path = root / relative
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit(f"[FAIL] candidate file changed: {path}: {actual}")
PY

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] RoboRacer arc/straight ${CANDIDATE_CAP} m/s candidate hashes match"
  echo "[OK] normal command decel=2.2 m/s^2; predictive/PP/OOB/checkpoint/fault decel=4.0 m/s^2"
  echo "[OK] standard/tight bends=3.0 m/s; frozen Stable RoboRacer remains untouched"
  exit 0
fi

PREPARE_ONLY=false
if [[ "$ACTION" == "--prepare-only" ]]; then
  PREPARE_ONLY=true
elif [[ "$ACTION" != "mpcc" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_$CANDIDATE}"
if [[ -z "${RESIDUAL_DYNAMICS_WS:-}" ]] \
   && [[ ! -f "$(dirname "$ROOT")/f1tenth_residual_ws/devel/setup.bash" ]]; then
  export RESIDUAL_DYNAMICS_WS="$ROOT"
fi
source "$ROOT/scripts/ros_env.sh"

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
  echo "[OK] RoboRacer arc/straight ${CANDIDATE_CAP} m/s solver is ready"
  exit 0
fi

"$ROOT/scripts/check_mpcc_topics.sh"

RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-$CANDIDATE_CAP}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" -v maximum="$CANDIDATE_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= maximum) }'; then
  echo "[FAIL] RESIDUAL_MPCC_SPEED_CAP must be in (0, $CANDIDATE_CAP]" >&2
  exit 2
fi

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
  echo "[RECORD] RoboRacer ${CANDIDATE_CAP} m/s candidate bag -> $BAG_PATH"
fi

echo "[ROBORACER_FAST_CANDIDATE] ISOLATED candidate; frozen stable_roboracer.sh is untouched"
echo "[ROBORACER_FAST_CANDIDATE] arc/round-arc/straight cap=${CANDIDATE_CAP} m/s; standard/tight=3.0 m/s"
echo "[ROBORACER_FAST_CANDIDATE] command decel: normal=2.2 m/s^2; predictive/PP/OOB/checkpoint/fault=4.0 m/s^2"
echo "[ROBORACER_FAST_CANDIDATE] runtime cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; PP recovery max remains 4.0 m/s"
echo "[ROBORACER_FAST_CANDIDATE] carry detection, arbitrary-position recovery and grid race-start are frozen ProMax"
echo "[WARNING] ${CANDIDATE_CAP} m/s candidate has not received operator hardware acceptance. Hold the physical E-stop."

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
  telemetry_path:="$ROOT/log/hardware_${CANDIDATE}_${STAMP}.jsonl"
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}"
  lap_log_path:="$ROOT/log/laps_${CANDIDATE}_${STAMP}.csv"
)

roslaunch f1tenth_dynamic_mpcc "$(basename "$LAUNCH")" "${LAUNCH_ARGS[@]}"
