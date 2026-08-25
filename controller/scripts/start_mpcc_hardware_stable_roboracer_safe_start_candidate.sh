#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STABLE="$ROOT/scripts/start_mpcc_hardware_stable_roboracer.sh"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_roboracer_safe_start_candidate.launch"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor_stable_roboracer_safe_start_candidate.py"
POLICY="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/safe_race_start.py"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"
RVIZ_CONFIG="$ROOT/src/f1tenth_dynamic_mpcc/rviz/stable_roboracer.rviz"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
OBSTACLE_WS="${ROBORACER_OBSTACLE_WS:-/home/tianbot/raw_cloud_obstacle_ws}"
OBSTACLE_START="$OBSTACLE_WS/start_raw_cloud_obstacle.sh"
OBSTACLE_NODE="/raw_cloud_obstacle_perception"

check_hash() {
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] Safe-start candidate file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

"$STABLE" --check-only
check_hash "f81bbeb9076e16be67c7aa0248420c5a4c2081cf656752ca1c043d4d4f5f04d9" "$LAUNCH"
check_hash "3c4a0d944c9bbaa866ab768d093929c19622e0e4e1aef1f132fe487b8a418b2b" "$SUPERVISOR"
check_hash "1aee41a11389a34d358ff86c57f63cca42cb23a33fe7dd75d7230e4a58afa15d" "$POLICY"

RACE_START_ENABLED="${ROBORACER_SAFE_RACE_START_ENABLED:-true}"
case "$RACE_START_ENABLED" in
  true|false) ;;
  *)
    echo "[FAIL] ROBORACER_SAFE_RACE_START_ENABLED must be true or false" >&2
    exit 2
    ;;
esac

ACTION="${1:-mpcc}"
if [[ "$ACTION" == "--check-only" ]]; then
  echo "[OK] Isolated RoboRacer safe-start candidate hashes match"
  echo "[OK] race-start switch=$RACE_START_ENABLED; release=0.35 m/s; PP<=1.60 m/s"
  echo "[OK] frozen start_mpcc_hardware_stable_roboracer.sh remains unchanged"
  exit 0
fi
if [[ "$ACTION" != "mpcc" && "$ACTION" != "--prepare-only" ]]; then
  echo "usage: $0 [mpcc|--check-only|--prepare-only]" >&2
  exit 2
fi

# Build/validate the exact frozen MPCC solver through its existing entry point.
"$STABLE" --prepare-only
if [[ "$ACTION" == "--prepare-only" ]]; then
  echo "[OK] RoboRacer safe-start candidate solver is ready"
  exit 0
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
python3 -c 'from point_lio_sam_lighterbev.msg import LocalizationStatus; from f1tenth_dynamic_mpcc.safe_race_start import SafeRaceStartPolicy' || {
  echo "[FAIL] safe-start Python dependencies are not importable" >&2
  exit 1
}

"$ROOT/scripts/check_mpcc_topics.sh"
RESIDUAL_MPCC_SPEED_CAP="${RESIDUAL_MPCC_SPEED_CAP:-4.0}"
if ! awk -v value="$RESIDUAL_MPCC_SPEED_CAP" \
  'BEGIN { exit !(value > 0.0 && value <= 4.0) }'; then
  echo "[FAIL] RESIDUAL_MPCC_SPEED_CAP must be in (0, 4.0]" >&2
  exit 2
fi

GENERATED_DIR="${MPCC_GENERATED_DIR:-$HOME/.cache/f1tenth_residual_mpcc/acados_stable_roboracer}"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROOT/log"
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

if [[ "${ROBORACER_OBSTACLE_PERCEPTION:-false}" == "true" ]]; then
  if [[ ! -f "$OBSTACLE_WS/devel/setup.bash" || ! -x "$OBSTACLE_START" ]]; then
    echo "[FAIL] obstacle-perception workspace is unavailable: $OBSTACLE_WS" >&2
    exit 1
  fi
  if rosnode list 2>/dev/null | grep -qx "$OBSTACLE_NODE"; then
    echo "[PERCEPTION] Reusing existing $OBSTACLE_NODE"
  else
    "$OBSTACLE_START" &
    OBSTACLE_PID=$!
    for _ in $(seq 1 30); do
      rosnode list 2>/dev/null | grep -qx "$OBSTACLE_NODE" && break
      kill -0 "$OBSTACLE_PID" 2>/dev/null || {
        echo "[FAIL] obstacle perception exited during startup" >&2
        exit 1
      }
      sleep 0.1
    done
    rosnode list 2>/dev/null | grep -qx "$OBSTACLE_NODE" || {
      echo "[FAIL] timed out waiting for $OBSTACLE_NODE" >&2
      exit 1
    }
  fi
else
  echo "[PERCEPTION] RoboRacer safe-start obstacle perception disabled"
fi

MODE_TAG="race_on"
if [[ "$RACE_START_ENABLED" == "false" ]]; then
  MODE_TAG="race_off"
fi
if [[ "${MPCC_RECORD_BAG:-false}" == "true" ]]; then
  mkdir -p "$ROOT/bags"
  BAG_PATH="${MPCC_RECORD_BAG_PATH:-$ROOT/bags/stable_roboracer_safe_start_${MODE_TAG}_${STAMP}.bag}"
  "$RECORDER" "$BAG_PATH" &
  RECORD_PID=$!
  echo "[RECORD] RoboRacer safe-start bag -> $BAG_PATH"
fi

echo "============================================================"
echo "[ROBORACER_SAFE_START] ISOLATED CANDIDATE; frozen RoboRacer is untouched"
echo "[ROBORACER_SAFE_START] race-start enabled=$RACE_START_ENABLED"
echo "[ROBORACER_SAFE_START] E-stop release probe=0.35 m/s"
echo "[ROBORACER_SAFE_START] PP max=1.60 m/s; PP accel/decel=1.00/3.00 m/s^2"
echo "[ROBORACER_SAFE_START] corridor slow/abort margin=0.15/0.02 m"
echo "[ROBORACER_SAFE_START] extracted-track safety gate is ENABLED"
echo "[ROBORACER_SAFE_START] runtime MPCC cap=$RESIDUAL_MPCC_SPEED_CAP m/s"
if [[ "$RACE_START_ENABLED" == "false" ]]; then
  echo "[ROBORACER_SAFE_START] grid launch mode OFF; placement falls back to guarded recovery"
fi
echo "[ROBORACER_SAFE_START] state=/automatic_relaunch_supervisor/state"
echo "[WARNING] Hold the physical E-stop until WAIT_RELEASE is shown."
echo "============================================================"

roslaunch f1tenth_dynamic_mpcc \
  hardware_mpcc_stable_roboracer_safe_start_candidate.launch \
  track:=raceline_smooth \
  controller_config:=candidates/stable_roboracer/controller.yaml \
  vehicle_config:=vehicle_stable_v3_racelineV3.yaml \
  formulation:=mpcc \
  allow_real_hardware:=true \
  race_start_enabled:="$RACE_START_ENABLED" \
  rviz:="${MPCC_REMOTE_RVIZ:-false}" \
  rviz_config:="$RVIZ_CONFIG" \
  runtime_speed_cap_mps:="$RESIDUAL_MPCC_SPEED_CAP" \
  generated_dir:="$GENERATED_DIR" \
  residual_model_path:="$RESIDUAL" \
  residual_feature_set:=markov_v1 \
  hardware_command_topic:="${AUTO_RELAUNCH_HARDWARE_COMMAND_TOPIC:-/tianracer/ackermann_cmd}" \
  telemetry_path:="$ROOT/log/hardware_stable_roboracer_safe_start_${MODE_TAG}_${STAMP}.jsonl" \
  lap_timer:="${AUTO_RELAUNCH_LAP_TIMER:-true}" \
  lap_log_path:="$ROOT/log/laps_stable_roboracer_safe_start_${MODE_TAG}_${STAMP}.csv"
