#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 BAG_PATH OUTPUT_JSON [MASTER_PORT=11320] [PLAY_RATE=1.0] [HOLD_SECONDS=2]" >&2
  exit 2
fi

BAG_PATH="$1"
OUTPUT_JSON="$2"
MASTER_PORT="${3:-11320}"
PLAY_RATE="${4:-1.0}"
HOLD_SECONDS="${5:-2}"
AVOIDANCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MPCC_ROOT="$(cd "$AVOIDANCE_DIR/.." && pwd)"
PERCEPTION_WS="${PERCEPTION_WS:-/home/tianbot/perception}"
LOCALIZATION_WS="${LOCALIZATION_WS:-/home/tianbot/localization_main}"
MAP_DIR="${AVOIDANCE_MAP_DIR:-$LOCALIZATION_WS/src/point_lio_sam_lighterbev/point_lio/competition_map}"
MODEL_PATH="${AVOIDANCE_MODEL_PATH:-$LOCALIZATION_WS/src/point_lio_sam_lighterbev/models/pca_mid360_finetuned.pt}"
DESCRIPTOR_CACHE="${AVOIDANCE_DESCRIPTOR_CACHE:-$MAP_DIR/lighterbev_descriptors_pca_mid360_finetuned.bin}"
BEV_CONFIG="${AVOIDANCE_BEV_CONFIG:-$LOCALIZATION_WS/src/point_lio_sam_lighterbev/config/bev_geometry.yaml}"
RACELINE="${AVOIDANCE_RACELINE:-$MPCC_ROOT/src/f1tenth_dynamic_mpcc/data/tracks/racelinev3/raceline.csv}"
ROS_LOG_DIR="${AVOIDANCE_TEST_LOG_DIR:-/tmp/avoidance_mid360_${MASTER_PORT}}"

required_files=(
  "$BAG_PATH"
  "$LOCALIZATION_WS/source_mid360_env.sh"
  "$MAP_DIR/poses_tum.txt"
  "$MAP_DIR/point_lio_map.pcd"
  "$MODEL_PATH"
  "$DESCRIPTOR_CACHE"
  "$BEV_CONFIG"
  "$RACELINE"
)
for path in "${required_files[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "[FAIL] required file does not exist: $path" >&2
    exit 1
  fi
done

RESTORE_NOUNSET=false
case "$-" in
  *u*) RESTORE_NOUNSET=true ;;
esac
set +u
source "$LOCALIZATION_WS/source_mid360_env.sh"
# The perception workspace was built directly on /opt/ros, so a normal source
# would replace the localization overlay.  --extend preserves Point-LIO while
# adding opponent_perception.
source "$PERCEPTION_WS/devel/setup.bash" --extend
if [[ "$RESTORE_NOUNSET" == "true" ]]; then
  set -u
fi
unset RESTORE_NOUNSET

export PYTHONPATH="$AVOIDANCE_DIR/python:$PERCEPTION_WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_MASTER_URI="http://127.0.0.1:$MASTER_PORT"
export ROS_IP="${AVOIDANCE_TEST_ROS_IP:-127.0.0.1}"
export ROS_LOG_DIR
unset ROS_HOSTNAME 2>/dev/null || true
mkdir -p "$ROS_LOG_DIR"

PIDS=()
cleanup() {
  local index pid
  # Stop consumers/roslaunch first and the isolated roscore last so roslaunch
  # can still unregister and reap the localization child processes.
  for ((index=${#PIDS[@]}-1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  local elapsed=0
  until rosnode list 2>/dev/null | grep -Fxq "$node"; do
    if ((elapsed >= timeout_s * 4)); then
      echo "[FAIL] node did not start: $node" >&2
      return 1
    fi
    sleep 0.25
    ((elapsed += 1)) || true
  done
}

roscore -p "$MASTER_PORT" >"$ROS_LOG_DIR/roscore.log" 2>&1 &
PIDS+=("$!")
for _ in {1..100}; do
  rosnode list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

echo "[mid360-test] starting Point-LIO localization on isolated master $ROS_MASTER_URI"
roslaunch point_lio_sam_lighterbev localization.launch \
  lidar:=mid360 rviz:=false run_frontend:=true use_sim_time:=true \
  lighterbev_device:=cpu local_map_backend:=octvox \
  scan_world_publish_en:=false scan_world_downsample_publish_en:=false \
  scan_bodyframe_pub_en:=true map_dir:="$MAP_DIR" \
  model_path:="$MODEL_PATH" descriptor_cache_path:="$DESCRIPTOR_CACHE" \
  bev_geometry_config:="$BEV_CONFIG" \
  >"$ROS_LOG_DIR/localization.log" 2>&1 &
PIDS+=("$!")

echo "[mid360-test] starting existing 3D opponent perception"
roslaunch opponent_perception perception.launch \
  >"$ROS_LOG_DIR/perception.log" 2>&1 &
PIDS+=("$!")

wait_for_node /laserMapping 30
wait_for_node /pointlio_robust_localizer 30
wait_for_node /opponent_perception 15

rosparam load "$AVOIDANCE_DIR/config/overtake_hardware.yaml" /overtake_shadow
rosparam set /overtake_shadow/raceline_csv "$RACELINE"
rosparam set /overtake_shadow/topics/odom /localization/odom
rosparam set /overtake_shadow/input/odom_timeout_s 1.0
rosparam set /overtake_shadow/input/target_timeout_s 1.0
rosparam load "$AVOIDANCE_DIR/config/state_visualizer.yaml" /avoidance_state_visualizer
rosparam set /avoidance_state_visualizer/raceline_csv "$RACELINE"
rosparam set /avoidance_state_visualizer/topics/odom /localization/odom
rosparam set /avoidance_state_visualizer/input/odom_timeout_s 1.0
rosparam set /avoidance_state_visualizer/input/target_timeout_s 1.0

python3 "$AVOIDANCE_DIR/scripts/overtake_shadow_node.py" \
  __name:=overtake_shadow >"$ROS_LOG_DIR/overtake_shadow.log" 2>&1 &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/avoidance_state_visualizer.py" \
  __name:=avoidance_state_visualizer >"$ROS_LOG_DIR/visualizer.log" 2>&1 &
PIDS+=("$!")
python3 "$AVOIDANCE_DIR/scripts/bag_test_observer.py" \
  __name:=avoidance_bag_test_observer \
  _output_path:="$OUTPUT_JSON" _bag_path:="$BAG_PATH" &
OBSERVER_PID="$!"
PIDS+=("$OBSERVER_PID")

sleep 3
echo "[mid360-test] replaying bag=$BAG_PATH rate=$PLAY_RATE"
rosbag play "$BAG_PATH" --quiet --clock --rate "$PLAY_RATE" \
  --topics /livox/lidar /livox/imu /tf /tf_static

if [[ "$HOLD_SECONDS" != "0" ]]; then
  echo "[mid360-test] holding visualization for ${HOLD_SECONDS}s"
  sleep "$HOLD_SECONDS"
fi
kill -INT "$OBSERVER_PID" 2>/dev/null || true
wait "$OBSERVER_PID" 2>/dev/null || true
echo "[mid360-test] result=$OUTPUT_JSON logs=$ROS_LOG_DIR"
