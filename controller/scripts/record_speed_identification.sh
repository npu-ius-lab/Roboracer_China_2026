#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 fitting|validation RUN_ID" >&2
  exit 2
fi
split="$1"
run_id="$2"
if [[ "$split" != "fitting" && "$split" != "validation" ]]; then
  echo "Split must be fitting or validation" >&2
  exit 2
fi
if ! rostopic type /localization/vehicle_odom 2>/dev/null | grep -qx nav_msgs/Odometry; then
  echo "[FAIL] /localization/vehicle_odom is unavailable" >&2
  exit 3
fi
if ! rostopic type /tianracer/ackermann_cmd 2>/dev/null | grep -qx ackermann_msgs/AckermannDrive; then
  echo "[FAIL] TianRacer driver does not expose the expected Ackermann command interface" >&2
  exit 3
fi
python3 - <<'PY'
import rospy
from nav_msgs.msg import Odometry

rospy.init_node("speed_identification_preflight", anonymous=True, disable_signals=True)
try:
    odom = rospy.wait_for_message("/localization/vehicle_odom", Odometry, timeout=5.0)
except rospy.ROSException as error:
    raise SystemExit(f"[FAIL] localization preflight timed out: {error}")
frames = (odom.header.frame_id.lstrip("/"), odom.child_frame_id.lstrip("/"))
if frames != ("map", "localization_base_link"):
    raise SystemExit(f"[FAIL] unexpected vehicle odometry frames: {frames}")
print("[OK] localized vehicle-centre odometry is live")
PY

out="$ROOT/speed_identification/data/$split"
mkdir -p "$out"
stamp="$(date +%Y%m%d_%H%M%S)"
prefix="$out/${run_id}_${stamp}"
echo "[RECORD] $prefix.bag"
echo "This script publishes no command. Stop with Ctrl-C."
exec rosbag record --buffsize=256 --chunksize=768 -O "$prefix" \
  /tianracer/ackermann_cmd \
  /localization/vehicle_odom \
  /tianracer/odom \
  /tianracer/imu \
  /livox/imu \
  /localization/status \
  /diagnostics \
  /tf /tf_static
