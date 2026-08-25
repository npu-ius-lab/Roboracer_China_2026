#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"

if ! rosnode list >/dev/null 2>&1; then
  echo "[FAIL] ROS master unavailable at $ROS_MASTER_URI" >&2
  exit 2
fi

check_type() {
  local topic="$1"
  local expected="$2"
  local actual
  actual="$(rostopic type "$topic" 2>/dev/null || true)"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] $topic expected $expected, got ${actual:-missing}" >&2
    return 1
  fi
  echo "[OK] $topic : $actual"
}

check_type /localization/vehicle_odom nav_msgs/Odometry
check_type /tianracer/odom nav_msgs/Odometry
check_type /tianracer/ackermann_cmd ackermann_msgs/AckermannDrive

python3 - <<'PY'
import rospy
from nav_msgs.msg import Odometry

rospy.init_node("f1tenth_mpcc_preflight", anonymous=True, disable_signals=True)
try:
    odom = rospy.wait_for_message("/localization/vehicle_odom", Odometry, timeout=5.0)
except rospy.ROSException:
    raise SystemExit(
        "[FAIL] /localization/vehicle_odom has the correct type but no fresh message for 5 s; "
        "check Point-LIO input /aft_mapped_to_init and /cloud_registered"
    )
frame = odom.header.frame_id.lstrip("/")
child = odom.child_frame_id.lstrip("/")
if (frame, child) != ("map", "localization_base_link"):
    raise SystemExit(
        f"[FAIL] /localization/vehicle_odom frames are {frame}->{child}, "
        "expected map->localization_base_link"
    )
print(f"[OK] /localization/vehicle_odom frames: {frame}->{child}")
PY

raw_info="$(rostopic info /tianracer/ackermann_cmd)"
existing_publishers="$(printf '%s\n' "$raw_info" | awk '
  /^Publishers:/ {inside=1; next}
  /^Subscribers:/ {inside=0}
  inside && /^ \* / {print}
')"
if [[ -n "$existing_publishers" ]]; then
  echo "[FAIL] Existing publisher(s) already own /tianracer/ackermann_cmd:" >&2
  printf '%s\n' "$existing_publishers" >&2
  echo "Stop old NMPC/teleop publishers before starting the new hardware gate." >&2
  exit 3
fi

echo "[OK] no competing publisher on /tianracer/ackermann_cmd"
echo "[PASS] MPCC topics and frames match the running vehicle stack"
