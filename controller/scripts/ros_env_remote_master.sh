#!/usr/bin/env bash
# Source on the visualization computer. The TianRacer is the ROS master.

ROS_MASTER_HOST="${ROS_MASTER_HOST:-192.168.43.59}"
ROS_LOCAL_IP="$(ip route get "$ROS_MASTER_HOST" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')"
if [[ -z "$ROS_LOCAL_IP" ]]; then
  echo "[FAIL] no route to ROS master $ROS_MASTER_HOST" >&2
  return 1 2>/dev/null || exit 1
fi

export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:11311"
export ROS_IP="$ROS_LOCAL_IP"
unset ROS_HOSTNAME
