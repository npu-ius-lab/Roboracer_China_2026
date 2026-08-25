#!/usr/bin/env bash
# Source on either computer. The TianRacer remains the ROS master while each
# node advertises the address that belongs to the machine running it.

ROS_MASTER_HOST="${ROS_MASTER_HOST:-192.168.43.59}"
ROS_LOCAL_IP="$(ip route get "$ROS_MASTER_HOST" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')"
if ip -o -4 address show | awk '{print $4}' | cut -d/ -f1 | grep -qx "$ROS_MASTER_HOST"; then
  ROS_LOCAL_IP="$ROS_MASTER_HOST"
fi
if [[ -z "$ROS_LOCAL_IP" ]]; then
  echo "[FAIL] no route to ROS master $ROS_MASTER_HOST" >&2
  return 1 2>/dev/null || exit 1
fi

export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:11311"
export ROS_IP="$ROS_LOCAL_IP"
unset ROS_HOSTNAME
unset ROS_LOCAL_IP
