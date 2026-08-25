#!/usr/bin/env bash

# Shared low-bandwidth perception/overtake topics for MPCC rosbag recorders.
# A topic with no active publisher adds no bag traffic; the recorder simply
# starts capturing it when the corresponding module comes online.
LIGHTWEIGHT_OVERTAKE_TOPICS=(
  /lio/relocalization/lighterbev_match
  /scan
  /semantic_region/state
  /perception/targets_detailed
  /perception/target_markers
  /overtake/diagnostics
  /overtake/selected_path
  /overtake/valid_area
  /avoidance/status
  /avoidance/decision
  /avoidance/markers
  /avoidance/local_plan_red
)

# An aligned PointCloud2 is intentionally opt-in for the shared recorder: it
# is required to reproduce perception bugs, but is much larger than the
# status/target topics above. start_60hz_shadow.sh enables it by default;
# unchanged controller launchers that source this file do not.
if [[ "${MPCC_RECORD_OVERTAKE_ALIGNED_SCAN:-false}" == "true" ]]; then
  LIGHTWEIGHT_OVERTAKE_TOPICS+=(/localization/aligned_scan)
fi
