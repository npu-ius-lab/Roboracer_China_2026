# RViz 2D Pose Estimate with 3D refinement

The robust localizer accepts the standard RViz `2D Pose Estimate` message on
`/initialpose`. The arrow describes the vehicle-center pose in the `map`
frame, matching AMCL semantics.

Processing is intentionally transactional:

1. Keep the vehicle stopped and click/drag the RViz arrow to provide x, y and
   yaw.
2. The backend uses exactly one recent body-frame lidar scan. It builds a
   local submap around the click and runs Nano-GICP from the 2D prior.
3. Score, correspondence, translation and rotation gates must all pass.
4. On acceptance only `map -> odom` changes; Point-LIO `odom` remains
   continuous. A rejected request leaves the current localization untouched.

Useful interfaces:

- input: `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
- accepted refined pose: `/localization/initialpose_refined`
- result/reason and registration quality: `/localization/status`
- corrected vehicle-center odometry: `/localization/vehicle_odom`

An accepted request reports `rviz_initialpose_gicp_accepted`. Common rejection
reasons include `initialpose_rejected_vehicle_moving`,
`initialpose_rejected_request_timeout`, `registration_quality_rejected` and
`initialpose_correction_limit_rejected`.
