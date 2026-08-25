# raw_cloud_obstacle_perception

Independent ROS1 node for high-rate raw Point-LIO body clouds. It does not
modify or replace `opponent_perception`.

Pipeline:

1. Subscribe to `/cloud_registered_body` (`sensor_msgs/PointCloud2`).
2. Pre-crop to `0 < x < 6 m`, `|y| < 1.5 m`, `z < 0.30 m`.
3. Use the newest `/localization/odom` orientation to remove roll and pitch.
4. Remove the ego box and retain `0 < z < 0.30 m`; compensated `z <= 0` is ground.
5. Transform points into `map` with the newest odometry and retain only the
   Stable RoboRacer `raceline_smooth` left/right corridor, inset by 0.08 m.
6. Limit projection to the current local track branch (`s=-1..+8 m`) so nearby
   U-complex branches do not leak into the ROI.
7. Publish the track-corridor ROI.
8. Voxelize, Euclidean-cluster once, reject oversized clusters, and publish all
   accepted cluster points as the obstacle cloud.

Outputs:

- `/raw_cloud_perception/roi_cloud`
- `/raw_cloud_perception/obstacle_cloud`

Start on the car:

```bash
/home/tianbot/raw_cloud_obstacle_ws/start_raw_cloud_obstacle.sh
```

Stop without affecting localization or the legacy perception package:

```bash
/home/tianbot/raw_cloud_obstacle_ws/stop_raw_cloud_obstacle.sh
```

The package lives in its own workspace, so rollback is simply stopping the
node and moving `/home/tianbot/raw_cloud_obstacle_ws` out of service.
