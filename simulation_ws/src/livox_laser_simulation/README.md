# Livox MID360 Gazebo 11 simulation

This package is a ROS Noetic/Gazebo 11 port of Livox's MIT-licensed
[`livox_laser_simulation`](https://github.com/Livox-SDK/livox_laser_simulation),
upstream commit `1cce1073633a062b92e30243a4c2920e45551bb5`.

The retained `mid360.csv` is Livox's non-repetitive MID360 scan pattern. The
ported C++ sensor performs one ODE ray pass and directly publishes both:

- `livox_ros_driver/CustomMsg` for Point-LIO;
- `sensor_msgs/PointCloud2` in the vehicle body frame for obstacle perception.

This removes the former rectangular block-laser approximation and both Python
point-cloud conversion passes. `samples` is the 24,000-point source window;
`downsample` controls deterministic ray thinning and is exposed as the
`mid360_downsample` argument of every RoboRacer simulation launcher.

The original copyright and MIT license are retained in [LICENSE](LICENSE).
