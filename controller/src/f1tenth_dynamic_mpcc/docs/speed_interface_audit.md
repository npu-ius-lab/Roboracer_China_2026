# Speed compensation interface audit

Audit date: 2026-08-12, ROS 1 Noetic, host `tianbot`.

## Selected interfaces

| Signal | Topic | Type | Timestamp/frame interpretation |
|---|---|---|---|
| Raw command | `/tianracer/ackermann_cmd` | `ackermann_msgs/AckermannDrive` | No header; use bag receipt time. `speed` is uncalibrated. |
| Vehicle-centre localization | `/localization/vehicle_odom` | `nav_msgs/Odometry` | Header stamp; `map -> localization_base_link`; body-axis twist at vehicle centre. |
| Original localization | `/localization/odom` | `nav_msgs/Odometry` | `map -> body`; body is the MID360/IMU origin. |
| Wheel odometry | `/tianracer/odom` | `nav_msgs/Odometry` | `tianracer/odom -> tianracer/base_link`; diagnostic reference until scale/slip are checked. |
| Chassis IMU | `/tianracer/imu` | `sensor_msgs/Imu` | `tianracer/imu_link`; approximately 50 Hz. |
| MID360 IMU | `/livox/imu` | `sensor_msgs/Imu` | `livox_frame`; approximately 200 Hz. |
| Diagnostics | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Advertised, but no live sample during audit. Battery remains optional. |

The physical order is `front -> MID360/body -> vehicle centre`. The measured
translation is `base -> body = (+0.135, 0, 0) m`. The adapter publishes
`body -> localization_base_link = (-0.135, 0, 0) m` and applies:

```text
vx_base = vx_body
vy_base = vy_body - 0.135 * yaw_rate
```

`tianracer/base_link` and `localization_base_link` denote the same physical
point but keep different names because they belong to different odometry trees.

## Observed rates and timing

- `/livox/lidar`: about 10 Hz.
- `/livox/imu`: about 200 Hz.
- `/aft_mapped_to_init`: about 47.5 Hz.
- `/cloud_registered`: about 10 Hz.
- `/tianracer/odom`: about 50 Hz, live header age about 0.01 s.
- `/tianracer/imu`: about 50 Hz, live header age about 0.01 s.

At audit time the new localization topic was advertised but not publishing,
because startup relocalization reported `no_lighterbev_candidate`. A run must
not begin until `/localization/vehicle_odom` is live and localized.

## Safety and data restrictions

- No collision topic is required by this vehicle.
- No command publisher was present during the audit.
- `2026-08-12-00-37-29.bag` contains only `/localization/odom`; it is invalid
  calibration data.
- Fit and validation bags must be separate.
- Calibration must clamp to its measured range and must not enter MPCC until
  independent validation passes.
