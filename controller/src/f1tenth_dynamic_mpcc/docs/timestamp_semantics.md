# TianRacer timestamp semantics audit

Date: 2026-08-12

This report defines the physical meaning of every timestamp used by the
low-latency state predictor. It must be updated if a driver or clock source is
changed.

## Clock domain

The online system uses ROS 1 wall time (`/use_sim_time` is not used during
hardware operation). PointLIO copies Livox LiDAR/IMU sensor timestamps through
its estimator, so `/livox/imu`, `/aft_mapped_to_init` and
`/localization/vehicle_odom` are numerically compatible ROS timestamps on the
current host. This establishes a compatible clock domain, but does not prove
sub-millisecond hardware synchronization: that still depends on MID360 time
synchronization and the Livox driver configuration.

If timestamps regress, lie in the future, or differ systematically after a
driver/firmware change, IMU correction must be disabled rather than presented
as exact latency compensation.

## Topic semantics

| Topic | Producer | `header.stamp` meaning | Observed rate | Online use |
|---|---|---|---:|---|
| `/livox/imu` | Livox driver | IMU sample time translated into ROS time | ~200 Hz | low-latency yaw-rate correction |
| `/aft_mapped_to_init` | PointLIO | estimator state time (`time_current` in high-rate mode) | currently <=50 Hz | localization frontend input |
| `/localization/vehicle_odom` | robust localizer | copied unchanged from PointLIO odometry | ~33 Hz in bags | delayed vehicle-centre measurement |
| `/tianracer/odom` | Tianbot serial driver | host receive/publish time, not MCU encoder sample time | ~50 Hz | small-gain low-latency `vx` correction |
| `/tianracer/ackermann_cmd` | command gate | message has no header; publisher ROS time is stored separately | 20 Hz | committed command history |

PointLIO source code sets high-rate odometry stamps from `time_current`. The
robust localizer copies the incoming stamp to both corrected odometry and
`/localization/vehicle_odom`; it does not stamp those outputs with callback
time. Therefore:

```text
pointlio_message_age = control_now - vehicle_odom.header.stamp
```

is the age of the represented state and must be evaluated per message. The
measured ~0.11 s median is monitoring evidence, not a fixed delay constant.

The Tianbot serial driver assigns `ros::Time::now()` after receiving an odom
packet. Its timestamp omits MCU encoder sampling, computation and serial
waiting time. Wheel odometry is consequently useful as a recent correction,
but its timestamp is not precise enough to act as ground truth for actuator
dead-time identification.

## Vehicle reference point

`/localization/vehicle_odom` is already expressed at the vehicle centre:

```text
frame_id:       map
child_frame_id: localization_base_link
```

The robust localizer has already applied the measured 0.135 m body/MID360 to
vehicle-centre rigid transform, including the angular-velocity cross-product
term for linear velocity. Consumers must **not** apply that transform again.

## Required telemetry

Every control tick records these distinct times:

```text
control_now
pointlio_header_stamp
pointlio_receive_time
pointlio_message_age
measurement_prediction_start/end/horizon
speed_dead_time
committed_prediction_start/end/horizon
mpcc_initial_state_timestamp
```

The receive time is captured in the odometry callback. The control time is
captured at the beginning of the MPCC timer callback. Neither replaces the
sensor/estimator timestamp.

## Validity policy

- Reject non-finite and future timestamps.
- Reject timestamp regression and large pose jumps.
- Do not repropagate beyond the configured maximum history.
- Treat Livox gyro correction as unavailable if its clock compatibility or
  sample freshness check fails.
- Never combine PointLIO measurement age and actuator dead time into one
  constant.

