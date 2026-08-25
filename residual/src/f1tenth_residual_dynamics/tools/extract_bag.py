#!/usr/bin/env python3
"""Extract an aligned residual-dynamics dataset from a ROS1 bag."""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

try:
    import rosbag
except ImportError as error:
    raise SystemExit("rosbag Python module missing; source /opt/ros/noetic/setup.bash") from error


TOPICS = (
    "/localization/vehicle_odom",
    "/tianracer/ackermann_cmd",
    "/f1tenth_mpcc/real/ackermann_cmd_stamped",
    "/tianracer/odom",
    "/livox/imu",
    "/f1tenth_mpcc/telemetry",
)

TELEMETRY_FIELDS = (
    "contour_error", "lag_error", "heading_error", "sideslip",
    "front_slip", "rear_slip", "published_speed", "published_steering",
    "pointlio_age", "solve_time", "failure_count", "solver_status",
    "solver_success", "track_slack", "tire_slack", "telemetry_vx",
    "telemetry_vy", "telemetry_yaw_rate", "near_track_slack", "far_track_slack",
    "track_slack_stage", "track_slack_time", "track_slack_side", "speed_cap_used",
    "speed_retry", "warm_start_reset", "failure_kind", "rti_iterations",
)


def stamp(message, bag_stamp):
    header = getattr(message, "header", None)
    if header is not None and not header.stamp.is_zero():
        return header.stamp.to_sec()
    return bag_stamp.to_sec()


def yaw_from_quaternion(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(sin_yaw, cos_yaw)


def zoh(times, values, query, max_age=np.inf):
    if not times:
        shape = (len(query),) + np.asarray(values).shape[1:]
        return np.full(shape, np.nan), np.full(len(query), np.inf)
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    order = np.argsort(times, kind="stable")
    times, values = times[order], values[order]
    indices = np.searchsorted(times, query, side="right") - 1
    valid = indices >= 0
    indices = np.clip(indices, 0, len(times) - 1)
    age = np.asarray(query) - times[indices]
    valid &= (age >= -1.0e-6) & (age <= max_age)
    output = values[indices].copy()
    output[~valid] = np.nan
    age[~valid] = np.inf
    return output, age


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-command-age", type=float, default=0.30)
    args = parser.parse_args()
    bag_path = args.bag.expanduser().resolve()
    output = (args.output or bag_path.with_suffix(".npz")).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    odom = []
    final_t, final_u = [], []
    request_t, request_u = [], []
    wheel_t, wheel_vx = [], []
    imu_t, imu_r = [], []
    telemetry_t, telemetry = [], []
    topic_counts = {}

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=TOPICS):
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            bag_time = bag_stamp.to_sec()
            if topic == "/localization/vehicle_odom":
                source_time = stamp(message, bag_stamp)
                pose, twist = message.pose.pose, message.twist.twist
                odom.append((source_time, bag_time, pose.position.x, pose.position.y,
                             yaw_from_quaternion(pose.orientation), twist.linear.x,
                             twist.linear.y, twist.angular.z))
            elif topic == "/tianracer/ackermann_cmd":
                final_t.append(bag_time)
                final_u.append((message.speed, message.steering_angle))
            elif topic == "/f1tenth_mpcc/real/ackermann_cmd_stamped":
                request_t.append(stamp(message, bag_stamp))
                request_u.append((message.drive.speed, message.drive.steering_angle))
            elif topic == "/tianracer/odom":
                wheel_t.append(stamp(message, bag_stamp))
                wheel_vx.append(message.twist.twist.linear.x)
            elif topic == "/livox/imu":
                imu_t.append(stamp(message, bag_stamp))
                imu_r.append(message.angular_velocity.z)
            elif topic == "/f1tenth_mpcc/telemetry":
                telemetry_t.append(bag_time)
                row = np.full(len(TELEMETRY_FIELDS), np.nan)
                values = np.asarray(message.data, dtype=float)
                row[:min(len(row), len(values))] = values[:len(row)]
                telemetry.append(row)

    if len(odom) < 10:
        raise SystemExit("not enough /localization/vehicle_odom samples in bag")
    odom = np.asarray(odom, dtype=float)
    # Preserve reception order and remove regressions/duplicates instead of sorting
    # them into an apparently valid trajectory.
    keep, last = [], -np.inf
    regressions = 0
    for index, value in enumerate(odom[:, 0]):
        if value > last + 1.0e-9:
            keep.append(index)
            last = value
        else:
            regressions += 1
    odom = odom[keep]
    t = odom[:, 0]

    final_u = np.asarray(final_u, dtype=float).reshape((-1, 2))
    request_u = np.asarray(request_u, dtype=float).reshape((-1, 2))
    telemetry = np.asarray(telemetry, dtype=float).reshape((-1, len(TELEMETRY_FIELDS)))
    if len(final_u) < 2:
        raise SystemExit("bag has no usable /tianracer/ackermann_cmd sequence")

    command, command_age = zoh(final_t, final_u, t, args.max_command_age)
    requested, request_age = zoh(request_t, request_u, t, args.max_command_age)
    # A manual-control bag has no MPCC request topic. Final command remains the
    # authoritative training input; requested fields stay NaN by design.
    wheel, wheel_age = zoh(wheel_t, wheel_vx, t, 0.15)
    imu, imu_age = zoh(imu_t, imu_r, t, 0.05)
    diagnostic, diagnostic_age = zoh(telemetry_t, telemetry, t, 0.15)

    valid_command = np.all(np.isfinite(command), axis=1)
    if np.count_nonzero(valid_command) < 10:
        raise SystemExit("not enough fresh /tianracer/ackermann_cmd samples")

    fields = {
        "t": t, "bag_t": odom[:, 1], "x": odom[:, 2], "y": odom[:, 3],
        "yaw": odom[:, 4], "vx": odom[:, 5], "vy": odom[:, 6],
        "yaw_rate": odom[:, 7], "cmd_speed": command[:, 0],
        "cmd_steer": command[:, 1], "command_age": command_age,
        "requested_speed": requested[:, 0], "requested_steer": requested[:, 1],
        "request_age": request_age, "wheel_vx": wheel, "wheel_age": wheel_age,
        "imu_yaw_rate": imu, "imu_age": imu_age,
    }
    for index, name in enumerate(TELEMETRY_FIELDS):
        fields[name] = diagnostic[:, index]
    fields["telemetry_age"] = diagnostic_age
    # Preserve the raw command sequence; it is needed for arbitrary-delay
    # histories during training, rather than only the odometry-aligned command.
    fields["command_t"] = np.asarray(final_t, dtype=float)
    fields["command_speed"] = final_u[:, 0]
    fields["command_steer"] = final_u[:, 1]

    np.savez_compressed(str(output), **fields)
    csv_path = output.with_suffix(".csv")
    row_keys = [key for key, value in fields.items() if np.ndim(value) == 1 and len(value) == len(t)]
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(row_keys)
        writer.writerows(zip(*(fields[key] for key in row_keys)))

    duration = float(t[-1] - t[0])
    metadata = {
        "schema_version": 1, "bag": str(bag_path), "npz": str(output),
        "csv": str(csv_path), "samples": int(len(t)), "duration_s": duration,
        "mean_odom_rate_hz": float((len(t) - 1) / duration),
        "timestamp_regressions_dropped": regressions,
        "valid_command_samples": int(np.count_nonzero(valid_command)),
        "topic_counts": topic_counts,
        "time_semantics": {
            "state": "vehicle_odom header stamp (sensor/source time)",
            "final_command": "rosbag receipt time; message has no header",
            "requested_command": "AckermannDriveStamped header stamp",
        },
    }
    with output.with_suffix(".json").open("w") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
