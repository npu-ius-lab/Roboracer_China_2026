#!/usr/bin/env python3
"""Read-only actuator candidate fit from a bag containing command and odom.

This tool never edits vehicle.yaml. Speed is fit as a first-order-plus-dead-time
model. Steering has no sensor feedback, so its result is explicitly only a
kinematic yaw-rate proxy and must not be promoted to an identified parameter.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def first_order(times, source_times, source, gain, tau, delay, initial):
    result = np.empty(len(times), dtype=float)
    result[0] = initial
    for index in range(1, len(times)):
        dt = times[index] - times[index - 1]
        midpoint = 0.5 * (times[index] + times[index - 1]) - delay
        command = np.interp(midpoint, source_times, source, left=0.0, right=0.0)
        decay = math.exp(-dt / tau)
        result[index] = decay * result[index - 1] + (1.0 - decay) * gain * command
    return result


def main() -> None:
    try:
        import rosbag
    except ImportError as exc:
        raise SystemExit("rosbag Python module is required") from exc
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--wheelbase", type=float, default=0.320)
    args = parser.parse_args()
    commands, odometry = [], []
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, receipt in bag.read_messages(
            topics=["/tianracer/ackermann_cmd", "/localization/vehicle_odom"]
        ):
            if topic == "/tianracer/ackermann_cmd":
                commands.append(
                    (receipt.to_sec(), message.speed, message.steering_angle)
                )
            else:
                odometry.append(
                    (
                        message.header.stamp.to_sec(),
                        receipt.to_sec(),
                        message.twist.twist.linear.x,
                        message.twist.twist.linear.y,
                        message.twist.twist.angular.z,
                    )
                )
    command = np.asarray(commands, dtype=float)
    odom = np.asarray(odometry, dtype=float)
    if len(command) < 20 or len(odom) < 30 or np.ptp(command[:, 1]) < 0.5:
        raise SystemExit("bag lacks an informative command/odom excitation")
    keep = (odom[:, 0] >= command[0, 0] - 0.30) & (
        odom[:, 0] <= command[-1, 0] + 0.30
    )
    odom = odom[keep]

    speed_candidates = []
    for delay in np.arange(0.0, 0.401, 0.01):
        def speed_residual(parameters):
            prediction = first_order(
                odom[:, 0], command[:, 0], command[:, 1],
                parameters[0], parameters[1], delay, odom[0, 2]
            )
            return prediction - odom[:, 2]

        fit = least_squares(
            speed_residual, [0.7, 0.15], bounds=([0.1, 0.02], [1.5, 1.0]),
            loss="soft_l1", f_scale=0.10
        )
        residual = speed_residual(fit.x)
        speed_candidates.append(
            (float(np.sqrt(np.mean(residual**2))), delay, *fit.x)
        )
    speed_rmse, speed_delay, speed_gain, speed_tau = min(speed_candidates)

    moving = (
        (odom[:, 2] > 0.5)
        & (np.abs(odom[:, 3]) < 0.4)
        & (np.abs(odom[:, 4]) < 3.0)
    )
    steering_candidates = []
    for delay in np.arange(0.0, 0.401, 0.01):
        def steering_residual(parameters):
            delta = first_order(
                odom[:, 0], command[:, 0], command[:, 2],
                parameters[0], parameters[1], delay, 0.0
            ) + parameters[2]
            yaw_rate = odom[:, 2] * np.tan(delta) / args.wheelbase
            return yaw_rate[moving] - odom[moving, 4]

        fit = least_squares(
            steering_residual, [1.0, 0.08, 0.0],
            bounds=([0.2, 0.02, -0.2], [3.0, 1.0, 0.2]),
            loss="soft_l1", f_scale=0.20
        )
        residual = steering_residual(fit.x)
        steering_candidates.append(
            (float(np.sqrt(np.mean(residual**2))), delay, *fit.x)
        )
    steer_rmse, steer_delay, steer_gain, steer_tau, steer_offset = min(
        steering_candidates
    )
    ages = odom[:, 1] - odom[:, 0]
    report = {
        "bag": str(args.bag.resolve()),
        "samples": {"command": len(command), "odom_used": len(odom)},
        "point_lio_age_s": {
            "mean": float(np.mean(ages)),
            "p95": float(np.quantile(ages, 0.95)),
            "max": float(np.max(ages)),
        },
        "speed_candidate_not_applied": {
            "static_gain": float(speed_gain),
            "time_constant_s": float(speed_tau),
            "dead_time_s": float(speed_delay),
            "fit_rmse_mps": float(speed_rmse),
        },
        "steering_kinematic_proxy_not_identification": {
            "static_gain": float(steer_gain),
            "offset_rad": float(steer_offset),
            "time_constant_s": float(steer_tau),
            "dead_time_s": float(steer_delay),
            "yaw_rate_fit_rmse_radps": float(steer_rmse),
            "moving_samples": int(np.sum(moving)),
            "warning": "No steering feedback; tire dynamics and steering cannot be separated.",
        },
    }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
