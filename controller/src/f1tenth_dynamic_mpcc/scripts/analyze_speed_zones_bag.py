#!/usr/bin/env python3
"""Summarize measured and commanded speeds by nearest raceline speed zone."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import rosbag

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack
from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("track", type=Path)
    parser.add_argument("--controller-config", type=Path)
    parser.add_argument("--vehicle-config", type=Path)
    args = parser.parse_args()
    with args.track.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    points = [(float(row["x_m"]), float(row["y_m"])) for row in rows]
    zones = [row.get("speed_zone", "standard") for row in rows]
    limits = [float(row.get("speed_limit_mps", "inf")) for row in rows]
    planned_limits = limits
    acceleration_limit = 1.5
    deceleration_limit = 2.0
    if bool(args.controller_config) != bool(args.vehicle_config):
        parser.error("controller and vehicle configs must be provided together")
    if args.controller_config:
        controller = load_yaml(args.controller_config)
        vehicle = VehicleParameters.from_yaml(args.vehicle_config, controller)
        planning = dict(controller["speed_planning"])
        planning.update(
            wheelbase_m=vehicle.wheelbase,
            max_steer_rad=vehicle.max_steer,
            max_steer_rate_radps=vehicle.max_steer_rate,
            max_accel_mps2=vehicle.max_accel,
            max_decel_mps2=vehicle.max_decel,
            lateral_accel_limit_mps2=vehicle.lateral_accel_limit,
        )
        runtime_track = PeriodicTrack(args.track, speed_planning=planning)
        planned_limits = list(runtime_track.speed_nodes)
        acceleration_limit = float(controller["publisher"]["acceleration_limit_mps2"])
        deceleration_limit = float(controller["publisher"]["deceleration_limit_mps2"])

    stats: dict[str, dict[str, float | int]] = defaultdict(
        lambda: {
            "odometry_samples": 0,
            "command_samples": 0,
            "maximum_measured_speed_mps": 0.0,
            "maximum_command_speed_mps": 0.0,
            "measured_over_local_limit_samples": 0,
            "command_over_local_limit_samples": 0,
            "maximum_simulated_hard_cap_command_mps": 0.0,
            "simulated_hard_cap_over_raw_limit_samples": 0,
        }
    )
    current_zone = "unknown"
    current_limit = math.inf
    current_planned_limit = math.inf
    simulated_command = 0.0
    previous_command_stamp: float | None = None
    topics = ["/localization/vehicle_odom", "/tianracer/ackermann_cmd"]
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            if topic == "/localization/vehicle_odom":
                px = float(message.pose.pose.position.x)
                py = float(message.pose.pose.position.y)
                index = min(
                    range(len(points)),
                    key=lambda i: (points[i][0] - px) ** 2 + (points[i][1] - py) ** 2,
                )
                current_zone = zones[index]
                current_limit = limits[index]
                current_planned_limit = planned_limits[index]
                speed = abs(float(message.twist.twist.linear.x))
                zone_stats = stats[current_zone]
                zone_stats["odometry_samples"] += 1
                zone_stats["maximum_measured_speed_mps"] = max(
                    float(zone_stats["maximum_measured_speed_mps"]), speed
                )
                if speed > current_limit + 0.05:
                    zone_stats["measured_over_local_limit_samples"] += 1
            elif current_zone != "unknown":
                speed = abs(float(message.speed))
                stamp = float(_.to_sec())
                dt = 0.02 if previous_command_stamp is None else max(
                    0.001, min(stamp - previous_command_stamp, 0.1)
                )
                previous_command_stamp = stamp
                target = min(6.0, current_planned_limit)
                change = target - simulated_command
                if change >= 0.0:
                    change = min(change, acceleration_limit * dt)
                else:
                    change = max(change, -deceleration_limit * dt)
                simulated_command += change
                zone_stats = stats[current_zone]
                zone_stats["command_samples"] += 1
                zone_stats["maximum_command_speed_mps"] = max(
                    float(zone_stats["maximum_command_speed_mps"]), speed
                )
                if speed > current_limit + 0.05:
                    zone_stats["command_over_local_limit_samples"] += 1
                zone_stats["maximum_simulated_hard_cap_command_mps"] = max(
                    float(zone_stats["maximum_simulated_hard_cap_command_mps"]),
                    simulated_command,
                )
                if simulated_command > current_limit + 0.05:
                    zone_stats["simulated_hard_cap_over_raw_limit_samples"] += 1

    print(json.dumps(stats, indent=2, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
