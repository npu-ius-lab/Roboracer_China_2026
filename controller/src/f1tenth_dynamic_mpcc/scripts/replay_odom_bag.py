#!/usr/bin/env python3
"""Audit a recorded Point-LIO odometry bag without publishing ROS topics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle


def main() -> None:
    try:
        import rosbag
    except ImportError as exc:
        raise SystemExit("rosbag Python module is required") from exc
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/localization/vehicle_odom")
    parser.add_argument(
        "--track", type=Path, default=root / "data/tracks/virtual_track/raceline.csv"
    )
    args = parser.parse_args()
    track = PeriodicTrack(args.track)
    rows = []
    theta = None
    with rosbag.Bag(str(args.bag)) as bag:
        for _, message, receipt in bag.read_messages(topics=[args.topic]):
            q = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            projection = track.project(
                message.pose.pose.position.x, message.pose.pose.position.y, theta
            )
            theta = projection.s
            rows.append(
                {
                    "receipt": receipt.to_sec(),
                    "stamp": message.header.stamp.to_sec(),
                    "frame": message.header.frame_id,
                    "child": message.child_frame_id,
                    "vx": message.twist.twist.linear.x,
                    "vy": message.twist.twist.linear.y,
                    "yaw_rate": message.twist.twist.angular.z,
                    "e_contour": projection.e_contour,
                    "e_lag": projection.e_lag,
                    "e_heading": float(wrap_angle(yaw - projection.psi_ref)),
                }
            )
    if not rows:
        raise SystemExit(f"no messages on {args.topic}")
    ages = [row["receipt"] - row["stamp"] for row in rows]
    dts = [rows[index]["receipt"] - rows[index - 1]["receipt"] for index in range(1, len(rows))]
    report = {
        "messages": len(rows),
        "duration_s": rows[-1]["receipt"] - rows[0]["receipt"],
        "rate_hz": (len(rows) - 1) / max(sum(dts), 1.0e-9),
        "frames": sorted({(row["frame"], row["child"]) for row in rows}),
        "age_mean_s": statistics.fmean(ages),
        "age_p95_s": float(np.quantile(ages, 0.95)),
        "age_max_s": max(ages),
        "vx_min_mps": min(row["vx"] for row in rows),
        "vx_max_mps": max(row["vx"] for row in rows),
        "max_abs_contour_error_m": max(abs(row["e_contour"]) for row in rows),
        "max_abs_heading_error_rad": max(abs(row["e_heading"]) for row in rows),
    }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
