#!/usr/bin/env python3
"""Counterfactual regression for the V5 Chaoche locked-path refresh guard.

The recorded vehicle stops shortly after the old first-pose guard rejects the
path, so this test only evaluates samples before the first MPCC command reaches
zero. It proves that the recorded failure contains a path whose first pose is
too far away while a later pose is still next to the vehicle.
"""

from __future__ import annotations

import argparse
import math

import rosbag


ODOM_TOPIC = "/localization/odom"
PATH_TOPIC = "/v5_chaoche/local_reference"
COMMAND_TOPIC = "/v5_chaoche/mpcc/ackermann_cmd_stamped"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--threshold", type=float, default=1.25)
    args = parser.parse_args()

    odom = None
    last_nonzero_command_time = None
    recovered = []
    with rosbag.Bag(args.bag) as bag:
        for topic, message, stamp in bag.read_messages(
            topics=[ODOM_TOPIC, PATH_TOPIC, COMMAND_TOPIC]
        ):
            if topic == ODOM_TOPIC:
                odom = message
                continue
            if topic == COMMAND_TOPIC:
                if abs(float(message.drive.speed)) > 1.0e-3:
                    last_nonzero_command_time = stamp.to_sec()
                continue
            if odom is None or not message.poses:
                continue
            if (
                last_nonzero_command_time is not None
                and stamp.to_sec() > last_nonzero_command_time + 0.25
            ):
                continue
            x = float(odom.pose.pose.position.x)
            y = float(odom.pose.pose.position.y)
            distances = [
                math.hypot(
                    float(pose.pose.position.x) - x,
                    float(pose.pose.position.y) - y,
                )
                for pose in message.poses
                if math.isfinite(float(pose.pose.position.x))
                and math.isfinite(float(pose.pose.position.y))
            ]
            if not distances:
                continue
            if distances[0] > args.threshold and min(distances) <= args.threshold:
                recovered.append((stamp.to_sec(), distances[0], min(distances)))

    if not recovered:
        raise SystemExit(
            "FAIL: bag contains no old-reject/new-accept locked-path sample"
        )
    first = recovered[0]
    print(
        "PASS: fixed guard accepts %d recorded refreshes; "
        "first t=%.3f front=%.3fm nearest=%.3fm"
        % (len(recovered), first[0], first[1], first[2])
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
