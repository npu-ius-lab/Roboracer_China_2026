#!/usr/bin/env python3
"""Publish a guarded straight-line Ackermann step sequence for calibration."""

from __future__ import annotations

import argparse
import signal
import sys
import time

import rospy
from ackermann_msgs.msg import AckermannDrive


UP_LEVELS = [0.0, 0.50, 0.75, 1.00, 1.25, 1.50, 2.00, 0.0]
DOWN_LEVELS = [0.0, 2.00, 1.50, 1.25, 1.00, 0.75, 0.50, 0.0]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Guarded ROS speed steps; keep the physical remote ready.")
    parser.add_argument("direction", choices=("up", "down"))
    parser.add_argument("--execute", action="store_true",
                        help="required acknowledgement that the vehicle may move")
    parser.add_argument("--hold", type=float, default=4.0,
                        help="seconds per nonzero plateau")
    parser.add_argument("--zero-hold", type=float, default=2.0,
                        help="seconds at initial/final zero")
    args = parser.parse_args()
    if not args.execute:
        raise SystemExit("Refusing to move: add --execute after clearing the straight test area")
    if args.hold < 3.0 or args.zero_hold < 1.0:
        raise SystemExit("hold must be >=3 s and zero-hold >=1 s")

    rospy.init_node("speed_identification_step_publisher", anonymous=False)
    topic = "/tianracer/ackermann_cmd"
    publishers = [item for item in rospy.get_published_topics() if item[0] == topic]
    if publishers:
        raise SystemExit(f"Refusing to start: another publisher already owns {topic}")
    publisher = rospy.Publisher(topic, AckermannDrive, queue_size=1)
    rate = rospy.Rate(20.0)
    stopping = False

    def publish(speed: float) -> None:
        publisher.publish(AckermannDrive(speed=speed, steering_angle=0.0))

    def stop_handler(_signum=None, _frame=None) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    time.sleep(0.5)
    if publisher.get_num_connections() < 1:
        raise SystemExit(f"No TianRacer subscriber on {topic}")

    levels = UP_LEVELS if args.direction == "up" else DOWN_LEVELS
    try:
        for index, level in enumerate(levels):
            if stopping or rospy.is_shutdown():
                break
            duration = args.zero_hold if level == 0.0 else args.hold
            rospy.logwarn("CALIBRATION STEP %d/%d: raw command %.2f for %.1f s",
                          index + 1, len(levels), level, duration)
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline and not stopping and not rospy.is_shutdown():
                publish(level)
                rate.sleep()
    finally:
        # Send enough zero commands to survive queue/transport jitter.
        for _ in range(20):
            publish(0.0)
            time.sleep(0.05)
        rospy.logwarn("Calibration sequence stopped; zero speed published")


if __name__ == "__main__":
    main()
