#!/usr/bin/env python3
"""Record a straight-line steering sine sweep for tire-stiffness identification.

Publishes a constant speed with a steering chirp (frequency sweeps from fmin
to fmax) and records all analysis topics, including the raw Point-LIO odom and
the 200 Hz Livox IMU. Run on an open straight area with the localization stack
alive. Ctrl-C or completion sends zero and finalizes the bag.
"""
from __future__ import annotations

import argparse
import math
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry


COMMAND_TOPIC = "/tianracer/ackermann_cmd"
STAMPED_TOPIC = "/f1tenth_identification/command_stamped"
ODOM_TOPIC = "/localization/vehicle_odom"
TOPICS = [
    COMMAND_TOPIC,
    STAMPED_TOPIC,
    ODOM_TOPIC,
    "/aft_mapped_to_init",
    "/localization/odom",
    "/tianracer/odom",
    "/livox/imu",
    "/tianracer/imu",
    "/localization/status",
    "/diagnostics",
    "/tf",
    "/tf_static",
]


def publish(raw_pub, stamped_pub, speed, steering):
    raw = AckermannDrive(speed=float(speed), steering_angle=float(steering))
    stamped = AckermannDriveStamped()
    stamped.header.stamp = rospy.Time.now()
    stamped.drive = raw
    raw_pub.publish(raw)
    stamped_pub.publish(stamped)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_id")
    parser.add_argument("--speed", type=float, default=2.5)
    parser.add_argument("--amplitude", type=float, default=0.15)
    parser.add_argument("--fmin", type=float, default=0.1)
    parser.add_argument("--fmax", type=float, default=1.2)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--pre-zero", type=float, default=2.0)
    parser.add_argument("--post-zero", type=float, default=3.0)
    args = parser.parse_args()

    if not 0.5 <= args.speed <= 3.0:
        raise SystemExit("--speed must be within [0.5, 3.0] m/s")
    if not 0.02 <= args.amplitude <= 0.28:
        raise SystemExit("--amplitude must be within [0.02, 0.28] rad")
    if not 0.02 <= args.fmin < args.fmax <= 2.0:
        raise SystemExit("need 0.02 <= fmin < fmax <= 2.0 Hz")

    rospy.init_node("record_steering_sweep")
    root = Path(__file__).resolve().parents[1]
    output_dir = root / "lateral_identification/data/steering_sweep"
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = output_dir / f"{args.run_id}_{stamp}"

    frames = None
    try:
        first = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=5.0)
        frames = (first.header.frame_id, first.child_frame_id)
    except rospy.ROSException as error:
        raise SystemExit(f"vehicle odometry is not live: {error}")
    if frames != ("map", "localization_base_link"):
        raise SystemExit(f"unexpected localization frames: {frames}")

    recorder = subprocess.Popen([
        "rosbag", "record", "--buffsize=256", "--chunksize=768", "-O",
        str(prefix), *TOPICS,
    ], start_new_session=True)
    raw_pub = rospy.Publisher(COMMAND_TOPIC, AckermannDrive, queue_size=1)
    stamped_pub = rospy.Publisher(STAMPED_TOPIC, AckermannDriveStamped, queue_size=10)
    rate = rospy.Rate(50)
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rospy.loginfo("ZERO %.1f s; sweep speed=%.2f m/s A=%.3f rad f=%.2f-%.2f Hz",
                  args.pre_zero, args.speed, args.amplitude, args.fmin, args.fmax)
    try:
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError("rosbag recorder exited during startup")
        deadline = time.monotonic() + 3.0
        while raw_pub.get_num_connections() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        if raw_pub.get_num_connections() < 1:
            raise RuntimeError("command publisher has no driver subscriber")

        pre_deadline = time.monotonic() + args.pre_zero
        while time.monotonic() < pre_deadline and not stop_requested[0]:
            publish(raw_pub, stamped_pub, 0.0, 0.0)
            rate.sleep()

        started = time.monotonic()
        total = max(args.duration, 1.0)
        while (not rospy.is_shutdown() and not stop_requested[0]
               and time.monotonic() - started < total):
            elapsed = min(time.monotonic() - started, total)
            f = args.fmin + (args.fmax - args.fmin) * elapsed / total
            phase = 2.0 * math.pi * (args.fmin * elapsed
                                     + 0.5 * (args.fmax - args.fmin)
                                     * elapsed * elapsed / total)
            # Scale down low-frequency amplitude so the lateral excursion stays
            # bounded inside a few metres while the chirp is slow.
            amplitude_scale = min(1.0, max(0.30, f / 0.6))
            publish(raw_pub, stamped_pub, args.speed,
                    args.amplitude * amplitude_scale * math.sin(phase))
            rate.sleep()
    finally:
        post_deadline = time.monotonic() + args.post_zero
        while time.monotonic() < post_deadline:
            publish(raw_pub, stamped_pub, 0.0, 0.0)
            rate.sleep()
        for _ in range(10):
            publish(raw_pub, stamped_pub, 0.0, 0.0)
            time.sleep(0.02)
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait(timeout=5.0)

    print(f"[STOPPED] zero command sent; bag={prefix}.bag")
    print(f"[SWEEP] speed={args.speed} amplitude={args.amplitude} "
          f"f={args.fmin}-{args.fmax} Hz")


if __name__ == "__main__":
    main()
