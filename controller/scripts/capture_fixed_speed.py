#!/usr/bin/env python3
"""Record identification signals while publishing one fixed raw command."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rosgraph
import rospy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry


TOPICS = [
    "/tianracer/ackermann_cmd", "/localization/vehicle_odom",
    "/tianracer/odom", "/tianracer/imu", "/livox/imu",
    "/localization/status", "/diagnostics", "/tf", "/tf_static",
]


def system_state():
    publishers, subscribers, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    return dict(publishers), dict(subscribers)


def publish_for(publisher, speed, steering, duration, stop_requested):
    rate = rospy.Rate(20.0)
    deadline = None if duration <= 0.0 else time.monotonic() + duration
    while not rospy.is_shutdown() and not stop_requested[0]:
        if deadline is not None and time.monotonic() >= deadline:
            break
        publisher.publish(AckermannDrive(speed=speed, steering_angle=steering))
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(
        description="Auto-record a fixed-speed straight-line calibration plateau.")
    parser.add_argument("split", choices=("fitting", "validation"))
    parser.add_argument("run_id", help="unique name, for example run01_050_up")
    parser.add_argument("speed", type=float, help="raw Ackermann speed command")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="fixed-speed seconds; 0 means hold until Ctrl-C")
    parser.add_argument("--pre-zero", type=float, default=2.0)
    parser.add_argument("--post-zero", type=float, default=2.0)
    parser.add_argument("--steering", type=float, default=0.0)
    parser.add_argument("--execute", action="store_true",
                        help="required acknowledgement that the vehicle may move")
    args = parser.parse_args()
    if not args.execute:
        raise SystemExit("Refusing to move: clear the straight area, hold the remote, then add --execute")
    if not 0.0 <= args.speed <= 3.0:
        raise SystemExit("speed must stay in [0, 3.0]")
    if args.duration != 0.0 and args.duration < 3.0:
        raise SystemExit("duration must be >=3 s, or 0 for continuous operation")
    if args.pre_zero < 1.0 or args.post_zero < 1.0:
        raise SystemExit("pre-zero and post-zero must each be >=1 s")

    rospy.init_node("speed_identification_fixed_capture", anonymous=False,
                    disable_signals=True)
    publishers, subscribers = system_state()
    existing = publishers.get("/tianracer/ackermann_cmd", [])
    if existing:
        raise SystemExit(f"command publisher already exists: {existing}")
    if not subscribers.get("/tianracer/ackermann_cmd"):
        raise SystemExit("TianRacer driver is not subscribed to the command topic")
    try:
        odom = rospy.wait_for_message("/localization/vehicle_odom", Odometry, timeout=5.0)
    except rospy.ROSException as error:
        raise SystemExit(f"vehicle-centre localization is not live: {error}")
    frames = (odom.header.frame_id.lstrip("/"), odom.child_frame_id.lstrip("/"))
    if frames != ("map", "localization_base_link"):
        raise SystemExit(f"unexpected localization frames: {frames}")

    root = Path(__file__).resolve().parents[1]
    output_dir = root / "speed_identification/data" / args.split
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = output_dir / f"{args.run_id}_{stamp}"
    recorder = subprocess.Popen([
        "rosbag", "record", "--buffsize=256", "--chunksize=768", "-O", str(prefix),
        *TOPICS,
    ], start_new_session=True)
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    publisher = rospy.Publisher("/tianracer/ackermann_cmd", AckermannDrive, queue_size=1)
    started = time.time()
    try:
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError("rosbag recorder exited during startup")
        deadline = time.monotonic() + 3.0
        while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        if publisher.get_num_connections() < 1:
            raise RuntimeError("no command subscriber connected")
        rospy.logwarn("ZERO %.1f s; recording %s.bag", args.pre_zero, prefix)
        publish_for(publisher, 0.0, args.steering, args.pre_zero, stop_requested)
        if not stop_requested[0]:
            text = "until Ctrl-C" if args.duration == 0.0 else f"{args.duration:.1f} s"
            rospy.logwarn("FIXED RAW COMMAND %.2f, steering %.3f, %s",
                          args.speed, args.steering, text)
            publish_for(publisher, args.speed, args.steering, args.duration, stop_requested)
    finally:
        # Ctrl-C still produces a deterministic zero-command tail.
        stop_requested[0] = False
        rospy.logwarn("FINAL ZERO %.1f s", args.post_zero)
        publish_for(publisher, 0.0, args.steering, args.post_zero, stop_requested)
        for _ in range(10):
            publisher.publish(AckermannDrive(speed=0.0, steering_angle=args.steering))
            time.sleep(0.05)
        if recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
            try:
                recorder.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(recorder.pid, signal.SIGTERM)
                recorder.wait(timeout=5.0)
        metadata = {
            "split": args.split, "run_id": args.run_id,
            "raw_speed_command": args.speed, "steering_command": args.steering,
            "duration_s": args.duration, "pre_zero_s": args.pre_zero,
            "post_zero_s": args.post_zero, "started_wall_time": started,
            "bag": str(prefix.with_suffix(".bag")),
        }
        prefix.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        rospy.logwarn("STOPPED; zero speed sent; bag=%s.bag", prefix)


if __name__ == "__main__":
    main()
