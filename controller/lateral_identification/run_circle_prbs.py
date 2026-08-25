#!/usr/bin/env python3
"""Fixed-radius circle with small steering PRBS for actuator identification."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rosgraph
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


COMMAND_TOPIC = "/tianracer/ackermann_cmd"
STAMPED_TOPIC = "/f1tenth_identification/command_stamped"
ODOM_TOPIC = "/localization/vehicle_odom"
TOPICS = [
    # Constant-circle / steering identification inputs.
    COMMAND_TOPIC, STAMPED_TOPIC, ODOM_TOPIC,
    "/aft_mapped_to_init", "/localization/odom", "/tianracer/odom",
    "/livox/imu", "/tianracer/imu", "/localization/status",
    "/lio/relocalization/lighterbev_match",
    # Residual-dynamics and MPCC analysis compatibility.  The MPCC-only topics
    # are expected to have no messages when this standalone command publisher
    # is correctly running with MPCC stopped.
    "/f1tenth_mpcc/real/ackermann_cmd_stamped",
    "/f1tenth_mpcc/telemetry", "/f1tenth_mpcc/prediction",
    "/f1tenth_mpcc/markers",
    "/diagnostics", "/tf", "/tf_static",
]
WHEELBASE_M = 0.320
STEERING_GAIN = 1.147
STEERING_BIAS = -0.015
PRBS_SIGNS = (1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1)
PRBS_DWELL_S = (0.25, 0.35, 0.45, 0.60)


def command_for_effective_angle(effective_angle: float) -> float:
    """Invert the current effective command-to-wheel-angle fit."""
    return (effective_angle - STEERING_BIAS) / STEERING_GAIN


def scheduled_excitation(elapsed: float, amplitude: float) -> float:
    cycle = sum(PRBS_DWELL_S[index % len(PRBS_DWELL_S)]
                for index in range(len(PRBS_SIGNS)))
    cursor = elapsed % cycle
    for index, sign in enumerate(PRBS_SIGNS):
        dwell = PRBS_DWELL_S[index % len(PRBS_DWELL_S)]
        if cursor < dwell:
            return amplitude * sign
        cursor -= dwell
    return amplitude * PRBS_SIGNS[-1]


class OdomMonitor:
    def __init__(self) -> None:
        self.last_receipt = time.monotonic()
        self.vx = 0.0
        self.position = None
        self.sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.callback, queue_size=10)

    def callback(self, message: Odometry) -> None:
        self.last_receipt = time.monotonic()
        self.vx = float(message.twist.twist.linear.x)
        self.position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def check(self) -> None:
        if time.monotonic() - self.last_receipt > 0.35:
            raise RuntimeError("vehicle odometry became stale")


def publish(raw_pub, stamped_pub, speed: float, steering: float) -> None:
    stamp = rospy.Time.now()
    raw = AckermannDrive(speed=float(speed), steering_angle=float(steering))
    stamped = AckermannDriveStamped()
    stamped.header.stamp = stamp
    stamped.header.frame_id = "localization_base_link"
    stamped.drive = raw
    raw_pub.publish(raw)
    stamped_pub.publish(stamped)


def system_state():
    publishers, subscribers, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    return dict(publishers), dict(subscribers)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record fixed-radius circle motion with optional steering PRBS excitation.")
    parser.add_argument("run_id")
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--max-speed", type=float, default=2.5,
        help="maximum accepted speed for this experiment (default: 2.5 m/s)")
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--excitation-deg", type=float, default=1.5)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--pre-zero", type=float, default=2.0)
    parser.add_argument("--post-zero", type=float, default=2.0)
    parser.add_argument("--max-lateral-accel", type=float, default=3.3)
    parser.add_argument(
        "--output-dir", type=Path,
        help="bag/metadata directory (default: lateral_identification/data/circle_prbs)")
    parser.add_argument("--plan", action="store_true")
    args = parser.parse_args()

    if not 1.0 <= args.radius <= 10.0:
        raise SystemExit("--radius must be within [1.0, 10.0] m")
    if not 0.2 <= args.max_speed <= 10.0:
        raise SystemExit("--max-speed must be within [0.2, 10.0] m/s")
    if not 0.2 <= args.speed <= args.max_speed:
        raise SystemExit(
            f"--speed must be within [0.2, {args.max_speed:.1f}] m/s for this circle")
    if not 0.0 <= args.excitation_deg <= 3.0:
        raise SystemExit("--excitation-deg must be within [0, 3] degrees")
    if args.duration < 10.0:
        raise SystemExit("--duration must be at least 10 s")

    sign = 1.0 if args.direction == "left" else -1.0
    effective_angle = sign * math.atan(WHEELBASE_M / args.radius)
    base_command = command_for_effective_angle(effective_angle)
    excitation = math.radians(args.excitation_deg)
    lateral_accel = args.speed * args.speed / args.radius
    circumference = 2.0 * math.pi * args.radius
    lap_time = circumference / args.speed
    if lateral_accel > args.max_lateral_accel:
        raise SystemExit(
            f"estimated lateral acceleration {lateral_accel:.2f} m/s^2 exceeds "
            f"limit {args.max_lateral_accel:.2f} m/s^2")
    if abs(base_command) + excitation > math.radians(35.0):
        raise SystemExit("base command plus excitation exceeds 35 degrees")

    profile_name = "circle_constant" if args.excitation_deg == 0.0 else "circle_prbs"
    print(f"Profile: {profile_name} radius={args.radius:.2f} m direction={args.direction}")
    print(f"Effective base wheel angle: {math.degrees(effective_angle):+.3f} deg")
    print(f"Base command: {math.degrees(base_command):+.3f} deg")
    print(f"PRBS excitation: +/-{args.excitation_deg:.3f} deg")
    print(f"Speed command: {args.speed:.3f} m/s")
    print(f"Estimated lateral acceleration: {lateral_accel:.3f} m/s^2")
    print(f"Estimated lap time: {lap_time:.2f} s; duration: {args.duration:.1f} s")
    if args.plan:
        return

    rospy.init_node("f1tenth_circle_prbs_identification", anonymous=False,
                    disable_signals=True)
    publishers, subscribers = system_state()
    existing = publishers.get(COMMAND_TOPIC, [])
    if existing:
        raise SystemExit(f"refusing command conflict; existing publishers: {existing}")
    if not subscribers.get(COMMAND_TOPIC):
        raise SystemExit("TianRacer driver is not subscribed to /tianracer/ackermann_cmd")

    monitor = OdomMonitor()
    try:
        first = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=5.0)
    except rospy.ROSException as error:
        raise SystemExit(f"vehicle odometry is not live: {error}")
    if monitor.position is None:
        monitor.callback(first)
    time.sleep(0.2)
    monitor.check()
    if abs(monitor.vx) > 0.15:
        raise SystemExit(f"vehicle must be stationary before test; vx={monitor.vx:.2f}")

    root = Path(__file__).resolve().parents[1]
    output_dir = (args.output_dir or
                  root / "lateral_identification/data/circle_prbs").expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_dir / args.run_id
    if prefix.with_suffix(".bag").exists():
        raise SystemExit(f"output already exists: {prefix}.bag")

    recorder = subprocess.Popen([
        "rosbag", "record", "--buffsize=256", "--chunksize=768", "-O",
        str(prefix), *TOPICS,
    ], start_new_session=True)
    raw_pub = rospy.Publisher(COMMAND_TOPIC, AckermannDrive, queue_size=1)
    stamped_pub = rospy.Publisher(STAMPED_TOPIC, AckermannDriveStamped, queue_size=10)
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rate = rospy.Rate(50.0)
    started = time.time()
    active_start = None
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
            publish(raw_pub, stamped_pub, 0.0, base_command)
            rate.sleep()

        active_start = time.monotonic()
        while (not rospy.is_shutdown() and not stop_requested[0]
               and time.monotonic() - active_start < args.duration):
            monitor.check()
            elapsed = time.monotonic() - active_start
            steering = base_command + sign * scheduled_excitation(elapsed, excitation)
            publish(raw_pub, stamped_pub, args.speed, steering)
            rate.sleep()
    finally:
        # Immediate zero command prevents a high-speed software ramp from
        # carrying the car through the rest of the circle after interruption.
        post_deadline = time.monotonic() + args.post_zero
        while time.monotonic() < post_deadline:
            publish(raw_pub, stamped_pub, 0.0, 0.0)
            rate.sleep()
        for _ in range(10):
            publish(raw_pub, stamped_pub, 0.0, 0.0)
            time.sleep(0.02)
        if recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
            try:
                recorder.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(recorder.pid, signal.SIGTERM)
                recorder.wait(timeout=5.0)

    metadata = {
        "run_id": args.run_id,
        "profile": profile_name,
        "bag": str(prefix.with_suffix(".bag")),
        "radius_m": args.radius,
        "wheelbase_m": WHEELBASE_M,
        "direction": args.direction,
        "speed_command_mps": args.speed,
        "speed_limit_mps": args.max_speed,
        "base_effective_angle_rad": effective_angle,
        "base_command_angle_rad": base_command,
        "excitation_rad": excitation,
        "duration_s": args.duration,
        "estimated_lateral_accel_mps2": lateral_accel,
        "lateral_accel_limit_mps2": args.max_lateral_accel,
        "estimated_lap_time_s": lap_time,
        "recorded_topics": TOPICS,
        "dataset_uses": ["constant_circle_identification", "residual_dynamics"],
        "started_wall_time": started,
    }
    prefix.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[STOPPED] zero command sent; bag={prefix}.bag")
    print(f"[METADATA] {prefix}.json")


if __name__ == "__main__":
    main()
