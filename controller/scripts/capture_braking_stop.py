#!/usr/bin/env python3
"""Record one compact straight-line acceleration-to-stop identification run.

This node deliberately does not start MPCC.  It owns the raw Ackermann topic,
commands the requested speed, and switches directly to zero as soon as the
localized vehicle speed reaches the trigger.  Every exit path sends a zero
command tail and finalizes the rosbag.
"""

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
from std_msgs.msg import String


COMMAND_TOPIC = "/tianracer/ackermann_cmd"
STAMPED_TOPIC = "/f1tenth_identification/command_stamped"
PHASE_TOPIC = "/f1tenth_identification/braking_phase"
ODOM_TOPIC = "/localization/vehicle_odom"
TOPICS = [
    COMMAND_TOPIC, STAMPED_TOPIC, PHASE_TOPIC,
    ODOM_TOPIC, "/localization/odom", "/localization/status",
    "/tianracer/odom", "/tianracer/imu", "/livox/imu",
    "/diagnostics", "/tf", "/tf_static",
]


class Monitor:
    def __init__(self, maximum_speed: float):
        self.maximum_speed = maximum_speed
        self.receipt = 0.0
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.yaw_rate = 0.0
        self.frame = self.child = ""
        self.seen = False
        self.jump = False
        self._last_position = None
        self.subscriber = rospy.Subscriber(ODOM_TOPIC, Odometry, self.callback, queue_size=20)

    def callback(self, message: Odometry) -> None:
        position = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self._last_position is not None:
            if math.hypot(position[0] - self._last_position[0],
                          position[1] - self._last_position[1]) > 0.50:
                self.jump = True
        self._last_position = position
        self.x, self.y = position
        q = message.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.vx = message.twist.twist.linear.x
        self.vy = message.twist.twist.linear.y
        self.yaw_rate = message.twist.twist.angular.z
        self.frame = message.header.frame_id.lstrip("/")
        self.child = message.child_frame_id.lstrip("/")
        self.receipt = time.monotonic()
        self.seen = True

    def check_fresh(self) -> None:
        if not self.seen or time.monotonic() - self.receipt > 0.35:
            raise RuntimeError("vehicle odometry became stale")
        if self.jump:
            raise RuntimeError("localization pose jump exceeded 0.50 m")
        if abs(self.vx) > self.maximum_speed + 0.80:
            raise RuntimeError("measured speed exceeded safety bound: %.2f m/s" % self.vx)


def system_state():
    publishers, subscribers, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    return dict(publishers), dict(subscribers)


def stop_recorder(recorder: subprocess.Popen) -> None:
    if recorder.poll() is not None:
        return
    os.killpg(recorder.pid, signal.SIGINT)
    try:
        recorder.wait(timeout=12.0)
    except subprocess.TimeoutExpired:
        os.killpg(recorder.pid, signal.SIGTERM)
        recorder.wait(timeout=5.0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compact straight-line braking identification with automatic rosbag recording")
    parser.add_argument("run_id", nargs="?", default=None)
    parser.add_argument("--target-speed", type=float, default=5.0)
    parser.add_argument("--trigger-speed", type=float, default=4.8,
                        help="measured vx that immediately triggers zero command")
    parser.add_argument("--course-length", type=float, default=20.0,
                        help="clear usable distance from the initial front axle position")
    parser.add_argument("--pre-zero", type=float, default=3.0)
    parser.add_argument("--post-zero", type=float, default=3.0)
    parser.add_argument("--acceleration-timeout", type=float, default=4.0)
    parser.add_argument("--braking-timeout", type=float, default=7.0)
    parser.add_argument("--max-cross-track", type=float, default=0.75)
    parser.add_argument("--max-heading-change", type=float, default=0.35)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--plan", action="store_true")
    parser.add_argument("--execute", action="store_true",
                        help="required acknowledgement that the vehicle will move")
    parser.add_argument("--high-speed-confirm", action="store_true",
                        help="second acknowledgement required above 3 m/s")
    args = parser.parse_args()

    if not 1.0 <= args.target_speed <= 5.0:
        raise SystemExit("--target-speed must be within [1.0, 5.0] m/s")
    if not 0.7 * args.target_speed <= args.trigger_speed <= args.target_speed:
        raise SystemExit("--trigger-speed must be within [0.7*target, target]")
    if args.course_length < 18.0 and args.target_speed > 4.5:
        raise SystemExit("5 m/s testing requires at least 18 m clear distance")
    if args.pre_zero < 2.0 or args.post_zero < 2.0:
        raise SystemExit("pre/post zero must each be at least 2 s")

    # Used only as a conservative early-braking watchdog, not as an identified result.
    watchdog_decel = 1.50
    reaction_allowance = 0.25
    terminal_margin = 2.0
    trigger_stop_reserve = (
        args.trigger_speed ** 2 / (2.0 * watchdog_decel)
        + reaction_allowance * args.trigger_speed + terminal_margin
    )
    print("Compact braking-identification plan")
    print("  command: 0 -> %.2f m/s; measured vx >= %.2f m/s -> immediate 0"
          % (args.target_speed, args.trigger_speed))
    print("  clear course: %.1f m; reserved stopping distance: %.1f m"
          % (args.course_length, trigger_stop_reserve))
    print("  no high-speed hold; steering command remains exactly zero")
    print("  MPCC must NOT be running; keep the physical E-stop in hand")
    if trigger_stop_reserve >= args.course_length - 3.0:
        raise SystemExit("course is too short for the configured trigger and safety reserve")
    if args.plan:
        return
    if not args.execute:
        raise SystemExit("refusing to move: add --execute after clearing the test course")
    if args.target_speed > 3.0 and not args.high_speed_confirm:
        raise SystemExit("above 3 m/s also requires --high-speed-confirm")

    rospy.init_node("braking_stop_identification", anonymous=False, disable_signals=True)
    publishers, subscribers = system_state()
    existing = publishers.get(COMMAND_TOPIC, [])
    if existing:
        raise SystemExit("refusing command conflict; existing publishers: %s" % existing)
    if not subscribers.get(COMMAND_TOPIC):
        raise SystemExit("TianRacer driver is not subscribed to %s" % COMMAND_TOPIC)

    monitor = Monitor(args.target_speed)
    try:
        first = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=5.0)
    except rospy.ROSException as error:
        raise SystemExit("vehicle-centre localization is not live: %s" % error)
    if not monitor.seen:
        monitor.callback(first)
    time.sleep(0.2)
    monitor.check_fresh()
    if (monitor.frame, monitor.child) != ("map", "localization_base_link"):
        raise SystemExit("unexpected localization frames: %s" % ((monitor.frame, monitor.child),))
    if abs(monitor.vx) > 0.15:
        raise SystemExit("vehicle must be stationary; measured vx=%.2f m/s" % monitor.vx)

    root = Path(__file__).resolve().parents[1]
    output_dir = (args.output_dir or root / "braking_identification/data").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_id = args.run_id or "stop_%sp%s_mps_%s" % (
        int(args.target_speed), int(round(10 * args.target_speed)) % 10, stamp)
    prefix = output_dir / run_id
    if prefix.with_suffix(".bag").exists():
        raise SystemExit("output already exists: %s.bag" % prefix)

    recorder = subprocess.Popen([
        "rosbag", "record", "--buffsize=256", "--chunksize=768", "-O", str(prefix),
        *TOPICS,
    ], start_new_session=True)
    command_pub = rospy.Publisher(COMMAND_TOPIC, AckermannDrive, queue_size=1)
    stamped_pub = rospy.Publisher(STAMPED_TOPIC, AckermannDriveStamped, queue_size=10)
    phase_pub = rospy.Publisher(PHASE_TOPIC, String, queue_size=1, latch=True)
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def publish(speed: float) -> None:
        raw = AckermannDrive(speed=speed, steering_angle=0.0)
        stamped = AckermannDriveStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = "localization_base_link"
        stamped.drive = raw
        command_pub.publish(raw)
        stamped_pub.publish(stamped)

    def phase(name: str, **details) -> None:
        message = {"phase": name, "stamp": rospy.Time.now().to_sec(), **details}
        phase_pub.publish(String(data=json.dumps(message, sort_keys=True)))
        rospy.logwarn("BRAKING IDENTIFICATION PHASE: %s", name)

    started_wall = time.time()
    start_x, start_y, start_yaw = monitor.x, monitor.y, monitor.yaw
    reached_trigger = False
    stopped = False
    completed = False
    abort_reason = ""
    maximum_speed = 0.0
    maximum_along = 0.0
    maximum_cross = 0.0
    brake_start = None
    rate = rospy.Rate(50.0)

    def geometry():
        dx, dy = monitor.x - start_x, monitor.y - start_y
        along = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        cross = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        heading = math.atan2(math.sin(monitor.yaw - start_yaw),
                             math.cos(monitor.yaw - start_yaw))
        return along, cross, heading

    def runtime_check(allow_direction_abort: bool = True):
        nonlocal maximum_speed, maximum_along, maximum_cross
        monitor.check_fresh()
        along, cross, heading = geometry()
        maximum_speed = max(maximum_speed, monitor.vx)
        maximum_along = max(maximum_along, along)
        maximum_cross = max(maximum_cross, abs(cross))
        if allow_direction_abort and abs(cross) > args.max_cross_track:
            raise RuntimeError("cross-track displacement exceeded %.2f m" % args.max_cross_track)
        if allow_direction_abort and abs(heading) > args.max_heading_change:
            raise RuntimeError("heading change exceeded %.2f rad" % args.max_heading_change)
        return along

    try:
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError("rosbag recorder exited during startup")
        deadline = time.monotonic() + 3.0
        while command_pub.get_num_connections() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        if command_pub.get_num_connections() < 1:
            raise RuntimeError("no command subscriber connected")

        phase("pre_zero")
        end = time.monotonic() + args.pre_zero
        while time.monotonic() < end and not stop_requested[0] and not rospy.is_shutdown():
            runtime_check()
            if abs(monitor.vx) > 0.15:
                raise RuntimeError("vehicle moved during pre-zero")
            publish(0.0)
            rate.sleep()

        phase("accelerate", target_speed=args.target_speed,
              trigger_speed=args.trigger_speed)
        acceleration_started = time.monotonic()
        trigger_count = 0
        while not stop_requested[0] and not rospy.is_shutdown():
            along = runtime_check()
            speed_for_reserve = max(0.0, monitor.vx)
            live_reserve = (speed_for_reserve ** 2 / (2.0 * watchdog_decel)
                            + reaction_allowance * speed_for_reserve + terminal_margin)
            if args.course_length - along <= live_reserve:
                abort_reason = "distance_watchdog_braked_before_trigger"
                break
            if monitor.vx >= args.trigger_speed:
                trigger_count += 1
            else:
                trigger_count = 0
            if trigger_count >= 5:
                reached_trigger = True
                break
            if time.monotonic() - acceleration_started >= args.acceleration_timeout:
                abort_reason = "acceleration_timeout_braked_before_trigger"
                break
            publish(args.target_speed)
            rate.sleep()

        brake_start = rospy.Time.now().to_sec()
        phase("brake_zero", reached_trigger=reached_trigger,
              measured_speed=monitor.vx, along_m=geometry()[0], reason=abort_reason)
        stop_count = 0
        braking_started = time.monotonic()
        while not rospy.is_shutdown():
            runtime_check(allow_direction_abort=False)
            publish(0.0)
            if abs(monitor.vx) <= 0.15:
                stop_count += 1
            else:
                stop_count = 0
            if stop_count >= 25:
                stopped = True
                break
            if time.monotonic() - braking_started >= args.braking_timeout:
                abort_reason = abort_reason or "braking_timeout"
                break
            rate.sleep()

        phase("post_zero", stopped=stopped)
        end = time.monotonic() + args.post_zero
        while time.monotonic() < end and not rospy.is_shutdown():
            publish(0.0)
            rate.sleep()
        completed = reached_trigger and stopped and not abort_reason
    except Exception as error:
        abort_reason = str(error)
        rospy.logerr("BRAKING IDENTIFICATION ABORT: %s", abort_reason)
    finally:
        phase("finished", completed=completed, abort_reason=abort_reason)
        for _ in range(100):
            publish(0.0)
            time.sleep(0.02)
        stop_recorder(recorder)
        metadata = {
            "run_id": run_id,
            "bag": str(prefix.with_suffix(".bag")),
            "target_speed_mps": args.target_speed,
            "trigger_speed_mps": args.trigger_speed,
            "course_length_m": args.course_length,
            "watchdog_decel_mps2": watchdog_decel,
            "trigger_stop_reserve_m": trigger_stop_reserve,
            "reached_trigger": reached_trigger,
            "stopped": stopped,
            "completed": completed,
            "abort_reason": abort_reason,
            "maximum_measured_speed_mps": maximum_speed,
            "maximum_along_distance_m": maximum_along,
            "maximum_abs_cross_track_m": maximum_cross,
            "brake_start_ros_time": brake_start,
            "started_wall_time": started_wall,
        }
        prefix.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print("bag: %s.bag" % prefix)
        print("metadata: %s.json" % prefix)
        print("completed: %s; max speed %.2f m/s; distance %.2f m"
              % (completed, maximum_speed, maximum_along))
        if not completed:
            raise SystemExit("run was safely stopped but is not an accepted identification run: %s"
                             % (abort_reason or "trigger/stop condition not met"))


if __name__ == "__main__":
    main()
