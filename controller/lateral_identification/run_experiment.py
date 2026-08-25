#!/usr/bin/env python3
"""Safely publish staged lateral-identification commands and record a ROS bag."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import List

import rosgraph
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_PYTHON = ROOT / "src/f1tenth_dynamic_mpcc/python"
if str(PACKAGE_PYTHON) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PYTHON))

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack
from safety_predictor import AsyncSafetyPredictor
from track_guided_core import PROFILES, SafeExcitationGenerator, TrackGuidedController


COMMAND_TOPIC = "/tianracer/ackermann_cmd"
STAMPED_COMMAND_TOPIC = "/f1tenth_identification/command_stamped"
PHASE_TOPIC = "/f1tenth_identification/phase"
REFERENCE_PATH_TOPIC = "/f1tenth_identification/reference_path"
ACTUAL_PATH_TOPIC = "/f1tenth_identification/actual_path"
DEBUG_TOPIC = "/f1tenth_identification/debug"
ODOM_TOPIC = "/localization/vehicle_odom"
IMU_TOPIC = "/livox/imu"
RECORD_TOPICS = [
    COMMAND_TOPIC, STAMPED_COMMAND_TOPIC, PHASE_TOPIC,
    REFERENCE_PATH_TOPIC, ACTUAL_PATH_TOPIC, DEBUG_TOPIC,
    ODOM_TOPIC, "/tianracer/odom", IMU_TOPIC, "/tianracer/imu",
    "/localization/status", "/diagnostics", "/tf", "/tf_static",
]


@dataclass
class Segment:
    name: str
    duration: float
    speed: float
    kind: str = "constant"
    steering: float = 0.0
    amplitude: float = 0.0
    frequency_start: float = 0.0
    frequency_end: float = 0.0

    def target_steering(self, elapsed: float) -> float:
        if self.kind == "constant":
            return self.steering
        if self.kind == "chirp":
            slope = (self.frequency_end - self.frequency_start) / self.duration
            phase = 2.0 * math.pi * (
                self.frequency_start * elapsed + 0.5 * slope * elapsed * elapsed
            )
            return self.amplitude * math.sin(phase)
        raise ValueError(f"unknown segment kind: {self.kind}")


def actuator_profile() -> List[Segment]:
    segments = [
        Segment("zero_start", 2.0, 0.0),
        Segment("straight_settle", 2.0, 0.8),
    ]
    for magnitude in (0.06, 0.12, 0.18):
        label = f"{int(round(100 * magnitude)):02d}"
        segments.extend([
            Segment(f"left_{label}", 3.0, 0.8, steering=magnitude),
            Segment(f"center_after_left_{label}", 1.0, 0.8),
            Segment(f"right_{label}", 3.0, 0.8, steering=-magnitude),
            Segment(f"center_after_right_{label}", 1.0, 0.8),
        ])
    for index in range(10):
        steering = 0.10 if index % 2 == 0 else -0.10
        segments.append(Segment(f"step_{index + 1:02d}", 0.8, 0.8, steering=steering))
    segments.extend([Segment("center_finish", 1.0, 0.8), Segment("zero_end", 2.0, 0.0)])
    return segments


def tire_profile(speed: float, amplitude: float, name: str) -> List[Segment]:
    return [
        Segment("zero_start", 2.0, 0.0),
        Segment("straight_settle", 2.5, speed),
        Segment(f"{name}_chirp_up", 10.0, speed, "chirp", amplitude=amplitude,
                frequency_start=0.20, frequency_end=1.20),
        Segment("center_mid", 1.5, speed),
        Segment(f"{name}_chirp_down", 10.0, speed, "chirp", amplitude=amplitude,
                frequency_start=1.20, frequency_end=0.20),
        Segment("center_finish", 1.5, speed),
        Segment("zero_end", 2.0, 0.0),
    ]


def build_profile(name: str) -> List[Segment]:
    if name == "actuator":
        return actuator_profile()
    if name == "tire_low":
        return tire_profile(1.0, 0.12, name)
    if name == "tire_mid":
        return tire_profile(1.5, 0.10, name)
    if name == "tire_high":
        return tire_profile(2.0, 0.08, name)
    raise ValueError(name)


def approach(current: float, target: float, maximum_change: float) -> float:
    return current + max(-maximum_change, min(maximum_change, target - current))


class SafetyMonitor:
    def __init__(self, maximum_speed: float):
        self.maximum_speed = maximum_speed
        self.last_receipt = time.monotonic()
        self.vx = self.vy = self.yaw_rate = 0.0
        self.measurement_age = 0.0
        self.measurement_stamp = 0.0
        self.frame = self.child = ""
        self.position = None
        self.yaw = 0.0
        self.jump = False
        self.imu_seen = False
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback, queue_size=10)
        self.imu_sub = rospy.Subscriber(IMU_TOPIC, Imu, self.imu_callback, queue_size=50)

    def odom_callback(self, message: Odometry) -> None:
        next_position = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.position is not None:
            distance = math.hypot(next_position[0] - self.position[0], next_position[1] - self.position[1])
            if distance > 0.50:
                self.jump = True
        self.position = next_position
        orientation = message.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.last_receipt = time.monotonic()
        self.measurement_stamp = message.header.stamp.to_sec()
        if self.measurement_stamp > 0.0:
            self.measurement_age = max(
                0.0, min(0.30, rospy.Time.now().to_sec() - self.measurement_stamp)
            )
        self.vx = message.twist.twist.linear.x
        self.vy = message.twist.twist.linear.y
        self.yaw_rate = message.twist.twist.angular.z
        self.frame = message.header.frame_id.lstrip("/")
        self.child = message.child_frame_id.lstrip("/")

    def predicted_pose(self):
        """Extrapolate delayed map pose to now using body-frame odometry.

        Point-LIO's header timestamp describes the measurement, while the
        callback arrives roughly 0.11 s later.  At mid speed, using the raw
        pose would make the tracker follow a point 15 cm behind the vehicle.
        """
        age = self.measurement_age
        yaw_mid = self.yaw + 0.5 * self.yaw_rate * age
        x = self.position[0] + age * (
            math.cos(yaw_mid) * self.vx - math.sin(yaw_mid) * self.vy
        )
        y = self.position[1] + age * (
            math.sin(yaw_mid) * self.vx + math.cos(yaw_mid) * self.vy
        )
        yaw = math.atan2(
            math.sin(self.yaw + self.yaw_rate * age),
            math.cos(self.yaw + self.yaw_rate * age),
        )
        return x, y, yaw, self.vx, self.vy, self.yaw_rate, age

    def imu_callback(self, _message: Imu) -> None:
        self.imu_seen = True

    def check(self) -> None:
        if time.monotonic() - self.last_receipt > 0.35:
            raise RuntimeError("vehicle odometry became stale")
        if self.jump:
            raise RuntimeError("localization pose jump exceeded 0.50 m")
        if abs(self.vx) > self.maximum_speed + 0.70:
            raise RuntimeError(f"measured speed exceeded safety bound: {self.vx:.2f} m/s")
        if abs(self.yaw_rate) > 5.0:
            raise RuntimeError(f"yaw rate exceeded safety bound: {self.yaw_rate:.2f} rad/s")


def system_state():
    publishers, subscribers, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    return dict(publishers), dict(subscribers)


def publish(command_pub, stamped_pub, speed: float, steering: float) -> None:
    stamp = rospy.Time.now()
    raw = AckermannDrive(speed=speed, steering_angle=steering)
    stamped = AckermannDriveStamped()
    stamped.header.stamp = stamp
    stamped.header.frame_id = "localization_base_link"
    stamped.drive = raw
    command_pub.publish(raw)
    stamped_pub.publish(stamped)


def print_plan(profile: str, segments: List[Segment]) -> None:
    print(f"Profile: {profile}")
    print(f"Total duration: {sum(s.duration for s in segments):.1f} s")
    for index, segment in enumerate(segments, 1):
        description = (
            f"steering={segment.steering:+.3f} rad" if segment.kind == "constant"
            else f"chirp={segment.amplitude:.3f} rad, {segment.frequency_start:.2f}"
                 f"->{segment.frequency_end:.2f} Hz"
        )
        print(f"  {index:02d} {segment.name:26s} {segment.duration:4.1f}s "
              f"speed={segment.speed:.2f} m/s {description}")


def print_guided_plan(profile_name: str, track: PeriodicTrack,
                      controller: TrackGuidedController) -> None:
    profile = controller.profile
    samples = [track.geometry(index * track.length / 1000.0) for index in range(1000)]
    left = [sample[4] for sample in samples]
    right = [sample[5] for sample in samples]
    boundary_x = []
    boundary_y = []
    for sample in samples:
        x, y, yaw, _, width_left, width_right, _ = sample
        boundary_x.extend((x - math.sin(yaw) * width_left,
                           x + math.sin(yaw) * width_right))
        boundary_y.extend((y + math.cos(yaw) * width_left,
                           y - math.cos(yaw) * width_right))
    print(f"Profile: {profile_name} (track-guided)")
    print(f"Track length: {track.length:.3f} m; stop after "
          f"{profile.maximum_laps:.1f} lap(s)")
    print(f"Track width: {min(a + b for a, b in zip(left, right)):.3f}"
          f"..{max(a + b for a, b in zip(left, right)):.3f} m")
    print(f"Boundary envelope: {max(boundary_x) - min(boundary_x):.3f} x "
          f"{max(boundary_y) - min(boundary_y):.3f} m")
    print(f"Vehicle width: {2.0 * controller.half_width:.3f} m")
    print(f"Speed cap: {profile.speed_mps:.2f} m/s")
    print(f"Excitation: {profile.excitation_kind}, +/-{profile.excitation_rad:.3f} rad; "
          "enters where |curvature| < 0.20 rad/m, margin >= 0.32 m")
    print("Excitation hold hysteresis: |curvature| < 0.22 rad/m, margin >= 0.28 m")
    print(f"Excitation warmup: {profile.warmup_s:.1f} s of safe straight driving")
    preview_distance = max(1.8, 1.5 * profile.speed_mps)
    print(
        "Speed policy: cap applies on straights; "
        f"{preview_distance:.2f} m speed-dependent curvature preview slows before bends"
    )
    print(f"Safety: start margin >= {controller.start_margin:.2f} m, current margin >= "
          f"{controller.current_abort_margin:.2f} m, 1 s predicted margin >= "
          f"{controller.predicted_abort_margin:.2f} m")


def reference_path(track: PeriodicTrack) -> RosPath:
    message = RosPath()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    for index in range(520):
        s = index * track.length / 519.0
        x, y = track.position(s)
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def append_actual_path(message: RosPath, monitor: SafetyMonitor) -> None:
    x, y, yaw, _, _, _, _ = monitor.predicted_pose()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x, pose.pose.position.y = x, y
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    message.header = pose.header
    message.poses.append(pose)


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
    parser = argparse.ArgumentParser()
    old_profiles = ("actuator", "tire_low", "tire_mid", "tire_high")
    parser.add_argument("profile", choices=old_profiles + tuple(PROFILES),
                        nargs="?", default="track_actuator")
    parser.add_argument("run_id", nargs="?", default="")
    parser.add_argument("--plan", action="store_true", help="print commands without publishing")
    parser.add_argument("--no-analyze", action="store_true")
    parser.add_argument(
        "--speed-cap", type=float,
        help="Override the track-guided profile's straight-line speed cap (m/s).",
    )
    parser.add_argument(
        "--excitation-rad", type=float,
        help="Override the signed steering chirp/plateau amplitude (rad).",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--track", type=Path,
        default=ROOT / "src/f1tenth_dynamic_mpcc/data/tracks/virtual_track/raceline.csv",
    )
    args = parser.parse_args()

    if args.profile in old_profiles:
        segments = build_profile(args.profile)
        print_plan(args.profile, segments)
        if args.plan:
            return
        raise SystemExit(
            "UNSAFE OPEN-LOOP PROFILE DISABLED after a field-size collision. "
            "Use track_actuator or track_tire_low/mid/high instead."
        )

    requested_profile = PROFILES[args.profile]
    if args.speed_cap is not None and not (0.25 <= args.speed_cap <= 3.5):
        raise SystemExit("--speed-cap must be within [0.25, 3.5] m/s")
    if args.excitation_rad is not None and not (0.0 <= args.excitation_rad <= 0.16):
        raise SystemExit("--excitation-rad must be within [0.0, 0.16] rad")
    profile = replace(
        requested_profile,
        speed_mps=args.speed_cap if args.speed_cap is not None else requested_profile.speed_mps,
        excitation_rad=(args.excitation_rad if args.excitation_rad is not None
                        else requested_profile.excitation_rad),
    )
    track = PeriodicTrack(args.track.expanduser().resolve())
    controller = TrackGuidedController(track, profile)
    excitation_generator = SafeExcitationGenerator(profile)
    print_guided_plan(args.profile, track, controller)
    if args.plan:
        return

    rospy.init_node("f1tenth_lateral_identification", anonymous=False, disable_signals=True)
    publishers, subscribers = system_state()
    existing = publishers.get(COMMAND_TOPIC, [])
    if existing:
        raise SystemExit(f"refusing command conflict; existing publishers: {existing}")
    if not subscribers.get(COMMAND_TOPIC):
        raise SystemExit("TianRacer driver is not subscribed to /tianracer/ackermann_cmd")

    maximum_profile_speed = profile.speed_mps
    monitor = SafetyMonitor(maximum_profile_speed)
    try:
        first_odom = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=5.0)
    except rospy.ROSException as error:
        raise SystemExit(f"vehicle-centre odometry is not live: {error}")
    if monitor.position is None:
        monitor.odom_callback(first_odom)
    time.sleep(0.20)
    if (monitor.frame, monitor.child) != ("map", "localization_base_link"):
        raise SystemExit(f"unexpected odometry frames: {(monitor.frame, monitor.child)}")
    if abs(monitor.vx) > 0.15:
        raise SystemExit(f"vehicle must be stationary before identification; vx={monitor.vx:.2f}")

    initial_state = monitor.predicted_pose()
    initial = controller.command(
        initial_state[0], initial_state[1], initial_state[2], initial_state[3],
        0.0, None, allow_excitation=False,
    )
    if initial.physical_margin < controller.start_margin:
        raise SystemExit(
            f"vehicle is too close to track boundary: body margin="
            f"{initial.physical_margin:.3f} m, required={controller.start_margin:.3f} m"
        )
    if abs(initial.heading_error) > 0.35:
        raise SystemExit(
            f"vehicle heading differs from raceline by {initial.heading_error:.3f} rad; "
            "place it facing the forward race direction"
        )

    output_dir = (args.output_dir or ROOT / "lateral_identification/data").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_text = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_id = args.run_id or f"{args.profile}_{stamp_text}"
    prefix = output_dir / run_id
    if prefix.with_suffix(".bag").exists():
        raise SystemExit(f"output already exists: {prefix}.bag")

    recorder = subprocess.Popen([
        "rosbag", "record", "--buffsize=256", "--chunksize=768", "-O", str(prefix),
        *RECORD_TOPICS,
    ], start_new_session=True)
    command_pub = rospy.Publisher(COMMAND_TOPIC, AckermannDrive, queue_size=1)
    stamped_pub = rospy.Publisher(STAMPED_COMMAND_TOPIC, AckermannDriveStamped, queue_size=10)
    phase_pub = rospy.Publisher(PHASE_TOPIC, String, queue_size=1, latch=True)
    reference_pub = rospy.Publisher(REFERENCE_PATH_TOPIC, RosPath, queue_size=1, latch=True)
    actual_pub = rospy.Publisher(ACTUAL_PATH_TOPIC, RosPath, queue_size=1, latch=True)
    debug_pub = rospy.Publisher(DEBUG_TOPIC, Float32MultiArray, queue_size=10)
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rate_hz = 50.0
    current_speed = current_steering = 0.0
    stop_steering = 0.0
    started_ros = rospy.Time.now().to_sec()
    abort_reason = ""
    completed = False
    hard_abort = False
    minimum_margin = initial.physical_margin
    minimum_predicted_margin = math.inf
    minimum_excitation_predicted_margin = math.inf
    maximum_abs_contour = abs(initial.contour_error)
    completed_distance = 0.0
    excitation_suppression_count = 0
    predictor = None
    latest_prediction = None
    predictor_compute_times = []
    maximum_control_loop_dt = 0.0
    missed_control_deadlines = 0
    actual_path = RosPath()
    actual_path.header.frame_id = "map"
    try:
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError("rosbag recorder exited during startup")
        deadline = time.monotonic() + 3.0
        while command_pub.get_num_connections() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        if command_pub.get_num_connections() < 1:
            raise RuntimeError("command publisher has no driver subscriber")
        if not monitor.imu_seen:
            rospy.logwarn("/livox/imu not observed yet; analysis will fall back to odometry yaw rate")
        predictor = AsyncSafetyPredictor(args.track.expanduser().resolve(), profile)
        initial_state = monitor.predicted_pose()
        initial = controller.command(
            initial_state[0], initial_state[1], initial_state[2], initial_state[3],
            0.0, initial.s, allow_excitation=False,
        )
        if initial.physical_margin < controller.start_margin:
            raise RuntimeError(
                "vehicle moved too close to track boundary during startup: "
                f"margin={initial.physical_margin:.3f} m"
            )
        initial_excitation = (
            profile.excitation_rad
            if controller.excitation_allowed(initial) and profile.excitation_rad > 0.0
            else 0.0
        )
        predictor.submit(
            initial_state[0], initial_state[1], initial_state[2],
            max(initial_state[3], 0.25), 0.0, initial.s, 0.0,
            initial_excitation,
        )
        latest_prediction = predictor.wait(timeout=5.0)
        if latest_prediction is None:
            raise RuntimeError("asynchronous safety predictor startup timed out")
        if latest_prediction.error:
            raise RuntimeError(
                f"asynchronous safety predictor failed: {latest_prediction.error}"
            )
        monitor.check()
        predictor_compute_times.append(latest_prediction.compute_seconds)
        minimum_predicted_margin = min(
            minimum_predicted_margin, latest_prediction.baseline_margin
        )
        minimum_excitation_predicted_margin = min(
            minimum_excitation_predicted_margin,
            latest_prediction.excitation_margin,
        )
        if latest_prediction.baseline_margin < controller.predicted_abort_margin:
            raise RuntimeError(
                "initial one-second predicted body margin "
                f"{latest_prediction.baseline_margin:.3f} m is below "
                f"{controller.predicted_abort_margin:.3f} m"
            )
        reference_pub.publish(reference_path(track))
        phase_pub.publish(String(data=json.dumps({
            "profile": args.profile, "phase": "track_following",
            "track_length_m": track.length, "speed_cap_mps": maximum_profile_speed,
            "debug_fields": [
                "base_steering_rad", "excitation_rad", "target_steering_rad",
                "sent_steering_rad", "sent_speed_mps", "measured_vx_mps",
                "contour_error_m", "physical_margin_m", "predicted_margin_m",
                "excitation_predicted_margin_m", "prediction_age_s",
                "prediction_compute_s", "excitation_allowed",
                "excitation_active_time_s",
            ],
        }, sort_keys=True)))
        run_start = time.monotonic()
        last_loop_time = run_start
        next_tick = run_start
        next_log = run_start
        next_prediction_request = run_start
        latest_predicted_margin = latest_prediction.baseline_margin
        latest_excitation_predicted_margin = latest_prediction.excitation_margin
        prediction_age = max(0.0, run_start - latest_prediction.submitted_monotonic)
        excitation_section_active = False
        tick = 0
        start_s = initial.s
        s_guess = initial.s
        while not rospy.is_shutdown():
            if stop_requested[0]:
                abort_reason = "operator_stop"
                break
            monitor.check()
            loop_time = time.monotonic()
            elapsed = loop_time - run_start
            loop_dt = loop_time - last_loop_time
            last_loop_time = loop_time
            maximum_control_loop_dt = max(maximum_control_loop_dt, loop_dt)
            if elapsed > profile.timeout_s:
                raise RuntimeError("track-guided run exceeded profile timeout")
            prediction_result = predictor.poll()
            if prediction_result is not None:
                if prediction_result.error:
                    raise RuntimeError(
                        f"asynchronous safety predictor failed: {prediction_result.error}"
                    )
                latest_prediction = prediction_result
                predictor_compute_times.append(prediction_result.compute_seconds)
                latest_predicted_margin = prediction_result.baseline_margin
                latest_excitation_predicted_margin = prediction_result.excitation_margin
                minimum_predicted_margin = min(
                    minimum_predicted_margin, latest_predicted_margin
                )
                minimum_excitation_predicted_margin = min(
                    minimum_excitation_predicted_margin,
                    latest_excitation_predicted_margin,
                )
                if latest_predicted_margin < controller.predicted_abort_margin:
                    raise RuntimeError(
                        f"one-second predicted body margin {latest_predicted_margin:.3f} m "
                        f"is below {controller.predicted_abort_margin:.3f} m"
                    )
                if (
                    abs(prediction_result.evaluated_excitation_rad) > 1.0e-9
                    and latest_excitation_predicted_margin
                    < controller.excitation_predicted_margin
                ):
                    excitation_suppression_count += 1
            prediction_age = max(
                0.0, loop_time - latest_prediction.submitted_monotonic
            )
            if prediction_age > 1.0:
                raise RuntimeError(
                    f"asynchronous safety prediction became stale: {prediction_age:.3f} s"
                )
            state_now = monitor.predicted_pose()
            baseline = controller.command(
                state_now[0], state_now[1], state_now[2], state_now[3],
                elapsed, s_guess, allow_excitation=False,
            )
            baseline_state_allows_excitation = controller.excitation_allowed(
                baseline, holding=excitation_section_active
            )
            if (
                not predictor.in_flight
                and loop_time >= next_prediction_request
            ):
                worst_excitation = (
                    profile.excitation_rad
                    if baseline_state_allows_excitation and profile.excitation_rad > 0.0
                    else 0.0
                )
                predictor.submit(
                    state_now[0], state_now[1], state_now[2],
                    max(state_now[3], current_speed, 0.25), elapsed, baseline.s,
                    current_steering, worst_excitation,
                )
                next_prediction_request = loop_time + 0.10
            prediction_fresh = prediction_age <= 0.40
            prediction_covered_excitation = (
                abs(latest_prediction.evaluated_excitation_rad) > 1.0e-9
            )
            state_gate = (
                baseline_state_allows_excitation
                and prediction_fresh
                and prediction_covered_excitation
                and latest_excitation_predicted_margin
                >= controller.excitation_predicted_margin
            )
            excitation_section_active = state_gate
            excitation = excitation_generator.step(loop_dt, state_gate)
            target = controller.command(
                state_now[0], state_now[1], state_now[2], state_now[3],
                elapsed, s_guess, allow_excitation=False,
                excitation_override=excitation,
            )
            # The command is a cap, not a measured velocity. On the TianRacer
            # the lower-level loop can overshoot it at high speed; feed that
            # observation back into the identification command before the
            # normal slew limiter compounds the overshoot.
            overspeed = max(0.0, monitor.vx - profile.speed_mps)
            if overspeed > 0.05:
                target = replace(
                    target,
                    speed=min(
                        target.speed,
                        max(0.25, profile.speed_mps - min(0.60, 2.0 * overspeed)),
                    ),
                )
            if not prediction_fresh:
                target = replace(target, speed=min(target.speed, 0.25), excitation=0.0,
                                 steering=target.base_steering)
            s_guess = target.s
            completed_distance = target.s - start_s
            minimum_margin = min(minimum_margin, target.physical_margin)
            maximum_abs_contour = max(maximum_abs_contour, abs(target.contour_error))
            stop_steering = target.base_steering
            if target.physical_margin < controller.current_abort_margin:
                raise RuntimeError(
                    f"body-to-boundary margin {target.physical_margin:.3f} m is below "
                    f"{controller.current_abort_margin:.3f} m"
                )
            if elapsed > 3.0 and completed_distance < -0.20:
                raise RuntimeError("vehicle is progressing opposite to raceline direction")
            speed_slew = 0.60 if target.speed >= current_speed else 1.20
            current_speed = approach(current_speed, target.speed, speed_slew / rate_hz)
            current_steering = approach(
                current_steering, target.steering, controller.max_steer_rate / rate_hz
            )
            publish(command_pub, stamped_pub, current_speed, current_steering)
            debug_pub.publish(Float32MultiArray(data=[
                target.base_steering,
                target.excitation,
                target.steering,
                current_steering,
                current_speed,
                monitor.vx,
                target.contour_error,
                target.physical_margin,
                latest_predicted_margin,
                latest_excitation_predicted_margin,
                prediction_age,
                latest_prediction.compute_seconds,
                1.0 if state_gate else 0.0,
                excitation_generator.active_time,
            ]))
            if tick % 5 == 0:
                append_actual_path(actual_path, monitor)
                actual_pub.publish(actual_path)
            if time.monotonic() >= next_log:
                progress = 100.0 * max(completed_distance, 0.0) / track.length
                rospy.logwarn(
                    "TRACK IDENT %.1f%% vx=%.2f cmd=%.2f steer=%+.3f excite=%+.3f "
                    "ey=%+.3f margin=%.3f predicted=%.3f excite_pred=%.3f "
                    "pred_age=%.3f compute=%.3f",
                    progress, monitor.vx, current_speed, current_steering,
                    target.excitation, target.contour_error, target.physical_margin,
                    latest_predicted_margin, latest_excitation_predicted_margin,
                    prediction_age, latest_prediction.compute_seconds,
                )
                next_log += 1.0
            if completed_distance >= track.length * profile.maximum_laps:
                completed = True
                break
            tick += 1
            next_tick += 1.0 / rate_hz
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                missed_control_deadlines += 1
                next_tick = time.monotonic()
    except Exception as error:
        abort_reason = str(error)
        hard_abort = abort_reason != "operator_stop"
        rospy.logerr("IDENTIFICATION ABORT: %s", error)
    finally:
        phase_pub.publish(String(data=json.dumps({"profile": args.profile, "phase": "final_stop"})))
        if hard_abort:
            # A predicted boundary violation is already a late event at high
            # speed. Do not spend 1--2 m travelling under a software speed
            # ramp; command zero immediately and keep the last path-directed
            # steering for the short lower-level braking transient.
            for _ in range(75):
                publish(command_pub, stamped_pub, 0.0, stop_steering)
                time.sleep(1.0 / rate_hz)
        else:
            stop_deadline = time.monotonic() + 3.0
            next_tick = time.monotonic()
            while time.monotonic() < stop_deadline:
                current_speed = approach(current_speed, 0.0, 1.5 / rate_hz)
                if max(abs(monitor.vx), current_speed) > 0.25:
                    current_steering = approach(
                        current_steering, stop_steering, controller.max_steer_rate / rate_hz
                    )
                else:
                    current_steering = approach(current_steering, 0.0, 0.75 / rate_hz)
                publish(command_pub, stamped_pub, current_speed, current_steering)
                next_tick += 1.0 / rate_hz
                time.sleep(max(0.0, next_tick - time.monotonic()))
        for _ in range(20):
            publish(command_pub, stamped_pub, 0.0, 0.0)
            time.sleep(0.02)
        stop_recorder(recorder)
        if predictor is not None:
            predictor.close()

    metadata = {
        "profile": args.profile, "run_id": run_id, "completed": completed,
        "abort_reason": abort_reason, "started_ros_time": started_ros,
        "bag": str(prefix.with_suffix(".bag")), "command_topic": COMMAND_TOPIC,
        "stamped_command_topic": STAMPED_COMMAND_TOPIC,
        "odom_topic": ODOM_TOPIC, "imu_topic": IMU_TOPIC,
        "wheelbase_m": controller.wheelbase,
        "body_width_m": 2.0 * controller.half_width,
        "track_csv": str(args.track.expanduser().resolve()),
        "track_length_m": track.length,
        "completed_distance_m": completed_distance,
        "minimum_physical_margin_m": minimum_margin,
        "minimum_predicted_margin_m": minimum_predicted_margin,
        "minimum_excitation_predicted_margin_m": minimum_excitation_predicted_margin,
        "maximum_abs_contour_error_m": maximum_abs_contour,
        "speed_cap_mps": maximum_profile_speed,
        "effective_profile": asdict(profile),
        "excitation_rad": profile.excitation_rad,
        "excitation_active_time_s": excitation_generator.active_time,
        "excitation_enabled_time_s": excitation_generator.enabled_time,
        "excitation_transition_count": excitation_generator.transitions,
        "excitation_suppression_count": excitation_suppression_count,
        "control_loop": {
            "target_hz": rate_hz,
            "maximum_loop_dt_s": maximum_control_loop_dt,
            "missed_deadlines": missed_control_deadlines,
        },
        "safety_predictor": {
            "mode": "spawned_process",
            "samples": len(predictor_compute_times),
            "mean_compute_s": (
                sum(predictor_compute_times) / len(predictor_compute_times)
                if predictor_compute_times else None
            ),
            "maximum_compute_s": (
                max(predictor_compute_times) if predictor_compute_times else None
            ),
            "freshness_limit_s": 0.40,
            "abort_age_s": 1.0,
        },
        "identified_steering_model": {
            "gain": controller.steering_gain,
            "bias_rad": controller.steering_bias,
            "time_constant_s": controller.steering_tau,
            "dead_time_s": controller.steering_delay,
        },
        "safety": {
            "start_margin_m": controller.start_margin,
            "current_abort_margin_m": controller.current_abort_margin,
            "predicted_abort_margin_m": controller.predicted_abort_margin,
            "excitation_predicted_margin_m": controller.excitation_predicted_margin,
            "prediction_horizon_s": controller.prediction_horizon,
            "actuator_model_envelope": [asdict(model) for model in controller.safety_models],
        },
    }
    metadata_path = prefix.with_suffix(".metadata.json")
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    print(f"[STOPPED] zero command sent; bag={prefix}.bag")
    print(f"[METADATA] {metadata_path}")
    if completed and not args.no_analyze:
        analyzer = Path(__file__).with_name("analyze_bag.py")
        result = subprocess.run([sys.executable, str(analyzer), str(prefix.with_suffix(".bag"))])
        if result.returncode:
            print("[WARN] automatic analysis failed; the bag is intact and can be analyzed later", file=sys.stderr)
    if abort_reason and abort_reason != "operator_stop":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
