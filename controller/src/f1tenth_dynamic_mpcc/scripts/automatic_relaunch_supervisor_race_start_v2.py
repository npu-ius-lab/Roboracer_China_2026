#!/usr/bin/env python3
"""V3pro race-start-v2 supervisor with guarded carry/relaunch hardware mux."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
from typing import Optional

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerResponse

from f1tenth_dynamic_mpcc.automatic_relaunch import (
    CarryDetector,
    ForwardRecoveryPlan,
    GroundStabilityDetector,
    HandoffGate,
    RelaunchMotionSample,
    WheelReleaseDetector,
    assess_forward_recovery_placement,
    blend_scalar,
    is_race_start_candidate,
    planned_recovery_speed,
)
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle

try:
    from point_lio_sam_lighterbev.msg import LocalizationStatus
except ImportError:  # pragma: no cover - deployment preflight reports this.
    LocalizationStatus = None


def orientation_rpy(message) -> tuple[float, float, float]:
    q = message.orientation
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return roll, pitch, yaw


class AutomaticRelaunchSupervisor:
    ZERO_STATES = {"WAIT_GROUND", "CARRY_LOCKED", "ENABLING"}

    def __init__(self) -> None:
        if not bool(rospy.get_param("~allow_real_hardware", False)):
            raise RuntimeError("allow_real_hardware must be true")
        track_csv = Path(rospy.get_param("~track_csv")).expanduser().resolve()
        vehicle_config = Path(rospy.get_param("~vehicle_config")).expanduser().resolve()
        vehicle = load_yaml(vehicle_config)
        self.track = PeriodicTrack(track_csv)
        self.body_width = float(vehicle["vehicle"]["ego_width"])
        self.wheelbase = float(vehicle["vehicle"]["wheelbase"])
        self.max_steer = float(vehicle["limits"]["max_steer"])
        self.boundary_buffer = float(rospy.get_param("~boundary_buffer_m", 0.08))
        self.maximum_heading_error = float(
            rospy.get_param("~maximum_heading_error_rad", 0.60)
        )
        self.maximum_pp_angle = float(
            rospy.get_param("~maximum_pp_recovery_angle_rad", math.radians(60.0))
        )
        self.minimum_recovery_path_margin = float(
            rospy.get_param("~minimum_recovery_path_margin_m", 0.05)
        )
        self.recovery_lookahead_minimum = float(
            rospy.get_param("~recovery_lookahead_minimum_m", 0.45)
        )
        self.recovery_lookahead_maximum = float(
            rospy.get_param("~recovery_lookahead_maximum_m", 1.20)
        )
        self.maximum_stationary_speed = float(
            rospy.get_param("~maximum_stationary_speed_mps", 0.12)
        )
        self.probe_speed = float(rospy.get_param("~probe_speed_mps", 0.15))
        self.probe_lookahead = float(rospy.get_param("~probe_lookahead_m", 0.50))
        self.probe_lookahead_speed_gain = float(
            rospy.get_param("~probe_lookahead_speed_gain_s", 0.25)
        )
        self.probe_lookahead_maximum = float(
            rospy.get_param("~probe_lookahead_maximum_m", 1.20)
        )
        self.probe_steer_limit = float(
            rospy.get_param("~probe_steering_limit_rad", 0.10)
        )
        self.probe_steering_rate_limit = float(
            rospy.get_param("~probe_steering_rate_limit_radps", 3.0)
        )
        self.recovery_minimum_speed = float(
            rospy.get_param("~recovery_minimum_speed_mps", 1.20)
        )
        self.recovery_speed_preview = float(
            rospy.get_param("~recovery_speed_preview_m", 3.0)
        )
        self.recovery_lateral_acceleration_limit = float(
            rospy.get_param("~recovery_lateral_acceleration_limit_mps2", 2.0)
        )
        # Race starts use the normal V3 track-profile lateral limit.  Keep the
        # arbitrary-position recovery limit separate and conservative.
        self.race_start_lateral_acceleration_limit = float(rospy.get_param(
            "~race_start_lateral_acceleration_limit_mps2",
            self.recovery_lateral_acceleration_limit,
        ))
        self.recovery_handoff_minimum_speed = float(
            rospy.get_param("~recovery_handoff_minimum_speed_mps", 1.00)
        )
        self.recovery_handoff_speed_margin = float(
            rospy.get_param("~recovery_handoff_speed_margin_mps", 0.20)
        )
        self.race_start_enabled = bool(
            rospy.get_param("~race_start_enabled", True)
        )
        self.race_start_center_x = float(
            rospy.get_param("~race_start_center_x_m", 0.0)
        )
        self.race_start_center_y = float(
            rospy.get_param("~race_start_center_y_m", 0.0)
        )
        self.race_start_radius = float(
            rospy.get_param("~race_start_radius_m", 1.20)
        )
        self.race_start_s = float(rospy.get_param("~race_start_s_m", 0.0))
        self.race_start_s_tolerance = float(
            rospy.get_param("~race_start_s_tolerance_m", 1.50)
        )
        self.race_start_maximum_contour_error = float(
            rospy.get_param("~race_start_maximum_contour_error_m", 0.30)
        )
        self.race_start_maximum_heading_error = math.radians(float(
            rospy.get_param("~race_start_maximum_heading_error_deg", 10.0)
        ))
        self.race_start_minimum_vehicle_margin = float(
            rospy.get_param("~race_start_minimum_vehicle_margin_m", 0.05)
        )
        self.race_start_minimum_path_margin = float(
            rospy.get_param("~race_start_minimum_path_margin_m", 0.05)
        )
        self.race_start_merge_lookahead = float(
            rospy.get_param("~race_start_merge_lookahead_m", 2.50)
        )
        self.race_start_speed = float(
            rospy.get_param("~race_start_speed_mps", 3.0)
        )
        self.race_start_handoff_speed = float(
            rospy.get_param("~race_start_handoff_speed_mps", 2.70)
        )
        self.race_start_dynamic_handoff = bool(
            rospy.get_param("~race_start_dynamic_handoff", False)
        )
        self.handoff = HandoffGate(
            minimum_speed_mps=float(
                rospy.get_param("~handoff_minimum_mpcc_speed_mps", 1.80)
            ),
            maximum_steering_difference_rad=float(rospy.get_param(
                "~handoff_maximum_steering_difference_rad", 0.08
            )),
            required_samples=int(
                rospy.get_param("~handoff_required_samples", 3)
            ),
        )
        self.handoff_blend_duration = float(
            rospy.get_param("~handoff_blend_duration_s", 0.20)
        )
        if self.handoff_blend_duration <= 0.0:
            raise ValueError("handoff blend duration must be positive")
        if (self.probe_lookahead <= 0.0 or
                self.probe_lookahead_maximum < self.probe_lookahead or
                self.probe_lookahead_speed_gain < 0.0 or
                self.probe_steering_rate_limit <= 0.0 or
                self.recovery_minimum_speed <= 0.0 or
                self.recovery_minimum_speed > self.probe_speed or
                self.recovery_speed_preview <= 0.0 or
                self.recovery_lateral_acceleration_limit <= 0.0 or
                self.race_start_lateral_acceleration_limit <= 0.0 or
                self.recovery_handoff_minimum_speed <= 0.0 or
                self.recovery_handoff_speed_margin < 0.0 or
                self.race_start_radius <= 0.0 or
                self.race_start_s_tolerance <= 0.0 or
                self.race_start_minimum_vehicle_margin < 0.0 or
                self.race_start_minimum_path_margin < 0.0 or
                self.race_start_merge_lookahead <= 0.0 or
                self.race_start_speed < self.probe_speed or
                self.race_start_handoff_speed <= 0.0 or
                self.race_start_handoff_speed > self.race_start_speed):
            raise ValueError("invalid recovery or race-start configuration")
        self.command_timeout = float(rospy.get_param("~command_timeout_s", 0.20))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout_s", 0.20))
        self.require_localization_status = bool(
            rospy.get_param("~require_localization_status", True)
        )

        self.carry = CarryDetector(
            window_s=float(rospy.get_param("~carry_window_s", 0.25)),
            maximum_stationary_wheel_speed_mps=float(
                rospy.get_param("~carry_wheel_speed_mps", 0.10)
            ),
            minimum_z_span_m=float(rospy.get_param("~carry_z_span_m", 0.10)),
            minimum_xy_span_m=float(rospy.get_param("~carry_xy_span_m", 0.15)),
            minimum_yaw_span_rad=float(
                rospy.get_param("~carry_yaw_span_rad", 0.20)
            ),
            minimum_tilt_rad=math.radians(
                float(rospy.get_param("~carry_tilt_deg", 8.0))
            ),
            minimum_gyro_xy_radps=float(
                rospy.get_param("~carry_gyro_xy_radps", 0.80)
            ),
            confirmation_samples=int(
                rospy.get_param("~carry_confirmation_samples", 3)
            ),
        )
        self.ground = GroundStabilityDetector(
            window_s=float(rospy.get_param("~ground_window_s", 0.30)),
            maximum_z_span_m=float(rospy.get_param("~ground_z_span_m", 0.025)),
            maximum_xy_span_m=float(rospy.get_param("~ground_xy_span_m", 0.040)),
            maximum_yaw_span_rad=float(
                rospy.get_param("~ground_yaw_span_rad", 0.060)
            ),
            maximum_tilt_rad=math.radians(
                float(rospy.get_param("~ground_tilt_deg", 6.0))
            ),
            maximum_gyro_xy_radps=float(
                rospy.get_param("~ground_gyro_xy_radps", 0.20)
            ),
            maximum_wheel_speed_mps=float(
                rospy.get_param("~ground_wheel_speed_mps", 0.10)
            ),
        )
        self.release = WheelReleaseDetector(
            float(rospy.get_param("~release_wheel_speed_mps", 0.08)),
            int(rospy.get_param("~release_required_samples", 3)),
        )

        self.lock = threading.RLock()
        self.state = "WAIT_GROUND"
        self.state_reason = "waiting_for_initial_placement"
        self.controller_disabled = True
        self.disable_pending = False
        self.enable_pending = False
        self.unstable_ready_samples = 0
        self.latest_raw = None
        self.latest_vehicle = None
        self.latest_wheel = (0.0, 0.0)
        self.latest_gyro = (0.0, 0.0)
        self.latest_mpcc_command: Optional[AckermannDriveStamped] = None
        self.latest_mpcc_command_time = 0.0
        self.localization_healthy = not self.require_localization_status
        self.ready_placement = None
        self.latest_recovery_plan: Optional[ForwardRecoveryPlan] = None
        self.ready_recovery_plan: Optional[ForwardRecoveryPlan] = None
        self.recovery_progress = None
        self.last_sample = None
        self.last_metrics = None
        self.launch_mode = "recovery"
        self.last_probe_speed = 0.0
        self.last_probe_steering = 0.0
        self.last_probe_command_time = 0.0
        self.last_probe_lookahead = self.probe_lookahead
        self.last_recovery_profile_speed = self.probe_speed
        self.last_handoff_required_speed = self.handoff.minimum_speed_mps
        self.handoff_blend_start_time = 0.0
        self.handoff_blend_start_speed = self.probe_speed
        self.handoff_blend_start_steering = 0.0
        self.handoff_blend_required_speed = self.handoff.minimum_speed_mps
        self.handoff_blend_progress = 0.0

        self.enable_service_name = str(
            rospy.get_param("~enable_service", "/f1tenth_dynamic_mpcc/set_enabled")
        )
        self.enable_client = rospy.ServiceProxy(self.enable_service_name, SetBool)
        self.hardware_pub = rospy.Publisher(
            str(rospy.get_param("~hardware_command_topic", "/tianracer/ackermann_cmd")),
            AckermannDrive,
            queue_size=1,
        )
        self.status_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        rospy.Subscriber(
            str(rospy.get_param("~mpcc_command_topic",
                                "/f1tenth_mpcc/quick_relaunch/mpcc_cmd_stamped")),
            AckermannDriveStamped,
            self.mpcc_command_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            str(rospy.get_param("~vehicle_odom_topic", "/localization/vehicle_odom")),
            Odometry,
            self.vehicle_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            str(rospy.get_param("~lidar_odom_topic", "/localization/odom")),
            Odometry,
            self.raw_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            str(rospy.get_param("~wheel_odom_topic", "/tianracer/odom")),
            Odometry,
            self.wheel_callback,
            queue_size=5,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            str(rospy.get_param("~imu_topic", "/tianracer/imu")),
            Imu,
            self.imu_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        if LocalizationStatus is not None:
            rospy.Subscriber(
                str(rospy.get_param("~localization_status_topic",
                                    "/localization/status")),
                LocalizationStatus,
                self.localization_status_callback,
                queue_size=1,
            )
        elif self.require_localization_status:
            raise RuntimeError("LocalizationStatus ROS message is unavailable")

        self.rearm_service = rospy.Service("~rearm", Trigger, self.rearm_callback)
        self.status_service = rospy.Service("~status", Trigger, self.status_callback)
        rate = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.timer_callback)
        rospy.on_shutdown(self.publish_zero)
        self.publish_zero()
        rospy.logwarn(
            "Automatic relaunch supervisor owns REAL commands: initial state=%s; "
            "probe=%.2f m/s; carry/ground windows=%.2f/%.2f s",
            self.state,
            self.probe_speed,
            self.carry.window_s,
            self.ground.window_s,
        )

    def mpcc_command_callback(self, message: AckermannDriveStamped) -> None:
        with self.lock:
            self.latest_mpcc_command = message
            self.latest_mpcc_command_time = rospy.get_time()
            if self.state == "HANDOFF_PP":
                pp_speed = max(self.last_probe_speed, self.recovery_minimum_speed)
                pp_steering = self.last_probe_steering
                if (self.launch_mode == "race_start" and
                        not self.race_start_dynamic_handoff):
                    required_speed = self.race_start_handoff_speed
                else:
                    required_speed = max(
                        self.recovery_handoff_minimum_speed,
                        min(
                            self.handoff.minimum_speed_mps,
                            pp_speed - self.recovery_handoff_speed_margin,
                        ),
                    )
                self.last_handoff_required_speed = required_speed
                ready = self.handoff.update(
                    pp_steering,
                    float(message.drive.speed),
                    float(message.drive.steering_angle),
                    required_speed,
                )
                self.state_reason = (
                    "pure_pp_waiting_for_mpcc_speed_and_steering_match"
                )
                if ready:
                    self.state = "BLEND_TO_MPCC"
                    self.state_reason = "blending_pure_pp_to_mpcc"
                    self.handoff_blend_start_time = self.latest_mpcc_command_time
                    self.handoff_blend_start_speed = pp_speed
                    self.handoff_blend_start_steering = pp_steering
                    self.handoff_blend_required_speed = required_speed
                    self.handoff_blend_progress = 0.0
                    rospy.logwarn(
                        "MPCC HANDOFF MATCHED: mode=%s PP=%.2f MPCC=%.2f m/s "
                        "required=%.2f steering_diff=%.3f rad for %d samples; "
                        "blending for %.2f s",
                        self.launch_mode,
                        pp_speed,
                        float(message.drive.speed),
                        required_speed,
                        self.handoff.steering_difference_rad,
                        self.handoff.matching_samples,
                        self.handoff_blend_duration,
                    )

    def _complete_handoff_locked(self) -> None:
        self.state = "RUNNING"
        self.state_reason = "checkpoint_pp_blended_to_mpcc"
        self.ready_placement = None
        self.ready_recovery_plan = None
        self.recovery_progress = None
        self.handoff_blend_progress = 1.0
        self.carry.reset()
        self.ground.reset()
        self.release.reset()
        rospy.logwarn(
            "SMOOTH MPCC HANDOFF COMPLETE: speed and steering are continuous"
        )

    def _reset_handoff_locked(self, reset_probe: bool = False) -> None:
        self.handoff.reset()
        self.last_handoff_required_speed = self.handoff.minimum_speed_mps
        self.handoff_blend_start_time = 0.0
        self.handoff_blend_start_speed = self.probe_speed
        self.handoff_blend_start_steering = 0.0
        self.handoff_blend_required_speed = self.handoff.minimum_speed_mps
        self.handoff_blend_progress = 0.0
        if reset_probe:
            self.last_probe_speed = 0.0
            self.last_probe_steering = 0.0
            self.last_probe_command_time = 0.0
            self.last_probe_lookahead = self.probe_lookahead
            self.last_recovery_profile_speed = self.probe_speed

    def raw_callback(self, message: Odometry) -> None:
        roll, pitch, _ = orientation_rpy(message.pose.pose)
        with self.lock:
            self.latest_raw = (
                rospy.get_time(),
                float(message.pose.pose.position.z),
                roll,
                pitch,
            )

    def wheel_callback(self, message: Odometry) -> None:
        with self.lock:
            self.latest_wheel = (
                rospy.get_time(), float(message.twist.twist.linear.x)
            )

    def imu_callback(self, message: Imu) -> None:
        gyro = math.hypot(
            float(message.angular_velocity.x), float(message.angular_velocity.y)
        )
        with self.lock:
            self.latest_gyro = (rospy.get_time(), gyro)

    def localization_status_callback(self, message) -> None:
        healthy = bool(
            message.localized and message.frontend_healthy and message.map_loaded
        )
        with self.lock:
            self.localization_healthy = healthy

    def vehicle_callback(self, message: Odometry) -> None:
        now = rospy.get_time()
        _, _, yaw = orientation_rpy(message.pose.pose)
        with self.lock:
            self.latest_vehicle = (
                now,
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                yaw,
                float(message.twist.twist.linear.x),
            )
            if self.latest_raw is None or now - self.latest_raw[0] > self.sensor_timeout:
                return
            if now - self.latest_wheel[0] > self.sensor_timeout:
                return
            gyro = self.latest_gyro[1] if now - self.latest_gyro[0] <= self.sensor_timeout else 0.0
            sample = RelaunchMotionSample(
                stamp=now,
                x_m=self.latest_vehicle[1],
                y_m=self.latest_vehicle[2],
                z_m=self.latest_raw[1],
                roll_rad=self.latest_raw[2],
                pitch_rad=self.latest_raw[3],
                yaw_rad=yaw,
                wheel_speed_mps=self.latest_wheel[1],
                gyro_xy_radps=gyro,
            )
            self.last_sample = sample
            self._update_state(sample)

    def _recovery_plan(self, sample: RelaunchMotionSample) -> ForwardRecoveryPlan:
        return assess_forward_recovery_placement(
            self.track,
            self.body_width,
            sample.x_m,
            sample.y_m,
            sample.yaw_rad,
            sample.wheel_speed_mps,
            boundary_buffer_m=self.boundary_buffer,
            maximum_direct_heading_error_rad=self.maximum_heading_error,
            maximum_pp_angle_rad=self.maximum_pp_angle,
            maximum_stationary_speed_mps=self.maximum_stationary_speed,
            lookahead_minimum_m=self.recovery_lookahead_minimum,
            lookahead_maximum_m=self.recovery_lookahead_maximum,
            minimum_path_margin_m=self.minimum_recovery_path_margin,
        )

    def _lock_for_carry(self, reason: str) -> None:
        self.state = "CARRY_LOCKED"
        self.state_reason = reason
        self.ready_placement = None
        self.ready_recovery_plan = None
        self.recovery_progress = None
        self.controller_disabled = False
        self.disable_pending = True
        self.enable_pending = False
        self.unstable_ready_samples = 0
        self.launch_mode = "recovery"
        self._reset_handoff_locked(reset_probe=True)
        self.ground.reset()
        self.release.reset()

    def _update_state(self, sample: RelaunchMotionSample) -> None:
        carry = self.carry.update(sample)
        if self.state in {"RUNNING", "HANDOFF_PP", "BLEND_TO_MPCC"}:
            if carry.detected:
                self._lock_for_carry(carry.reason)
                rospy.logerr("CARRY DETECTED: %s; hardware output locked", carry.reason)
            return

        plan = self._recovery_plan(sample)
        self.latest_recovery_plan = plan
        placement = plan.placement
        if self.state in {"WAIT_GROUND", "CARRY_LOCKED"}:
            ready, reason, metrics = self.ground.update(
                sample, placement, self.localization_healthy
            )
            self.last_metrics = metrics
            self.state_reason = reason
            if ready and self.controller_disabled:
                self.state = "WAIT_RELEASE"
                self.state_reason = "ground_ready_waiting_for_estop_release"
                self.ready_placement = placement
                self.ready_recovery_plan = plan
                self.recovery_progress = placement.projection.s
                self.launch_mode = (
                    "race_start"
                    if self.race_start_enabled and is_race_start_candidate(
                        plan,
                        sample.x_m,
                        sample.y_m,
                        self.track.length,
                        center_x_m=self.race_start_center_x,
                        center_y_m=self.race_start_center_y,
                        radius_m=self.race_start_radius,
                        target_s_m=self.race_start_s,
                        s_tolerance_m=self.race_start_s_tolerance,
                        maximum_contour_error_m=(
                            self.race_start_maximum_contour_error
                        ),
                        maximum_heading_error_rad=(
                            self.race_start_maximum_heading_error
                        ),
                        minimum_vehicle_margin_m=(
                            self.race_start_minimum_vehicle_margin
                        ),
                        minimum_path_margin_m=(
                            self.race_start_minimum_path_margin
                        ),
                    ) else "recovery"
                )
                self.release.reset()
                self.unstable_ready_samples = 0
                rospy.logwarn(
                    "GROUND READY: global s=%.3f e=%.3f heading=%.3f margin=%.3f; "
                    "PP bearing=%.3f path_margin=%.3f plan=%s launch=%s; "
                    "arming %.2f m/s",
                    placement.projection.s_wrapped,
                    placement.projection.e_contour,
                    placement.heading_error_rad,
                    placement.vehicle_margin_m,
                    plan.bearing_error_rad,
                    plan.path_minimum_margin_m,
                    "recovery" if plan.recovery_required else "direct",
                    self.launch_mode,
                    (self.race_start_speed if self.launch_mode == "race_start"
                     else self.probe_speed),
                )
            return

        if self.state == "WAIT_RELEASE":
            if self.release.update(sample.wheel_speed_mps):
                self.state = "ENABLING"
                self.state_reason = "wheel_motion_confirmed_enabling_mpcc"
                self.enable_pending = True
                rospy.logwarn("E-STOP RELEASE DETECTED from wheel motion; enabling MPCC")
                return
            ready, reason, metrics = self.ground.update(
                sample, placement, self.localization_healthy
            )
            self.last_metrics = metrics
            if abs(sample.wheel_speed_mps) < 0.04 and not ready:
                self.unstable_ready_samples += 1
            else:
                self.unstable_ready_samples = 0
            if self.unstable_ready_samples >= 3:
                self.state = "CARRY_LOCKED"
                self.state_reason = "placement_moved_after_ready:" + reason
                self.ready_placement = None
                self.ready_recovery_plan = None
                self.recovery_progress = None
                self.ground.reset()
                self.release.reset()
                rospy.logwarn("Placement moved after READY; release probe removed")

    def _probe_command(self) -> tuple[float, float]:
        base_speed = (
            self.race_start_speed
            if self.launch_mode == "race_start" else self.probe_speed
        )
        if (self.last_sample is None or self.ready_placement is None or
                self.ready_recovery_plan is None):
            return base_speed, 0.0
        sample = self.last_sample
        if abs(sample.wheel_speed_mps) < 0.04:
            projection = self.ready_placement.projection
            if self.launch_mode == "race_start":
                target_x, target_y = self.track.position(
                    projection.s + self.race_start_merge_lookahead
                )
            else:
                target_x = self.ready_recovery_plan.target_x_m
                target_y = self.ready_recovery_plan.target_y_m
        else:
            projection = self.track.project(
                sample.x_m, sample.y_m, self.recovery_progress
            )
            self.recovery_progress = projection.s
            current_heading_error = float(wrap_angle(
                sample.yaw_rad - projection.psi_ref
            ))
            if self.launch_mode == "race_start":
                speed = planned_recovery_speed(
                    self.track,
                    projection.s,
                    base_speed,
                    minimum_speed_mps=self.recovery_minimum_speed,
                    preview_distance_m=self.recovery_speed_preview,
                    lateral_acceleration_limit_mps2=(
                        self.race_start_lateral_acceleration_limit
                    ),
                )
            else:
                speed = planned_recovery_speed(
                    self.track,
                    projection.s,
                    base_speed,
                    minimum_speed_mps=self.recovery_minimum_speed,
                    preview_distance_m=self.recovery_speed_preview,
                    lateral_acceleration_limit_mps2=(
                        self.recovery_lateral_acceleration_limit
                    ),
                    contour_error_m=projection.e_contour,
                    heading_error_rad=current_heading_error,
                )
            if self.launch_mode == "race_start":
                lookahead = self.race_start_merge_lookahead
            else:
                lookahead = min(
                    self.probe_lookahead_maximum,
                    self.probe_lookahead + self.probe_lookahead_speed_gain * max(
                        0.0, speed - self.recovery_minimum_speed
                    ),
                )
            target_x, target_y = self.track.position(projection.s + lookahead)
            self.last_probe_lookahead = lookahead
        if abs(sample.wheel_speed_mps) < 0.04:
            if self.launch_mode == "race_start":
                speed = base_speed
            else:
                speed = planned_recovery_speed(
                    self.track,
                    projection.s,
                    base_speed,
                    minimum_speed_mps=self.recovery_minimum_speed,
                    preview_distance_m=self.recovery_speed_preview,
                    lateral_acceleration_limit_mps2=(
                        self.recovery_lateral_acceleration_limit
                    ),
                    contour_error_m=projection.e_contour,
                    heading_error_rad=self.ready_placement.heading_error_rad,
                )
            self.last_probe_lookahead = float(math.hypot(
                float(target_x) - sample.x_m, float(target_y) - sample.y_m
            ))
        dx, dy = float(target_x) - sample.x_m, float(target_y) - sample.y_m
        alpha = float(wrap_angle(math.atan2(dy, dx) - sample.yaw_rad))
        desired_steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha), max(math.hypot(dx, dy), 0.05)
        )
        desired_steering = float(max(
            -self.probe_steer_limit,
            min(self.probe_steer_limit, desired_steering),
        ))
        now = rospy.get_time()
        if self.last_probe_command_time > 0.0:
            dt = max(0.0, min(0.10, now - self.last_probe_command_time))
            maximum_step = self.probe_steering_rate_limit * dt
            steering = max(
                self.last_probe_steering - maximum_step,
                min(self.last_probe_steering + maximum_step, desired_steering),
            )
        else:
            steering = desired_steering
        self.last_probe_speed = float(speed)
        self.last_probe_steering = float(steering)
        self.last_probe_command_time = now
        self.last_recovery_profile_speed = float(speed)
        return self.last_probe_speed, self.last_probe_steering

    def _call_enable(self, enabled: bool) -> tuple[bool, str]:
        try:
            response = self.enable_client(SetBoolRequest(data=enabled))
            return bool(response.success), str(response.message)
        except rospy.ServiceException as error:
            return False, str(error)

    def _process_service_actions(self) -> None:
        with self.lock:
            disable = self.disable_pending
            enable = self.enable_pending
            self.disable_pending = False
            self.enable_pending = False
        if disable:
            success, message = self._call_enable(False)
            with self.lock:
                self.controller_disabled = success
                if not success:
                    self.disable_pending = True
                    self.state_reason = "controller_disable_failed:" + message
            if success:
                rospy.logwarn("MPCC disabled and predictor history cleared")
        if enable:
            success, message = self._call_enable(True)
            with self.lock:
                if success:
                    # Keep the supervisor's 2 m/s pure pursuit alive until the
                    # controller has reached a compatible speed and steering
                    # command. A startup/observer reset must not stop a car
                    # that has just relaunched onto the racing surface.
                    self.state = "HANDOFF_PP"
                    self.state_reason = (
                        "pure_pp_waiting_for_mpcc_speed_and_steering_match"
                    )
                    self.controller_disabled = False
                    self._reset_handoff_locked()
                    self.carry.reset()
                    self.ground.reset()
                    self.release.reset()
                else:
                    self.state = "CARRY_LOCKED"
                    self.state_reason = "controller_enable_failed:" + message
                    self.controller_disabled = True
                    self.ground.reset()
            if success:
                rospy.logwarn(
                    "AUTOMATIC RELAUNCH ACTIVE: mode=%s holding adaptive PP; "
                    "base=%.2f m/s steering agreement=%.3f rad",
                    self.launch_mode,
                    (self.race_start_speed if self.launch_mode == "race_start"
                     else self.probe_speed),
                    self.handoff.maximum_steering_difference_rad,
                )
            else:
                rospy.logerr("MPCC enable failed: %s", message)

    def _output_command(self) -> AckermannDrive:
        command = AckermannDrive()
        with self.lock:
            state = self.state
            if state in {"WAIT_RELEASE", "ENABLING", "HANDOFF_PP"}:
                command.speed, command.steering_angle = self._probe_command()
            elif state == "BLEND_TO_MPCC":
                now = rospy.get_time()
                command_fresh = (
                    self.latest_mpcc_command is not None and
                    now - self.latest_mpcc_command_time <= self.command_timeout
                )
                if not command_fresh:
                    self.state = "HANDOFF_PP"
                    self.state_reason = "mpcc_command_stale_reholding_pure_pp"
                    self._reset_handoff_locked()
                    command.speed, command.steering_angle = self._probe_command()
                    rospy.logwarn("MPCC command stale during blend; returning to pure PP")
                else:
                    target = self.latest_mpcc_command.drive
                    if (float(target.speed) + 0.10 <
                            self.handoff_blend_required_speed):
                        self.state = "HANDOFF_PP"
                        self.state_reason = (
                            "mpcc_speed_dropped_reholding_pure_pp"
                        )
                        self._reset_handoff_locked()
                        command.speed, command.steering_angle = (
                            self._probe_command()
                        )
                        rospy.logwarn(
                            "MPCC speed dropped during blend; returning to pure PP"
                        )
                        return command
                    progress = (
                        (now - self.handoff_blend_start_time) /
                        self.handoff_blend_duration
                    )
                    self.handoff_blend_progress = max(
                        0.0, min(1.0, float(progress))
                    )
                    command.speed = blend_scalar(
                        self.handoff_blend_start_speed,
                        target.speed,
                        self.handoff_blend_progress,
                    )
                    command.steering_angle = blend_scalar(
                        self.handoff_blend_start_steering,
                        target.steering_angle,
                        self.handoff_blend_progress,
                    )
                    if self.handoff_blend_progress >= 1.0:
                        self._complete_handoff_locked()
            elif state == "RUNNING" and self.latest_mpcc_command is not None:
                if rospy.get_time() - self.latest_mpcc_command_time <= self.command_timeout:
                    command = self.latest_mpcc_command.drive
        return command

    def _status_dictionary(self) -> dict:
        with self.lock:
            placement = self.ready_placement
            plan = self.ready_recovery_plan or self.latest_recovery_plan
            return {
                "state": self.state,
                "reason": self.state_reason,
                "controller_disabled": self.controller_disabled,
                "localization_healthy": self.localization_healthy,
                "s": None if placement is None else placement.projection.s_wrapped,
                "contour_error": None if placement is None else placement.projection.e_contour,
                "heading_error": None if placement is None else placement.heading_error_rad,
                "vehicle_margin": None if placement is None else placement.vehicle_margin_m,
                "pp_target_s": None if plan is None else plan.target_s,
                "pp_bearing_error": None if plan is None else plan.bearing_error_rad,
                "pp_target_heading_error": (
                    None if plan is None else plan.target_heading_error_rad
                ),
                "pp_path_margin": (
                    None if plan is None else plan.path_minimum_margin_m
                ),
                "pp_recovery_required": (
                    None if plan is None else plan.recovery_required
                ),
                "launch_mode": self.launch_mode,
                "race_start_dynamic_handoff": self.race_start_dynamic_handoff,
                "race_start_lateral_acceleration_limit": (
                    self.race_start_lateral_acceleration_limit
                ),
                "pp_command_speed": self.last_probe_speed,
                "pp_command_steering": self.last_probe_steering,
                "pp_lookahead": self.last_probe_lookahead,
                "handoff_matching_samples": self.handoff.matching_samples,
                "handoff_required_samples": self.handoff.required_samples,
                "handoff_steering_difference": (
                    None if not math.isfinite(
                        self.handoff.steering_difference_rad
                    ) else self.handoff.steering_difference_rad
                ),
                "handoff_maximum_steering_difference": (
                    self.handoff.maximum_steering_difference_rad
                ),
                "handoff_minimum_mpcc_speed": self.last_handoff_required_speed,
                "handoff_blend_progress": self.handoff_blend_progress,
                "wheel_speed": self.latest_wheel[1],
            }

    def timer_callback(self, _event) -> None:
        self._process_service_actions()
        self.hardware_pub.publish(self._output_command())
        self.status_pub.publish(String(data=json.dumps(
            self._status_dictionary(), sort_keys=True
        )))

    def rearm_callback(self, _request) -> TriggerResponse:
        with self.lock:
            self._lock_for_carry("manual_rearm")
        return TriggerResponse(True, "hardware locked; waiting for a stable placement")

    def status_callback(self, _request) -> TriggerResponse:
        status = self._status_dictionary()
        return TriggerResponse(
            success=status["state"] in {
                "WAIT_RELEASE", "HANDOFF_PP", "BLEND_TO_MPCC", "RUNNING"
            },
            message=json.dumps(status, sort_keys=True),
        )

    def publish_zero(self) -> None:
        if hasattr(self, "hardware_pub"):
            self.hardware_pub.publish(AckermannDrive())


def main() -> None:
    rospy.init_node("automatic_relaunch_supervisor")
    try:
        AutomaticRelaunchSupervisor()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("Automatic relaunch supervisor failed: %s", error)
        raise


if __name__ == "__main__":
    main()
