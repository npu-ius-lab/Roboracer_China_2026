#!/usr/bin/env python3
"""ROS wrapper for timestamp-compensated Dynamic MPCC.

The default output is an isolated debug topic. Selecting TianRacer's stamped
command topic requires both allow_real_command_topic=true in the immutable
launch configuration and an explicit runtime parameter.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_dynamic_mpcc.acados_solver import AcadosDynamicMPCC
from f1tenth_dynamic_mpcc.command_manager import CommandManager
from f1tenth_dynamic_mpcc.command_history_buffer import (
    CommandHistoryBuffer,
    PublishedCommandSample,
)
from f1tenth_dynamic_mpcc.config import (
    apply_runtime_speed_cap,
    load_yaml,
    package_root,
)
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.startup_state_machine import (
    SAFE_DECEL,
    STOPPED,
    StartupStateMachine,
)
from f1tenth_dynamic_mpcc.steering_actuator_model import SteeringActuatorModel
from f1tenth_dynamic_mpcc.steering_state_observer import (
    INVALID,
    MODEL_ONLY,
    SteeringStateObserver,
)
from f1tenth_dynamic_mpcc.telemetry import JsonlTelemetry
from f1tenth_dynamic_mpcc.timestamp_guard import benign_reorder
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class DynamicMPCCNode:
    def __init__(self) -> None:
        root = package_root()
        controller_path = Path(
            rospy.get_param("~controller_config", str(root / "config/controller.yaml"))
        )
        vehicle_path = Path(
            rospy.get_param("~vehicle_config", str(root / "config/vehicle.yaml"))
        )
        track_path = Path(
            rospy.get_param(
                "~track_csv", str(root / "data/tracks/virtual_track/raceline.csv")
            )
        )
        self.controller = load_yaml(controller_path)
        # Python executes the same OCP more slowly than the C++ runtime. Keep
        # the fixed-rate publisher's freshness window configurable per launch.
        self.controller["publisher"]["command_timeout_s"] = float(
            rospy.get_param(
                "~command_timeout_s",
                self.controller["publisher"]["command_timeout_s"],
            )
        )
        # The Python OCP cannot currently sustain the 20 Hz deadline at the
        # full 3 m/s hardware cap. Apply a launch-local cap before constructing
        # the speed profile, vehicle limits, solver and startup state machine.
        runtime_speed_cap = rospy.get_param("~runtime_speed_cap_mps", None)
        if runtime_speed_cap is not None:
            apply_runtime_speed_cap(self.controller, float(runtime_speed_cap))
        self.vehicle_config = load_yaml(vehicle_path)
        self.vehicle = VehicleParameters.from_yaml(vehicle_path, self.controller)
        self.steering_actuator = SteeringActuatorModel.from_config(self.vehicle_config)
        self.steering_observer = SteeringStateObserver.from_config(
            self.vehicle_config, self.steering_actuator, self.vehicle
        )
        speed_planning = dict(self.controller.get("speed_planning", {}))
        speed_planning.update(
            wheelbase_m=self.vehicle.wheelbase,
            max_steer_rad=self.vehicle.max_steer,
            max_steer_rate_radps=self.vehicle.max_steer_rate,
            max_accel_mps2=self.vehicle.max_accel,
            max_decel_mps2=self.vehicle.max_decel,
            lateral_accel_limit_mps2=self.vehicle.lateral_accel_limit,
        )
        self.track = PeriodicTrack(track_path, speed_planning=speed_planning)
        self.model = DynamicBicycleModel(self.vehicle)
        self.command_history = CommandHistoryBuffer(retention_s=2.0)
        self.state_predictor = LowLatencyVehicleStatePredictor(
            self.model, integration_step_s=0.01,
            command_history=self.command_history,
        )
        self.formulation = str(rospy.get_param("~formulation", "mpcc"))
        generated_dir = Path(
            rospy.get_param(
                "~generated_dir",
                str(Path.home() / ".cache/f1tenth_dynamic_mpcc/acados"),
            )
        )
        self.solver = AcadosDynamicMPCC(
            self.track,
            self.vehicle,
            self.controller,
            generated_dir,
            formulation=self.formulation,
            build=bool(rospy.get_param("~build_solver", True)),
        )

        topics = self.controller["topics"]
        output = self.controller.get("output", {})
        frames = self.controller["frames"]
        timing = self.controller["timing"]
        safety = self.controller["safety"]
        self.odom_topic = str(rospy.get_param("~odom_topic", topics["odom"]))
        self.command_topic = str(rospy.get_param("~command_topic", topics["command"]))
        self.collision_topic = str(rospy.get_param("~collision_topic", topics["collision"]))
        self.world_frame = str(rospy.get_param("~world_frame", frames["world"])).lstrip("/")
        self.body_frame = str(rospy.get_param("~body_frame", frames["body"])).lstrip("/")
        self.control_rate_hz = float(timing["control_rate_hz"])
        self.odom_timeout = float(timing["odom_timeout_s"])
        self.max_prediction = float(timing["max_state_prediction_s"])
        prediction_cfg = self.controller["localization_prediction"]
        self.max_repropagation = float(prediction_cfg["max_repropagation_s"])
        self.pointlio_warning_age = float(prediction_cfg["warning_age_s"])
        self.pointlio_hard_age = float(prediction_cfg["hard_age_s"])
        self.benign_reorder_tolerance = float(
            prediction_cfg.get("benign_reorder_tolerance_s", 0.001)
        )
        self.benign_reorder_position_tolerance = float(
            prediction_cfg.get("benign_reorder_position_tolerance_m", 0.05)
        )
        self.benign_reorder_yaw_tolerance = float(
            prediction_cfg.get("benign_reorder_yaw_tolerance_rad", 0.05)
        )
        self.actuation_prediction = float(timing["actuation_prediction_s"])
        if abs(self.actuation_prediction) > 1.0e-9:
            raise ValueError(
                "actuation_prediction_s must be zero; speed_dead_time_s owns "
                "the committed actuator horizon"
            )
        self.collision_timeout = float(safety["collision_timeout_s"])
        self.collision_required = bool(
            rospy.get_param("~collision_required", safety.get("collision_required", False))
        )
        self.latency_reduction_start = float(
            safety["latency_speed_reduction_start_s"]
        )
        self.latency_speed_limit = float(safety["latency_speed_limit_mps"])
        self.observer_model_only_speed_limit = float(
            safety.get("observer_model_only_speed_limit_mps", 1.0)
        )
        self.observer_invalid_speed_limit = float(
            safety.get("observer_invalid_speed_limit_mps", 0.35)
        )
        self.real_topic = "/tianracer/ackermann_cmd_stamped"
        allow_real_config = bool(safety.get("allow_real_command_topic", False))
        allow_real_runtime = bool(rospy.get_param("~allow_real_command_topic", False))
        if self.command_topic == self.real_topic and not (
            allow_real_config and allow_real_runtime
        ):
            raise RuntimeError(
                "real command topic requested without both configuration and runtime approval"
            )

        self._lock = threading.Lock()
        self._odom_state: np.ndarray | None = None
        self._odom_stamp: float | None = None
        self._odom_receive_stamp: float | None = None
        self._last_odom_stamp: float | None = None
        self._theta_guess: float | None = None
        self._last_control = np.zeros(3)  # speed, published delta_c rate, v_theta
        self._collision = False
        self._collision_received_monotonic = float("-inf")
        self.enabled = bool(safety["start_enabled"] and rospy.get_param("~start_enabled", False))
        race_speed_cap = min(
            float(safety["global_speed_max_mps"]),
            float(safety["baseline_speed_max_mps"])
            if self.formulation == "baseline"
            else self.vehicle.max_speed,
        )
        self.startup = StartupStateMachine(
            self.controller,
            race_speed_cap,
            min(
                self.vehicle.max_steer_rate,
                float(self.controller["startup"]["steering_rate_max_radps"]),
            ),
        )
        self.command_manager = CommandManager(self.controller, self.vehicle.max_steer)
        self._runtime_speed_cap = race_speed_cap
        self.startup.reset(rospy.Time.now().to_sec())
        self._last_published = None
        self._last_solver_result = None
        self._last_solver_reason = ""
        self._pointlio_valid = False
        self._localization_jump_count = 0
        self.prediction_marker_period = 1.0 / max(
            float(output.get("prediction_marker_rate_hz", 5.0)), 0.1
        )
        self.full_prediction_log_period = 1.0 / max(
            float(output.get("full_prediction_log_rate_hz", 2.0)), 0.1
        )
        self._last_prediction_marker_time = float("-inf")
        self._last_full_prediction_log_time = float("-inf")
        self.telemetry = JsonlTelemetry(
            rospy.get_param("~telemetry_path", ""),
            queue_size=int(output.get("telemetry_queue_size", 512)),
        )

        self.command_pub = rospy.Publisher(
            self.command_topic, AckermannDriveStamped, queue_size=1
        )
        self.diagnostic_pub = rospy.Publisher(
            str(topics["telemetry"]), Float32MultiArray, queue_size=5
        )
        self.marker_pub = rospy.Publisher(
            str(topics["markers"]), MarkerArray, queue_size=1, latch=True
        )
        self.prediction_marker_pub = rospy.Publisher(
            str(topics.get("prediction_markers", "/f1tenth_mpcc/prediction")),
            MarkerArray,
            queue_size=1,
        )
        self._track_markers = self._build_track_markers()
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        self.imu_sub = rospy.Subscriber(
            "/livox/imu", Imu, self._imu_callback, queue_size=50,
            tcp_nodelay=True,
        )
        self.wheel_odom_sub = rospy.Subscriber(
            str(topics["wheel_odom"]), Odometry, self._wheel_odom_callback,
            queue_size=10, tcp_nodelay=True,
        )
        self.collision_sub = None
        if self.collision_required:
            self._collision = True
            self.collision_sub = rospy.Subscriber(
                self.collision_topic, Bool, self._collision_callback, queue_size=1
            )
        self.enable_service = rospy.Service("~set_enabled", SetBool, self._enable_callback)
        self.solver_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.control_rate_hz), self._control_tick
        )
        self.publisher_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / float(self.controller["publisher"]["rate_hz"])),
            self._publisher_tick,
        )
        rospy.on_shutdown(self._shutdown)
        self._publish_stop()
        self.marker_pub.publish(MarkerArray(markers=self._track_markers))
        rospy.loginfo(
            "Dynamic MPCC ready formulation=%s odom=%s command=%s enabled=%s",
            self.formulation,
            self.odom_topic,
            self.command_topic,
            self.enabled,
        )
        rospy.loginfo(
            "Speed profile source=%s range=%.3f..%.3f m/s cap=%.3f m/s",
            self.track.speed_profile_source,
            float(np.min(self.track.speed_nodes)),
            float(np.max(self.track.speed_nodes)),
            float(self.controller["speed_planning"]["max_speed_mps"]),
        )

    def _shutdown(self) -> None:
        self._publish_stop()
        self.telemetry.close()

    def _collision_callback(self, message: Bool) -> None:
        with self._lock:
            self._collision = bool(message.data)
            self._collision_received_monotonic = time.monotonic()

    def _imu_callback(self, message: Imu) -> None:
        stamp = message.header.stamp.to_sec()
        yaw_rate = message.angular_velocity.z
        if math.isfinite(stamp) and math.isfinite(yaw_rate):
            self.state_predictor.push_imu(stamp, yaw_rate)

    def _wheel_odom_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp.to_sec()
        vx = message.twist.twist.linear.x
        if math.isfinite(stamp) and math.isfinite(vx):
            self.state_predictor.push_wheel(stamp, vx)

    def _odom_callback(self, message: Odometry) -> None:
        receive_stamp = rospy.Time.now().to_sec()
        frame = message.header.frame_id.lstrip("/")
        child = message.child_frame_id.lstrip("/")
        if frame != self.world_frame or child != self.body_frame:
            rospy.logerr_throttle(
                1.0,
                "Rejecting odometry frames %s -> %s; expected %s -> %s",
                frame,
                child,
                self.world_frame,
                self.body_frame,
            )
            return
        stamp = (
            message.header.stamp.to_sec()
            if message.header.stamp != rospy.Time()
            else rospy.Time.now().to_sec()
        )
        values = np.asarray(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw_from_odometry(message),
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            rospy.logerr_throttle(1.0, "Rejecting non-finite Point-LIO odometry")
            self.steering_observer.invalidate()
            return
        with self._lock:
            previous_state = None if self._odom_state is None else self._odom_state.copy()
            previous_stamp = self._odom_stamp
        timestamp_regression = previous_stamp is not None and stamp <= previous_stamp
        pose_jump = False
        position_jump = 0.0
        yaw_jump = 0.0
        if previous_state is not None:
            position_jump = float(np.linalg.norm(values[:2] - previous_state[:2]))
            yaw_jump = abs(float(wrap_angle(values[2] - previous_state[2])))
            pose_jump = (
                position_jump > float(self.controller["startup"]["localization_jump_distance_m"])
                or yaw_jump > float(self.controller["startup"]["localization_jump_yaw_rad"])
            )
        if benign_reorder(
            previous_stamp,
            stamp,
            position_jump,
            yaw_jump,
            self.benign_reorder_tolerance,
            self.benign_reorder_position_tolerance,
            self.benign_reorder_yaw_tolerance,
        ):
            # Drop only this stale sample. Preserve the latest valid state,
            # observer and startup mode; the next forward sample continues
            # normally without forcing SAFE_DECEL.
            rospy.logwarn_throttle(
                5.0,
                "Dropping benign Point-LIO reorder dt=%.6fs dpos=%.4fm dyaw=%.4frad",
                stamp - float(previous_stamp),
                position_jump,
                yaw_jump,
            )
            return
        pointlio_valid = not timestamp_regression and not pose_jump
        if not pointlio_valid:
            self.steering_observer.invalidate()
            with self._lock:
                self._pointlio_valid = False
                self._localization_jump_count += 1
                # For a forward-timestamp pose reset, seed the new location so
                # that the next stable sample can revalidate the stream. Never
                # replace state on an out-of-order timestamp.
                if pose_jump and not timestamp_regression:
                    projection = self.track.project(values[0], values[1], None)
                    self._theta_guess = projection.s
                    self._odom_state = np.r_[
                        values,
                        self.steering_observer.estimate().delta_hat,
                        float(self.command_manager.published[1]),
                        projection.s,
                    ]
                    self._odom_stamp = stamp
                    self._odom_receive_stamp = receive_stamp
            if self.enabled:
                self.startup.force_safe_decel(rospy.Time.now().to_sec())
            rospy.logwarn_throttle(1.0, "Rejecting Point-LIO timestamp regression or pose jump")
            return
        observer = self.steering_observer.update(
            stamp, values[3], values[4], values[5]
        )
        with self._lock:
            projection = self.track.project(values[0], values[1], self._theta_guess)
            self._theta_guess = projection.s
            published_steering = float(self.command_manager.published[1])
            self._odom_state = np.r_[
                values, observer.delta_hat, published_steering, projection.s
            ]
            self._odom_stamp = stamp
            self._odom_receive_stamp = receive_stamp
            self._last_odom_stamp = stamp
            self._pointlio_valid = True
        self.startup.update_observer(
            rospy.Time.now().to_sec(), observer.valid, observer.confidence, pointlio_valid
        )

    def _enable_callback(self, request: SetBool.Request) -> SetBoolResponse:
        with self._lock:
            if request.data and self._odom_state is None:
                return SetBoolResponse(False, "cannot enable before valid odometry")
            self.enabled = bool(request.data)
            if self.enabled:
                self.startup.reset(rospy.Time.now().to_sec())
            if not self.enabled:
                self._last_control[:] = 0.0
                self.state_predictor.clear()
        if not request.data:
            self._publish_stop()
        return SetBoolResponse(True, "enabled" if request.data else "disabled")

    def _publish(self, control: np.ndarray, stamp: rospy.Time | None = None) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = rospy.Time.now() if stamp is None else stamp
        message.header.frame_id = self.body_frame
        message.drive.speed = float(control[0])
        message.drive.steering_angle = float(control[1])
        self.command_pub.publish(message)

    def _publish_stop(self) -> None:
        steering = (
            float(self.command_manager.published[1])
            if hasattr(self, "command_manager")
            else 0.0
        )
        self._publish(np.asarray([0.0, steering, 0.0]))

    def _publisher_tick(self, _event: rospy.TimerEvent) -> None:
        now_ros = rospy.Time.now()
        now = now_ros.to_sec()
        profile = self.startup.profile(now)
        with self._lock:
            runtime_speed_cap = self._runtime_speed_cap
        profile = replace(profile, speed_cap=min(profile.speed_cap, runtime_speed_cap))
        if not self.enabled:
            profile = profile.__class__(STOPPED, 0.0, 0.0,
                float(self.controller["fallback"]["steering_recenter_rate_radps"]),
                0.0, float("inf"))
        published = self.command_manager.tick(now, profile)
        command = np.asarray(
            [published.speed, published.steering, published.virtual_speed], dtype=float
        )
        previous_steering = (
            published.steering
            if self._last_published is None
            else self._last_published.steering
        )
        steering_rate = (published.steering - previous_steering) / published.publish_dt
        model_control = np.asarray(
            [published.speed, steering_rate, published.virtual_speed], dtype=float
        )
        with self._lock:
            self._last_control = model_control
            self._last_published = published
        self.command_history.push(PublishedCommandSample(
            now,
            published.speed,
            published.steering,
            published.virtual_speed,
            steering_rate,
        ))
        self.steering_observer.push_command(now, published.steering)
        self._publish(command, now_ros)

    def _line_marker(
        self, marker_id: int, name: str, points: list[Point], rgba: tuple[float, ...]
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.ns = "f1tenth_dynamic_mpcc"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.text = name
        marker.points = points
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float = 0.0) -> Point:
        point = Point()
        point.x, point.y, point.z = float(x), float(y), float(z)
        return point

    def _build_track_markers(self) -> list[Marker]:
        samples = np.linspace(0.0, self.track.length, 501)
        center, left, right, control_left, control_right = [], [], [], [], []
        control_margin = (
            self.vehicle.body_width / 2.0
            + float(self.controller["safety"]["track_control_margin_m"])
            + 0.05
        )
        for s in samples:
            x, y = self.track.position(float(s))
            yaw = float(self.track.tangent(float(s)))
            nx, ny = -math.sin(yaw), math.cos(yaw)
            wl = float(self.track.width_left(float(s)))
            wr = float(self.track.width_right(float(s)))
            center.append(self._point(x, y, 0.02))
            left.append(self._point(x + nx * wl, y + ny * wl, 0.01))
            right.append(self._point(x - nx * wr, y - ny * wr, 0.01))
            control_left.append(
                self._point(x + nx * max(wl - control_margin, 0.0),
                            y + ny * max(wl - control_margin, 0.0), 0.03)
            )
            control_right.append(
                self._point(x - nx * max(wr - control_margin, 0.0),
                            y - ny * max(wr - control_margin, 0.0), 0.03)
            )
        return [
            self._line_marker(0, "raceline", center, (0.2, 0.8, 1.0, 1.0)),
            self._line_marker(1, "track_left", left, (1.0, 0.2, 0.2, 1.0)),
            self._line_marker(2, "track_right", right, (1.0, 0.2, 0.2, 1.0)),
            self._line_marker(3, "control_left", control_left, (1.0, 0.8, 0.1, 0.9)),
            self._line_marker(4, "control_right", control_right, (1.0, 0.8, 0.1, 0.9)),
        ]

    def _publish_prediction_marker(self, result, now: float) -> None:
        if (
            result is None
            or now - self._last_prediction_marker_time < self.prediction_marker_period
            or not np.all(np.isfinite(result.states))
        ):
            return
        predicted = [self._point(state[0], state[1], 0.06) for state in result.states]
        marker = self._line_marker(
            10, "predicted_trajectory", predicted, (0.3, 1.0, 0.3, 1.0)
        )
        stamp = rospy.Time.now()
        marker.header.stamp = stamp
        self.prediction_marker_pub.publish(MarkerArray(markers=[marker]))
        self._last_prediction_marker_time = now

    def _control_tick(self, _event: rospy.TimerEvent) -> None:
        now_ros = rospy.Time.now()
        now = now_ros.to_sec()
        with self._lock:
            if not self.enabled:
                return
            if self._odom_state is None or self._odom_stamp is None:
                return
            measured = self._odom_state.copy()
            measurement_stamp = self._odom_stamp
            measurement_receive_stamp = self._odom_receive_stamp
            collision = self._collision
            collision_age = time.monotonic() - self._collision_received_monotonic
            last_control = self._last_control.copy()
            pointlio_valid = self._pointlio_valid
        if not pointlio_valid:
            self.command_manager.note_failure()
            self.startup.note_solver(False, now)
            self.startup.force_safe_decel(now)
            rospy.logwarn_throttle(1.0, "MPCC safe decel: Point-LIO stream not revalidated")
            return
        if self.collision_required and (
            collision_age > self.collision_timeout or collision
        ):
            self.command_manager.note_failure()
            self.startup.note_solver(False, now)
            self.startup.force_safe_decel(now)
            rospy.logwarn_throttle(1.0, "MPCC stopping: collision state unsafe or stale")
            return
        age = now - measurement_stamp
        if age > min(self.odom_timeout, self.pointlio_hard_age) or age < -0.02:
            self.command_manager.note_failure()
            self.startup.note_solver(False, now)
            self.startup.force_safe_decel(now)
            rospy.logwarn_throttle(1.0, "MPCC stopping: odom age %.3fs", age)
            return
        # Keep timestamp diagnostics valid even when prediction or solving throws.
        # A successful committed-horizon prediction replaces these defaults.
        committed_output_stamp = now
        committed_prediction_horizon = 0.0
        try:
            prediction = self.state_predictor.correct_delayed_measurement_and_repropagate(
                measured, measurement_stamp, now, self.max_repropagation,
                fallback_control=last_control,
            )
            committed = self.state_predictor.predict_committed_horizon(
                prediction.state, now, fallback_control=last_control
            )
            committed_output_stamp = committed.output_stamp
            committed_prediction_horizon = committed.prediction_horizon
            compensated = committed.state
            projection = self.track.project(compensated[0], compensated[1], compensated[8])
            compensated[7] = float(self.command_manager.published[1])
            compensated[8] = projection.s
            observer = self.steering_observer.estimate()
            observer_age = (
                float("inf")
                if observer.timestamp is None
                else max(now - observer.timestamp, 0.0)
            )
            observer_stale = observer_age > self.steering_observer.p.dropout_timeout_s
            profile = self.startup.profile(now)
            runtime_speed_cap = profile.speed_cap
            latency_limited = age > self.pointlio_warning_age
            if latency_limited:
                runtime_speed_cap = min(runtime_speed_cap, self.latency_speed_limit)
            if observer_stale or not observer.valid or observer.mode == INVALID:
                runtime_speed_cap = min(runtime_speed_cap, self.observer_invalid_speed_limit)
            elif observer.mode == MODEL_ONLY:
                runtime_speed_cap = min(
                    runtime_speed_cap, self.observer_model_only_speed_limit
                )
            solve_profile = replace(profile, speed_cap=runtime_speed_cap)
            with self._lock:
                self._runtime_speed_cap = runtime_speed_cap
            result = self.solver.solve(
                compensated,
                speed_cap=solve_profile.speed_cap,
                steering_rate_cap=solve_profile.steering_rate_cap,
                progress_scale=solve_profile.progress_scale,
            )
            if result.success:
                self.command_manager.set_solution(result.states, result.controls, now)
                self.startup.note_solver(True, now)
                failure_reason = ""
            else:
                self.command_manager.note_failure()
                self.startup.note_solver(False, now)
                failure_reason = result.reason
        except Exception as exc:
            result = None
            self.command_manager.note_failure()
            self.startup.note_solver(False, now)
            failure_reason = str(exc)
            projection = self.track.project(measured[0], measured[1], measured[8])
            compensated = measured
        published = self.command_manager.published
        command = published.copy()
        observer = self.steering_observer.estimate()
        observer_age = (
            float("inf")
            if observer.timestamp is None
            else max(now - observer.timestamp, 0.0)
        )
        observer_stale = observer_age > self.steering_observer.p.dropout_timeout_s
        self._publish_prediction_marker(result, now)

        alpha_f, alpha_r = self.model.slip_angles(compensated)
        psi_ref = float(self.track.tangent(projection.s))
        heading_error = float(wrap_angle(compensated[2] - psi_ref))
        beta = math.atan2(compensated[4], math.hypot(compensated[3], self.vehicle.vx_regularization))
        self.diagnostic_pub.publish(
            Float32MultiArray(
                data=[
                    float(projection.e_contour),
                    float(projection.e_lag),
                    heading_error,
                    beta,
                    alpha_f,
                    alpha_r,
                    float(command[0]),
                    float(command[1]),
                    float(age),
                    float("nan") if result is None else float(result.solve_time_s),
                    float(self.startup.failure_count),
                ]
            )
        )
        include_prediction = (
            result is not None
            and now - self._last_full_prediction_log_time
            >= self.full_prediction_log_period
        )
        profile = self.startup.profile(now)
        last_published = self._last_published
        record = {
                "timestamp": now,
                "timing": {
                    "control_now": now,
                    "pointlio_header_stamp": measurement_stamp,
                    "pointlio_receive_time": measurement_receive_stamp,
                    "pointlio_message_age": age,
                    "measurement_prediction_start": measurement_stamp,
                    "measurement_prediction_end": now,
                    "measurement_prediction_horizon": age,
                    "speed_dead_time": self.vehicle.speed_dead_time,
                    "committed_prediction_start": now,
                    "committed_prediction_end": committed_output_stamp,
                    "committed_prediction_horizon": committed_prediction_horizon,
                    "mpcc_initial_state_timestamp": committed_output_stamp,
                },
                "measurement_timestamp": measurement_stamp,
                "measurement_age_s": age,
                "state_measured": measured,
                "state_compensated": compensated,
                "track": {
                    "theta": projection.s,
                    "x_ref": projection.x_ref,
                    "y_ref": projection.y_ref,
                    "psi_ref": projection.psi_ref,
                    "kappa": float(self.track.curvature(projection.s)),
                    "w_left": float(self.track.width_left(projection.s)),
                    "w_right": float(self.track.width_right(projection.s)),
                },
                "errors": {
                    "contour": projection.e_contour,
                    "lag": projection.e_lag,
                    "heading": heading_error,
                    "beta": beta,
                    "alpha_f": alpha_f,
                    "alpha_r": alpha_r,
                },
                "command": command,
                "actuator": {
                    "speed_command": command[0],
                    "steering_command": command[1],
                    "steering_target": observer.delta_target,
                    "steering_prediction": observer.delta_pred,
                    "steering_pseudo": observer.delta_pseudo,
                    "steering_kinematic": observer.delta_kinematic,
                    "steering_dynamic": observer.delta_dynamic,
                    "steering_estimate": observer.delta_hat,
                    "steering_innovation": observer.innovation,
                    "steering_estimate_variance": observer.variance,
                    "steering_observer_confidence": observer.confidence,
                    "steering_observer_valid": observer.valid,
                    "steering_observer_mode": observer.mode,
                    "steering_observer_age_s": observer_age,
                    "steering_observer_stale": observer_stale,
                    "steering_correction_gain": observer.correction_gain,
                },
                "latency_speed_limited": latency_limited,
                "mode": self.startup.mode,
                "control_mode": self.startup.mode,
                "failure_reason": failure_reason,
                "startup": {
                    "ramp_lambda": profile.ramp_lambda,
                    "current_speed_cap": profile.speed_cap,
                    "current_steering_rate_cap": profile.steering_rate_cap,
                    "current_progress_weight_scale": profile.progress_scale,
                    "current_progress_weight": (
                        float(self.controller["cost"][
                            "progress_reward_debug"
                            if self.formulation == "baseline"
                            or str(self.controller.get("mode", "debug")).lower() == "debug"
                            else "progress_reward_race"
                        ]) * profile.progress_scale
                    ),
                    "observer_valid_samples": self.startup.observer_valid_samples,
                    "localization_jump_count": self._localization_jump_count,
                },
                "publisher": None if last_published is None else {
                    "published_speed": last_published.speed,
                    "published_steering": last_published.steering,
                    "actual_publish_dt": last_published.publish_dt,
                    "slew_limiter_active": last_published.slew_limiter_active,
                    "source": last_published.source,
                },
                "fallback": {
                    "failure_count": self.startup.failure_count,
                    "fallback_level": (
                        2 if self.startup.mode == SAFE_DECEL
                        else 3 if self.startup.mode == STOPPED
                        else 1 if self.startup.failure_count else 0
                    ),
                    "shifted_previous_solution_used":
                        self.command_manager.shifted_previous_solution_used,
                },
                "solver": None
                if result is None
                else {
                    "status": result.status,
                    "success": result.success,
                    "solve_time_s": result.solve_time_s,
                    "objective": result.objective,
                    "track_slack_max": result.track_slack_max,
                    "tire_slack_max": result.tire_slack_max,
                    "qp_iterations": result.qp_iterations,
                    "rti_iterations": result.rti_iterations,
                    "first_steer_rate_radps": result.first_steer_rate_radps,
                    "requested_speed": result.controls[0, 0],
                    "requested_delta_c": result.states[1, 7],
                    "requested_delta_c_rate": result.controls[0, 1],
                    "prediction_logged": include_prediction,
                },
            }
        if include_prediction:
            record["solver"]["predicted_states"] = result.states
            record["solver"]["predicted_controls"] = result.controls
            self._last_full_prediction_log_time = now
        if not self.telemetry.write(record):
            rospy.logwarn_throttle(
                5.0,
                "MPCC telemetry queue full; dropped records=%d",
                self.telemetry.dropped_records,
            )


def main() -> None:
    rospy.init_node("f1tenth_dynamic_mpcc")
    DynamicMPCCNode()
    rospy.spin()


if __name__ == "__main__":
    main()
