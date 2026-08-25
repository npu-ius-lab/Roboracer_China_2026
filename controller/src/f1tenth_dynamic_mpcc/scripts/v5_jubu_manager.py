#!/usr/bin/env python3
"""V5 JUBU behavior manager and fail-closed local-reference interface."""

from __future__ import annotations

import copy
from collections import deque
import json
import math
import threading

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from opponent_perception.msg import TargetArray
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_v5_jubu.state_machine import (
    ABORT,
    FOLLOW,
    FREE,
    PASS,
    PREPARE,
    RETURN,
    OvertakeStateMachine,
    StateMachineConfig,
    StateObservation,
)
from f1tenth_v5_jubu.track import PeriodicTrack
from f1tenth_v5_jubu.time_sync import (
    TimedPlanarState,
    body_to_world_vector,
    body_to_world_xy,
    interpolate_timed_state,
    propagate_xy,
)
from f1tenth_dynamic_mpcc.v5_jubu import (
    SpeedPolicy,
    TemporalTargetConfirmation,
    inputs_healthy,
)


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class V5JubuManager:
    # PREPARE is a confirmation dwell. Lateral motion begins only in PASS.
    # ABORT intentionally removes the reference and commands a zero speed cap.
    ACTIVE_REFERENCE_STATES = (PASS, RETURN)
    ACTIONS = {
        FREE: ("CLEAR", "无障碍"),
        FOLLOW: ("WAIT", "避障等待"),
        PREPARE: ("WAIT_PREPARE", "等待并准备超车"),
        PASS: ("OVERTAKE", "超车"),
        RETURN: ("RETURN", "超车后返回"),
        ABORT: ("AVOID_WAIT", "中止并避障等待"),
    }

    def __init__(self) -> None:
        self.track = PeriodicTrack.from_csv(rospy.get_param("~raceline_csv"))
        self.world_frame = rospy.get_param("~frames/world", "map")
        self.odom_twist_frame = rospy.get_param("~input/odom_twist_frame", "body")
        self.odom_timeout = float(rospy.get_param("~input/odom_timeout_s", 0.30))
        legacy_target_timeout = float(rospy.get_param("~input/target_timeout_s", 0.45))
        self.target_message_timeout = float(
            rospy.get_param("~input/target_message_timeout_s", legacy_target_timeout)
        )
        self.target_measurement_max_age = float(
            rospy.get_param(
                "~input/target_measurement_max_age_s", max(0.80, legacy_target_timeout)
            )
        )
        self.odom_history_duration = float(
            rospy.get_param("~input/odom_history_duration_s", 2.0)
        )
        self.maximum_odom_bracket_gap = float(
            rospy.get_param("~input/maximum_odom_bracket_gap_s", 0.30)
        )
        self.maximum_odom_extrapolation = float(
            rospy.get_param("~input/maximum_odom_extrapolation_s", 0.10)
        )
        if self.odom_twist_frame not in ("body", "world"):
            raise ValueError("input/odom_twist_frame must be body or world")
        self.obstacle_half_width = float(
            rospy.get_param("~classification/obstacle_half_width_m", 0.18)
        )
        self.track_overlap_margin = float(
            rospy.get_param("~classification/track_overlap_margin_m", 0.03)
        )
        self.maximum_projection_distance = float(
            rospy.get_param("~classification/maximum_projection_distance_m", 1.20)
        )
        self.reference_hold_timeout = float(
            rospy.get_param("~state_machine/reference_hold_timeout_s", 1.50)
        )
        self.health_timeout = float(
            rospy.get_param("~control/health_timeout_s", 1.00)
        )
        self.cruise_speed_cap = float(
            rospy.get_param("~control/cruise_speed_cap_mps", 3.00)
        )
        self.pass_speed_cap = float(
            rospy.get_param("~control/pass_speed_cap_mps", 2.20)
        )
        self.follow_minimum_speed = float(
            rospy.get_param("~control/follow_minimum_speed_mps", 0.40)
        )
        self.follow_gap = float(
            rospy.get_param("~control/follow_gap_m", 0.75)
        )
        self.follow_gain = float(
            rospy.get_param("~control/follow_gap_gain", 0.55)
        )
        self.speed_policy = SpeedPolicy(
            cruise_speed_cap=self.cruise_speed_cap,
            pass_speed_cap=self.pass_speed_cap,
            follow_minimum_speed=self.follow_minimum_speed,
            follow_gap=self.follow_gap,
            follow_gain=self.follow_gain,
        )
        if self.health_timeout <= 0.0:
            raise ValueError("invalid V5 JUBU health timeout")
        self.target_confirmation = TemporalTargetConfirmation(
            minimum_frames=int(rospy.get_param("~confirmation/minimum_frames", 2)),
            minimum_duration=float(
                rospy.get_param("~confirmation/minimum_duration_s", 0.15)
            ),
            maximum_gap=float(rospy.get_param("~confirmation/maximum_gap_s", 0.80)),
        )
        sm_config = StateMachineConfig(
            prepare_time=float(rospy.get_param("~state_machine/prepare_time", 0.40)),
            follow_target_loss_timeout=float(
                rospy.get_param("~state_machine/follow_target_loss_timeout", 1.00)
            ),
            return_confirm_gap=float(
                rospy.get_param("~state_machine/return_confirm_gap", 0.45)
            ),
            return_ey_tolerance=float(
                rospy.get_param("~state_machine/return_ey_tolerance", 0.10)
            ),
            abort_center_distance=float(
                rospy.get_param("~state_machine/abort_center_distance", 0.30)
            ),
            abort_recovery_distance=float(
                rospy.get_param("~state_machine/abort_recovery_distance", 0.55)
            ),
        )
        self.state_machine = OvertakeStateMachine(sm_config)

        self.lock = threading.Lock()
        self.odom = None
        self.odom_received = None
        self.odom_history = deque()
        self.targets = None
        self.targets_received = None
        self.selected_path = Path()
        self.accepted_path = Path()
        self.accepted_path_at = None
        self.last_boundary_safe = True
        self.last_payload = {}
        self.diagnostic_received = None

        topics = rospy.get_param("~topics", {})
        self.marker_pub = rospy.Publisher(
            topics.get("markers", "/v5_jubu/markers"), MarkerArray, queue_size=1
        )
        self.track_marker_pub = rospy.Publisher(
            topics.get("track_markers", "/v5_jubu/track_markers"),
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.red_path_pub = rospy.Publisher(
            topics.get("red_local_plan", "/v5_jubu/local_reference"),
            Path,
            queue_size=1,
            latch=True,
        )
        self.status_pub = rospy.Publisher(
            topics.get("status", "/v5_jubu/status"), String, queue_size=10, latch=True
        )
        self.decision_pub = rospy.Publisher(
            topics.get("decision", "/v5_jubu/decision"), String, queue_size=10, latch=True
        )
        self.control_state_pub = rospy.Publisher(
            topics.get("control_state", "/v5_jubu/state"),
            String,
            queue_size=1,
            latch=True,
        )
        self.speed_cap_pub = rospy.Publisher(
            topics.get("speed_cap", "/v5_jubu/local_speed_cap"),
            Float32,
            queue_size=1,
            latch=True,
        )
        self.health_pub = rospy.Publisher(
            topics.get("health", "/v5_jubu/health"),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.odom_sub = rospy.Subscriber(
            topics.get("odom", "/localization/odom"),
            Odometry,
            self.odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.targets_sub = rospy.Subscriber(
            topics.get("targets", "/perception/targets_detailed"),
            TargetArray,
            self.targets_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.path_sub = rospy.Subscriber(
            topics.get("selected_path", "/v5_jubu/selected_path"),
            Path,
            self.path_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.diagnostic_sub = rospy.Subscriber(
            topics.get("diagnostics", "/v5_jubu/candidate_diagnostics"),
            String,
            self.diagnostic_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self.timer = rospy.Timer(rospy.Duration(0.10), self.timer_callback)
        self.track_marker_pub.publish(self.make_track_markers(rospy.Time.now()))
        rospy.logwarn(
            "V5 JUBU manager ready; local-reference authority only, no chassis publisher."
        )

    def odom_callback(self, message: Odometry) -> None:
        received = rospy.Time.now()
        state = self.timed_state_from_odom(message, received)
        with self.lock:
            self.odom = copy.deepcopy(message)
            self.odom_received = received
            if self.odom_history and state.stamp < self.odom_history[-1].stamp:
                self.odom_history.clear()
            self.odom_history.append(state)
            oldest_allowed = state.stamp - self.odom_history_duration
            while self.odom_history and self.odom_history[0].stamp < oldest_allowed:
                self.odom_history.popleft()

    def timed_state_from_odom(
        self, message: Odometry, fallback_stamp: rospy.Time
    ) -> TimedPlanarState:
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        vx = float(message.twist.twist.linear.x)
        vy = float(message.twist.twist.linear.y)
        if self.odom_twist_frame == "body":
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            vx, vy = cos_yaw * vx - sin_yaw * vy, sin_yaw * vx + cos_yaw * vy
        stamp = message.header.stamp
        if stamp == rospy.Time():
            stamp = fallback_stamp
        return TimedPlanarState(
            stamp=stamp.to_sec(),
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            yaw=yaw,
            vx=vx,
            vy=vy,
        )

    def targets_callback(self, message: TargetArray) -> None:
        with self.lock:
            self.targets = copy.deepcopy(message)
            self.targets_received = rospy.Time.now()

    def path_callback(self, message: Path) -> None:
        now = rospy.Time.now()
        with self.lock:
            self.selected_path = copy.deepcopy(message)
            if message.poses:
                self.accepted_path = copy.deepcopy(message)
                self.accepted_path_at = now

    def diagnostic_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Invalid overtaking diagnostic JSON: %s", error)
            return
        now = rospy.Time.now()
        with self.lock:
            self.diagnostic_received = now
            accepted_path_at = self.accepted_path_at
            accepted_available = bool(self.accepted_path.poses)
            previous_state = self.state_machine.state
        if accepted_path_at is not None:
            accepted_available = bool(
                accepted_available
                and (now - accepted_path_at).to_sec() <= self.reference_hold_timeout
            )

        selected_side = str(payload.get("selected_side", "none"))
        selected_summary = payload.get(selected_side, {}) if selected_side in ("left", "right") else {}
        boundary_clearance = selected_summary.get("minimum_boundary_clearance")
        if boundary_clearance is not None:
            self.last_boundary_safe = float(boundary_clearance) >= -1.0e-6
        boundary_safe = self.last_boundary_safe if accepted_available else True
        target_visible = bool(payload.get("target_visible", False))
        center_distance = payload.get("target_center_distance")
        if center_distance is None:
            center_distance = math.inf
        target_id = int(payload.get("target_id", -1))
        measurement_stamp = float(payload.get("target_measurement_stamp_s", now.to_sec()))
        locked_target_id = self.target_confirmation.target_id
        target_confirmed = False
        if (
            target_visible
            and bool(payload.get("relevant", False))
            and bool(payload.get("overtake_authorized", False))
            and target_id >= 0
        ):
            target_confirmed = self.target_confirmation.update(
                target_id, measurement_stamp, now.to_sec()
            )
        else:
            self.target_confirmation.reset()
        if (
            previous_state in self.ACTIVE_REFERENCE_STATES
            and locked_target_id is not None
            and target_id >= 0
            and target_id != locked_target_id
        ):
            target_visible = False
            payload["reason"] = "target_identity_changed"
        observation = StateObservation(
            target_visible=target_visible,
            relevant=bool(payload.get("relevant", False)),
            valid_area=bool(payload.get("valid_overtake_area", False)),
            authorized=bool(payload.get("overtake_authorized", False)),
            candidate_ready=bool(
                target_confirmed and selected_side in ("left", "right")
            ),
            reference_available=accepted_available,
            boundary_safe=boundary_safe,
            opponent_signed_delta=float(payload.get("opponent_signed_delta", math.inf)),
            ego_ey=float(payload.get("ego_ey", 0.0)),
            center_distance=float(center_distance),
        )
        state = self.state_machine.update(now.to_sec(), observation)
        with self.lock:
            self.last_payload = payload
            if state == FREE and previous_state != FREE:
                self.accepted_path = Path()
                self.accepted_path_at = None
        self.publish(now, state, payload)

    def timer_callback(self, _event) -> None:
        now = rospy.Time.now()
        with self.lock:
            received = self.odom_received
            payload = dict(self.last_payload)
        if received is None or (now - received).to_sec() > self.odom_timeout:
            payload.update(
                valid=False,
                target_visible=False,
                relevant=False,
                reason="odom_missing_or_stale",
            )
        if not self.planner_healthy(now) and self.state_machine.state in self.ACTIVE_REFERENCE_STATES:
            self.state_machine._transition(ABORT, now.to_sec())
            payload.update(valid=False, reason="planner_health_timeout")
        self.publish(now, self.state_machine.state, payload)

    def planner_healthy(self, now: rospy.Time) -> bool:
        with self.lock:
            odom_received = self.odom_received
            targets_received = self.targets_received
            diagnostic_received = self.diagnostic_received
        return inputs_healthy(
            now.to_sec(),
            None if odom_received is None else odom_received.to_sec(),
            None if targets_received is None else targets_received.to_sec(),
            None if diagnostic_received is None else diagnostic_received.to_sec(),
            self.odom_timeout,
            self.target_message_timeout,
            self.health_timeout,
        )

    def speed_cap(self, state: str, payload: dict, healthy: bool) -> float:
        return self.speed_policy.cap(state, payload, healthy)

    def classify_targets(self, now: rospy.Time):
        with self.lock:
            odom = copy.deepcopy(self.odom)
            odom_received = self.odom_received
            targets = copy.deepcopy(self.targets)
            targets_received = self.targets_received
            odom_history = list(self.odom_history)
        if (
            odom is None
            or targets is None
            or odom_received is None
            or targets_received is None
            or (now - odom_received).to_sec() > self.odom_timeout
            or (now - targets_received).to_sec() > self.target_message_timeout
        ):
            return [], None
        target_stamp = targets.header.stamp
        if target_stamp == rospy.Time():
            target_stamp = targets_received
        target_age = max(0.0, (now - target_stamp).to_sec())
        if target_age > self.target_measurement_max_age:
            return [], odom
        measurement_ego = interpolate_timed_state(
            odom_history,
            target_stamp.to_sec(),
            maximum_bracket_gap=self.maximum_odom_bracket_gap,
            maximum_extrapolation=self.maximum_odom_extrapolation,
        )
        if measurement_ego is None:
            return [], odom
        current_ego = self.timed_state_from_odom(odom, odom_received)
        if measurement_ego.stamp > current_ego.stamp:
            current_ego = measurement_ego
        prediction_dt = max(0.0, current_ego.stamp - measurement_ego.stamp)
        ego_projection = self.track.project(current_ego.x, current_ego.y)
        classified = []
        for target in targets.targets:
            local_x = float(target.pose.position.x)
            local_y = float(target.pose.position.y)
            map_x, map_y = body_to_world_xy(local_x, local_y, measurement_ego)
            map_vx, map_vy = body_to_world_vector(
                float(target.twist.linear.x),
                float(target.twist.linear.y),
                measurement_ego.yaw,
            )
            map_x, map_y = propagate_xy(
                map_x, map_y, map_vx, map_vy, prediction_dt
            )
            projection = self.track.project_near(
                map_x,
                map_y,
                ego_projection.s,
                max(8.0, math.hypot(local_x, local_y) + 2.0),
            )
            lower = -projection.width_right - self.track_overlap_margin
            upper = projection.width_left + self.track_overlap_margin
            footprint_low = projection.ey - self.obstacle_half_width
            footprint_high = projection.ey + self.obstacle_half_width
            overlaps_track = footprint_high >= lower and footprint_low <= upper
            on_track = bool(
                projection.distance <= self.maximum_projection_distance and overlaps_track
            )
            classified.append(
                {
                    "id": int(target.id),
                    "map_x": map_x,
                    "map_y": map_y,
                    "local_x": local_x,
                    "local_y": local_y,
                    "yaw": measurement_ego.yaw
                    + yaw_from_quaternion(target.pose.orientation),
                    "speed": math.hypot(map_vx, map_vy),
                    "s": projection.s,
                    "ey": projection.ey,
                    "on_track": on_track,
                    "measurement_age_s": target_age,
                    "prediction_dt_s": prediction_dt,
                }
            )
        return classified, odom

    def publish(self, now: rospy.Time, state: str, payload: dict) -> None:
        classified, odom = self.classify_targets(now)
        healthy = self.planner_healthy(now)
        speed_cap = self.speed_cap(state, payload, healthy)
        on_track_count = sum(1 for item in classified if item["on_track"])
        off_track_count = len(classified) - on_track_count
        action, action_zh = self.ACTIONS[state]
        output = {
            "stamp": now.to_sec(),
            "state": state,
            "action": action,
            "action_zh": action_zh,
            "target_visible": bool(payload.get("target_visible", False)),
            "on_track_obstacles": on_track_count,
            "off_track_obstacles": off_track_count,
            "selected_side": payload.get("selected_side", "none"),
            "valid_overtake_area": bool(payload.get("valid_overtake_area", False)),
            "valid_overtake_sides": payload.get("valid_overtake_sides", []),
            "reason": payload.get("reason", "waiting_for_diagnostics"),
            "command_authority": False,
            "local_reference_authority": True,
            "planner_healthy": healthy,
            "speed_cap_mps": speed_cap,
            "confirmation_hits": self.target_confirmation.hits,
            "confirmation_target_id": self.target_confirmation.target_id,
            "target_state_source": "/perception/targets_detailed",
            "uses_target_truth": False,
        }
        self.status_pub.publish(String(data=json.dumps(output, sort_keys=True)))
        self.decision_pub.publish(String(data=f"{action} | {action_zh}"))
        self.control_state_pub.publish(
            String(data="GLOBAL" if state == FREE else state)
        )
        self.speed_cap_pub.publish(Float32(data=speed_cap))
        self.health_pub.publish(Bool(data=healthy))
        self.publish_red_path(now, state)
        self.marker_pub.publish(
            self.make_dynamic_markers(now, state, action, action_zh, payload, classified, odom)
        )

    def publish_red_path(self, now: rospy.Time, state: str) -> None:
        with self.lock:
            path = copy.deepcopy(self.accepted_path)
            accepted_at = self.accepted_path_at
        valid = bool(
            state in self.ACTIVE_REFERENCE_STATES
            and path.poses
            and accepted_at is not None
            and (now - accepted_at).to_sec() <= self.reference_hold_timeout
        )
        if not valid:
            path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = now
        for pose in path.poses:
            pose.header = path.header
        self.red_path_pub.publish(path)

    def make_dynamic_markers(
        self, now, state, action, action_zh, payload, classified, odom
    ) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, item in enumerate(classified):
            cube = Marker()
            cube.header.frame_id = self.world_frame
            cube.header.stamp = now
            cube.ns = "classified_obstacles"
            cube.id = 100 + index
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = item["map_x"]
            cube.pose.position.y = item["map_y"]
            cube.pose.position.z = 0.14
            cube.pose.orientation.z = math.sin(item["yaw"] * 0.5)
            cube.pose.orientation.w = math.cos(item["yaw"] * 0.5)
            cube.scale.x = 0.40
            cube.scale.y = 0.35
            cube.scale.z = 0.24
            cube.color.a = 0.82
            if item["on_track"]:
                cube.color.r, cube.color.g, cube.color.b = 1.0, 0.08, 0.02
            else:
                cube.color.r, cube.color.g, cube.color.b = 0.70, 0.15, 1.0
            cube.lifetime = rospy.Duration(0.35)
            markers.markers.append(cube)

            label = Marker()
            label.header = cube.header
            label.ns = "obstacle_labels"
            label.id = 200 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = item["map_x"]
            label.pose.position.y = item["map_y"]
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.a = 1.0
            label.color.r, label.color.g, label.color.b = (
                (1.0, 0.15, 0.05) if item["on_track"] else (0.85, 0.40, 1.0)
            )
            kind = "ON-TRACK OBSTACLE" if item["on_track"] else "OFF-TRACK / IGNORE"
            label.text = f"{kind}  ID={item['id']}  ey={item['ey']:+.2f}m"
            label.lifetime = rospy.Duration(0.35)
            markers.markers.append(label)

        if odom is not None:
            ego = Marker()
            ego.header.frame_id = self.world_frame
            ego.header.stamp = now
            ego.ns = "ego_vehicle"
            ego.id = 1
            ego.type = Marker.CUBE
            ego.action = Marker.ADD
            ego.pose = copy.deepcopy(odom.pose.pose)
            ego.pose.position.z = 0.10
            ego.scale.x, ego.scale.y, ego.scale.z = 0.40, 0.24, 0.18
            ego.color.r, ego.color.g, ego.color.b, ego.color.a = 0.05, 0.75, 1.0, 0.85
            markers.markers.append(ego)

            status = Marker()
            status.header = ego.header
            status.ns = "avoidance_decision"
            status.id = 2
            status.type = Marker.TEXT_VIEW_FACING
            status.action = Marker.ADD
            status.pose.position.x = float(odom.pose.pose.position.x)
            status.pose.position.y = float(odom.pose.pose.position.y)
            status.pose.position.z = 0.90
            status.pose.orientation.w = 1.0
            status.scale.z = 0.28
            status.color.a = 1.0
            if state == PASS:
                status.color.r, status.color.g, status.color.b = 1.0, 0.05, 0.02
            elif state in (FOLLOW, PREPARE, ABORT):
                status.color.r, status.color.g, status.color.b = 1.0, 0.75, 0.05
            else:
                status.color.r, status.color.g, status.color.b = 0.05, 1.0, 0.20
            sides = "/".join(payload.get("valid_overtake_sides", [])) or "NONE"
            status.text = f"{action} | FSM={state} | PASS={sides}"
            markers.markers.append(status)

        with self.lock:
            path = copy.deepcopy(self.accepted_path)
            path_at = self.accepted_path_at
        if (
            state in self.ACTIVE_REFERENCE_STATES
            and path.poses
            and path_at is not None
            and (now - path_at).to_sec() <= self.reference_hold_timeout
        ):
            line = Marker()
            line.header.frame_id = self.world_frame
            line.header.stamp = now
            line.ns = "red_local_plan"
            line.id = 3
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.075
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.0, 0.0, 1.0
            for pose in path.poses:
                point = Point()
                point.x = pose.pose.position.x
                point.y = pose.pose.position.y
                point.z = 0.08
                line.points.append(point)
            markers.markers.append(line)
        return markers

    def make_track_markers(self, stamp: rospy.Time) -> MarkerArray:
        markers = MarkerArray()
        lines = []
        for marker_id, namespace, color, width in (
            (1, "raceline", (0.85, 0.85, 0.85), 0.025),
            (2, "left_boundary", (0.10, 0.45, 1.0), 0.045),
            (3, "right_boundary", (0.10, 0.45, 1.0), 0.045),
        ):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.95
            lines.append(marker)
        for x, y, yaw, left, right in zip(
            self.track.x,
            self.track.y,
            self.track.yaw,
            self.track.width_left,
            self.track.width_right,
        ):
            nx, ny = -math.sin(float(yaw)), math.cos(float(yaw))
            points = (
                (float(x), float(y)),
                (float(x) + float(left) * nx, float(y) + float(left) * ny),
                (float(x) - float(right) * nx, float(y) - float(right) * ny),
            )
            for marker, (px, py) in zip(lines, points):
                marker.points.append(Point(x=px, y=py, z=0.02))
        for marker in lines:
            if marker.points:
                marker.points.append(copy.deepcopy(marker.points[0]))
        markers.markers.extend(lines)
        return markers


def main() -> None:
    rospy.init_node("v5_jubu_manager")
    V5JubuManager()
    rospy.spin()


if __name__ == "__main__":
    main()
