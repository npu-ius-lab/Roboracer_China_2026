#!/usr/bin/env python3
"""V5 JUBU candidate generator; it publishes paths and never chassis commands."""

from __future__ import annotations

import copy
from collections import deque
import json
import math
import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from opponent_perception.msg import TargetArray
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_v5_jubu import OvertakePlanner, PeriodicTrack, PlannerConfig, VehicleState
from f1tenth_v5_jubu.time_sync import (
    TimedPlanarState,
    body_to_world_vector,
    body_to_world_xy,
    interpolate_timed_state,
    propagate_xy,
)


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class V5JubuCandidatePlanner:
    def __init__(self) -> None:
        raceline_csv = rospy.get_param("~raceline_csv")
        self.track = PeriodicTrack.from_csv(raceline_csv)
        planner_values = rospy.get_param("~planner", {})
        self.planner = OvertakePlanner(self.track, PlannerConfig.from_mapping(planner_values))

        self.world_frame = rospy.get_param("~frames/world", "map")
        self.odom_twist_frame = rospy.get_param("~input/odom_twist_frame", "body")
        self.odom_timeout = float(rospy.get_param("~input/odom_timeout_s", 0.20))
        legacy_target_timeout = float(rospy.get_param("~input/target_timeout_s", 0.35))
        self.target_message_timeout = float(
            rospy.get_param("~input/target_message_timeout_s", legacy_target_timeout)
        )
        self.target_loss_timeout = float(
            rospy.get_param("~input/target_loss_timeout_s", legacy_target_timeout)
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

        self.lock = threading.Lock()
        self.odom = None
        self.odom_received = None
        self.odom_history = deque()
        self.target_message_received = None
        self.confirmed_target_received = None
        self.active_watchdog_reason = None

        self.left_pub = rospy.Publisher(
            rospy.get_param("~topics/left_path", "/v5_jubu/candidate_left"), Path, queue_size=1
        )
        self.right_pub = rospy.Publisher(
            rospy.get_param("~topics/right_path", "/v5_jubu/candidate_right"), Path, queue_size=1
        )
        self.selected_pub = rospy.Publisher(
            rospy.get_param("~topics/selected_path", "/v5_jubu/selected_path"), Path, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            rospy.get_param("~topics/markers", "/v5_jubu/candidate_markers"), MarkerArray, queue_size=1
        )
        self.diagnostic_pub = rospy.Publisher(
            rospy.get_param("~topics/diagnostics", "/v5_jubu/candidate_diagnostics"), String, queue_size=1
        )
        self.valid_area_pub = rospy.Publisher(
            rospy.get_param("~topics/valid_area", "/v5_jubu/valid_area"),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.odom_sub = rospy.Subscriber(
            rospy.get_param("~topics/odom", "/localization/odom"),
            Odometry,
            self.odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.target_sub = rospy.Subscriber(
            rospy.get_param("~topics/targets", "/perception/targets_detailed"),
            TargetArray,
            self.targets_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.watchdog = rospy.Timer(rospy.Duration(0.10), self.watchdog_callback)
        rospy.logwarn(
            "V5 JUBU candidate planner ready: track=%.2fm; path-only, no command output.",
            self.track.length,
        )

    def odom_callback(self, message: Odometry) -> None:
        received = rospy.Time.now()
        state = self.timed_vehicle_from_odom(message, received)
        with self.lock:
            self.odom = copy.deepcopy(message)
            self.odom_received = received
            if self.odom_history and state.stamp < self.odom_history[-1].stamp:
                self.odom_history.clear()
            self.odom_history.append(state)
            oldest_allowed = state.stamp - self.odom_history_duration
            while self.odom_history and self.odom_history[0].stamp < oldest_allowed:
                self.odom_history.popleft()

    def vehicle_from_odom(self, message: Odometry) -> VehicleState:
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        vx = float(message.twist.twist.linear.x)
        vy = float(message.twist.twist.linear.y)
        if self.odom_twist_frame == "body":
            vx, vy = (
                math.cos(yaw) * vx - math.sin(yaw) * vy,
                math.sin(yaw) * vx + math.cos(yaw) * vy,
            )
        return VehicleState(
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            yaw=yaw,
            vx=vx,
            vy=vy,
        )

    def timed_vehicle_from_odom(
        self, message: Odometry, fallback_stamp: rospy.Time
    ) -> TimedPlanarState:
        state = self.vehicle_from_odom(message)
        stamp = message.header.stamp
        if stamp == rospy.Time():
            stamp = fallback_stamp
        return TimedPlanarState(
            stamp=stamp.to_sec(),
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            vx=state.vx,
            vy=state.vy,
        )

    @staticmethod
    def vehicle_from_timed(state: TimedPlanarState) -> VehicleState:
        return VehicleState(
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            vx=state.vx,
            vy=state.vy,
        )

    @staticmethod
    def opponent_from_target(
        target, measurement_ego: TimedPlanarState, prediction_dt: float
    ) -> VehicleState:
        local_x = float(target.pose.position.x)
        local_y = float(target.pose.position.y)
        local_vx = float(target.twist.linear.x)
        local_vy = float(target.twist.linear.y)
        world_x, world_y = body_to_world_xy(local_x, local_y, measurement_ego)
        world_vx, world_vy = body_to_world_vector(
            local_vx, local_vy, measurement_ego.yaw
        )
        world_x, world_y = propagate_xy(
            world_x, world_y, world_vx, world_vy, prediction_dt
        )
        target_yaw = measurement_ego.yaw + yaw_from_quaternion(target.pose.orientation)
        return VehicleState(
            x=world_x,
            y=world_y,
            yaw=target_yaw,
            vx=world_vx,
            vy=world_vy,
            target_id=int(target.id),
        )

    def targets_callback(self, message: TargetArray) -> None:
        now = rospy.Time.now()
        with self.lock:
            self.target_message_received = now
            self.active_watchdog_reason = None
            odom = copy.deepcopy(self.odom)
            odom_received = self.odom_received
            odom_history = list(self.odom_history)
        if odom is None or odom_received is None or (now - odom_received).to_sec() > self.odom_timeout:
            self.publish_invalid("odom_missing_or_stale", now)
            return
        target_stamp = message.header.stamp if message.header.stamp != rospy.Time() else now
        target_age = max(0.0, (now - target_stamp).to_sec())
        if target_age > self.target_measurement_max_age:
            self.publish_invalid(
                "targets_stale",
                now,
                target_measurement_age_s=target_age,
            )
            return
        if not message.targets:
            with self.lock:
                last_target = self.confirmed_target_received
            # Do not turn one missing radar scan into an instantaneous ABORT.
            # Target message reception and confirmed-target loss are separate:
            # an empty but timely array proves that the perception node is alive.
            if (
                last_target is None
                or (now - last_target).to_sec() > self.target_loss_timeout
            ):
                self.publish_invalid(
                    "no_confirmed_target",
                    now,
                    target_measurement_age_s=target_age,
                )
            return

        measurement_ego = interpolate_timed_state(
            odom_history,
            target_stamp.to_sec(),
            maximum_bracket_gap=self.maximum_odom_bracket_gap,
            maximum_extrapolation=self.maximum_odom_extrapolation,
        )
        if measurement_ego is None:
            self.publish_invalid(
                "target_odom_unavailable",
                now,
                target_measurement_age_s=target_age,
            )
            return

        with self.lock:
            self.confirmed_target_received = now

        current_ego = self.timed_vehicle_from_odom(odom, odom_received)
        # If a target happens to be newer than the latest odometry sample, use
        # the bounded extrapolated measurement-time ego as the planning state.
        if measurement_ego.stamp > current_ego.stamp:
            current_ego = measurement_ego
        prediction_dt = max(0.0, current_ego.stamp - measurement_ego.stamp)

        ego = self.vehicle_from_timed(current_ego)
        results = []
        for target in message.targets:
            opponent = self.opponent_from_target(target, measurement_ego, prediction_dt)
            result = self.planner.evaluate(ego, opponent)
            target_range = math.hypot(opponent.x - ego.x, opponent.y - ego.y)
            results.append((result, int(target.id), target_range))
        result, target_id, target_range = min(
            results,
            key=lambda item: item[0].delta_s if item[0].relevant else math.inf,
        )
        # If every detection is already behind, retain the nearest radar
        # target so PASS can confirm rear clearance without Gazebo truth.
        if not result.relevant:
            result, target_id, target_range = min(results, key=lambda item: item[2])
        self.publish_result(
            result,
            target_id,
            target_range,
            now,
            target_measurement_stamp_s=target_stamp.to_sec(),
            target_measurement_age_s=target_age,
            target_prediction_dt_s=prediction_dt,
            target_frame_id=message.header.frame_id,
        )

    def watchdog_callback(self, _event) -> None:
        now = rospy.Time.now()
        with self.lock:
            received = self.odom_received
            target_received = self.target_message_received
            previous_reason = self.active_watchdog_reason
        reason = None
        if received is not None and (now - received).to_sec() > self.odom_timeout:
            reason = "odom_watchdog_timeout"
        elif target_received is not None and (
            now - target_received
        ).to_sec() > self.target_message_timeout:
            reason = "target_message_watchdog_timeout"
        if reason == previous_reason:
            return
        with self.lock:
            self.active_watchdog_reason = reason
        if reason is not None:
            self.publish_invalid(reason, now)

    def path_message(self, candidate, stamp: rospy.Time) -> Path:
        message = Path()
        message.header.frame_id = self.world_frame
        message.header.stamp = stamp
        if candidate is None:
            return message
        for x, y, yaw in zip(candidate.x, candidate.y, candidate.yaw):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
            pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
            message.poses.append(pose)
        return message

    def candidate_marker(self, candidate, marker_id: int, stamp: rospy.Time, selected: bool) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = "overtake_candidates"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045 if selected else 0.025
        marker.color.a = 1.0 if selected else 0.65
        if not candidate.feasible:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.1, 0.1
        elif candidate.side == "left":
            marker.color.r, marker.color.g, marker.color.b = 0.1, 0.8, 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.1
        for x, y in zip(candidate.x, candidate.y):
            point = Point()
            point.x = float(x)
            point.y = float(y)
            marker.points.append(point)
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def area_status_marker(self, result, valid_sides: list[str], stamp: rospy.Time) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = "overtake_area_status"
        marker.id = 10
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(result.ego.x)
        marker.pose.position.y = float(result.ego.y)
        marker.pose.position.z = 0.72
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.24
        marker.color.a = 1.0
        if valid_sides:
            marker.color.r, marker.color.g, marker.color.b = 0.05, 1.0, 0.20
            sides = "/".join(side.upper() for side in valid_sides)
            marker.text = f"PASS AREA VALID: {sides}"
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.15, 0.05
            marker.text = "NO VALID PASS AREA"
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def publish_result(
        self,
        result,
        target_id: int,
        target_range: float,
        stamp: rospy.Time,
        **timing,
    ) -> None:
        valid_sides = [
            candidate.side for candidate in (result.left, result.right) if candidate.feasible
        ]
        valid_overtake_area = bool(result.relevant and valid_sides)
        overtake_authorized = bool(result.risk and result.selected is not None)
        self.left_pub.publish(self.path_message(result.left, stamp))
        self.right_pub.publish(self.path_message(result.right, stamp))
        self.selected_pub.publish(self.path_message(result.selected, stamp))
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        markers.markers.append(self.candidate_marker(result.left, 1, stamp, result.selected_side == "left"))
        markers.markers.append(self.candidate_marker(result.right, 2, stamp, result.selected_side == "right"))
        markers.markers.append(self.area_status_marker(result, valid_sides, stamp))
        self.marker_pub.publish(markers)
        self.valid_area_pub.publish(Bool(data=valid_overtake_area))
        payload = {
            "valid": True,
            "valid_overtake_area": valid_overtake_area,
            "valid_overtake_sides": valid_sides,
            "overtake_authorized": overtake_authorized,
            "area_validity_basis": "track_corridor_curvature_and_predicted_collision",
            "target_id": target_id,
            "target_visible": True,
            "target_state_source": "radar_target_array",
            "uses_target_truth": False,
            "relevant": bool(result.relevant),
            "risk": bool(result.risk),
            "reason": result.reason,
            "selected_side": result.selected_side,
            "ego_s": float(result.ego.s),
            "ego_ey": float(result.ego.ey),
            "opponent_s": float(result.opponent.s),
            "opponent_ey": float(result.opponent.ey),
            "ego_vs": float(result.ego_vs),
            "opponent_vs": float(result.opponent_vs),
            "delta_s": float(result.delta_s),
            "opponent_signed_delta": float(
                self.track.signed_delta(result.ego.s, result.opponent.s)
            ),
            "target_center_distance": float(target_range),
            "bumper_gap": float(result.bumper_gap),
            "closing_speed": float(result.closing_speed),
            "ttc": float(result.ttc) if math.isfinite(result.ttc) else None,
            "left": self.candidate_summary(result.left),
            "right": self.candidate_summary(result.right),
            "command_authority": False,
        }
        payload.update(timing)
        self.diagnostic_pub.publish(String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True)))

    @staticmethod
    def candidate_summary(candidate) -> dict:
        return {
            "feasible": bool(candidate.feasible),
            "reason": candidate.reason,
            "target_offset": float(candidate.target_offset) if math.isfinite(candidate.target_offset) else None,
            "minimum_boundary_clearance": float(candidate.minimum_boundary_clearance)
            if math.isfinite(candidate.minimum_boundary_clearance)
            else None,
            "minimum_collision_metric": float(candidate.minimum_collision_metric)
            if math.isfinite(candidate.minimum_collision_metric)
            else None,
            "maximum_curvature": float(candidate.maximum_curvature)
            if math.isfinite(candidate.maximum_curvature)
            else None,
            "score": float(candidate.score) if math.isfinite(candidate.score) else None,
        }

    def publish_invalid(self, reason: str, stamp: rospy.Time, **details) -> None:
        empty = Path()
        empty.header.frame_id = self.world_frame
        empty.header.stamp = stamp
        self.left_pub.publish(empty)
        self.right_pub.publish(empty)
        self.selected_pub.publish(empty)
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[clear]))
        self.valid_area_pub.publish(Bool(data=False))
        payload = {
            "valid": False,
            "valid_overtake_area": False,
            "valid_overtake_sides": [],
            "overtake_authorized": False,
            "reason": reason,
            "target_visible": False,
            "target_state_source": "radar_target_array",
            "uses_target_truth": False,
            "command_authority": False,
        }
        payload.update(details)
        self.diagnostic_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    rospy.init_node("v5_jubu_candidate_planner")
    V5JubuCandidatePlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
