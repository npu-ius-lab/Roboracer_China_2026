#!/usr/bin/env python3
"""V3 Pro Max target fusion with world-velocity estimation for slow objects."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import importlib.util
import json
import math
from pathlib import Path
import sys

import rospy
from opponent_perception.msg import Target, TargetArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


BASE_PATH = (
    Path(__file__).resolve().parents[2]
    / "v5_chaoche"
    / "scripts"
    / "static_target_fusion.py"
)
SPEC = importlib.util.spec_from_file_location("v5_static_target_fusion", BASE_PATH)
if SPEC is None or SPEC.loader is None:
    raise ImportError(f"cannot load base static fusion: {BASE_PATH}")
BASE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BASE
SPEC.loader.exec_module(BASE)


@dataclass
class MotionHistory:
    observations: int
    last_stamp: float
    map_x: float
    map_y: float
    vx: float = 0.0
    vy: float = 0.0
    velocity_valid: bool = False


class RoboRacerTargetFusion(BASE.StaticTargetFusion):
    def __init__(self) -> None:
        super().__init__()
        self.slow_id_base = int(rospy.get_param("~slow_target_id_base", 500000000))
        self.minimum_static_observations = int(
            rospy.get_param("~motion/minimum_static_observations", 6)
        )
        self.maximum_static_speed = float(
            rospy.get_param("~motion/maximum_static_speed_mps", 0.08)
        )
        self.maximum_candidate_speed = float(
            rospy.get_param("~motion/maximum_candidate_speed_mps", 3.0)
        )
        self.velocity_alpha = float(rospy.get_param("~motion/velocity_alpha", 0.30))
        self.last_slow_count = 0
        rospy.logwarn(
            "V3 Pro Max fusion estimates candidate world velocity before static classification"
        )

    @staticmethod
    def map_xy(pose, odom):
        yaw = BASE.yaw_from_quaternion(odom.pose.pose.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        local_x = float(pose.position.x)
        local_y = float(pose.position.y)
        return (
            float(odom.pose.pose.position.x) + c * local_x - s * local_y,
            float(odom.pose.pose.position.y) + s * local_x + c * local_y,
        )

    def update_motion(self, marker, odom, stamp: float) -> MotionHistory:
        marker_id = int(marker.id)
        map_x, map_y = self.map_xy(marker.pose, odom)
        previous = self.histories.get(marker_id)
        if (
            previous is None
            or not isinstance(previous, MotionHistory)
            or stamp < previous.last_stamp
            or stamp - previous.last_stamp > self.maximum_observation_gap
        ):
            history = MotionHistory(1, stamp, map_x, map_y)
        else:
            dt = stamp - previous.last_stamp
            vx = previous.vx
            vy = previous.vy
            valid = previous.velocity_valid
            if dt >= 0.05:
                raw_vx = (map_x - previous.map_x) / dt
                raw_vy = (map_y - previous.map_y) / dt
                if math.hypot(raw_vx, raw_vy) <= self.maximum_candidate_speed:
                    if valid:
                        a = self.velocity_alpha
                        vx = (1.0 - a) * vx + a * raw_vx
                        vy = (1.0 - a) * vy + a * raw_vy
                    else:
                        vx, vy = raw_vx, raw_vy
                    valid = True
            history = MotionHistory(
                previous.observations + 1,
                stamp,
                map_x,
                map_y,
                vx,
                vy,
                valid,
            )
        self.histories[marker_id] = history
        return history

    @staticmethod
    def body_velocity(history: MotionHistory, odom):
        yaw = BASE.yaw_from_quaternion(odom.pose.pose.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (
            c * history.vx + s * history.vy,
            -s * history.vx + c * history.vy,
        )

    def markers_callback(self, message: MarkerArray) -> None:
        now = rospy.Time.now()
        with self.lock:
            self.marker_received = now
            odom = copy.deepcopy(self.odom)
            odom_received = self.odom_received
            dynamic = copy.deepcopy(self.dynamic_targets)
            dynamic_received = self.dynamic_received

            measurement_stamp = dynamic.header.stamp
            if measurement_stamp == rospy.Time():
                measurement_stamp = now
            stamp = measurement_stamp.to_sec()
            compact = [
                marker
                for marker in message.markers
                if self.marker_is_compact_candidate(marker)
            ]
            visible_ids = {int(marker.id) for marker in compact}
            for marker_id in list(self.histories):
                history = self.histories[marker_id]
                if (
                    marker_id not in visible_ids
                    and stamp - history.last_stamp > self.maximum_observation_gap
                ):
                    del self.histories[marker_id]

            odom_fresh = bool(
                odom is not None
                and odom_received is not None
                and (now - odom_received).to_sec() <= self.odom_timeout
            )
            dynamic_fresh = bool(
                dynamic_received is not None
                and (now - dynamic_received).to_sec() <= self.input_timeout
            )

            fused = TargetArray()
            fused.header = copy.deepcopy(dynamic.header)
            fused.header.stamp = measurement_stamp
            fused.header.frame_id = dynamic.header.frame_id or "body_leveled"
            ignored_start_gate = 0
            if dynamic_fresh:
                for target in dynamic.targets:
                    if odom_fresh and self.pose_is_start_gate(target.pose, odom):
                        ignored_start_gate += 1
                    else:
                        fused.targets.append(copy.deepcopy(target))

            dynamic_xy = [
                (float(target.pose.position.x), float(target.pose.position.y))
                for target in fused.targets
            ]
            display_markers = MarkerArray()
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            display_markers.markers.append(delete_all)
            static_count = 0
            slow_count = 0
            rejected_boundary = 0

            if odom_fresh:
                for marker in compact:
                    history = self.update_motion(marker, odom, stamp)
                    if history.observations < self.minimum_observations:
                        continue
                    if self.pose_is_start_gate(marker.pose, odom):
                        ignored_start_gate += 1
                        continue
                    if not self.candidate_inside_track(marker, odom):
                        rejected_boundary += 1
                        continue
                    x = float(marker.pose.position.x)
                    y = float(marker.pose.position.y)
                    if any(
                        math.hypot(x - dx, y - dy) <= self.dynamic_merge_distance
                        for dx, dy in dynamic_xy
                    ):
                        continue
                    if not history.velocity_valid:
                        continue

                    speed = math.hypot(history.vx, history.vy)
                    confirmed_static = bool(
                        history.observations >= self.minimum_static_observations
                        and speed <= self.maximum_static_speed
                    )
                    target = Target()
                    target.pose = copy.deepcopy(marker.pose)
                    if confirmed_static:
                        target.id = self.static_id_base + int(marker.id)
                        static_count += 1
                    else:
                        target.id = self.slow_id_base + int(marker.id)
                        target.twist.linear.x, target.twist.linear.y = self.body_velocity(
                            history, odom
                        )
                        slow_count += 1
                    fused.targets.append(target)

                    display = copy.deepcopy(marker)
                    display.header.stamp = measurement_stamp
                    display.ns = (
                        "roboracer_static_obstacles"
                        if confirmed_static
                        else "roboracer_slow_candidates"
                    )
                    display.id = target.id
                    display.color.r = 1.0
                    display.color.g = 0.05 if confirmed_static else 0.55
                    display.color.b = 0.05
                    display.color.a = 0.85
                    display.lifetime = rospy.Duration(0.9)
                    display_markers.markers.append(display)

            self.last_static_count = static_count
            self.last_slow_count = slow_count
            self.last_rejected_boundary = rejected_boundary
            self.last_ignored_start_gate = ignored_start_gate

        self.output_pub.publish(fused)
        self.static_marker_pub.publish(display_markers)

    def status_callback(self, _event) -> None:
        now = rospy.Time.now()
        with self.lock:
            odom_age = (
                (now - self.odom_received).to_sec()
                if self.odom_received is not None
                else math.inf
            )
            dynamic_age = (
                (now - self.dynamic_received).to_sec()
                if self.dynamic_received is not None
                else math.inf
            )
            marker_age = (
                (now - self.marker_received).to_sec()
                if self.marker_received is not None
                else math.inf
            )
            healthy = bool(
                odom_age <= self.odom_timeout
                and dynamic_age <= self.input_timeout
                and marker_age <= self.input_timeout
            )
            if odom_age > self.odom_timeout:
                reason = "odom_missing_or_stale"
            elif dynamic_age > self.input_timeout:
                reason = "dynamic_target_stream_missing_or_stale"
            elif marker_age > self.input_timeout:
                reason = "candidate_marker_stream_missing_or_stale"
            else:
                reason = "healthy"
            payload = {
                "healthy": healthy,
                "reason": reason,
                "odom_age_s": odom_age if math.isfinite(odom_age) else None,
                "dynamic_age_s": dynamic_age if math.isfinite(dynamic_age) else None,
                "marker_age_s": marker_age if math.isfinite(marker_age) else None,
                "static_count": self.last_static_count,
                "slow_candidate_count": self.last_slow_count,
                "rejected_boundary_count": self.last_rejected_boundary,
                "ignored_start_gate_count": self.last_ignored_start_gate,
                "command_authority": False,
            }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    rospy.init_node("roboracer_chaoche_static_target_fusion")
    RoboRacerTargetFusion()
    rospy.spin()


if __name__ == "__main__":
    main()
