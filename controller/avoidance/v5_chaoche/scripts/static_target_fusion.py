#!/usr/bin/env python3
"""Fuse dynamic opponents and compact static perception candidates.

The existing C++ MID360 perception node already performs ground removal,
Euclidean clustering and vehicle-size filtering.  It publishes clusters that
have not demonstrated world motion as ``candidates`` markers.  This V5-only
adapter confirms those candidates, rejects track-boundary fragments, and
publishes them as zero-velocity targets alongside dynamic opponents.

This node has no command publisher.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import json
import math
import threading

import rospy
from nav_msgs.msg import Odometry
from opponent_perception.msg import Target, TargetArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_overtake.track import PeriodicTrack


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


@dataclass
class CandidateHistory:
    observations: int
    last_stamp: float


class StaticTargetFusion:
    def __init__(self) -> None:
        self.track = PeriodicTrack.from_csv(rospy.get_param("~raceline_csv"))
        self.static_id_base = int(rospy.get_param("~static_id_base", 1000000000))
        self.minimum_observations = int(rospy.get_param("~minimum_observations", 3))
        self.maximum_observation_gap = float(
            rospy.get_param("~maximum_observation_gap_s", 0.80)
        )
        self.input_timeout = float(rospy.get_param("~input_timeout_s", 0.80))
        self.odom_timeout = float(rospy.get_param("~odom_timeout_s", 0.30))
        self.minimum_x = float(rospy.get_param("~filter/minimum_x_m", 0.35))
        self.maximum_x = float(rospy.get_param("~filter/maximum_x_m", 5.50))
        self.maximum_abs_y = float(rospy.get_param("~filter/maximum_abs_y_m", 1.20))
        self.maximum_length = float(rospy.get_param("~filter/maximum_length_m", 0.55))
        self.maximum_width = float(rospy.get_param("~filter/maximum_width_m", 0.45))
        self.minimum_extent = float(rospy.get_param("~filter/minimum_extent_m", 0.06))
        self.track_margin = float(rospy.get_param("~filter/track_margin_m", 0.03))
        self.dynamic_merge_distance = float(
            rospy.get_param("~filter/dynamic_merge_distance_m", 0.45)
        )
        self.start_gate_ignore_enabled = bool(
            rospy.get_param("~start_gate_ignore/enabled", True)
        )
        self.start_gate_s = float(
            rospy.get_param("~start_gate_ignore/center_s_m", 87.55)
        )
        self.start_gate_half_s = float(
            rospy.get_param("~start_gate_ignore/half_length_s_m", 0.70)
        )
        self.start_gate_ey = float(
            rospy.get_param("~start_gate_ignore/center_ey_m", 0.46)
        )
        self.start_gate_half_ey = float(
            rospy.get_param("~start_gate_ignore/half_width_ey_m", 0.18)
        )

        self.lock = threading.Lock()
        self.odom = None
        self.odom_received = None
        self.dynamic_targets = TargetArray()
        self.dynamic_received = None
        self.marker_received = None
        self.histories = {}
        self.last_static_count = 0
        self.last_rejected_boundary = 0
        self.last_ignored_start_gate = 0

        topics = rospy.get_param("~topics", {})
        self.output_pub = rospy.Publisher(
            topics.get("output", "/v5_chaoche/perception/targets_detailed"),
            TargetArray,
            queue_size=1,
        )
        self.static_marker_pub = rospy.Publisher(
            topics.get(
                "static_markers", "/v5_chaoche/perception/static_obstacle_markers"
            ),
            MarkerArray,
            queue_size=1,
        )
        self.status_pub = rospy.Publisher(
            topics.get("status", "/v5_chaoche/perception/fusion_status"),
            String,
            queue_size=10,
            latch=True,
        )
        rospy.Subscriber(
            topics.get("odom", "/localization/odom"),
            Odometry,
            self.odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            topics.get("dynamic_targets", "/v5_chaoche/perception/targets_dynamic"),
            TargetArray,
            self.dynamic_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            topics.get("perception_markers", "/v5_chaoche/perception/target_markers"),
            MarkerArray,
            self.markers_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.status_timer = rospy.Timer(rospy.Duration(0.10), self.status_callback)
        rospy.logwarn(
            "V5 Chaoche compact-static fusion ready; command_authority=false"
        )

    def odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.odom = copy.deepcopy(message)
            self.odom_received = rospy.Time.now()

    def dynamic_callback(self, message: TargetArray) -> None:
        with self.lock:
            self.dynamic_targets = copy.deepcopy(message)
            self.dynamic_received = rospy.Time.now()

    def marker_is_compact_candidate(self, marker: Marker) -> bool:
        return bool(
            marker.action == Marker.ADD
            and marker.ns == "candidates"
            and self.minimum_x <= float(marker.pose.position.x) <= self.maximum_x
            and abs(float(marker.pose.position.y)) <= self.maximum_abs_y
            and self.minimum_extent <= float(marker.scale.x) <= self.maximum_length
            and self.minimum_extent <= float(marker.scale.y) <= self.maximum_width
        )

    def candidate_inside_track(self, marker: Marker, odom: Odometry) -> bool:
        projection, ego_projection = self.project_local_pose(marker.pose, odom)
        # A conservative lateral radius prevents wall segments from becoming
        # fake obstacles. Objects outside the drivable corridor need no path
        # deviation and are intentionally not fused.
        radius = 0.5 * math.hypot(float(marker.scale.x), float(marker.scale.y))
        lower = -projection.width_right + radius + self.track_margin
        upper = projection.width_left - radius - self.track_margin
        delta_s = self.track.forward_delta(ego_projection.s, projection.s)
        return bool(
            projection.distance <= 1.0
            and 0.0 <= delta_s <= self.maximum_x + 1.0
            and lower <= projection.ey <= upper
        )

    def project_local_pose(self, pose, odom: Odometry):
        yaw = yaw_from_quaternion(odom.pose.pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = float(pose.position.x)
        local_y = float(pose.position.y)
        map_x = float(odom.pose.pose.position.x) + cos_yaw * local_x - sin_yaw * local_y
        map_y = float(odom.pose.pose.position.y) + sin_yaw * local_x + cos_yaw * local_y
        ego_projection = self.track.project(
            float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
        )
        projection = self.track.project_near(map_x, map_y, ego_projection.s, 7.0)
        return projection, ego_projection

    def pose_is_start_gate(self, pose, odom: Odometry) -> bool:
        if not self.start_gate_ignore_enabled:
            return False
        projection, _ = self.project_local_pose(pose, odom)
        return bool(
            abs(self.track.signed_delta(self.start_gate_s, projection.s))
            <= self.start_gate_half_s
            and abs(projection.ey - self.start_gate_ey) <= self.start_gate_half_ey
        )

    def update_history(self, marker_id: int, stamp: float) -> int:
        previous = self.histories.get(marker_id)
        if previous is None or stamp - previous.last_stamp > self.maximum_observation_gap:
            history = CandidateHistory(1, stamp)
        elif stamp + 1.0e-6 < previous.last_stamp:
            history = CandidateHistory(1, stamp)
        else:
            history = CandidateHistory(previous.observations + 1, stamp)
        self.histories[marker_id] = history
        return history.observations

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
            compact = [m for m in message.markers if self.marker_is_compact_candidate(m)]
            visible_ids = {int(marker.id) for marker in compact}
            for marker_id in list(self.histories):
                history = self.histories[marker_id]
                if marker_id not in visible_ids and stamp - history.last_stamp > self.maximum_observation_gap:
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
                        continue
                    fused.targets.append(copy.deepcopy(target))

            dynamic_xy = [
                (float(target.pose.position.x), float(target.pose.position.y))
                for target in fused.targets
            ]
            static_markers = MarkerArray()
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            static_markers.markers.append(delete_all)
            static_count = 0
            rejected_boundary = 0
            if odom_fresh:
                for marker in compact:
                    observations = self.update_history(int(marker.id), stamp)
                    if observations < self.minimum_observations:
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

                    target = Target()
                    target.id = self.static_id_base + int(marker.id)
                    target.pose = copy.deepcopy(marker.pose)
                    # A zero target twist means stationary in the world, not
                    # stationary relative to the moving ego frame.
                    fused.targets.append(target)
                    static_count += 1

                    display = copy.deepcopy(marker)
                    display.header.stamp = measurement_stamp
                    display.ns = "v5_chaoche_static_obstacles"
                    display.id = target.id
                    display.color.r = 1.0
                    display.color.g = 0.05
                    display.color.b = 0.05
                    display.color.a = 0.85
                    display.lifetime = rospy.Duration(0.9)
                    static_markers.markers.append(display)

            # Both products are emitted from the same perception cloud: retain
            # its measurement stamp so the planner can compensate ego motion.
            fused.header.stamp = measurement_stamp
            if not fused.header.frame_id:
                fused.header.frame_id = "body_leveled"
            self.last_static_count = static_count
            self.last_rejected_boundary = rejected_boundary
            self.last_ignored_start_gate = ignored_start_gate

        self.output_pub.publish(fused)
        self.static_marker_pub.publish(static_markers)

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
                "rejected_boundary_count": self.last_rejected_boundary,
                "ignored_start_gate_count": self.last_ignored_start_gate,
                "command_authority": False,
            }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    rospy.init_node("v5_chaoche_static_target_fusion")
    StaticTargetFusion()
    rospy.spin()


if __name__ == "__main__":
    main()
