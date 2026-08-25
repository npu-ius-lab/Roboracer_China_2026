#!/usr/bin/env python3
"""Bag-only MID360 LaserScan adapter for downstream shadow-mode testing.

The real car uses /perception/targets_detailed. Historical bags did not record
that topic, so this adapter derives track-compensated moving clusters from the
recorded /scan and republishes the same TargetArray interface. It is never
started by the hardware launcher.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from opponent_perception.msg import Target, TargetArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_overtake.track import PeriodicTrack


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


@dataclass
class Detection:
    map_x: float
    map_y: float
    local_x: float
    local_y: float
    span: float
    on_track: bool


@dataclass
class Track:
    identifier: int
    map_x: float
    map_y: float
    vx: float
    vy: float
    stamp: rospy.Time
    hits: int
    dynamic_streak: int
    span: float
    on_track: bool


class OfflineScanTargetAdapter:
    def __init__(self) -> None:
        self.track_model = PeriodicTrack.from_csv(rospy.get_param("~raceline_csv"))
        self.target_frame = rospy.get_param("~frames/target", "body_leveled")
        self.x_offset = float(rospy.get_param("~scan/x_offset_m", 0.135))
        self.y_offset = float(rospy.get_param("~scan/y_offset_m", 0.0))
        self.minimum_range = float(rospy.get_param("~scan/minimum_range_m", 0.25))
        self.maximum_range = float(rospy.get_param("~scan/maximum_range_m", 6.0))
        self.maximum_lateral = float(rospy.get_param("~scan/maximum_lateral_m", 1.6))
        self.cluster_gap = float(rospy.get_param("~scan/cluster_gap_m", 0.20))
        self.minimum_cluster_points = int(
            rospy.get_param("~scan/minimum_cluster_points", 2)
        )
        self.maximum_cluster_span = float(
            rospy.get_param("~scan/maximum_cluster_span_m", 0.75)
        )
        self.association_gate = float(
            rospy.get_param("~tracking/association_gate_m", 0.65)
        )
        self.minimum_hits = int(rospy.get_param("~tracking/minimum_hits", 3))
        self.maximum_age = float(rospy.get_param("~tracking/maximum_age_s", 0.60))
        self.minimum_dynamic_speed = float(
            rospy.get_param("~tracking/minimum_dynamic_speed_mps", 0.12)
        )
        self.velocity_alpha = float(rospy.get_param("~tracking/velocity_alpha", 0.45))
        self.corridor_inner_margin = float(
            rospy.get_param("~classification/corridor_inner_margin_m", 0.08)
        )
        self.maximum_projection_distance = float(
            rospy.get_param("~classification/maximum_projection_distance_m", 1.25)
        )

        topics = rospy.get_param("~topics", {})
        self.lock = threading.Lock()
        self.odom = None
        self.next_id = 1
        self.tracks: list[Track] = []
        self.target_pub = rospy.Publisher(
            topics.get("targets", "/perception/targets_detailed"),
            TargetArray,
            queue_size=1,
        )
        self.marker_pub = rospy.Publisher(
            topics.get("markers", "/perception/target_markers"),
            MarkerArray,
            queue_size=1,
        )
        self.odom_sub = rospy.Subscriber(
            topics.get("odom", "/localization/odom"),
            Odometry,
            self.odom_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.scan_sub = rospy.Subscriber(
            topics.get("scan", "/scan"),
            LaserScan,
            self.scan_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.logwarn(
            "Offline scan target adapter ready. BAG TEST ONLY; hardware launcher does not start it."
        )

    def odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.odom = message

    def scan_callback(self, scan: LaserScan) -> None:
        with self.lock:
            odom = self.odom
        if odom is None:
            return
        stamp = scan.header.stamp if scan.header.stamp != rospy.Time() else rospy.Time.now()
        yaw = yaw_from_quaternion(odom.pose.pose.orientation)
        ego_x = float(odom.pose.pose.position.x)
        ego_y = float(odom.pose.pose.position.y)
        ego_projection = self.track_model.project(ego_x, ego_y)
        detections = self.extract_detections(scan, ego_x, ego_y, yaw, ego_projection.s)
        self.update_tracks(detections, stamp)
        self.publish(stamp, ego_x, ego_y, yaw)

    def extract_detections(self, scan, ego_x, ego_y, yaw, ego_s):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        groups = []
        current = []
        previous = None
        for index, raw_range in enumerate(scan.ranges):
            value = float(raw_range)
            if not math.isfinite(value) or value < self.minimum_range or value > self.maximum_range:
                if current:
                    groups.append(current)
                    current = []
                previous = None
                continue
            angle = float(scan.angle_min + index * scan.angle_increment)
            local_x = self.x_offset + value * math.cos(angle)
            local_y = self.y_offset + value * math.sin(angle)
            if local_x <= 0.20 or abs(local_y) > self.maximum_lateral:
                if current:
                    groups.append(current)
                    current = []
                previous = None
                continue
            point = np.array([local_x, local_y], dtype=float)
            if previous is not None and float(np.linalg.norm(point - previous)) > self.cluster_gap:
                if current:
                    groups.append(current)
                current = []
            current.append(point)
            previous = point
        if current:
            groups.append(current)

        detections = []
        for group in groups:
            if len(group) < self.minimum_cluster_points:
                continue
            points = np.asarray(group, dtype=float)
            span = float(np.max(np.linalg.norm(points - points[0], axis=1)))
            if span > self.maximum_cluster_span:
                continue
            center = np.median(points, axis=0)
            map_x = ego_x + cos_yaw * center[0] - sin_yaw * center[1]
            map_y = ego_y + sin_yaw * center[0] + cos_yaw * center[1]
            projection = self.track_model.project_near(
                float(map_x), float(map_y), ego_s, self.maximum_range + 2.0
            )
            if projection.distance > self.maximum_projection_distance:
                continue
            on_track = bool(
                -projection.width_right + self.corridor_inner_margin
                <= projection.ey
                <= projection.width_left - self.corridor_inner_margin
            )
            detections.append(
                Detection(
                    map_x=float(map_x),
                    map_y=float(map_y),
                    local_x=float(center[0]),
                    local_y=float(center[1]),
                    span=span,
                    on_track=on_track,
                )
            )
        return detections

    def update_tracks(self, detections: list[Detection], stamp: rospy.Time) -> None:
        self.tracks = [
            track for track in self.tracks if (stamp - track.stamp).to_sec() <= self.maximum_age
        ]
        unmatched = set(range(len(detections)))
        updated = []
        for track in self.tracks:
            dt = max((stamp - track.stamp).to_sec(), 1.0e-3)
            prediction = np.array(
                [track.map_x + track.vx * dt, track.map_y + track.vy * dt], dtype=float
            )
            choices = []
            for index in unmatched:
                detection = detections[index]
                distance = float(
                    np.linalg.norm(prediction - np.array([detection.map_x, detection.map_y]))
                )
                choices.append((distance, index))
            if not choices:
                updated.append(track)
                continue
            distance, index = min(choices)
            if distance > self.association_gate:
                updated.append(track)
                continue
            unmatched.remove(index)
            detection = detections[index]
            raw_vx = (detection.map_x - track.map_x) / dt
            raw_vy = (detection.map_y - track.map_y) / dt
            raw_speed = math.hypot(raw_vx, raw_vy)
            if raw_speed > 8.0:
                raw_vx, raw_vy = track.vx, track.vy
            vx = (1.0 - self.velocity_alpha) * track.vx + self.velocity_alpha * raw_vx
            vy = (1.0 - self.velocity_alpha) * track.vy + self.velocity_alpha * raw_vy
            speed = math.hypot(vx, vy)
            updated.append(
                Track(
                    identifier=track.identifier,
                    map_x=detection.map_x,
                    map_y=detection.map_y,
                    vx=vx,
                    vy=vy,
                    stamp=stamp,
                    hits=track.hits + 1,
                    dynamic_streak=track.dynamic_streak + 1
                    if speed >= self.minimum_dynamic_speed
                    else 0,
                    span=detection.span,
                    on_track=detection.on_track,
                )
            )
        for index in unmatched:
            detection = detections[index]
            updated.append(
                Track(
                    identifier=self.next_id,
                    map_x=detection.map_x,
                    map_y=detection.map_y,
                    vx=0.0,
                    vy=0.0,
                    stamp=stamp,
                    hits=1,
                    dynamic_streak=0,
                    span=detection.span,
                    on_track=detection.on_track,
                )
            )
            self.next_id += 1
        self.tracks = updated

    def publish(self, stamp, ego_x, ego_y, yaw) -> None:
        message = TargetArray()
        message.header.stamp = stamp
        message.header.frame_id = self.target_frame
        markers = MarkerArray()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        for track in self.tracks:
            confirmed = bool(
                track.hits >= self.minimum_hits
                and track.dynamic_streak >= 2
                and math.hypot(track.vx, track.vy) >= self.minimum_dynamic_speed
            )
            local_x = cos_yaw * (track.map_x - ego_x) + sin_yaw * (track.map_y - ego_y)
            local_y = -sin_yaw * (track.map_x - ego_x) + cos_yaw * (track.map_y - ego_y)
            if local_x <= 0.20 or local_x > self.maximum_range or abs(local_y) > self.maximum_lateral:
                continue
            if confirmed:
                target = Target()
                target.id = track.identifier
                target.pose.position.x = local_x
                target.pose.position.y = local_y
                target.pose.position.z = 0.12
                local_vx = cos_yaw * track.vx + sin_yaw * track.vy
                local_vy = -sin_yaw * track.vx + cos_yaw * track.vy
                target.twist.linear.x = local_vx
                target.twist.linear.y = local_vy
                heading = math.atan2(local_vy, local_vx) if math.hypot(local_vx, local_vy) > 0.1 else 0.0
                target.pose.orientation.z = math.sin(heading * 0.5)
                target.pose.orientation.w = math.cos(heading * 0.5)
                message.targets.append(target)

            marker = Marker()
            marker.header = message.header
            marker.ns = "offline_scan_tracks"
            marker.id = track.identifier
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = local_x
            marker.pose.position.y = local_y
            marker.pose.position.z = 0.12
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.18, min(0.45, track.span))
            marker.scale.y = 0.22
            marker.scale.z = 0.20
            marker.color.a = 0.80 if confirmed else 0.25
            if track.on_track:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.20, 0.05
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.70, 0.20, 1.0
            marker.lifetime = rospy.Duration(0.30)
            markers.markers.append(marker)
        self.target_pub.publish(message)
        self.marker_pub.publish(markers)


def main() -> None:
    rospy.init_node("offline_scan_target_adapter")
    OfflineScanTargetAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
