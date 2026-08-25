#!/usr/bin/env python3
"""Synthetic ROI-only PASS -> disappearance clearance -> RETURN scenario."""

import csv
import json
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String


def load_track(path):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    return [{key: float(row[key]) for key in (
        "s_m", "x_m", "y_m", "psi_rad")}
        for row in rows]


def sample(rows, progress):
    progress = max(rows[0]["s_m"], min(progress, rows[-1]["s_m"]))
    upper = 1
    while upper < len(rows) and rows[upper]["s_m"] < progress:
        upper += 1
    upper = min(upper, len(rows) - 1)
    lower = upper - 1
    gap = rows[upper]["s_m"] - rows[lower]["s_m"]
    ratio = 0.0 if gap <= 0.0 else (progress - rows[lower]["s_m"]) / gap
    yaw_delta = math.atan2(
        math.sin(rows[upper]["psi_rad"] - rows[lower]["psi_rad"]),
        math.cos(rows[upper]["psi_rad"] - rows[lower]["psi_rad"]))
    return (
        rows[lower]["x_m"] + ratio * (rows[upper]["x_m"] - rows[lower]["x_m"]),
        rows[lower]["y_m"] + ratio * (rows[upper]["y_m"] - rows[lower]["y_m"]),
        rows[lower]["psi_rad"] + ratio * yaw_delta,
    )


def main():
    rows = load_track(sys.argv[1])
    rospy.init_node("roboracer_react_synthetic_scenario")
    committed_side = {"value": None}

    def status_callback(message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        side = status.get("selected_side")
        if status.get("state") == "PASS" and side in ("left", "right"):
            committed_side["value"] = side

    rospy.Subscriber("/roboracer_react/status", String, status_callback,
                     queue_size=5)
    odom_pub = rospy.Publisher("/localization/odom", Odometry, queue_size=5)
    cloud_pub = rospy.Publisher("/perception/roi_cloud", PointCloud2,
                                queue_size=2)
    rospy.sleep(0.5)
    rate = rospy.Rate(20)
    start = rospy.get_time()
    # Final straight of raceline_smooth; the 8 m planning horizon remains in
    # the configured low-curvature pass zone across the lap seam.
    start_s = 80.0
    while not rospy.is_shutdown():
        elapsed = rospy.get_time() - start
        if elapsed >= 11.0:
            break
        progress = start_s + 0.35 * elapsed
        # Emulate a closed-loop MPCC response: lateral motion begins shortly
        # after the planner first commits to PASS instead of waiting several
        # seconds while the obstacle keeps approaching on the centreline.
        direction = 1.0 if committed_side["value"] == "left" else -1.0
        if elapsed < 1.5 or committed_side["value"] is None:
            ego_ey = 0.0
        elif elapsed < 3.5:
            ego_ey = direction * 0.28 * (elapsed - 1.5) / 2.0
        elif elapsed < 8.0:
            ego_ey = direction * 0.28
        else:
            ego_ey = direction * 0.28 * max(
                0.0, 1.0 - (elapsed - 8.0) / 2.0)
        center_x, center_y, yaw = sample(rows, progress)
        x = center_x - math.sin(yaw) * ego_ey
        y = center_y + math.cos(yaw) * ego_ey
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(0.5 * yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * yaw)
        odom.twist.twist.linear.x = 0.35
        odom_pub.publish(odom)
        if int(elapsed * 20.0) % 4 == 0:
            header = Header(stamp=stamp, frame_id="body_leveled")
            occupied = 1.0 <= elapsed < 6.0
            points = []
            if occupied:
                obstacle_x, obstacle_y, _ = sample(rows, progress + 2.45)
                dx = obstacle_x - x
                dy = obstacle_y - y
                local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
                local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                for ix in range(-3, 4):
                    for iy in range(-3, 4):
                        points.append((local_x + 0.035 * ix,
                                       local_y + 0.035 * iy, 0.12))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        rate.sleep()


if __name__ == "__main__":
    main()
