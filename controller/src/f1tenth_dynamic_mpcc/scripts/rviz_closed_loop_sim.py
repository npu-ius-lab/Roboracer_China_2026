#!/usr/bin/env python3
"""ROS plant for visualising the Dynamic MPCC closed loop in RViz."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class ClosedLoopPlant:
    def __init__(self) -> None:
        package = Path(__file__).resolve().parents[1]
        controller_path = Path(rospy.get_param(
            "~controller_config", str(package / "config/controller.yaml")
        )).expanduser()
        vehicle_path = Path(rospy.get_param(
            "~vehicle_config", str(package / "config/vehicle.yaml")
        )).expanduser()
        track_path = Path(rospy.get_param(
            "~track_csv", str(package / "data/tracks/virtual_track/raceline.csv")
        )).expanduser()

        controller = load_yaml(controller_path)
        self.vehicle = VehicleParameters.from_yaml(vehicle_path, controller)
        speed_planning = dict(controller.get("speed_planning", {}))
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

        x0, y0 = self.track.position(0.0)
        yaw0 = float(self.track.tangent(0.0))
        self.state = np.asarray([x0, y0, yaw0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.speed_command = 0.0
        self.steering_command = 0.0
        self.last_command_time = rospy.Time(0)
        self.last_update = rospy.Time.now()
        self.trail: list[Point] = []
        self.last_trail_s = -1.0

        telemetry_topic = rospy.get_param(
            "~telemetry_topic", "/f1tenth_mpcc/telemetry"
        )
        rospy.Subscriber(
            telemetry_topic, Float32MultiArray, self.command_callback,
            queue_size=1, tcp_nodelay=True,
        )
        self.vehicle_odom_pub = rospy.Publisher(
            "/localization/vehicle_odom", Odometry, queue_size=1
        )
        self.wheel_odom_pub = rospy.Publisher(
            "/tianracer/odom", Odometry, queue_size=1
        )
        self.imu_pub = rospy.Publisher("/livox/imu", Imu, queue_size=5)
        self.marker_pub = rospy.Publisher(
            "/f1tenth_mpcc/markers", MarkerArray, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.005), self.tick)
        rospy.loginfo(
            "Dynamic bicycle RViz plant ready: telemetry=%s, update=200 Hz",
            telemetry_topic,
        )

    def command_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) < 8:
            return
        self.speed_command = float(np.clip(
            message.data[6], 0.0, self.vehicle.max_speed
        ))
        self.steering_command = float(np.clip(
            message.data[7],
            -self.vehicle.max_steer,
            self.vehicle.max_steer,
        ))
        self.last_command_time = rospy.Time.now()

    def tick(self, event: rospy.TimerEvent) -> None:
        now = event.current_real
        dt = float(np.clip((now - self.last_update).to_sec(), 0.001, 0.02))
        self.last_update = now
        if (now - self.last_command_time).to_sec() > 0.30:
            self.speed_command = 0.0

        steering_rate = float(np.clip(
            (self.steering_command - self.state[7]) / dt,
            -self.vehicle.max_steer_rate,
            self.vehicle.max_steer_rate,
        ))
        projected = self.track.project(self.state[0], self.state[1], self.state[8])
        virtual_speed = max(self.speed_command, 0.20)
        control = np.asarray([self.speed_command, steering_rate, virtual_speed])
        self.state = self.model.step(self.state, control, dt)
        projected = self.track.project(self.state[0], self.state[1], projected.s)
        self.state[8] = projected.s

        # Sensor output is 50 Hz; the plant itself remains at 200 Hz.
        if (
            event.last_real is not None
            and int(now.to_nsec() // 20_000_000)
            == int(event.last_real.to_nsec() // 20_000_000)
        ):
            return
        self.publish_sensors(now)
        self.publish_markers(now)

    def odometry(self, stamp: rospy.Time) -> Odometry:
        x, y, yaw, vx, vy, yaw_rate = self.state[:6]
        qx, qy, qz, qw = quaternion_from_yaw(float(yaw))
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "localization_base_link"
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = float(vx)
        message.twist.twist.linear.y = float(vy)
        message.twist.twist.angular.z = float(yaw_rate)
        return message

    def publish_sensors(self, stamp: rospy.Time) -> None:
        odom = self.odometry(stamp)
        self.vehicle_odom_pub.publish(odom)
        self.wheel_odom_pub.publish(odom)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "localization_base_link"
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity.z = odom.twist.twist.angular.z
        self.imu_pub.publish(imu)

    @staticmethod
    def base_marker(stamp: rospy.Time, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "map"
        marker.ns = "f1tenth_dynamic_mpcc_sim"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_markers(self, stamp: rospy.Time) -> None:
        x, y, yaw = (float(value) for value in self.state[:3])
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        body = self.base_marker(stamp, 0, Marker.CUBE)
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.08
        body.pose.orientation.x = qx
        body.pose.orientation.y = qy
        body.pose.orientation.z = qz
        body.pose.orientation.w = qw
        body.scale.x = 0.40
        body.scale.y = self.vehicle.body_width
        body.scale.z = 0.12
        body.color.r = 1.0
        body.color.g = 0.55
        body.color.b = 0.05
        body.color.a = 1.0

        nose = self.base_marker(stamp, 1, Marker.ARROW)
        nose.pose.position.x = x
        nose.pose.position.y = y
        nose.pose.position.z = 0.18
        nose.pose.orientation = body.pose.orientation
        nose.scale.x = 0.55
        nose.scale.y = 0.08
        nose.scale.z = 0.08
        nose.color.r = 1.0
        nose.color.g = 0.95
        nose.color.b = 0.15
        nose.color.a = 1.0

        if self.state[8] - self.last_trail_s >= 0.04 or not self.trail:
            point = Point(x=x, y=y, z=0.04)
            self.trail.append(point)
            self.trail = self.trail[-2500:]
            self.last_trail_s = float(self.state[8])
        trail = self.base_marker(stamp, 2, Marker.LINE_STRIP)
        trail.scale.x = 0.035
        trail.color.r = 1.0
        trail.color.g = 0.75
        trail.color.b = 0.10
        trail.color.a = 0.9
        trail.points = self.trail
        self.marker_pub.publish(MarkerArray(markers=[body, nose, trail]))


def main() -> None:
    rospy.init_node("f1tenth_mpcc_closed_loop_plant")
    ClosedLoopPlant()
    rospy.spin()


if __name__ == "__main__":
    main()
