#!/usr/bin/env python3
"""Publish Gazebo vehicle motion in the on-car Livox IMU format."""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class LivoxImuAdapter:
    def __init__(self):
        self.imu_pub = rospy.Publisher("/livox/imu", Imu, queue_size=200)
        self.last_odom = None
        self.last_speed = 0.0
        self.last_stamp = None
        self.accel_x = 0.0
        rospy.Subscriber("/localization/ground_truth_odom", Odometry,
                         self.odom_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.005), self.imu_callback)

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        speed = msg.twist.twist.linear.x
        if self.last_stamp is not None:
            dt = stamp.to_sec() - self.last_stamp.to_sec()
            if 0.0 < dt < 0.25:
                raw = (speed - self.last_speed) / dt
                self.accel_x = max(-40.0, min(40.0, raw))
        self.last_odom = msg
        self.last_speed = speed
        self.last_stamp = stamp

    def imu_callback(self, _event):
        if self.last_odom is None:
            return
        speed = self.last_odom.twist.twist.linear.x
        yaw_rate = self.last_odom.twist.twist.angular.z
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "livox_imu"
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.z = yaw_rate
        # The on-car Livox driver/config represents acceleration in g.
        msg.linear_acceleration.x = self.accel_x / 9.81
        msg.linear_acceleration.y = speed * yaw_rate / 9.81
        msg.linear_acceleration.z = 1.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        self.imu_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("mid360_imu_adapter")
    LivoxImuAdapter()
    rospy.spin()
