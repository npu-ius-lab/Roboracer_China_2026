#!/usr/bin/env python3
"""Closed-loop bicycle plant that moves the parameter-matched Gazebo model."""
import math
import threading

import rospy
import tf2_ros
from ackermann_msgs.msg import AckermannDriveStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class VehicleBridge:
    def __init__(self):
        self.model = rospy.get_param("~model_name", "roboracer")
        self.wheelbase = rospy.get_param("~wheelbase", 0.320)
        self.speed_tau = rospy.get_param("~speed_time_constant", 0.20)
        self.steer_tau = rospy.get_param("~steering_time_constant", 0.062671845)
        self.max_accel = rospy.get_param("~max_accel", 2.4)
        self.max_decel = rospy.get_param("~max_decel", 4.0)
        self.max_steer = rospy.get_param("~max_steer", 0.55)
        self.speed_cap = rospy.get_param("~speed_cap", 4.0)
        self.timeout = rospy.get_param("~command_timeout", 0.5)
        self.rate_hz = rospy.get_param("~publish_rate", 100.0)
        self.x = rospy.get_param("~initial_x", 0.106211745)
        self.y = rospy.get_param("~initial_y", 0.171355458)
        self.yaw = rospy.get_param("~initial_yaw", 0.024190847)
        self.speed = 0.0
        self.steer = 0.0
        self.command_speed = 0.0
        self.command_steer = 0.0
        self.last_command = rospy.Time(0)
        self.lock = threading.Lock()
        command_topic = rospy.get_param("~command_topic")
        rospy.Subscriber(command_topic, AckermannDriveStamped, self.command, queue_size=1)
        self.odom_pubs = [rospy.Publisher(t, Odometry, queue_size=5) for t in
                          ("/localization/vehicle_odom", "/localization/odom",
                           "/localization/ground_truth_odom", "/tianracer/odom")]
        self.tf = tf2_ros.TransformBroadcaster()
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def command(self, msg):
        with self.lock:
            self.command_speed = clamp(msg.drive.speed, -self.speed_cap, self.speed_cap)
            self.command_steer = clamp(msg.drive.steering_angle, -self.max_steer, self.max_steer)
            self.last_command = rospy.Time.now()

    def step(self, dt):
        with self.lock:
            target_speed = self.command_speed if (rospy.Time.now() - self.last_command).to_sec() <= self.timeout else 0.0
            target_steer = self.command_steer if target_speed != 0.0 else 0.0
        desired_accel = (target_speed - self.speed) / max(self.speed_tau, 1e-3)
        accel = clamp(desired_accel, -self.max_decel, self.max_accel)
        self.speed += accel * dt
        steer_rate = (target_steer - self.steer) / max(self.steer_tau, 1e-3)
        self.steer = clamp(self.steer + steer_rate * dt, -self.max_steer, self.max_steer)
        yaw_rate = self.speed * math.tan(self.steer) / self.wheelbase
        self.yaw += yaw_rate * dt
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        return yaw_rate

    def publish(self, yaw_rate):
        now = rospy.Time.now()
        qz, qw = math.sin(self.yaw * 0.5), math.cos(self.yaw * 0.5)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, "map", "localization_base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.speed, yaw_rate
        for pub in self.odom_pubs:
            pub.publish(odom)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = now, "map", "localization_base_link"
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z, transform.transform.rotation.w = qz, qw
        self.tf.sendTransform(transform)
        state = ModelState(model_name=self.model, reference_frame="world")
        state.pose.position.x, state.pose.position.y, state.pose.position.z = self.x, self.y, 0.060
        state.pose.orientation.z, state.pose.orientation.w = qz, qw
        state.twist.linear.x = self.speed * math.cos(self.yaw)
        state.twist.linear.y = self.speed * math.sin(self.yaw)
        state.twist.angular.z = yaw_rate
        try:
            self.set_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "set_model_state failed: %s", exc)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        previous = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = clamp((now - previous).to_sec(), 0.0, 0.05)
            previous = now
            self.publish(self.step(dt))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("roboracer_gazebo_vehicle_bridge")
    try:
        VehicleBridge().run()
    except rospy.ROSInterruptException:
        pass
