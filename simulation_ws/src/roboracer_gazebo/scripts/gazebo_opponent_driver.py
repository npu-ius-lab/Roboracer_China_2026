#!/usr/bin/env python3
"""Drive the H2H opponent around the measured raceline in Gazebo."""
import bisect
import csv
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class OpponentDriver:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "opponent")
        self.speed = float(rospy.get_param("~speed_mps", 1.35))
        self.start_s = float(rospy.get_param("~start_s_m", 8.0))
        csv_path = rospy.get_param("~raceline_csv")
        with open(csv_path, newline="") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) < 3:
            raise RuntimeError("opponent raceline is too short")
        self.s = [float(row["s_m"]) for row in rows]
        self.x = [float(row["x_m"]) for row in rows]
        self.y = [float(row["y_m"]) for row in rows]
        self.yaw = [float(row["psi_rad"]) for row in rows]
        self.length = self.s[-1]
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.started = None
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)

    def sample(self, progress):
        progress %= self.length
        upper = bisect.bisect_right(self.s, progress)
        if upper == 0:
            return self.x[0], self.y[0], self.yaw[0]
        if upper >= len(self.s):
            upper = len(self.s) - 1
        lower = upper - 1
        span = max(1e-9, self.s[upper] - self.s[lower])
        ratio = (progress - self.s[lower]) / span
        dyaw = math.atan2(math.sin(self.yaw[upper] - self.yaw[lower]),
                          math.cos(self.yaw[upper] - self.yaw[lower]))
        return (self.x[lower] + ratio * (self.x[upper] - self.x[lower]),
                self.y[lower] + ratio * (self.y[upper] - self.y[lower]),
                self.yaw[lower] + ratio * dyaw)

    def update(self, event):
        if self.started is None or event.current_real < self.started:
            self.started = event.current_real
        elapsed = max(0.0, (event.current_real - self.started).to_sec())
        x, y, yaw = self.sample(self.start_s + self.speed * elapsed)
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.15
        state.pose.orientation.z = math.sin(0.5 * yaw)
        state.pose.orientation.w = math.cos(0.5 * yaw)
        state.twist.linear.x = self.speed * math.cos(yaw)
        state.twist.linear.y = self.speed * math.sin(yaw)
        try:
            self.set_state(state)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "opponent update failed: %s", error)


if __name__ == "__main__":
    rospy.init_node("gazebo_opponent_driver")
    OpponentDriver()
    rospy.spin()
