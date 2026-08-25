#!/usr/bin/env python3
"""Fail-fast acceptance probe for the Gazebo/controller/perception graph."""
import sys
import rospy
from livox_ros_driver.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def wait(topic, msg_type, timeout):
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("missing topic: %s", topic)
        return None


if __name__ == "__main__":
    rospy.init_node("roboracer_simulation_healthcheck")
    timeout = rospy.get_param("~timeout", 30.0)
    ok = wait("/clock", rospy.msg.AnyMsg, timeout) is not None
    lidar = wait("/livox/lidar", CustomMsg, timeout)
    cloud = wait("/cloud_registered_body", PointCloud2, timeout)
    ok &= lidar is not None and lidar.point_num > 0
    ok &= cloud is not None and cloud.width > 0
    ok &= wait("/livox/imu", rospy.msg.AnyMsg, timeout) is not None
    ok &= wait("/aft_mapped_to_init", Odometry, timeout) is not None
    odom = wait("/localization/vehicle_odom", Odometry, timeout)
    ok &= odom is not None
    if lidar is not None and cloud is not None:
        rospy.loginfo("MID360 frame: livox=%d points, body_cloud=%d points",
                      lidar.point_num, cloud.width)
    rospy.loginfo("RoboRacer Gazebo healthcheck: %s", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
