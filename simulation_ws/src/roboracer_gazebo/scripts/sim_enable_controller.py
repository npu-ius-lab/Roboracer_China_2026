#!/usr/bin/env python3
"""Enable an MPCC node only inside the isolated localhost simulation graph."""
import os
import rospy
from std_srvs.srv import SetBool


if __name__ == "__main__":
    rospy.init_node("roboracer_sim_enable_controller")
    master = os.environ.get("ROS_MASTER_URI", "")
    if "127.0.0.1" not in master and "localhost" not in master:
        raise RuntimeError("simulation controller refuses a non-local ROS master: " + master)
    service = rospy.get_param("~service")
    rospy.wait_for_service(service, timeout=60.0)
    client = rospy.ServiceProxy(service, SetBool)
    deadline = rospy.Time.now() + rospy.Duration(60.0)
    response = None
    try:
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            response = client(True)
            if response.success:
                break
            rospy.logwarn_throttle(2.0, "waiting for controller readiness: %s", response.message)
            rospy.sleep(0.25)
    except rospy.ROSInterruptException:
        pass
    if response is None or not response.success:
        raise RuntimeError("controller did not become ready: " + (response.message if response else "no response"))
    rospy.loginfo("simulation controller enabled through %s", service)
    rospy.spin()
