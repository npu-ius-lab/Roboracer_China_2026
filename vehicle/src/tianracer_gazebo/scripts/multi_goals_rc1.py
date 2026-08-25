#! /usr/bin/env python3
# @Source: https://www.guyuehome.com/35146
# @Time: 2023/10/20 17:02:46
# @Author: Jeff Wang(Lr_2002)
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42

"""
the brief:     the timeout is controlled by the time out parameter of the wait_for_result of SimpleActionClient
tuning method: adjust the timeout period of the wait_for_result to ensure that the path length between the target points is similar
"""

import os
import rospy, rospkg
import actionlib
import waypoint_race.utils as utils

import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")

class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        """
        init RaceStateMachine
        filename: path to your yaml file
        reapeat: determine whether to visit waypoints repeatly(usually True in 110)
        """
        self._waypoints = utils.get_waypoints(filename) # gets the value of a series of target points

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # simple customer action
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat


        # the following is to display the target point
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def move_to_next(self):
        pos = self._get_next_destination()

        if not pos:
            rospy.loginfo("Finishing Race")
            return True
        # convert the target point information read from the file into the format of the goal of move base
        goal = utils.create_move_base_goal(pos)
        rospy.loginfo("Move to %s" % pos['name'])

        # send the goal to the move base
        self._ac_move_base.send_goal(goal)
        self._ac_move_base.wait_for_result(rospy.Duration(8.5))        # wait for the result of the action server， control the timeout time
        result = self._ac_move_base.get_result()
        rospy.loginfo("Result : %s" % result)

        return False

    def _get_next_destination(self):
        """
        determine what's the next goal point according to repeat 
        """
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destination

    def spin(self):
        rospy.sleep(1.0)
        self._pub_viz_marker.publish(self._viz_markers)
        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = self.move_to_next()
            rospy.sleep(0)

if __name__ == '__main__':
    rospy.init_node('race')
    
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, f"scripts/waypoint_race/{world}_points.yaml")
        print(f"yaml: {filename}")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename",filename)
    repeat = rospy.get_param('~repeat', True)

    m = RaceStateMachine(filename, repeat)
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Finished')