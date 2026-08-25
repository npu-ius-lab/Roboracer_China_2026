#!/usr/bin/env python3
"""Small test-only observer for isolated synthetic and rosbag replays."""

import collections
import json
import statistics

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


class Observer:
    def __init__(self):
        self.states = collections.Counter()
        self.transitions = []
        self.current_state = None
        self.caps_by_state = collections.defaultdict(list)
        self.mpcc_speeds_by_state = collections.defaultdict(list)
        self.nonempty_paths = 0
        self.empty_paths = 0
        self.status_messages = 0
        self.target_id_used = set()
        self.motion_classification = set()
        self.reasons = collections.Counter()
        self.maximum_occupancy = 0
        rospy.Subscriber("/roboracer_react_fast/state", String,
                         self.state_callback, queue_size=50)
        rospy.Subscriber("/roboracer_react_fast/local_speed_cap", Float32,
                         self.cap_callback, queue_size=50)
        rospy.Subscriber("/roboracer_react_fast/mpcc/ackermann_cmd_stamped",
                         AckermannDriveStamped, self.mpcc_callback,
                         queue_size=100)
        rospy.Subscriber("/roboracer_react_fast/local_plan_red", Path,
                         self.path_callback, queue_size=20)
        rospy.Subscriber("/roboracer_react_fast/status", String,
                         self.status_callback, queue_size=50)
        rospy.on_shutdown(self.report)

    def state_callback(self, message):
        self.states[message.data] += 1
        if self.current_state != message.data:
            self.transitions.append((round(rospy.get_time(), 3), message.data))
            self.current_state = message.data

    def cap_callback(self, message):
        self.caps_by_state[self.current_state or "BEFORE_STATE"].append(
            round(message.data, 3))

    def mpcc_callback(self, message):
        self.mpcc_speeds_by_state[self.current_state or "BEFORE_STATE"].append(
            round(message.drive.speed, 3))

    def path_callback(self, message):
        if message.poses:
            self.nonempty_paths += 1
        else:
            self.empty_paths += 1

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.status_messages += 1
        self.target_id_used.add(status.get("target_id_used"))
        self.motion_classification.add(status.get("motion_classification"))
        self.reasons[status.get("reason")] += 1
        self.maximum_occupancy = max(
            self.maximum_occupancy, int(status.get("occupancy_count", 0)))

    def report(self):
        caps = {
            state: {
                "minimum": min(values),
                "mean": statistics.fmean(values),
                "maximum": max(values),
            }
            for state, values in self.caps_by_state.items() if values
        }
        mpcc_speeds = {
            state: {
                "minimum": min(values),
                "mean": statistics.fmean(values),
                "maximum": max(values),
            }
            for state, values in self.mpcc_speeds_by_state.items() if values
        }
        print(json.dumps({
            "states": dict(self.states),
            "transitions": self.transitions,
            "caps_by_state": caps,
            "mpcc_speeds_by_state": mpcc_speeds,
            "nonempty_red_paths": self.nonempty_paths,
            "empty_red_paths": self.empty_paths,
            "status_messages": self.status_messages,
            "target_id_used": sorted(self.target_id_used, key=str),
            "motion_classification": sorted(
                self.motion_classification, key=str),
            "reasons": dict(self.reasons),
            "maximum_occupancy": self.maximum_occupancy,
        }, sort_keys=True), flush=True)


if __name__ == "__main__":
    rospy.init_node("roboracer_react_fast_bag_observer")
    Observer()
    rospy.spin()
