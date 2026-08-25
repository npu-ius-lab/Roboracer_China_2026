#!/usr/bin/env python3
"""Collect compact decision metrics during an offline bag replay."""

from __future__ import annotations

import json
import math
import signal

import rospy
from opponent_perception.msg import TargetArray
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def summary(values):
    if not values:
        return None
    ordered = sorted(values)

    def percentile(q):
        position = (len(ordered) - 1) * q
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)

    return {
        "min": ordered[0],
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "max": ordered[-1],
    }


class BagTestObserver:
    def __init__(self) -> None:
        self.output_path = rospy.get_param("~output_path")
        self.bag_path = rospy.get_param("~bag_path", "")
        self.samples = 0
        self.actions = {}
        self.states = {}
        self.reasons = {}
        self.on_track_samples = 0
        self.off_track_samples = 0
        self.overtake_samples = 0
        self.wait_samples = 0
        self.first_overtake_stamp = None
        self.last_overtake_stamp = None
        self.maximum_on_track = 0
        self.maximum_off_track = 0
        self.latest = {}
        self.diagnostic_samples = 0
        self.diagnostic_reasons = {}
        self.valid_diagnostic_samples = 0
        self.measurement_ages = []
        self.prediction_dts = []
        self.target_messages = 0
        self.nonempty_target_messages = 0
        self.target_samples = {}
        self.marker_messages = 0
        self.candidate_ids = set()
        self.confirmed_marker_ids = set()
        self.candidate_z = []
        self.candidate_width = []
        self.candidate_height = []
        self.invalid_candidate_quaternions = 0
        self.transitions = []
        self._last_transition_key = None
        self.subscriber = rospy.Subscriber(
            "/avoidance/status", String, self.callback, queue_size=100
        )
        self.diagnostic_subscriber = rospy.Subscriber(
            "/overtake/diagnostics", String, self.diagnostic_callback, queue_size=100
        )
        self.target_subscriber = rospy.Subscriber(
            "/perception/targets_detailed", TargetArray, self.target_callback, queue_size=100
        )
        self.marker_subscriber = rospy.Subscriber(
            "/perception/target_markers", MarkerArray, self.marker_callback, queue_size=100
        )
        rospy.on_shutdown(self.write)

    def target_callback(self, message: TargetArray) -> None:
        self.target_messages += 1
        self.nonempty_target_messages += int(bool(message.targets))
        for target in message.targets:
            samples = self.target_samples.setdefault(
                int(target.id), {"x": [], "y": [], "z": [], "range": [], "speed": []}
            )
            x = float(target.pose.position.x)
            y = float(target.pose.position.y)
            z = float(target.pose.position.z)
            vx = float(target.twist.linear.x)
            vy = float(target.twist.linear.y)
            vz = float(target.twist.linear.z)
            samples["x"].append(x)
            samples["y"].append(y)
            samples["z"].append(z)
            samples["range"].append(math.hypot(x, y))
            samples["speed"].append(math.sqrt(vx * vx + vy * vy + vz * vz))

    def marker_callback(self, message: MarkerArray) -> None:
        self.marker_messages += 1
        for marker in message.markers:
            if marker.ns == "candidates":
                self.candidate_ids.add(int(marker.id))
                self.candidate_z.append(float(marker.pose.position.z))
                self.candidate_width.append(float(marker.scale.y))
                self.candidate_height.append(float(marker.scale.z))
                q = marker.pose.orientation
                if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                    self.invalid_candidate_quaternions += 1
            elif marker.ns == "opponent_targets":
                self.confirmed_marker_ids.add(int(marker.id))

    def diagnostic_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.diagnostic_samples += 1
        reason = str(payload.get("reason", "UNKNOWN"))
        self.diagnostic_reasons[reason] = self.diagnostic_reasons.get(reason, 0) + 1
        self.valid_diagnostic_samples += int(bool(payload.get("valid", False)))
        measurement_age = payload.get("target_measurement_age_s")
        if measurement_age is not None:
            self.measurement_ages.append(float(measurement_age))
        prediction_dt = payload.get("target_prediction_dt_s")
        if prediction_dt is not None:
            self.prediction_dts.append(float(prediction_dt))

    def callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.samples += 1
        action = str(payload.get("action", "UNKNOWN"))
        state = str(payload.get("state", "UNKNOWN"))
        reason = str(payload.get("reason", "UNKNOWN"))
        stamp = float(payload.get("stamp", rospy.Time.now().to_sec()))
        self.actions[action] = self.actions.get(action, 0) + 1
        self.states[state] = self.states.get(state, 0) + 1
        self.reasons[reason] = self.reasons.get(reason, 0) + 1
        on_track = int(payload.get("on_track_obstacles", 0))
        off_track = int(payload.get("off_track_obstacles", 0))
        self.maximum_on_track = max(self.maximum_on_track, on_track)
        self.maximum_off_track = max(self.maximum_off_track, off_track)
        self.on_track_samples += int(on_track > 0)
        self.off_track_samples += int(off_track > 0)
        if action == "OVERTAKE":
            self.overtake_samples += 1
            if self.first_overtake_stamp is None:
                self.first_overtake_stamp = stamp
            self.last_overtake_stamp = stamp
        if action.startswith("WAIT") or action == "AVOID_WAIT":
            self.wait_samples += 1
        transition_key = (state, action, reason)
        if transition_key != self._last_transition_key:
            if len(self.transitions) < 500:
                self.transitions.append(
                    {
                        "stamp": stamp,
                        "state": state,
                        "action": action,
                        "reason": reason,
                        "on_track_obstacles": on_track,
                        "off_track_obstacles": off_track,
                        "selected_side": payload.get("selected_side", "none"),
                    }
                )
            self._last_transition_key = transition_key
        self.latest = payload

    def write(self) -> None:
        targets = []
        for target_id, samples in sorted(self.target_samples.items()):
            targets.append(
                {
                    "id": target_id,
                    "samples": len(samples["x"]),
                    "x": summary(samples["x"]),
                    "y": summary(samples["y"]),
                    "z": summary(samples["z"]),
                    "range": summary(samples["range"]),
                    "speed": summary(samples["speed"]),
                }
            )
        result = {
            "bag": self.bag_path,
            "samples": self.samples,
            "actions": self.actions,
            "states": self.states,
            "reasons": self.reasons,
            "on_track_samples": self.on_track_samples,
            "off_track_samples": self.off_track_samples,
            "maximum_on_track_obstacles": self.maximum_on_track,
            "maximum_off_track_obstacles": self.maximum_off_track,
            "overtake_samples": self.overtake_samples,
            "wait_samples": self.wait_samples,
            "first_overtake_stamp": self.first_overtake_stamp,
            "last_overtake_stamp": self.last_overtake_stamp,
            "latest": self.latest,
            "transitions": self.transitions,
            "diagnostics": {
                "samples": self.diagnostic_samples,
                "valid_samples": self.valid_diagnostic_samples,
                "reasons": self.diagnostic_reasons,
                "target_measurement_age_s": summary(self.measurement_ages),
                "target_prediction_dt_s": summary(self.prediction_dts),
            },
            "perception": {
                "target_messages": self.target_messages,
                "nonempty_target_messages": self.nonempty_target_messages,
                "unique_target_ids": len(self.target_samples),
                "targets": targets,
                "marker_messages": self.marker_messages,
                "unique_candidate_ids": len(self.candidate_ids),
                "unique_confirmed_marker_ids": len(self.confirmed_marker_ids),
                "candidate_z": summary(self.candidate_z),
                "candidate_width": summary(self.candidate_width),
                "candidate_height": summary(self.candidate_height),
                "invalid_candidate_quaternions": self.invalid_candidate_quaternions,
            },
            "command_authority": False,
        }
        try:
            with open(self.output_path, "w", encoding="utf-8") as stream:
                json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
                stream.write("\n")
        except OSError as error:
            rospy.logerr("Unable to write bag test result %s: %s", self.output_path, error)


def main() -> None:
    rospy.init_node("avoidance_bag_test_observer")
    observer = BagTestObserver()
    signal.signal(signal.SIGTERM, lambda *_args: rospy.signal_shutdown("SIGTERM"))
    rospy.spin()


if __name__ == "__main__":
    main()
