#!/usr/bin/env python3
"""Explicitly armed, fail-closed adapter from MPCC Stamped commands to hardware."""

from __future__ import annotations

import threading
import time

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class CommandGate:
    def __init__(self) -> None:
        if not bool(rospy.get_param("~allow_real_hardware", False)):
            raise RuntimeError("allow_real_hardware must be explicitly true")
        self.input_topic = str(rospy.get_param("~input_topic", "/f1tenth_mpcc/real/ackermann_cmd_stamped"))
        self.output_topic = str(rospy.get_param("~output_topic", "/tianracer/ackermann_cmd"))
        self.collision_topic = str(rospy.get_param("~collision_topic", "/tianracer/collision"))
        self.collision_required = bool(rospy.get_param("~collision_required", False))
        self.command_timeout = float(rospy.get_param("~command_timeout_s", 0.20))
        self.collision_timeout = float(rospy.get_param("~collision_timeout_s", 0.50))
        self.rate = float(rospy.get_param("~publish_rate_hz", 20.0))
        self._lock = threading.Lock()
        self._enabled = bool(rospy.get_param("~start_enabled", False))
        self._command = None
        self._command_time = float("-inf")
        self._collision = False
        self._collision_time = float("-inf")
        self.pub = rospy.Publisher(self.output_topic, AckermannDrive, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, AckermannDriveStamped, self._command_cb, queue_size=1)
        self.collision_sub = None
        if self.collision_required:
            self._collision = True
            self.collision_sub = rospy.Subscriber(
                self.collision_topic, Bool, self._collision_cb, queue_size=1
            )
        self.service = rospy.Service("~set_enabled", SetBool, self._enable_cb)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.rate), self._tick)
        rospy.on_shutdown(self._stop)
        self._stop()
        rospy.logwarn(
            "MPCC command gate ready output=%s enabled=%s",
            self.output_topic,
            self._enabled,
        )

    def _command_cb(self, message: AckermannDriveStamped) -> None:
        command = AckermannDrive()
        command.steering_angle = message.drive.steering_angle
        command.speed = message.drive.speed
        with self._lock:
            self._command = command
            self._command_time = time.monotonic()

    def _collision_cb(self, message: Bool) -> None:
        with self._lock:
            self._collision = bool(message.data)
            self._collision_time = time.monotonic()

    def _enable_cb(self, request: SetBool.Request) -> SetBoolResponse:
        with self._lock:
            now = time.monotonic()
            safe = (
                self._command is not None
                and now - self._command_time <= self.command_timeout
                and (
                    not self.collision_required
                    or (
                        now - self._collision_time <= self.collision_timeout
                        and not self._collision
                    )
                )
            )
            if request.data and not safe:
                requirement = (
                    "fresh command and clear collision state"
                    if self.collision_required
                    else "fresh command"
                )
                return SetBoolResponse(False, requirement + " required")
            self._enabled = bool(request.data)
        if not request.data:
            self._stop()
        return SetBoolResponse(True, "enabled" if request.data else "disabled")

    def _stop(self) -> None:
        self.pub.publish(AckermannDrive())

    def _tick(self, _event) -> None:
        with self._lock:
            now = time.monotonic()
            safe = (
                self._enabled
                and self._command is not None
                and now - self._command_time <= self.command_timeout
                and (
                    not self.collision_required
                    or (
                        now - self._collision_time <= self.collision_timeout
                        and not self._collision
                    )
                )
            )
            command = self._command if safe else None
        self.pub.publish(command if command is not None else AckermannDrive())


def main() -> None:
    rospy.init_node("f1tenth_mpcc_command_gate")
    CommandGate()
    rospy.spin()


if __name__ == "__main__":
    main()
