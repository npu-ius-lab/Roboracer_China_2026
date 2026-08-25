#!/usr/bin/env python3
"""Automatically enable the quick-relaunch candidate after safe placement."""

from __future__ import annotations

import math
from pathlib import Path
import threading
from typing import Optional

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerResponse

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.quick_relaunch import (
    PlacementAssessment,
    StablePlacementWindow,
    assess_placement,
    placement_relocated,
)
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


def message_yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CheckpointStartGuard:
    def __init__(self) -> None:
        track_csv = Path(rospy.get_param("~track_csv")).expanduser().resolve()
        vehicle_config = Path(rospy.get_param("~vehicle_config")).expanduser().resolve()
        vehicle = load_yaml(vehicle_config)["vehicle"]
        self.body_width = float(vehicle["ego_width"])
        self.track = PeriodicTrack(track_csv)
        self.world_frame = str(rospy.get_param("~world_frame", "map")).lstrip("/")
        self.body_frame = str(
            rospy.get_param("~body_frame", "localization_base_link")
        ).lstrip("/")
        self.boundary_buffer = float(rospy.get_param("~boundary_buffer_m", 0.08))
        self.maximum_heading_error = float(
            rospy.get_param("~maximum_heading_error_rad", 0.55)
        )
        self.maximum_stationary_speed = float(
            rospy.get_param("~maximum_stationary_speed_mps", 0.12)
        )
        self.minimum_relocation_distance = float(
            rospy.get_param("~minimum_relocation_distance_m", 0.25)
        )
        self.minimum_relocation_yaw = float(
            rospy.get_param("~minimum_relocation_yaw_rad", 0.35)
        )
        required_samples = int(rospy.get_param("~required_stable_samples", 8))
        self.window = StablePlacementWindow(
            required_samples=required_samples,
            maximum_position_span_m=float(
                rospy.get_param("~maximum_position_span_m", 0.06)
            ),
            maximum_yaw_span_rad=float(
                rospy.get_param("~maximum_yaw_span_rad", 0.06)
            ),
            maximum_progress_span_m=float(
                rospy.get_param("~maximum_progress_span_m", 0.10)
            ),
            track_length_m=self.track.length,
        )
        self.auto_enable = bool(rospy.get_param("~auto_enable", True))
        self.enable_service_name = str(
            rospy.get_param("~enable_service", "/f1tenth_dynamic_mpcc/set_enabled")
        )
        self.enable_client = rospy.ServiceProxy(self.enable_service_name, SetBool)
        self.lock = threading.Lock()
        self.ready = False
        self.enable_requested = False
        self.enabled = False
        self.manual_release_required = False
        self.latest: Optional[PlacementAssessment] = None
        self.latest_pose = (0.0, 0.0, 0.0)
        self.last_enable_pose: Optional[tuple[float, float, float]] = None
        self.relocation_reference: Optional[tuple[float, float, float]] = None
        self.relocation_required = False
        odom_topic = str(
            rospy.get_param("~odom_topic", "/localization/vehicle_odom")
        )
        self.subscriber = rospy.Subscriber(
            odom_topic, Odometry, self.odom_callback, queue_size=1,
            tcp_nodelay=True,
        )
        self.rearm_service = rospy.Service("~rearm", Trigger, self.rearm_callback)
        self.release_service = rospy.Service(
            "~release", Trigger, self.release_callback
        )
        self.status_service = rospy.Service("~status", Trigger, self.status_callback)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.loginfo(
            "Quick-relaunch guard ready track=%s samples=%d heading<=%.2f "
            "buffer=%.2f auto_enable=%s",
            track_csv,
            required_samples,
            self.maximum_heading_error,
            self.boundary_buffer,
            self.auto_enable,
        )

    def odom_callback(self, message: Odometry) -> None:
        frame = message.header.frame_id.lstrip("/")
        child = message.child_frame_id.lstrip("/")
        if frame != self.world_frame or child != self.body_frame:
            rospy.logerr_throttle(
                1.0,
                "Quick-relaunch guard rejecting frames %s -> %s",
                frame,
                child,
            )
            with self.lock:
                self.ready = False
                self.window.reset()
            return
        x_m = float(message.pose.pose.position.x)
        y_m = float(message.pose.pose.position.y)
        yaw_rad = message_yaw(message)
        twist = message.twist.twist
        speed_mps = math.hypot(float(twist.linear.x), float(twist.linear.y))
        try:
            assessment = assess_placement(
                self.track,
                self.body_width,
                x_m,
                y_m,
                yaw_rad,
                speed_mps,
                boundary_buffer_m=self.boundary_buffer,
                maximum_heading_error_rad=self.maximum_heading_error,
                maximum_stationary_speed_mps=self.maximum_stationary_speed,
            )
        except (ValueError, FloatingPointError) as error:
            rospy.logwarn_throttle(1.0, "Placement assessment failed: %s", error)
            return
        with self.lock:
            self.latest = assessment
            self.latest_pose = (x_m, y_m, yaw_rad)
            if self.relocation_required and self.relocation_reference is not None:
                relocated = placement_relocated(
                    self.relocation_reference,
                    self.latest_pose,
                    self.minimum_relocation_distance,
                    self.minimum_relocation_yaw,
                )
                if relocated:
                    self.relocation_required = False
                    self.window.reset()
                else:
                    self.window.reset()
                    self.ready = False
            if not self.relocation_required:
                self.ready = self.window.update(
                    assessment, x_m, y_m, yaw_rad
                )
            sample_count = self.window.sample_count
            relocation_required = self.relocation_required
        rospy.loginfo_throttle(
            0.5,
            "Quick-relaunch placement=%s s=%.2f e=%.3f heading=%.3f "
            "margin=%.3f speed=%.3f stable=%d/%d relocation_required=%s",
            assessment.reason,
            assessment.projection.s_wrapped,
            assessment.projection.e_contour,
            assessment.heading_error_rad,
            assessment.vehicle_margin_m,
            assessment.speed_mps,
            sample_count,
            self.window.required_samples,
            relocation_required,
        )

    def timer_callback(self, _event: rospy.timer.TimerEvent) -> None:
        with self.lock:
            should_enable = (
                self.auto_enable
                and self.ready
                and not self.manual_release_required
                and not self.enable_requested
                and not self.enabled
            )
            if should_enable:
                self.enable_requested = True
        if not should_enable:
            return
        try:
            response = self.enable_client(SetBoolRequest(data=True))
            success = bool(response.success)
            message = str(response.message)
        except rospy.ServiceException as error:
            success = False
            message = str(error)
        with self.lock:
            self.enabled = success
            self.enable_requested = False
            if success:
                self.last_enable_pose = self.latest_pose
        if success:
            rospy.logwarn(
                "QUICK RELAUNCH ENABLED: placement stable; checkpoint PP will "
                "reacquire globally and hand off to MPCC"
            )
        else:
            rospy.logwarn_throttle(
                1.0, "Quick-relaunch enable service not ready: %s", message
            )

    def rearm_callback(self, _request) -> TriggerResponse:
        try:
            response = self.enable_client(SetBoolRequest(data=False))
            disabled = bool(response.success)
            message = str(response.message)
        except rospy.ServiceException as error:
            disabled = False
            message = str(error)
        with self.lock:
            self.window.reset()
            self.ready = False
            self.enable_requested = False
            self.enabled = False
            self.manual_release_required = True
            self.relocation_reference = self.last_enable_pose
            self.relocation_required = self.relocation_reference is not None
        if disabled:
            return TriggerResponse(
                success=True,
                message=(
                    "controller disabled; keep the physical emergency stop active, "
                    "move the car and place it forward and fully inside the track; "
                    "after status reports ready, release the physical emergency "
                    "stop and call /checkpoint_start_guard/release"
                ),
            )
        return TriggerResponse(
            success=False, message="failed to disable controller: " + message
        )

    def release_callback(self, _request) -> TriggerResponse:
        with self.lock:
            ready = self.ready
            relocation_required = self.relocation_required
            already_enabled = self.enabled
            in_flight = self.enable_requested
            if ready and not relocation_required and not already_enabled and not in_flight:
                self.enable_requested = True
        if already_enabled:
            return TriggerResponse(success=True, message="controller already enabled")
        if in_flight:
            return TriggerResponse(success=False, message="enable request already in progress")
        if not ready or relocation_required:
            return TriggerResponse(
                success=False,
                message="placement is not ready; check /checkpoint_start_guard/status",
            )
        try:
            response = self.enable_client(SetBoolRequest(data=True))
            success = bool(response.success)
            message = str(response.message)
        except rospy.ServiceException as error:
            success = False
            message = str(error)
        with self.lock:
            self.enabled = success
            self.enable_requested = False
            if success:
                self.manual_release_required = False
                self.last_enable_pose = self.latest_pose
        return TriggerResponse(success=success, message=message)

    def status_callback(self, _request) -> TriggerResponse:
        with self.lock:
            assessment = self.latest
            ready = self.ready
            enabled = self.enabled
            samples = self.window.sample_count
            relocation_required = self.relocation_required
            manual_release_required = self.manual_release_required
        if assessment is None:
            return TriggerResponse(success=False, message="waiting for odometry")
        return TriggerResponse(
            success=ready,
            message=(
                f"reason={assessment.reason} ready={ready} enabled={enabled} "
                f"relocation_required={relocation_required} "
                f"manual_release_required={manual_release_required} "
                f"samples={samples}/{self.window.required_samples} "
                f"s={assessment.projection.s_wrapped:.3f} "
                f"e={assessment.projection.e_contour:.3f} "
                f"heading={assessment.heading_error_rad:.3f} "
                f"margin={assessment.vehicle_margin_m:.3f}"
            ),
        )


def main() -> None:
    rospy.init_node("checkpoint_start_guard")
    CheckpointStartGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
