#!/usr/bin/env python3
"""Automatic relaunch supervisor that deliberately retains Pure Pursuit.

This is an opt-in hardware debugging node.  It reuses the accepted carry,
ground-placement, race-start and controller reset state machine verbatim, but
does not hand hardware ownership from Pure Pursuit to MPCC after release.
MPCC remains enabled in the background so its telemetry can be compared with
the command actually sent by Pure Pursuit.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import rospkg
import rospy


def _load_base_module():
    package = Path(rospkg.RosPack().get_path("f1tenth_dynamic_mpcc"))
    source = package / "scripts" / "automatic_relaunch_supervisor.py"
    spec = importlib.util.spec_from_file_location(
        "f1tenth_dynamic_mpcc_automatic_relaunch_supervisor", source
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load automatic relaunch supervisor: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_base = _load_base_module()


class PurePursuitDebugSupervisor(_base.AutomaticRelaunchSupervisor):
    """Keep the accepted relaunch safety state machine in continuous PP mode."""

    def __init__(self) -> None:
        self.debug_pp_speed = float(
            rospy.get_param("~debug_pp_speed_mps", 3.0)
        )
        self.debug_pp_minimum_speed = float(
            rospy.get_param("~debug_pp_minimum_speed_mps", 1.20)
        )
        self.debug_pp_lookahead = float(
            rospy.get_param("~debug_pp_lookahead_m", 0.90)
        )
        self.debug_pp_lookahead_speed_gain = float(
            rospy.get_param("~debug_pp_lookahead_speed_gain_s", 0.25)
        )
        self.debug_pp_lookahead_maximum = float(
            rospy.get_param("~debug_pp_lookahead_maximum_m", 1.20)
        )
        self.debug_pp_speed_preview = float(
            rospy.get_param("~debug_pp_speed_preview_m", 3.0)
        )
        self.debug_pp_lateral_acceleration_limit = float(
            rospy.get_param("~debug_pp_lateral_acceleration_limit_mps2", 2.0)
        )
        values = (
            self.debug_pp_speed,
            self.debug_pp_minimum_speed,
            self.debug_pp_lookahead,
            self.debug_pp_lookahead_maximum,
            self.debug_pp_speed_preview,
            self.debug_pp_lateral_acceleration_limit,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("Pure Pursuit debug parameters must be finite and positive")
        if self.debug_pp_minimum_speed > self.debug_pp_speed:
            raise ValueError("Pure Pursuit debug minimum speed exceeds its speed cap")
        if self.debug_pp_lookahead_maximum < self.debug_pp_lookahead:
            raise ValueError("Pure Pursuit debug maximum lookahead is too small")
        if (not math.isfinite(self.debug_pp_lookahead_speed_gain) or
                self.debug_pp_lookahead_speed_gain < 0.0):
            raise ValueError("Pure Pursuit debug lookahead speed gain is invalid")
        super().__init__()
        rospy.logwarn(
            "PURE PURSUIT DEBUG OWNS HARDWARE continuously: speed<=%.2f m/s "
            "lookahead=%.2f+%.2f*(v-vmin), max=%.2f m; MPCC is telemetry only",
            self.debug_pp_speed,
            self.debug_pp_lookahead,
            self.debug_pp_lookahead_speed_gain,
            self.debug_pp_lookahead_maximum,
        )

    def mpcc_command_callback(self, message) -> None:
        """Record the MPCC comparison command without permitting a handoff."""
        with self.lock:
            self.latest_mpcc_command = message
            self.latest_mpcc_command_time = rospy.get_time()
            if self.state == "HANDOFF_PP":
                self.state_reason = "pure_pursuit_debug_continuous_tracking"

    def _process_service_actions(self) -> None:
        super()._process_service_actions()
        with self.lock:
            if self.state == "HANDOFF_PP":
                self.state_reason = "pure_pursuit_debug_continuous_tracking"

    def _probe_command(self) -> tuple[float, float]:
        # WAIT_RELEASE and ENABLING intentionally retain the accepted start or
        # relocation plan.  Once the controller is enabled, temporarily select
        # the existing adaptive recovery PP profile for continuous full-track
        # debugging.  Restoring every field keeps launch-mode diagnostics and
        # the base supervisor's behavior intact.
        if self.state != "HANDOFF_PP":
            return super()._probe_command()
        saved = (
            self.launch_mode,
            self.probe_speed,
            self.recovery_minimum_speed,
            self.probe_lookahead,
            self.probe_lookahead_speed_gain,
            self.probe_lookahead_maximum,
            self.recovery_speed_preview,
            self.recovery_lateral_acceleration_limit,
        )
        self.launch_mode = "recovery"
        self.probe_speed = self.debug_pp_speed
        self.recovery_minimum_speed = self.debug_pp_minimum_speed
        self.probe_lookahead = self.debug_pp_lookahead
        self.probe_lookahead_speed_gain = self.debug_pp_lookahead_speed_gain
        self.probe_lookahead_maximum = self.debug_pp_lookahead_maximum
        self.recovery_speed_preview = self.debug_pp_speed_preview
        self.recovery_lateral_acceleration_limit = (
            self.debug_pp_lateral_acceleration_limit
        )
        try:
            return super()._probe_command()
        finally:
            (
                self.launch_mode,
                self.probe_speed,
                self.recovery_minimum_speed,
                self.probe_lookahead,
                self.probe_lookahead_speed_gain,
                self.probe_lookahead_maximum,
                self.recovery_speed_preview,
                self.recovery_lateral_acceleration_limit,
            ) = saved

    def _status_dictionary(self) -> dict:
        status = super()._status_dictionary()
        status.update({
            "pure_pursuit_debug": True,
            "pure_pursuit_debug_speed_cap": self.debug_pp_speed,
            "pure_pursuit_debug_minimum_speed": self.debug_pp_minimum_speed,
            "pure_pursuit_debug_lookahead": self.debug_pp_lookahead,
            "pure_pursuit_debug_lookahead_speed_gain": (
                self.debug_pp_lookahead_speed_gain
            ),
            "pure_pursuit_debug_lookahead_maximum": (
                self.debug_pp_lookahead_maximum
            ),
        })
        return status


def main() -> None:
    rospy.init_node("automatic_relaunch_supervisor")
    try:
        PurePursuitDebugSupervisor()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("Pure Pursuit debug supervisor failed: %s", error)
        raise


if __name__ == "__main__":
    main()
