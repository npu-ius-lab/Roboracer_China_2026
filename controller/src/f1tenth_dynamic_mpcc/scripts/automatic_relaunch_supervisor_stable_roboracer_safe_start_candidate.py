#!/usr/bin/env python3
"""Isolated low-speed, corridor-guarded start candidate for RoboRacer."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import rospy

# Work both from the source package and from catkin's generated relay script.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from automatic_relaunch_supervisor_stable_roboracer import (  # noqa: E402
    AutomaticRelaunchSupervisor,
)

from f1tenth_dynamic_mpcc.safe_race_start import (  # noqa: E402
    SafeRaceStartPolicy,
)
from f1tenth_dynamic_mpcc.track_model import wrap_angle  # noqa: E402


class SafeStartAutomaticRelaunchSupervisor(AutomaticRelaunchSupervisor):
    """Keep PP and MPCC startup speeds compatible and guard the corridor."""

    def __init__(self) -> None:
        self.safe_start_policy = SafeRaceStartPolicy(
            release_probe_speed_mps=float(rospy.get_param(
                "~safe_release_probe_speed_mps", 0.35
            )),
            pp_maximum_speed_mps=float(rospy.get_param(
                "~safe_pp_maximum_speed_mps", 1.60
            )),
            pp_acceleration_limit_mps2=float(rospy.get_param(
                "~safe_pp_acceleration_limit_mps2", 1.00
            )),
            pp_deceleration_limit_mps2=float(rospy.get_param(
                "~safe_pp_deceleration_limit_mps2", 3.00
            )),
            corridor_slow_margin_m=float(rospy.get_param(
                "~safe_corridor_slow_margin_m", 0.15
            )),
            corridor_abort_margin_m=float(rospy.get_param(
                "~safe_corridor_abort_margin_m", 0.02
            )),
            lateral_error_speed_gain=float(rospy.get_param(
                "~safe_lateral_error_speed_gain", 1.50
            )),
            lateral_error_minimum_speed_mps=float(rospy.get_param(
                "~safe_lateral_error_minimum_speed_mps", 0.80
            )),
            handoff_abort_heading_error_rad=float(rospy.get_param(
                "~safe_handoff_abort_heading_error_rad", 0.35
            )),
        )
        self.safe_start_policy.validate()
        self.safe_projection_s = None
        self.safe_corridor_margin = None
        self.safe_contour_error = None
        self.safe_heading_error = None
        super().__init__()
        if self.race_start_speed > self.safe_start_policy.pp_maximum_speed_mps:
            raise ValueError(
                "race_start_speed_mps exceeds safe_pp_maximum_speed_mps"
            )
        rospy.logwarn(
            "SAFE START CANDIDATE ACTIVE: race_start=%s; release probe=%.2f m/s; "
            "PP max=%.2f m/s; accel/decel=%.2f/%.2f m/s^2; "
            "corridor slow/abort=%.2f/%.2f m",
            self.race_start_enabled,
            self.safe_start_policy.release_probe_speed_mps,
            self.safe_start_policy.pp_maximum_speed_mps,
            self.safe_start_policy.pp_acceleration_limit_mps2,
            self.safe_start_policy.pp_deceleration_limit_mps2,
            self.safe_start_policy.corridor_slow_margin_m,
            self.safe_start_policy.corridor_abort_margin_m,
        )

    def _update_safe_geometry(self, sample) -> None:
        projection = self.track.project(
            sample.x_m, sample.y_m, self.safe_projection_s
        )
        self.safe_projection_s = projection.s
        half_width = 0.5 * self.body_width
        left_margin = (
            float(self.track.width_left(projection.s_wrapped))
            - projection.e_contour - half_width - self.boundary_buffer
        )
        right_margin = (
            float(self.track.width_right(projection.s_wrapped))
            + projection.e_contour - half_width - self.boundary_buffer
        )
        self.safe_corridor_margin = min(left_margin, right_margin)
        self.safe_contour_error = projection.e_contour
        self.safe_heading_error = float(wrap_angle(
            sample.yaw_rad - projection.psi_ref
        ))

    def _update_state(self, sample) -> None:
        self._update_safe_geometry(sample)
        reason = self.safe_start_policy.abort_reason(
            self.state,
            self.safe_corridor_margin,
            self.safe_heading_error,
        )
        if reason is not None:
            margin = self.safe_corridor_margin
            heading = self.safe_heading_error
            self._lock_for_carry(reason)
            rospy.logerr(
                "SAFE START ABORT: %s; margin=%.3f m heading_error=%.3f rad; "
                "hardware output locked",
                reason,
                margin,
                heading,
            )
            return
        super()._update_state(sample)

    def _probe_command(self) -> tuple[float, float]:
        previous_speed = self.last_probe_speed
        previous_time = self.last_probe_command_time
        nominal_speed, steering = super()._probe_command()
        target_speed = self.safe_start_policy.target_speed(
            self.state,
            nominal_speed,
            self.safe_corridor_margin,
            self.safe_contour_error,
        )
        if previous_time > 0.0:
            target_speed = self.safe_start_policy.rate_limit(
                previous_speed,
                target_speed,
                self.last_probe_command_time - previous_time,
            )
        self.last_probe_speed = float(target_speed)
        self.last_recovery_profile_speed = float(target_speed)
        return self.last_probe_speed, steering

    def _status_dictionary(self) -> dict:
        status = super()._status_dictionary()
        status.update({
            "safe_start_candidate": True,
            "safe_release_probe_speed": (
                self.safe_start_policy.release_probe_speed_mps
            ),
            "safe_pp_maximum_speed": (
                self.safe_start_policy.pp_maximum_speed_mps
            ),
            "safe_corridor_margin": self.safe_corridor_margin,
            "safe_contour_error": self.safe_contour_error,
            "safe_heading_error": self.safe_heading_error,
        })
        return status


def main() -> None:
    rospy.init_node("automatic_relaunch_supervisor")
    try:
        SafeStartAutomaticRelaunchSupervisor()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("Safe-start relaunch supervisor failed: %s", error)
        raise


if __name__ == "__main__":
    main()
