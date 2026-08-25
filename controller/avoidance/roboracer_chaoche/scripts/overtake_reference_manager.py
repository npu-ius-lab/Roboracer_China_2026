#!/usr/bin/env python3
"""V3 Pro Max wrapper around the proven locked-reference manager.

It keeps the V5 implementation immutable while fixing the bag-observed
maximum-hold deadlock for this new additive variant.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys

import rospy
from nav_msgs.msg import Path as RosPath


SCRIPT_DIR = str(Path(__file__).resolve().parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
BASE_PATH = (
    Path(__file__).resolve().parents[2]
    / "v5_chaoche"
    / "scripts"
    / "overtake_reference_manager.py"
)
SPEC = importlib.util.spec_from_file_location("v5_locked_reference_manager", BASE_PATH)
if SPEC is None or SPEC.loader is None:
    raise ImportError(f"cannot load base reference manager: {BASE_PATH}")
BASE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BASE
SPEC.loader.exec_module(BASE)


class RoboRacerReferenceManager(BASE.OvertakeReferenceManager):
    def __init__(self) -> None:
        super().__init__()
        self.configured_maximum_hold = self.maximum_hold
        # The subclass resolves timeout before delegating. Disable the base
        # early return that retained an old local path forever.
        self.maximum_hold = 1.0e9
        self.timeout_follow_until = None
        rospy.logwarn(
            "V3 Pro Max Chaoche manager: dropout hold + centered timeout release"
        )

    def update(self, now: rospy.Time):
        odom = self.odom
        odom_fresh = bool(
            odom is not None
            and self.odom_received is not None
            and (now - self.odom_received).to_sec() <= self.odom_timeout
        )
        timed_out = bool(
            self.locked_path.poses
            and self.locked_at is not None
            and (now - self.locked_at).to_sec() > self.configured_maximum_hold
        )
        if timed_out and odom_fresh:
            projection = self.track.project(
                float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
            )
            if abs(float(projection.ey)) <= self.return_ey_tolerance:
                self.clear_locked("maximum_hold_centered_release")
                diagnostic = dict(self.planner_diagnostic)
                target_relevant = bool(
                    diagnostic.get("valid", False)
                    and diagnostic.get("target_visible", False)
                    and diagnostic.get("relevant", False)
                )
                if target_relevant:
                    cap, details = self.follow_speed_cap(now)
                    self.mode = "FOLLOW"
                    self.timeout_follow_until = now + rospy.Duration(1.0)
                    status = {
                        "reason": "maximum_hold_released_to_follow",
                        "upstream_state": self.upstream.get("state", "FREE"),
                        "odom_fresh": True,
                        "fusion_healthy": self.fusion_is_healthy(now),
                        "supervisor_state": self.supervisor_state,
                    }
                    status.update(details)
                    return RosPath(), "FOLLOW", cap, status
                return RosPath(), "GLOBAL", 0.0, {
                    "reason": "maximum_hold_centered_global_release",
                    "odom_fresh": True,
                }

        if (
            not self.locked_path.poses
            and self.timeout_follow_until is not None
            and now <= self.timeout_follow_until
        ):
            diagnostic = dict(self.planner_diagnostic)
            if diagnostic.get("target_visible", False) and diagnostic.get(
                "relevant", False
            ):
                cap, details = self.follow_speed_cap(now)
                status = {
                    "reason": "post_timeout_follow_hold",
                    "upstream_state": self.upstream.get("state", "FREE"),
                    "odom_fresh": odom_fresh,
                    "fusion_healthy": self.fusion_is_healthy(now),
                    "supervisor_state": self.supervisor_state,
                }
                status.update(details)
                return RosPath(), "FOLLOW", cap, status
        return super().update(now)


def main() -> None:
    rospy.init_node("roboracer_chaoche_reference_manager")
    RoboRacerReferenceManager()
    rospy.spin()


if __name__ == "__main__":
    main()
