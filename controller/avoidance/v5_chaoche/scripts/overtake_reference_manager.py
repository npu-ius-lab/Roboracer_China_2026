#!/usr/bin/env python3
"""Lock an overtaking path until a confirmed return to the global raceline.

This node owns reference selection only.  It never publishes an Ackermann
command: the local path is consumed by the V5 Chaoche MPCC executable.
"""

from __future__ import annotations

import copy
import json
import math
import threading

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32, String

from f1tenth_overtake.track import PeriodicTrack
from follow_speed_controller import FollowSpeedConfig, FollowSpeedController


ACTIVE_STATES = ("PREPARE", "PASS", "RETURN", "ABORT")


class OvertakeReferenceManager:
    def __init__(self) -> None:
        self.track = PeriodicTrack.from_csv(rospy.get_param("~raceline_csv"))
        self.world_frame = rospy.get_param("~world_frame", "map")
        self.candidate_timeout = float(rospy.get_param("~candidate_timeout_s", 0.80))
        self.odom_timeout = float(rospy.get_param("~odom_timeout_s", 0.30))
        self.status_timeout = float(rospy.get_param("~status_timeout_s", 0.35))
        self.maximum_hold = float(rospy.get_param("~maximum_hold_s", 12.0))
        self.committed_pass_target_loss = float(
            rospy.get_param("~committed_pass_target_loss_s", 2.50)
        )
        self.clear_confirmation = float(
            rospy.get_param("~clear_confirmation_s", 0.60)
        )
        self.return_ey_tolerance = float(
            rospy.get_param("~return_ey_tolerance_m", 0.12)
        )
        self.end_distance = float(rospy.get_param("~path_end_distance_m", 0.35))
        self.speed_caps = {
            "PREPARE": float(rospy.get_param("~speed_caps/prepare", 1.40)),
            "PASS": float(rospy.get_param("~speed_caps/pass", 3.00)),
            "RETURN": float(rospy.get_param("~speed_caps/return", 2.50)),
            "ABORT": float(rospy.get_param("~speed_caps/abort", 0.80)),
        }
        self.require_fusion_health = bool(
            rospy.get_param("~require_fusion_health", True)
        )
        self.fusion_health_timeout = float(
            rospy.get_param("~fusion_health_timeout_s", 0.45)
        )
        self.require_supervisor_running = bool(
            rospy.get_param("~require_supervisor_running_for_maneuver", True)
        )
        self.follow_controller = FollowSpeedController(
            FollowSpeedConfig(
                standstill_gap=float(
                    rospy.get_param("~follow/standstill_gap_m", 0.70)
                ),
                hard_stop_gap=float(
                    rospy.get_param("~follow/hard_stop_gap_m", 0.35)
                ),
                time_headway=float(
                    rospy.get_param("~follow/time_headway_s", 0.35)
                ),
                gap_gain=float(rospy.get_param("~follow/gap_gain", 0.90)),
                maximum_speed=float(
                    rospy.get_param("~follow/maximum_speed_mps", 3.00)
                ),
                maximum_acceleration=float(
                    rospy.get_param("~follow/maximum_acceleration_mps2", 1.50)
                ),
                maximum_deceleration=float(
                    rospy.get_param("~follow/maximum_deceleration_mps2", 3.50)
                ),
                target_speed_alpha=float(
                    rospy.get_param("~follow/target_speed_alpha", 0.35)
                ),
                maximum_target_speed=float(
                    rospy.get_param("~follow/maximum_target_speed_mps", 3.50)
                ),
            )
        )
        self.follow_diagnostic_timeout = float(
            rospy.get_param("~follow/diagnostic_timeout_s", 0.45)
        )

        self.lock = threading.Lock()
        self.odom = None
        self.odom_received = None
        self.staged_path = Path()
        self.staged_at = None
        self.locked_path = Path()
        self.locked_at = None
        self.return_start_index = 0
        self.progress_index = 0
        self.upstream = {}
        self.upstream_received = None
        self.planner_diagnostic = {}
        self.planner_diagnostic_received = None
        self.fusion_health = {}
        self.fusion_health_received = None
        self.supervisor_state = "UNKNOWN"
        self.supervisor_received = None
        self.mode = "GLOBAL"
        self.no_obstacle_since = None
        self.blind_pass_since = None

        self.reference_pub = rospy.Publisher(
            rospy.get_param("~topics/reference", "/v5_chaoche/local_reference"),
            Path,
            queue_size=1,
            latch=True,
        )
        self.red_path_pub = rospy.Publisher(
            rospy.get_param("~topics/red_path", "/v5_chaoche/local_plan_red"),
            Path,
            queue_size=1,
            latch=True,
        )
        self.speed_pub = rospy.Publisher(
            rospy.get_param("~topics/speed_cap", "/v5_chaoche/local_speed_cap"),
            Float32,
            queue_size=1,
            latch=True,
        )
        self.state_pub = rospy.Publisher(
            rospy.get_param("~topics/state", "/v5_chaoche/state"),
            String,
            queue_size=1,
            latch=True,
        )
        self.active_pub = rospy.Publisher(
            rospy.get_param("~topics/active", "/v5_chaoche/reference_active"),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.status_pub = rospy.Publisher(
            rospy.get_param("~topics/status", "/v5_chaoche/reference_status"),
            String,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(
            rospy.get_param("~topics/odom", "/localization/odom"),
            Odometry,
            self.odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~topics/planner_diagnostics",
                "/v5_chaoche/overtake/diagnostics",
            ),
            String,
            self.planner_diagnostic_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~topics/fusion_status",
                "/v5_chaoche/perception/fusion_status",
            ),
            String,
            self.fusion_status_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~topics/supervisor_state",
                "/automatic_relaunch_supervisor_v5_chaoche/state",
            ),
            String,
            self.supervisor_state_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/candidate", "/overtake/selected_path"),
            Path,
            self.candidate_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/upstream_status", "/avoidance/status"),
            String,
            self.status_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self.timer = rospy.Timer(rospy.Duration(0.10), self.timer_callback)
        rospy.logwarn(
            "V5 Chaoche reference manager ready: path lock + confirmed global return; command_authority=false"
        )

    def odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.odom = copy.deepcopy(message)
            self.odom_received = rospy.Time.now()

    def candidate_callback(self, message: Path) -> None:
        if not message.poses:
            return
        with self.lock:
            # A candidate may keep changing while following.  Once a maneuver
            # starts, it cannot replace the path currently owned by MPCC.
            if not self.locked_path.poses:
                self.staged_path = copy.deepcopy(message)
                self.staged_at = rospy.Time.now()

    def status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Invalid avoidance status JSON: %s", error)
            return
        if not isinstance(value, dict):
            return
        with self.lock:
            self.upstream = value
            self.upstream_received = rospy.Time.now()

    def planner_diagnostic_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(value, dict):
            return
        with self.lock:
            self.planner_diagnostic = value
            self.planner_diagnostic_received = rospy.Time.now()

    def fusion_status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(value, dict):
            return
        with self.lock:
            self.fusion_health = value
            self.fusion_health_received = rospy.Time.now()

    def supervisor_state_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(value, dict):
            return
        with self.lock:
            self.supervisor_state = str(value.get("state", "UNKNOWN"))
            self.supervisor_received = rospy.Time.now()

    def compute_return_start(self, path: Path, ego_s: float) -> int:
        lateral = []
        reference_s = ego_s
        for pose in path.poses:
            point = pose.pose.position
            projection = self.track.project_near(
                float(point.x), float(point.y), reference_s, 6.0
            )
            reference_s = projection.s
            lateral.append(abs(float(projection.ey)))
        if not lateral:
            return 0
        apex = max(range(len(lateral)), key=lateral.__getitem__)
        apex_value = lateral[apex]
        threshold = max(self.return_ey_tolerance * 1.5, 0.70 * apex_value)
        for index in range(apex + 1, len(lateral)):
            if lateral[index] <= threshold:
                return index
        return max(apex, int(0.65 * (len(lateral) - 1)))

    @staticmethod
    def nearest_index(path: Path, odom: Odometry, previous: int) -> int:
        if not path.poses:
            return 0
        x = float(odom.pose.pose.position.x)
        y = float(odom.pose.pose.position.y)
        begin = max(0, previous - 3)
        end = min(len(path.poses), max(previous + 35, begin + 1))
        index = min(
            range(begin, end),
            key=lambda item: (
                path.poses[item].pose.position.x - x
            ) ** 2
            + (path.poses[item].pose.position.y - y) ** 2,
        )
        return max(previous, index)

    def lock_candidate(self, now: rospy.Time, odom: Odometry) -> bool:
        if (
            not self.staged_path.poses
            or self.staged_at is None
            or (now - self.staged_at).to_sec() > self.candidate_timeout
        ):
            return False
        self.locked_path = copy.deepcopy(self.staged_path)
        self.locked_at = now
        self.progress_index = 0
        ego_projection = self.track.project(
            float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
        )
        self.return_start_index = self.compute_return_start(
            self.locked_path, ego_projection.s
        )
        rospy.logwarn(
            "V5 Chaoche path LOCKED poses=%d return_start=%d",
            len(self.locked_path.poses),
            self.return_start_index,
        )
        return True

    def clear_locked(self, reason: str) -> None:
        self.locked_path = Path()
        self.locked_at = None
        self.staged_path = Path()
        self.staged_at = None
        self.progress_index = 0
        self.return_start_index = 0
        self.mode = "GLOBAL"
        self.no_obstacle_since = None
        self.blind_pass_since = None
        rospy.loginfo("V5 Chaoche returned to GLOBAL MPCC: %s", reason)

    def fusion_is_healthy(self, now: rospy.Time) -> bool:
        if not self.require_fusion_health:
            return True
        return bool(
            self.fusion_health_received is not None
            and (now - self.fusion_health_received).to_sec()
            <= self.fusion_health_timeout
            and self.fusion_health.get("healthy", False)
        )

    def supervisor_allows_maneuver(self) -> bool:
        if not self.require_supervisor_running:
            return True
        return self.supervisor_state == "RUNNING"

    def follow_speed_cap(self, now: rospy.Time) -> tuple[float, dict]:
        diagnostic = dict(self.planner_diagnostic)
        received = self.planner_diagnostic_received
        fresh = bool(
            received is not None
            and (now - received).to_sec() <= self.follow_diagnostic_timeout
            and diagnostic.get("valid", False)
            and diagnostic.get("target_visible", False)
            and diagnostic.get("relevant", False)
        )
        cap, details = self.follow_controller.update(
            now.to_sec(),
            diagnostic.get("bumper_gap"),
            diagnostic.get("ego_vs"),
            diagnostic.get("opponent_vs"),
            diagnostic.get("target_id"),
            fresh,
        )
        return cap, details

    def update(self, now: rospy.Time) -> tuple[Path, str, float, dict]:
        odom = self.odom
        received = self.odom_received
        upstream = dict(self.upstream)
        upstream_received = self.upstream_received
        upstream_state = str(upstream.get("state", "FREE"))
        odom_fresh = bool(
            odom is not None
            and received is not None
            and (now - received).to_sec() <= self.odom_timeout
        )

        fusion_healthy = self.fusion_is_healthy(now)
        supervisor_ready = self.supervisor_allows_maneuver()
        if self.locked_path.poses and self.supervisor_state in (
            "CARRY_LOCKED",
            "WAIT_GROUND",
            "WAIT_RELEASE",
        ):
            self.clear_locked("supervisor_recovery_invalidated_maneuver")

        if not fusion_healthy:
            self.mode = "ABORT"
            return copy.deepcopy(self.locked_path), "ABORT", 0.0, {
                "reason": "static_and_dynamic_perception_unhealthy_stop",
                "upstream_state": upstream_state,
                "odom_fresh": odom_fresh,
                "fusion_healthy": False,
                "supervisor_state": self.supervisor_state,
            }

        if not self.locked_path.poses:
            if (
                upstream_state in ("PREPARE", "PASS")
                and odom_fresh
                and supervisor_ready
            ):
                if self.lock_candidate(now, odom):
                    self.mode = upstream_state
            if not self.locked_path.poses:
                following = upstream_state in ("FOLLOW", "PREPARE", "PASS")
                self.mode = "FOLLOW" if following else "GLOBAL"
                speed_cap = 0.0
                follow_details = {}
                if following:
                    speed_cap, follow_details = self.follow_speed_cap(now)
                else:
                    self.follow_controller.reset()
                status = {
                    "reason": "waiting_for_locked_candidate",
                    "upstream_state": upstream_state,
                    "odom_fresh": odom_fresh,
                    "fusion_healthy": True,
                    "supervisor_state": self.supervisor_state,
                }
                status.update(follow_details)
                return Path(), self.mode, speed_cap, status

        if not odom_fresh:
            return copy.deepcopy(self.locked_path), "ABORT", 0.0, {
                "reason": "stale_odometry_hold_and_stop",
                "upstream_state": upstream_state,
                "odom_fresh": False,
            }
        if self.locked_at is not None and (now - self.locked_at).to_sec() > self.maximum_hold:
            self.mode = "ABORT"
            return copy.deepcopy(self.locked_path), "ABORT", self.speed_caps["ABORT"], {
                "reason": "maximum_maneuver_hold_exceeded",
                "upstream_state": upstream_state,
                "odom_fresh": True,
                "fusion_healthy": True,
                "supervisor_state": self.supervisor_state,
            }

        status_fresh = bool(
            upstream_received is not None
            and (now - upstream_received).to_sec() <= self.status_timeout
        )

        self.progress_index = self.nearest_index(
            self.locked_path, odom, self.progress_index
        )
        on_track = int(upstream.get("on_track_obstacles", 0))
        target_visible = bool(upstream.get("target_visible", False))
        obstacle_clear = status_fresh and on_track == 0 and not target_visible
        if obstacle_clear:
            if self.no_obstacle_since is None:
                self.no_obstacle_since = now
        else:
            self.no_obstacle_since = None
        clear_confirmed = bool(
            self.no_obstacle_since is not None
            and (now - self.no_obstacle_since).to_sec() >= self.clear_confirmation
        )

        ego_projection = self.track.project(
            float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
        )
        end_pose = self.locked_path.poses[-1].pose.position
        end_distance = math.hypot(
            float(end_pose.x) - float(odom.pose.pose.position.x),
            float(end_pose.y) - float(odom.pose.pose.position.y),
        )
        return_started = self.progress_index >= self.return_start_index
        near_end = bool(
            self.progress_index >= len(self.locked_path.poses) - 3
            or end_distance <= self.end_distance
        )
        centered = abs(float(ego_projection.ey)) <= self.return_ey_tolerance

        previous_mode = self.mode
        upstream_reason = str(upstream.get("reason", ""))
        committed_target_loss = bool(
            previous_mode == "PASS"
            and upstream_state == "ABORT"
            and upstream_reason in (
                "no_confirmed_target",
                "opponent_not_ahead_on_track",
                "targets_stale",
                "target_odom_unavailable",
                "target_message_watchdog_timeout",
            )
            and not return_started
        )
        if committed_target_loss:
            if self.blind_pass_since is None:
                self.blind_pass_since = now
        else:
            self.blind_pass_since = None
        blind_pass_held = bool(
            committed_target_loss
            and self.blind_pass_since is not None
            and (now - self.blind_pass_since).to_sec()
            <= self.committed_pass_target_loss
        )

        if not status_fresh:
            self.mode = "ABORT"
        elif blind_pass_held:
            # The MID360 detector is forward-only. Once the lateral pass is
            # committed, losing the target beside the car is expected; keep
            # the collision-checked locked path until its return segment.
            self.mode = "PASS"
        elif upstream_state == "PASS" and not return_started:
            self.mode = "PASS"
        elif upstream_state == "PREPARE" and not return_started:
            self.mode = "PREPARE"
        elif return_started:
            self.mode = "RETURN"
        elif upstream_state == "ABORT":
            self.mode = "ABORT"
        elif upstream_state == "RETURN":
            # Opponent clearance can be confirmed before MPCC reaches the
            # return portion of the locked geometric path. Keep pass speed and
            # the same reference until geometric progress catches up; dropping
            # to ABORT here was the observed mid-pass 0.8 m/s slowdown.
            self.mode = "RETURN" if return_started else "PASS"
        elif upstream_state == "FREE":
            # Never snap straight to the global line merely because the
            # forward radar loses the opponent after passing it.
            self.mode = "RETURN" if return_started else "ABORT"

        if near_end and centered and clear_confirmed:
            self.clear_locked("path_end_centered_and_obstacle_clear")
            return Path(), "GLOBAL", 0.0, {
                "reason": "return_confirmed",
                "upstream_state": upstream_state,
                "odom_fresh": True,
            }

        speed_cap = self.speed_caps.get(self.mode, self.speed_caps["ABORT"])
        return copy.deepcopy(self.locked_path), self.mode, speed_cap, {
            "reason": "locked_local_mpcc",
            "upstream_state": upstream_state,
            "odom_fresh": True,
            "progress_index": self.progress_index,
            "path_size": len(self.locked_path.poses),
            "return_start_index": self.return_start_index,
            "return_started": return_started,
            "near_end": near_end,
            "centered": centered,
            "clear_confirmed": clear_confirmed,
            "status_fresh": status_fresh,
            "fusion_healthy": True,
            "supervisor_state": self.supervisor_state,
            "blind_pass_held": blind_pass_held,
            "ego_ey": float(ego_projection.ey),
        }

    def timer_callback(self, _event) -> None:
        now = rospy.Time.now()
        with self.lock:
            path, state, speed_cap, status = self.update(now)
            active = bool(path.poses and state in ACTIVE_STATES)
            path.header.frame_id = self.world_frame
            path.header.stamp = now
            for pose in path.poses:
                pose.header = path.header
            status.update(
                stamp=now.to_sec(),
                state=state,
                active=active,
                speed_cap_mps=speed_cap,
                command_authority=False,
                controller="MPCC" if active else "GLOBAL_MPCC",
            )
        # State is published before the path so the MPCC subscriber never sees
        # a local reference without maneuver context.
        self.state_pub.publish(String(data=state))
        self.speed_pub.publish(Float32(data=speed_cap))
        self.reference_pub.publish(path)
        self.red_path_pub.publish(path)
        self.active_pub.publish(Bool(data=active))
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main() -> None:
    rospy.init_node("v5_chaoche_reference_manager")
    OvertakeReferenceManager()
    rospy.spin()


if __name__ == "__main__":
    main()
