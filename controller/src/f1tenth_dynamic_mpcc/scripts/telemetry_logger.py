#!/usr/bin/env python3
"""Non-real-time JSONL logger for the C++ MPCC diagnostics topic."""

import json
import queue
import threading
from pathlib import Path

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray


FIELDS = (
    "contour_error", "lag_error", "heading_error", "sideslip",
    "front_slip", "rear_slip", "published_speed", "published_steering",
    "pointlio_age", "solve_time", "failure_count", "solver_status",
    "solver_success", "track_slack", "tire_slack", "vx", "vy", "yaw_rate",
    "near_track_slack", "far_track_slack", "track_slack_stage",
    "track_slack_time", "track_slack_side", "speed_cap_used", "speed_retry",
    "warm_start_reset", "failure_kind", "rti_iterations",
    "speed_input_cap", "trigger_track_slack", "trigger_track_slack_stage",
    "trigger_track_slack_time", "trigger_track_slack_side",
    "state_prediction_horizon", "committed_horizon", "mpcc_state_lead",
    "objective",
    "cost_contour", "cost_lag", "cost_heading", "cost_speed_prior",
    "cost_progress", "cost_control", "cost_delta_control",
    "cost_track_slack", "cost_tire_slack", "geometry_theta_shift",
    "geometry_heading_shift", "geometry_curvature_shift",
    "candidate_active", "candidate_activated", "candidate_released",
    "prediction_risk", "mpcc_minimum_margin", "candidate_minimum_margin",
    "candidate_margin_improvement", "candidate_speed", "candidate_steering",
    "candidate_lookahead", "candidate_reason_code",
    "candidate_initial_margin", "candidate_final_margin",
    "candidate_reentry_time", "candidate_recovery_mode",
    "raw_pointlio_vx", "raw_pointlio_vy", "raw_pointlio_yaw_rate",
    "low_speed_stationary", "low_speed_dynamic_blend",
    "warm_start_advance_s", "warm_start_shift_stages",
    "skipped_solve_ticks",
    "pp_trigger_risk", "near_prediction_risk", "far_prediction_risk",
    "prediction_risk_confirmation_cycles", "near_minimum_margin",
    "checkpoint_recovery_armed",
    "tracking_gate_scale", "tracking_gate_speed_cap",
    "tracking_gate_lateral_scale", "tracking_gate_heading_scale",
    "tracking_gate_margin_scale", "prediction_soft_margin_risk",
)


class Logger:
    def __init__(self):
        self.path = Path(rospy.get_param("~path")).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.queue = queue.Queue(maxsize=1024)
        self.last_command = None
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()
        rospy.Subscriber("/f1tenth_mpcc/telemetry", Float32MultiArray,
                         self.diagnostic, queue_size=100)
        rospy.Subscriber("/f1tenth_mpcc/real/ackermann_cmd_stamped",
                         AckermannDriveStamped, self.command, queue_size=20)

    def command(self, message):
        self.last_command = {
            "stamp": message.header.stamp.to_sec(),
            "speed": message.drive.speed,
            "steering": message.drive.steering_angle,
        }

    def diagnostic(self, message):
        values = {key: value for key, value in zip(FIELDS, message.data)}
        now = rospy.Time.now().to_sec()
        age = float(values.get("pointlio_age", 0.0))
        state_horizon = float(values.get("state_prediction_horizon", age))
        committed = float(values.get("committed_horizon", 0.0))
        values.update({
            "control_now": now,
            "pointlio_source_stamp": now - age,
            "measurement_prediction_start": now - state_horizon,
            "measurement_prediction_end": now,
            "committed_prediction_start": now,
            "committed_prediction_end": now + committed,
            "mpcc_initial_state_timestamp": now + committed,
        })
        record = {"timestamp": now, "diagnostic": values,
                  "command": self.last_command}
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            rospy.logwarn_throttle(5.0, "MPCC telemetry logger queue full")

    def _write(self):
        with self.path.open("a", buffering=64 * 1024) as stream:
            while not rospy.is_shutdown():
                try:
                    record = self.queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                stream.write(json.dumps(record, sort_keys=True) + "\n")
                self.queue.task_done()


if __name__ == "__main__":
    rospy.init_node("f1tenth_mpcc_telemetry_logger")
    Logger()
    rospy.spin()
