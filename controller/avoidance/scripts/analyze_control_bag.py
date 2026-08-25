#!/usr/bin/env python3
"""Summarize boundary/control events in a TianRacer hardware analysis bag.

This is a read-only diagnostic.  It does not start ROS nodes and does not
publish commands.  The stable-v3 telemetry layout is decoded from
mpcc_node_auto_relaunch.cpp.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, Optional

import rosbag


SUPERVISOR_TOPIC = "/automatic_relaunch_supervisor/state"
TELEMETRY_TOPIC = "/f1tenth_mpcc/telemetry"
COMMAND_TOPIC = "/tianracer/ackermann_cmd"
ODOM_TOPIC = "/localization/vehicle_odom"


def finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def update_extreme(
    current: Optional[Dict[str, float]], value: Any, stamp: float, mode: str
) -> Optional[Dict[str, float]]:
    number = finite(value)
    if number is None:
        return current
    if current is None or (mode == "min" and number < current["value"]) or (
        mode == "max" and number > current["value"]
    ):
        return {"stamp": stamp, "value": number}
    return current


def close_window(
    windows: list, start: Optional[float], end: float
) -> Optional[float]:
    if start is not None:
        windows.append({"start": start, "end": end, "duration": max(0.0, end - start)})
    return None


def decode_supervisor(message: Any) -> Dict[str, Any]:
    try:
        payload = json.loads(message.data)
    except (AttributeError, TypeError, ValueError):
        return {}
    return payload if isinstance(payload, dict) else {}


def analyze(path: str) -> Dict[str, Any]:
    with rosbag.Bag(path) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        info = bag.get_type_and_topic_info()[1]
        topic_counts = {name: int(item.message_count) for name, item in info.items()}

        state_counts: Counter = Counter()
        reason_counts: Counter = Counter()
        state_durations = defaultdict(float)
        transitions = []
        last_state_key = None
        last_supervisor_stamp = None
        last_supervisor_state = None
        minimum_vehicle_margin = None
        minimum_pp_margin = None
        maximum_abs_supervisor_contour = None
        maximum_abs_supervisor_heading = None
        minimum_wheel_speed = None
        maximum_wheel_speed = None
        disabled_samples = 0
        supervisor_samples = 0
        negative_margin_samples = 0

        telemetry_samples = 0
        maximum_abs_contour = None
        maximum_abs_heading = None
        maximum_abs_steering = None
        maximum_speed = None
        minimum_mpcc_margin = None
        minimum_near_margin = None
        maximum_track_slack = None
        maximum_near_track_slack = None
        maximum_far_track_slack = None
        candidate_active_samples = 0
        prediction_risk_samples = 0
        near_risk_samples = 0
        far_risk_samples = 0
        candidate_windows = []
        risk_windows = []
        candidate_start = None
        risk_start = None

        command_samples = 0
        maximum_command_speed = None
        maximum_abs_command_steering = None
        latest_command = {"stamp": None, "speed": None, "steering": None}

        odom_samples = 0
        distance_m = 0.0
        previous_xy = None
        latest_odom = None
        maximum_odom_speed = None

        topics = [SUPERVISOR_TOPIC, TELEMETRY_TOPIC, COMMAND_TOPIC, ODOM_TOPIC]
        for topic, message, recorded_at in bag.read_messages(topics=topics):
            stamp = recorded_at.to_sec()
            relative = stamp - start
            if topic == SUPERVISOR_TOPIC:
                payload = decode_supervisor(message)
                if not payload:
                    continue
                supervisor_samples += 1
                state = str(payload.get("state", "UNKNOWN"))
                reason = str(payload.get("reason", "UNKNOWN"))
                state_counts[state] += 1
                reason_counts[reason] += 1
                disabled_samples += int(bool(payload.get("controller_disabled", False)))
                vehicle_margin = finite(payload.get("vehicle_margin"))
                negative_margin_samples += int(vehicle_margin is not None and vehicle_margin < 0.0)
                minimum_vehicle_margin = update_extreme(
                    minimum_vehicle_margin, vehicle_margin, relative, "min"
                )
                minimum_pp_margin = update_extreme(
                    minimum_pp_margin, payload.get("pp_path_margin"), relative, "min"
                )
                contour = finite(payload.get("contour_error"))
                heading = finite(payload.get("heading_error"))
                maximum_abs_supervisor_contour = update_extreme(
                    maximum_abs_supervisor_contour,
                    abs(contour) if contour is not None else None,
                    relative,
                    "max",
                )
                maximum_abs_supervisor_heading = update_extreme(
                    maximum_abs_supervisor_heading,
                    abs(heading) if heading is not None else None,
                    relative,
                    "max",
                )
                minimum_wheel_speed = update_extreme(
                    minimum_wheel_speed, payload.get("wheel_speed"), relative, "min"
                )
                maximum_wheel_speed = update_extreme(
                    maximum_wheel_speed, payload.get("wheel_speed"), relative, "max"
                )
                if last_supervisor_stamp is not None and last_supervisor_state is not None:
                    state_durations[last_supervisor_state] += max(
                        0.0, stamp - last_supervisor_stamp
                    )
                last_supervisor_stamp = stamp
                last_supervisor_state = state
                state_key = (state, reason, bool(payload.get("controller_disabled", False)))
                if state_key != last_state_key:
                    transitions.append(
                        {
                            "stamp": relative,
                            "state": state,
                            "reason": reason,
                            "disabled": bool(payload.get("controller_disabled", False)),
                            "wheel_speed": finite(payload.get("wheel_speed")),
                            "vehicle_margin": vehicle_margin,
                            "contour_error": contour,
                            "heading_error": heading,
                            "s": finite(payload.get("s")),
                            "pp_path_margin": finite(payload.get("pp_path_margin")),
                        }
                    )
                    last_state_key = state_key
            elif topic == TELEMETRY_TOPIC:
                data = list(message.data)
                # Stable-v3 auto-relaunch records 78 values (indices 0..77).
                if len(data) < 78:
                    continue
                telemetry_samples += 1
                maximum_abs_contour = update_extreme(
                    maximum_abs_contour, abs(data[0]), relative, "max"
                )
                maximum_abs_heading = update_extreme(
                    maximum_abs_heading, abs(data[2]), relative, "max"
                )
                maximum_abs_steering = update_extreme(
                    maximum_abs_steering, abs(data[7]), relative, "max"
                )
                maximum_speed = update_extreme(maximum_speed, abs(data[15]), relative, "max")
                minimum_mpcc_margin = update_extreme(
                    minimum_mpcc_margin, data[53], relative, "min"
                )
                minimum_near_margin = update_extreme(
                    minimum_near_margin, data[77], relative, "min"
                )
                maximum_track_slack = update_extreme(
                    maximum_track_slack, data[13], relative, "max"
                )
                maximum_near_track_slack = update_extreme(
                    maximum_near_track_slack, data[18], relative, "max"
                )
                maximum_far_track_slack = update_extreme(
                    maximum_far_track_slack, data[19], relative, "max"
                )
                candidate_active = data[49] > 0.5
                prediction_risk = data[52] > 0.5
                near_risk = data[74] > 0.5
                far_risk = data[75] > 0.5
                candidate_active_samples += int(candidate_active)
                prediction_risk_samples += int(prediction_risk)
                near_risk_samples += int(near_risk)
                far_risk_samples += int(far_risk)
                if candidate_active and candidate_start is None:
                    candidate_start = relative
                elif not candidate_active and candidate_start is not None:
                    candidate_start = close_window(candidate_windows, candidate_start, relative)
                if prediction_risk and risk_start is None:
                    risk_start = relative
                elif not prediction_risk and risk_start is not None:
                    risk_start = close_window(risk_windows, risk_start, relative)
            elif topic == COMMAND_TOPIC:
                command_samples += 1
                speed = finite(message.speed)
                steering = finite(message.steering_angle)
                latest_command = {"stamp": relative, "speed": speed, "steering": steering}
                maximum_command_speed = update_extreme(
                    maximum_command_speed, speed, relative, "max"
                )
                maximum_abs_command_steering = update_extreme(
                    maximum_abs_command_steering,
                    abs(steering) if steering is not None else None,
                    relative,
                    "max",
                )
            elif topic == ODOM_TOPIC:
                odom_samples += 1
                x = float(message.pose.pose.position.x)
                y = float(message.pose.pose.position.y)
                vx = float(message.twist.twist.linear.x)
                vy = float(message.twist.twist.linear.y)
                speed = math.hypot(vx, vy)
                if previous_xy is not None:
                    step = math.hypot(x - previous_xy[0], y - previous_xy[1])
                    if step < 1.0:
                        distance_m += step
                previous_xy = (x, y)
                latest_odom = {"stamp": relative, "x": x, "y": y, "speed": speed}
                maximum_odom_speed = update_extreme(
                    maximum_odom_speed, speed, relative, "max"
                )

        duration = end - start
        if last_supervisor_stamp is not None and last_supervisor_state is not None:
            state_durations[last_supervisor_state] += max(0.0, end - last_supervisor_stamp)
        candidate_start = close_window(candidate_windows, candidate_start, duration)
        risk_start = close_window(risk_windows, risk_start, duration)

    def ratio(count: int, total: int) -> float:
        return float(count) / float(total) if total else 0.0

    return {
        "bag": path,
        "start": start,
        "end": end,
        "duration": duration,
        "topic_counts": topic_counts,
        "contains_perception_targets": "/perception/targets_detailed" in topic_counts,
        "contains_raw_mid360": "/livox/lidar" in topic_counts,
        "supervisor": {
            "samples": supervisor_samples,
            "state_counts": dict(state_counts),
            "state_durations": dict(state_durations),
            "reason_counts": dict(reason_counts),
            "disabled_ratio": ratio(disabled_samples, supervisor_samples),
            "negative_margin_ratio": ratio(negative_margin_samples, supervisor_samples),
            "minimum_vehicle_margin": minimum_vehicle_margin,
            "minimum_pp_path_margin": minimum_pp_margin,
            "maximum_abs_contour_error": maximum_abs_supervisor_contour,
            "maximum_abs_heading_error": maximum_abs_supervisor_heading,
            "minimum_wheel_speed": minimum_wheel_speed,
            "maximum_wheel_speed": maximum_wheel_speed,
            "transitions": transitions,
        },
        "telemetry": {
            "samples": telemetry_samples,
            "maximum_abs_contour_error": maximum_abs_contour,
            "maximum_abs_heading_error": maximum_abs_heading,
            "maximum_abs_steering": maximum_abs_steering,
            "maximum_speed": maximum_speed,
            "minimum_mpcc_margin": minimum_mpcc_margin,
            "minimum_near_margin": minimum_near_margin,
            "maximum_track_slack": maximum_track_slack,
            "maximum_near_track_slack": maximum_near_track_slack,
            "maximum_far_track_slack": maximum_far_track_slack,
            "candidate_active_ratio": ratio(candidate_active_samples, telemetry_samples),
            "prediction_risk_ratio": ratio(prediction_risk_samples, telemetry_samples),
            "near_risk_ratio": ratio(near_risk_samples, telemetry_samples),
            "far_risk_ratio": ratio(far_risk_samples, telemetry_samples),
            "candidate_windows": candidate_windows,
            "risk_windows": risk_windows,
        },
        "command": {
            "samples": command_samples,
            "maximum_speed": maximum_command_speed,
            "maximum_abs_steering": maximum_abs_command_steering,
            "latest": latest_command,
        },
        "odometry": {
            "samples": odom_samples,
            "distance_m": distance_m,
            "maximum_speed": maximum_odom_speed,
            "latest": latest_odom,
        },
        "diagnostic_note": (
            "Telemetry candidate_active is the stable-v3 boundary/recovery candidate, "
            "not the opponent-overtake state machine."
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = analyze(args.bag)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered)
            stream.write("\n")
    else:
        print(rendered)


if __name__ == "__main__":
    main()
