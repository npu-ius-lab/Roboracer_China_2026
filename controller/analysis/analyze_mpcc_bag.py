#!/usr/bin/env python3
"""Read-only summary for a residual MPCC hardware rosbag."""

import argparse
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial import cKDTree


FIELDS = (
    "contour_error", "lag_error", "heading_error", "sideslip",
    "front_slip", "rear_slip", "published_speed", "published_steering",
    "pointlio_age", "solve_time", "failure_count", "solver_status",
    "solver_success", "track_slack", "tire_slack", "vx", "vy", "yaw_rate",
    "near_track_slack", "far_track_slack", "track_slack_stage",
    "track_slack_time", "track_slack_side", "speed_cap_used", "speed_retry",
    "warm_start_reset", "failure_kind", "rti_iterations", "speed_input_cap",
    "trigger_track_slack", "trigger_track_slack_stage",
    "trigger_track_slack_time", "trigger_track_slack_side",
    "state_prediction_horizon", "committed_horizon", "mpcc_state_lead",
    "objective", "cost_contour", "cost_lag", "cost_heading",
    "cost_speed_prior", "cost_progress", "cost_control",
    "cost_delta_control", "cost_track_slack", "cost_tire_slack",
    "geometry_theta_shift", "geometry_heading_shift",
    "geometry_curvature_shift", "candidate_active", "candidate_activated",
    "candidate_released", "prediction_risk", "mpcc_minimum_margin",
    "candidate_minimum_margin", "candidate_margin_improvement",
    "candidate_speed", "candidate_steering", "candidate_lookahead",
    "candidate_reason_code", "candidate_initial_margin",
    "candidate_final_margin", "candidate_reentry_time",
    "candidate_recovery_mode", "raw_pointlio_vx", "raw_pointlio_vy",
    "raw_pointlio_yaw_rate", "low_speed_stationary",
    "low_speed_dynamic_blend", "warm_start_advance_s",
    "warm_start_shift_stages", "skipped_solve_ticks",
)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pct(values, quantiles=(0.5, 0.95, 0.99)):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {}
    result = {f"p{int(q * 100)}": float(np.quantile(values, q)) for q in quantiles}
    result.update(min=float(np.min(values)), max=float(np.max(values)), mean=float(np.mean(values)))
    return result


def rate_summary(stamps):
    stamps = np.asarray(stamps, dtype=float)
    dt = np.diff(stamps)
    dt = dt[dt > 1e-6]
    if not len(dt):
        return {"messages": int(len(stamps)), "rate_hz": 0.0}
    return {
        "messages": int(len(stamps)),
        "rate_hz": float((len(stamps) - 1) / (stamps[-1] - stamps[0])),
        "dt_p50_s": float(np.quantile(dt, 0.5)),
        "dt_p95_s": float(np.quantile(dt, 0.95)),
        "dt_max_s": float(np.max(dt)),
    }


def intervals(stamps, mask, minimum_duration=0.05):
    stamps = np.asarray(stamps)
    mask = np.asarray(mask, dtype=bool)
    output = []
    start = None
    for index, active in enumerate(mask):
        if active and start is None:
            start = index
        if start is not None and (not active or index == len(mask) - 1):
            end = index if active and index == len(mask) - 1 else index - 1
            duration = stamps[end] - stamps[start]
            if duration >= minimum_duration:
                output.append((float(stamps[start]), float(stamps[end]), float(duration)))
            start = None
    return output


def best_delay(command_t, command, response_t, response, speed_t=None, speed=None,
               maximum=0.6, model="direct"):
    start = max(command_t[0] + maximum, response_t[0])
    end = min(command_t[-1], response_t[-1])
    grid = np.arange(start, end, 0.01)
    measured = np.interp(grid, response_t, response)
    measured_speed = np.interp(grid, speed_t, speed) if speed_t is not None else None
    best = None
    for delay in np.arange(0.0, maximum + 0.0001, 0.005):
        delayed = np.interp(grid - delay, command_t, command)
        if model == "kinematic_yaw":
            source = measured_speed * np.tan(delayed) / 0.320
            mask = (measured_speed > 0.8) & (np.abs(delayed) > 0.025)
        else:
            source = delayed
            mask = np.ones(len(grid), dtype=bool)
            if measured_speed is not None:
                mask &= measured_speed > 0.4
        if np.count_nonzero(mask) < 100:
            continue
        x = source[mask]
        y = measured[mask]
        design = np.column_stack((x, np.ones(len(x))))
        gain, bias = np.linalg.lstsq(design, y, rcond=None)[0]
        fitted = gain * x + bias
        rmse = float(np.sqrt(np.mean((y - fitted) ** 2)))
        corr = float(np.corrcoef(x, y)[0, 1]) if np.std(x) > 1e-6 else 0.0
        candidate = (rmse, -abs(corr), float(delay), float(gain), float(bias), corr)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        return {}
    return {"delay_s": best[2], "gain": best[3], "bias": best[4],
            "correlation": best[5], "rmse": best[0],
            "interpretation": "equivalent alignment, includes actuator dynamics and closed-loop correlation"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--track", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    telemetry_t, telemetry = [], []
    command_t, command_speed, command_steer = [], [], []
    gate_t, gate_speed, gate_steer = [], [], []
    loc_t, loc_recv, loc_x, loc_y, loc_yaw, loc_vx, loc_vy, loc_r = ([] for _ in range(8))
    wheel_t, wheel_x, wheel_y, wheel_yaw, wheel_vx, wheel_r = ([] for _ in range(6))
    predictions = []
    status = []

    with rosbag.Bag(str(args.bag)) as bag:
        for topic, msg, bag_stamp in bag.read_messages():
            receive = bag_stamp.to_sec()
            if topic == "/f1tenth_mpcc/telemetry":
                telemetry_t.append(receive)
                telemetry.append(list(msg.data))
            elif topic == "/f1tenth_mpcc/real/ackermann_cmd_stamped":
                command_t.append(msg.header.stamp.to_sec())
                command_speed.append(msg.drive.speed)
                command_steer.append(msg.drive.steering_angle)
            elif topic == "/tianracer/ackermann_cmd":
                gate_t.append(receive)
                gate_speed.append(msg.speed)
                gate_steer.append(msg.steering_angle)
            elif topic == "/localization/vehicle_odom":
                loc_t.append(msg.header.stamp.to_sec())
                loc_recv.append(receive)
                loc_x.append(msg.pose.pose.position.x)
                loc_y.append(msg.pose.pose.position.y)
                loc_yaw.append(yaw(msg.pose.pose.orientation))
                loc_vx.append(msg.twist.twist.linear.x)
                loc_vy.append(msg.twist.twist.linear.y)
                loc_r.append(msg.twist.twist.angular.z)
            elif topic == "/tianracer/odom":
                wheel_t.append(msg.header.stamp.to_sec())
                wheel_x.append(msg.pose.pose.position.x)
                wheel_y.append(msg.pose.pose.position.y)
                wheel_yaw.append(yaw(msg.pose.pose.orientation))
                wheel_vx.append(msg.twist.twist.linear.x)
                wheel_r.append(msg.twist.twist.angular.z)
            elif topic == "/f1tenth_mpcc/prediction":
                for marker in msg.markers:
                    if marker.id == 10 and marker.points:
                        predictions.append((marker.header.stamp.to_sec(),
                                            [(p.x, p.y) for p in marker.points]))
            elif topic == "/localization/status":
                status.append({
                    "receive": receive, "stamp": msg.header.stamp.to_sec(),
                    "state": int(msg.state), "localized": bool(msg.localized),
                    "healthy": bool(msg.frontend_healthy), "score": float(msg.registration_score),
                    "ratio": float(msg.correspondence_ratio), "residual": float(msg.mean_residual),
                    "translation_update": float(msg.map_odom_translation_update),
                    "rotation_update_deg": float(msg.map_odom_rotation_update_deg),
                    "failures": int(msg.consecutive_failures),
                    "confirmations": int(msg.relocalization_confirmations),
                    "reason": str(msg.reason),
                })

    arrays = {name: np.asarray([row[index] if index < len(row) else np.nan
                                for row in telemetry], dtype=float)
              for index, name in enumerate(FIELDS)}
    telemetry_t = np.asarray(telemetry_t)
    command_t, command_speed, command_steer = map(np.asarray, (command_t, command_speed, command_steer))
    gate_t, gate_speed, gate_steer = map(np.asarray, (gate_t, gate_speed, gate_steer))
    loc_t, loc_recv, loc_x, loc_y, loc_yaw, loc_vx, loc_vy, loc_r = map(
        np.asarray, (loc_t, loc_recv, loc_x, loc_y, loc_yaw, loc_vx, loc_vy, loc_r))
    wheel_t, wheel_x, wheel_y, wheel_yaw, wheel_vx, wheel_r = map(
        np.asarray, (wheel_t, wheel_x, wheel_y, wheel_yaw, wheel_vx, wheel_r))
    t0 = min(telemetry_t[0], command_t[0], loc_recv[0])

    track = np.genfromtxt(args.track, delimiter=",", names=True)
    tree = cKDTree(np.column_stack((track["x_m"], track["y_m"])))
    _, nearest = tree.query(np.column_stack((loc_x, loc_y)))
    nx = -np.sin(track["psi_rad"][nearest])
    ny = np.cos(track["psi_rad"][nearest])
    actual_e = (loc_x - track["x_m"][nearest]) * nx + (loc_y - track["y_m"][nearest]) * ny
    width = np.where(actual_e >= 0.0, track["w_tr_left_m"][nearest], track["w_tr_right_m"][nearest])
    actual_control_margin = width - np.abs(actual_e) - 0.12 - 0.05
    actual_physical_margin = width - np.abs(actual_e) - 0.12
    track_length = float(np.max(track["s_m"]) + np.median(np.diff(track["s_m"])))
    unwrapped_s = np.unwrap(2.0 * np.pi * track["s_m"][nearest] / track_length) * track_length / (2.0 * np.pi)
    lap_number = np.floor((unwrapped_s - unwrapped_s[0]) / track_length).astype(int)

    loc_dt = np.diff(loc_t)
    loc_distance = np.hypot(np.diff(loc_x), np.diff(loc_y))
    loc_yaw_delta = np.diff(np.unwrap(loc_yaw))
    loc_yaw_jump = np.abs(loc_yaw_delta)
    normal_loc = (loc_dt > 0.005) & (loc_dt < 0.10)
    expected_distance = 0.5 * (np.hypot(loc_vx[:-1], loc_vy[:-1]) +
                               np.hypot(loc_vx[1:], loc_vy[1:])) * loc_dt
    translation_motion_residual = np.abs(loc_distance - expected_distance)
    expected_yaw_delta = 0.5 * (loc_r[:-1] + loc_r[1:]) * loc_dt
    yaw_motion_residual = np.abs(loc_yaw_delta - expected_yaw_delta)

    wheel_dt = np.diff(wheel_t)
    dx, dy = np.diff(wheel_x), np.diff(wheel_y)
    middle_yaw = 0.5 * (np.unwrap(wheel_yaw)[1:] + np.unwrap(wheel_yaw)[:-1])
    valid_wheel = (wheel_dt > 0.005) & (wheel_dt < 0.10) & (np.hypot(dx, dy) < 0.20)
    wheel_pose_speed = np.full(len(wheel_dt), np.nan)
    wheel_pose_speed[valid_wheel] = (dx[valid_wheel] * np.cos(middle_yaw[valid_wheel]) +
                                     dy[valid_wheel] * np.sin(middle_yaw[valid_wheel])) / wheel_dt[valid_wheel]

    prediction_errors = {horizon: [] for horizon in (0.13, 0.38, 0.63, 0.88, 1.13)}
    prediction_cross_track = {horizon: [] for horizon in prediction_errors}
    for stamp, points in predictions:
        for horizon in prediction_errors:
            index = int(round((horizon - 0.13) / 0.05))
            target = stamp + horizon
            if index < 0 or index >= len(points) or target < loc_t[0] or target > loc_t[-1]:
                continue
            actual_x = np.interp(target, loc_t, loc_x)
            actual_y = np.interp(target, loc_t, loc_y)
            actual_heading = np.interp(target, loc_t, np.unwrap(loc_yaw))
            if np.interp(target, loc_t, loc_vx) < 0.5:
                continue
            px, py = points[index]
            ex, ey = px - actual_x, py - actual_y
            prediction_errors[horizon].append(math.hypot(ex, ey))
            prediction_cross_track[horizon].append(-math.sin(actual_heading) * ex + math.cos(actual_heading) * ey)

    moving = arrays["vx"] > 0.5
    success = arrays["solver_success"] > 0.5
    risk = arrays["prediction_risk"] > 0.5
    candidate_active = arrays["candidate_active"] > 0.5
    actual_outside = actual_control_margin < 0.0
    physical_outside = actual_physical_margin < 0.0

    def actual_event(stamp):
        index = int(np.clip(np.searchsorted(loc_t, stamp), 0, len(loc_t) - 1))
        return {
            "time_s": float(stamp - t0),
            "track_s_m": float(track["s_m"][nearest[index]]),
            "x_m": float(loc_x[index]), "y_m": float(loc_y[index]),
            "contour_error_m": float(actual_e[index]),
            "control_margin_m": float(actual_control_margin[index]),
            "physical_margin_m": float(actual_physical_margin[index]),
            "pointlio_vx_mps": float(loc_vx[index]),
            "pointlio_vy_mps": float(loc_vy[index]),
            "pointlio_yaw_rate_radps": float(loc_r[index]),
            "command_speed_mps": float(np.interp(stamp, command_t, command_speed)),
            "command_steering_rad": float(np.interp(stamp, command_t, command_steer)),
        }

    selected_minima = []
    for index in np.argsort(actual_control_margin):
        if loc_vx[index] <= 0.5:
            continue
        if all(abs(loc_t[index] - loc_t[other]) > 0.75 for other in selected_minima):
            selected_minima.append(int(index))
        if len(selected_minima) == 10:
            break

    solver_failure_events = []
    for index in np.flatnonzero(~success):
        event = actual_event(telemetry_t[index])
        event.update({
            "solve_time_ms": float(1000.0 * arrays["solve_time"][index]),
            "acados_status": int(arrays["solver_status"][index]),
            "track_slack_m": float(arrays["track_slack"][index]),
            "prediction_risk": bool(risk[index]),
            "candidate_active": bool(candidate_active[index]),
        })
        solver_failure_events.append(event)

    candidate_activation_events = []
    for index in np.flatnonzero(arrays["candidate_activated"] > 0.5):
        event = actual_event(telemetry_t[index])
        event.update({
            "mpcc_predicted_margin_m": float(arrays["mpcc_minimum_margin"][index]),
            "candidate_predicted_margin_m": float(arrays["candidate_minimum_margin"][index]),
            "candidate_command_speed_mps": float(arrays["candidate_speed"][index]),
            "candidate_command_steering_rad": float(arrays["candidate_steering"][index]),
            "reason_code": int(arrays["candidate_reason_code"][index]),
        })
        candidate_activation_events.append(event)

    lap_summaries = []
    for lap in range(int(np.min(lap_number)), int(np.max(lap_number)) + 1):
        mask = lap_number == lap
        indices = np.flatnonzero(mask)
        if len(indices) < 10:
            continue
        start, end = indices[0], indices[-1]
        lap_summaries.append({
            "lap_index_from_record_start": int(lap),
            "duration_s": float(loc_t[end] - loc_t[start]),
            "progress_m": float(unwrapped_s[end] - unwrapped_s[start]),
            "mean_vx_mps": float(np.mean(loc_vx[mask])),
            "max_vx_mps": float(np.max(loc_vx[mask])),
            "contour_error_rms_m": float(np.sqrt(np.mean(actual_e[mask] ** 2))),
            "absolute_contour_error_p95_m": float(np.quantile(np.abs(actual_e[mask]), 0.95)),
            "minimum_control_margin_m": float(np.min(actual_control_margin[mask])),
            "minimum_physical_margin_m": float(np.min(actual_physical_margin[mask])),
            "candidate_activations": int(sum(
                loc_t[start] <= telemetry_t[index] <= loc_t[end]
                for index in np.flatnonzero(arrays["candidate_activated"] > 0.5))),
        })

    gate_speed_error = np.max(np.abs(gate_speed - np.interp(gate_t, command_t, command_speed)))
    gate_steer_error = np.max(np.abs(gate_steer - np.interp(gate_t, command_t, command_steer)))

    summary = {
        "bag": str(args.bag.resolve()),
        "duration_s": float(max(telemetry_t[-1], command_t[-1], loc_recv[-1]) - t0),
        "rates": {
            "mpcc_telemetry": rate_summary(telemetry_t),
            "mpcc_command": rate_summary(command_t),
            "hardware_gate_command": rate_summary(gate_t),
            "pointlio_vehicle_odom": rate_summary(loc_t),
            "wheel_odom": rate_summary(wheel_t),
        },
        "solver": {
            "status_counts": {str(k): int(v) for k, v in Counter(arrays["solver_status"].astype(int)).items()},
            "successful": int(np.count_nonzero(success)),
            "failed": int(np.count_nonzero(~success)),
            "solve_time_s": pct(arrays["solve_time"]),
            "deadline_misses_over_25ms": int(np.count_nonzero(arrays["solve_time"] > 0.025)),
            "maximum_failure_streak": int(np.nanmax(arrays["failure_count"])),
            "speed_retries": int(np.count_nonzero(arrays["speed_retry"] > 0.5)),
            "warm_start_resets": int(np.count_nonzero(arrays["warm_start_reset"] > 0.5)),
            "skipped_solve_ticks_final": int(arrays["skipped_solve_ticks"][-1]),
            "warm_start_advance_s": pct(arrays["warm_start_advance_s"]),
            "warm_start_shift_stages": pct(arrays["warm_start_shift_stages"]),
            "failure_events": solver_failure_events,
        },
        "tracking": {
            "moving_samples": int(np.count_nonzero(moving)),
            "absolute_contour_error_m": pct(np.abs(arrays["contour_error"][moving])),
            "signed_contour_error_m": pct(arrays["contour_error"][moving]),
            "absolute_heading_error_rad": pct(np.abs(arrays["heading_error"][moving])),
            "absolute_sideslip_rad": pct(np.abs(arrays["sideslip"][moving])),
            "actual_localization_contour_error_m": pct(actual_e[loc_vx > 0.5]),
            "actual_control_corridor_margin_m": pct(actual_control_margin[loc_vx > 0.5]),
            "actual_physical_boundary_margin_m": pct(actual_physical_margin[loc_vx > 0.5]),
            "control_corridor_outside_samples": int(np.count_nonzero(actual_outside & (loc_vx > 0.5))),
            "physical_boundary_outside_samples": int(np.count_nonzero(physical_outside & (loc_vx > 0.5))),
            "control_corridor_outside_intervals": [
                {"start_s": a - t0, "end_s": b - t0, "duration_s": d}
                for a, b, d in intervals(loc_t, actual_outside & (loc_vx > 0.5))
            ],
            "ten_lowest_separated_margin_events": [actual_event(loc_t[index]) for index in selected_minima],
            "laps": lap_summaries,
        },
        "safety": {
            "prediction_risk_samples": int(np.count_nonzero(risk)),
            "speed_retry_samples": int(np.count_nonzero(arrays["speed_retry"] > 0.5)),
            "candidate_active_samples": int(np.count_nonzero(candidate_active)),
            "candidate_activations": int(np.count_nonzero(arrays["candidate_activated"] > 0.5)),
            "candidate_releases": int(np.count_nonzero(arrays["candidate_released"] > 0.5)),
            "mpcc_minimum_margin_m": pct(arrays["mpcc_minimum_margin"][moving]),
            "track_slack_m": pct(arrays["track_slack"]),
            "risk_intervals": [
                {"start_s": a - t0, "end_s": b - t0, "duration_s": d}
                for a, b, d in intervals(telemetry_t, risk)
            ],
            "candidate_intervals": [
                {"start_s": a - t0, "end_s": b - t0, "duration_s": d}
                for a, b, d in intervals(telemetry_t, candidate_active)
            ],
            "candidate_activation_events": candidate_activation_events,
        },
        "speed_and_delay": {
            "command_speed_mps": pct(command_speed),
            "pointlio_vx_mps": pct(loc_vx),
            "wheel_twist_vx_mps": pct(wheel_vx),
            "wheel_pose_derived_speed_mps": pct(wheel_pose_speed),
            "pointlio_receive_age_s": pct(loc_recv - loc_t),
            "telemetry_pointlio_age_s": pct(arrays["pointlio_age"]),
            "longitudinal_equivalent_alignment": best_delay(
                command_t, command_speed, loc_t, loc_vx,
                speed_t=loc_t, speed=loc_vx, maximum=0.8),
            "steering_to_yaw_equivalent_alignment": best_delay(
                command_t, command_steer, loc_t, loc_r,
                speed_t=loc_t, speed=loc_vx, maximum=0.4, model="kinematic_yaw"),
            "wheel_to_pointlio_speed_alignment": best_delay(
                wheel_t, wheel_vx, loc_t, loc_vx,
                speed_t=loc_t, speed=loc_vx, maximum=0.4),
            "wheel_minus_pointlio_vx_mps": pct(
                np.interp(loc_t, wheel_t, wheel_vx) - loc_vx),
            "gate_max_speed_difference": float(gate_speed_error),
            "gate_max_steering_difference": float(gate_steer_error),
        },
        "prediction_vs_future_localization": {
            f"{horizon:.2f}s": {
                "position_error_m": pct(prediction_errors[horizon]),
                "signed_vehicle_lateral_error_m": pct(prediction_cross_track[horizon]),
                "samples": len(prediction_errors[horizon]),
            } for horizon in prediction_errors
        },
        "localization": {
            "translation_step_m": pct(loc_distance[normal_loc]),
            "yaw_step_deg": pct(np.degrees(loc_yaw_jump[normal_loc])),
            "translation_motion_residual_m": pct(translation_motion_residual[normal_loc]),
            "yaw_motion_residual_deg": pct(np.degrees(yaw_motion_residual[normal_loc])),
            "motion_residual_steps_over_0.08m": int(np.count_nonzero(
                normal_loc & (translation_motion_residual > 0.08))),
            "motion_residual_steps_over_5deg": int(np.count_nonzero(
                normal_loc & (yaw_motion_residual > math.radians(5.0)))),
            "status_state_counts": dict(Counter(str(row["state"]) for row in status)),
            "unlocalized_samples": int(sum(not row["localized"] for row in status)),
            "unhealthy_samples": int(sum(not row["healthy"] for row in status)),
            "consecutive_failures_max": max((row["failures"] for row in status), default=0),
            "relocalization_confirmations_max": max((row["confirmations"] for row in status), default=0),
            "map_odom_translation_update_m": pct([row["translation_update"] for row in status]),
            "map_odom_rotation_update_deg": pct([abs(row["rotation_update_deg"]) for row in status]),
            "reason_counts": dict(Counter(row["reason"] for row in status)),
        },
    }

    output = json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True)
    print(output)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n")


if __name__ == "__main__":
    main()
