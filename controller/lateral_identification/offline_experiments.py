#!/usr/bin/env python3
"""Reproducible offline steering-actuator experiments for TianRacer bags.

This tool deliberately does not edit controller or vehicle configuration.  It
audits the recorded streams, compares static-map hypotheses, profiles the
FOPDT actuator parameters, and evaluates candidates on bags held out from the
joint fit.  Later experiment stages append multi-step rollout and controller
simulation results to the same report.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy.optimize import least_squares

try:
    from .analyze_bag import message_stamp, read_bag, synchronized
    from .fit_core import (
        ActuatorFit,
        PreparedActuatorRun,
        evaluate_steering_actuator,
        first_order_response,
        fit_steering_actuator_joint,
        fit_tire_stiffness,
        prepare_actuator_run,
        smooth,
        zoh,
    )
except ImportError:  # Direct execution from lateral_identification/.
    from analyze_bag import message_stamp, read_bag, synchronized
    from fit_core import (
        ActuatorFit,
        PreparedActuatorRun,
        evaluate_steering_actuator,
        first_order_response,
        fit_steering_actuator_joint,
        fit_tire_stiffness,
        prepare_actuator_run,
        smooth,
        zoh,
    )


PRIMARY_PROFILES = (
    "track_static_060",
    "track_prbs_060",
    "track_prbs_100",
    "track_prbs_150",
)
TIMING_TOPICS = (
    "/f1tenth_identification/command_stamped",
    "/f1tenth_mpcc/real/ackermann_cmd_stamped",
    "/livox/imu",
    "/tianracer/imu",
    "/localization/vehicle_odom",
    "/tianracer/odom",
)


def finite(value):
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(item) for item in value]
    if isinstance(value, np.ndarray):
        return finite(value.tolist())
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def percentile_summary(values: Iterable[float]) -> Dict[str, float]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if len(array) == 0:
        return {"minimum": float("nan"), "median": float("nan"),
                "p95": float("nan"), "maximum": float("nan")}
    return {
        "minimum": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.quantile(array, 0.95)),
        "maximum": float(np.max(array)),
    }


def quaternion_yaw(quaternion) -> float:
    x, y, z, w = (
        float(quaternion.x), float(quaternion.y),
        float(quaternion.z), float(quaternion.w),
    )
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def longest_true_duration(mask: np.ndarray, dt: float) -> float:
    longest = current = 0
    for value in np.asarray(mask, dtype=bool):
        current = current + 1 if value else 0
        longest = max(longest, current)
    return float(longest * dt)


def contiguous_regions(mask: np.ndarray, minimum_samples: int = 1) -> List[slice]:
    mask = np.asarray(mask, dtype=bool)
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    return [slice(int(start), int(end)) for start, end in zip(starts, ends)
            if end - start >= minimum_samples]


def load_metadata(path: Path) -> Dict[str, object]:
    candidate = path.with_suffix(".metadata.json")
    if not candidate.exists():
        return {}
    return json.loads(candidate.read_text(encoding="utf-8"))


def load_json_lines(path: Path) -> List[Dict[str, object]]:
    if not path.exists():
        return []
    return [
        json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip().startswith("{")
    ]


def select_bags(data_dir: Path) -> Tuple[List[Path], List[Path], Dict[str, object]]:
    """Select one primary bag per profile and keep all others as holdouts."""
    records = []
    for path in sorted(data_dir.glob("*.bag")):
        metadata = load_metadata(path)
        profile = str(metadata.get("profile", ""))
        if profile not in PRIMARY_PROFILES:
            continue
        records.append((path, metadata, profile))
    primary, used = [], set()
    selection = {}
    for profile in PRIMARY_PROFILES:
        candidates = [record for record in records if record[2] == profile]
        if not candidates:
            raise ValueError("missing required profile: " + profile)
        # Prefer a completed run. Within the same completion class use the
        # greatest driven distance, then the newest file.
        selected = max(
            candidates,
            key=lambda record: (
                bool(record[1].get("completed", False)),
                float(record[1].get("completed_distance_m", 0.0)),
                record[0].stat().st_mtime,
            ),
        )
        primary.append(selected[0])
        used.add(selected[0].resolve())
        selection[profile] = {
            "selected": str(selected[0].resolve()),
            "completed": bool(selected[1].get("completed", False)),
            "completed_distance_m": float(
                selected[1].get("completed_distance_m", 0.0)
            ),
            "alternatives": [str(item[0].resolve()) for item in candidates
                             if item[0] != selected[0]],
        }
    holdouts = [record[0] for record in records
                if record[0].resolve() not in used]
    return primary, holdouts, selection


def read_auxiliary(path: Path) -> Dict[str, object]:
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError("ROS rosbag Python module is required: " + str(error))

    timing = defaultdict(lambda: {"stamps": [], "receipts": []})
    pose, wheel = [], []
    with rosbag.Bag(str(path)) as bag:
        bag_start, bag_end = bag.get_start_time(), bag.get_end_time()
        for topic, message, receipt in bag.read_messages(
                topics=list(TIMING_TOPICS)):
            receive = float(receipt.to_sec())
            stamp = float(message_stamp(message, receipt))
            timing[topic]["stamps"].append(stamp)
            timing[topic]["receipts"].append(receive)
            if topic == "/localization/vehicle_odom":
                position = message.pose.pose.position
                pose.append((
                    stamp, receive, float(position.x), float(position.y),
                    quaternion_yaw(message.pose.pose.orientation),
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.linear.y),
                    float(message.twist.twist.angular.z),
                ))
            elif topic == "/tianracer/odom":
                wheel.append((
                    stamp, receive,
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.angular.z),
                ))
    return {
        "bag_start": float(bag_start), "bag_end": float(bag_end),
        "timing": timing,
        "pose": np.asarray(pose, dtype=float),
        "wheel": np.asarray(wheel, dtype=float),
    }


def timing_summary(stamps: Sequence[float], receipts: Sequence[float]) -> Dict[str, object]:
    stamps = np.asarray(stamps, dtype=float)
    receipts = np.asarray(receipts, dtype=float)
    if len(stamps) < 2:
        return {"samples": int(len(stamps)), "rate_hz": float("nan")}
    receipt_dt = np.diff(receipts)
    stamp_dt = np.diff(stamps)
    span = max(float(receipts[-1] - receipts[0]), 1.0e-9)
    return {
        "samples": int(len(stamps)),
        "rate_hz": float((len(stamps) - 1) / span),
        "receipt_gap_s": percentile_summary(receipt_dt),
        "header_age_s": percentile_summary(receipts - stamps),
        "timestamp_regressions_or_duplicates": int(np.sum(stamp_dt <= 0.0)),
    }


def pose_quality(pose: np.ndarray) -> Dict[str, object]:
    if len(pose) < 3:
        return {"samples": int(len(pose)), "usable": False}
    stamp = pose[:, 0]
    dt = np.diff(stamp)
    distance = np.hypot(np.diff(pose[:, 2]), np.diff(pose[:, 3]))
    yaw_delta = np.diff(np.unwrap(pose[:, 4]))
    expected_distance = 0.5 * (
        np.hypot(pose[:-1, 5], pose[:-1, 6])
        + np.hypot(pose[1:, 5], pose[1:, 6])
    ) * np.maximum(dt, 0.0)
    expected_yaw = 0.5 * (pose[:-1, 7] + pose[1:, 7]) * np.maximum(dt, 0.0)
    valid = (dt > 0.005) & (dt < 0.10)
    translation_residual = np.abs(distance - expected_distance)
    yaw_residual = np.abs(yaw_delta - expected_yaw)
    return {
        "samples": int(len(pose)),
        "usable": bool(np.sum(valid) > 20),
        "positive_dt_fraction": float(np.mean(dt > 0.0)),
        "translation_step_m": percentile_summary(distance[valid]),
        "yaw_step_rad": percentile_summary(np.abs(yaw_delta[valid])),
        "translation_motion_residual_m": percentile_summary(
            translation_residual[valid]
        ),
        "yaw_motion_residual_rad": percentile_summary(yaw_residual[valid]),
        "translation_jump_events": int(np.sum(valid & (translation_residual > 0.08))),
        "yaw_jump_events": int(np.sum(valid & (yaw_residual > 0.12))),
    }


def audit_bag(path: Path, sync: Dict[str, np.ndarray], auxiliary: Dict[str, object]) -> Dict[str, object]:
    times = sync["times"]
    dt = float(np.median(np.diff(times)))
    vx = sync["vx"]
    moving = vx >= 0.45
    stopped = np.abs(vx) < 0.20
    metadata = load_metadata(path)
    topics = {
        topic: timing_summary(values["stamps"], values["receipts"])
        for topic, values in auxiliary["timing"].items()
    }
    return {
        "bag": str(path.resolve()),
        "profile": str(metadata.get("profile", "")),
        "completed": bool(metadata.get("completed", False)),
        "abort_reason": str(metadata.get("abort_reason", "")),
        "bag_duration_s": float(auxiliary["bag_end"] - auxiliary["bag_start"]),
        "overlap_duration_s": float(times[-1] - times[0]),
        "moving_duration_s": float(np.sum(moving) * dt),
        "stopped_duration_s": float(np.sum(stopped) * dt),
        "longest_stopped_interval_s": longest_true_duration(stopped, dt),
        "vx_mps": {
            "p05": float(np.quantile(vx, 0.05)),
            "median": float(np.median(vx)),
            "p95": float(np.quantile(vx, 0.95)),
        },
        "pointlio_receive_age_s": percentile_summary(sync["pointlio_age"]),
        "topics": topics,
        "pose_quality": pose_quality(auxiliary["pose"]),
        "metadata": {
            "completed_distance_m": float(metadata.get("completed_distance_m", 0.0)),
            "excitation_active_time_s": float(metadata.get("excitation_active_time_s", 0.0)),
            "excitation_transition_count": int(metadata.get("excitation_transition_count", 0)),
            "minimum_physical_margin_m": float(metadata.get("minimum_physical_margin_m", float("nan"))),
            "minimum_predicted_margin_m": float(metadata.get("minimum_predicted_margin_m", float("nan"))),
            "control_loop": metadata.get("control_loop", {}),
            "safety_predictor": metadata.get("safety_predictor", {}),
        },
    }


def static_model_value(kind: str, parameters: np.ndarray, command: np.ndarray) -> np.ndarray:
    command = np.asarray(command, dtype=float)
    if kind == "linear":
        gain, bias = parameters
        return gain * command + bias
    if kind == "split_gain":
        gain_negative, gain_positive, bias = parameters
        gain = np.where(command < 0.0, gain_negative, gain_positive)
        return gain * command + bias
    if kind == "deadband":
        gain, bias, deadband = parameters
        effective = np.sign(command) * np.maximum(np.abs(command) - deadband, 0.0)
        return gain * effective + bias
    raise ValueError("unknown static model: " + kind)


def fit_static_model(kind: str, command: np.ndarray, target: np.ndarray) -> Dict[str, object]:
    command = np.asarray(command, dtype=float)
    target = np.asarray(target, dtype=float)
    if kind == "linear":
        seed, lower, upper = [1.0, 0.0], [0.2, -0.15], [2.5, 0.15]
        names = ("gain", "bias_rad")
    elif kind == "split_gain":
        seed, lower, upper = [1.0, 1.0, 0.0], [0.2, 0.2, -0.15], [2.5, 2.5, 0.15]
        names = ("gain_negative", "gain_positive", "bias_rad")
    elif kind == "deadband":
        seed, lower, upper = [1.0, 0.0, 0.01], [0.2, -0.15, 0.0], [2.5, 0.15, 0.08]
        names = ("gain", "bias_rad", "deadband_rad")
    else:
        raise ValueError("unknown static model: " + kind)
    result = least_squares(
        lambda p: static_model_value(kind, p, command) - target,
        np.asarray(seed), bounds=(np.asarray(lower), np.asarray(upper)),
        loss="soft_l1", f_scale=0.015,
    )
    prediction = static_model_value(kind, result.x, command)
    residual = prediction - target
    rss = max(float(np.sum(np.square(residual))), 1.0e-15)
    count, parameter_count = len(command), len(result.x)
    aicc = count * math.log(rss / max(count, 1)) + 2.0 * parameter_count
    if count > parameter_count + 1:
        aicc += 2.0 * parameter_count * (parameter_count + 1) / (
            count - parameter_count - 1
        )
    return {
        "kind": kind,
        "parameters": {name: float(value) for name, value in zip(names, result.x)},
        "rmse_rad": float(np.sqrt(np.mean(np.square(residual)))),
        "maximum_abs_error_rad": float(np.max(np.abs(residual))),
        "aicc": float(aicc),
        "parameter_vector": result.x,
    }


def leave_one_plateau_out(kind: str, command: np.ndarray, target: np.ndarray) -> float:
    errors = []
    minimum = 5 if kind != "linear" else 4
    if len(command) < minimum:
        return float("nan")
    for held in range(len(command)):
        keep = np.arange(len(command)) != held
        try:
            fit = fit_static_model(kind, command[keep], target[keep])
            prediction = static_model_value(
                kind, np.asarray(fit["parameter_vector"]), command[[held]]
            )[0]
            errors.append(prediction - target[held])
        except (ValueError, RuntimeError):
            return float("nan")
    return float(np.sqrt(np.mean(np.square(errors))))


def static_mapping_experiment(run: PreparedActuatorRun) -> Dict[str, object]:
    regions = contiguous_regions(run.static_mask, minimum_samples=8)
    plateau_command, plateau_target, plateau_samples = [], [], []
    for region in regions:
        plateau_command.append(float(np.median(run.command[region])))
        plateau_target.append(float(np.median(run.delta_proxy[region])))
        plateau_samples.append(int(region.stop - region.start))
    command = np.asarray(plateau_command, dtype=float)
    target = np.asarray(plateau_target, dtype=float)
    if len(command) < 6:
        raise ValueError("not enough stable plateaus for static-map experiment")
    models = []
    for kind in ("linear", "split_gain", "deadband"):
        fit = fit_static_model(kind, command, target)
        fit["leave_one_plateau_out_rmse_rad"] = leave_one_plateau_out(
            kind, command, target
        )
        fit.pop("parameter_vector")
        models.append(fit)
    linear = next(item for item in models if item["kind"] == "linear")
    best_cv = min(
        models,
        key=lambda item: item["leave_one_plateau_out_rmse_rad"]
        if np.isfinite(item["leave_one_plateau_out_rmse_rad"]) else float("inf"),
    )
    # Hysteresis proxy: compare affine residuals while plateau command moves in
    # opposite directions. This is only an effective-motion diagnostic.
    linear_parameters = np.asarray([
        linear["parameters"]["gain"], linear["parameters"]["bias_rad"]
    ])
    residual = static_model_value("linear", linear_parameters, command) - target
    direction = np.sign(np.r_[0.0, np.diff(command)])
    rising, falling = residual[direction > 0.0], residual[direction < 0.0]
    hysteresis = (
        float(abs(np.mean(rising) - np.mean(falling)))
        if len(rising) >= 2 and len(falling) >= 2 else float("nan")
    )
    improvement = (
        1.0 - best_cv["leave_one_plateau_out_rmse_rad"]
        / max(linear["leave_one_plateau_out_rmse_rad"], 1.0e-12)
    )
    balanced = int(np.sum(command < -0.02)) >= 3 and int(np.sum(command > 0.02)) >= 3
    recommendation = "linear"
    if best_cv["kind"] != "linear" and improvement > 0.15 and balanced:
        recommendation = best_cv["kind"]
    return {
        "source_run": run.name,
        "plateau_count": int(len(command)),
        "plateau_samples": plateau_samples,
        "command_range_rad": [float(np.min(command)), float(np.max(command))],
        "negative_plateaus": int(np.sum(command < -0.02)),
        "positive_plateaus": int(np.sum(command > 0.02)),
        "near_zero_plateaus": int(np.sum(np.abs(command) <= 0.02)),
        "plateau_command_rad": command,
        "plateau_equivalent_angle_rad": target,
        "models": models,
        "hysteresis_proxy_rad": hysteresis,
        "directional_coverage_sufficient": balanced,
        "identifiability": (
            "two_sided" if balanced else "one_sided_total_command_only"
        ),
        "recommended_map_type": recommendation,
        "nonlinear_cv_improvement_fraction": float(improvement),
        "warning": (
            "Equivalent angle comes from yaw response; no steering-angle sensor is installed. "
            + ("" if balanced else
               "Negative total-command plateaus are absent, so left/right gain, deadband and true hysteresis are not identifiable.")
        ),
        "hysteresis_interpretation": (
            "The rising/falling residual difference is confounded by track location and closed-loop baseline steering; it is not a mechanical hysteresis estimate."
        ),
    }


def actuator_from_dict(values: Dict[str, object], source: str) -> ActuatorFit:
    return ActuatorFit(
        gain=float(values["static_gain"]),
        bias=float(values["offset_rad"]),
        tau=float(values["time_constant_s"]),
        delay=float(values["dead_time_s"]),
        static_rmse=float(values.get("static_delta_rmse_rad", float("nan"))),
        dynamic_rmse=float(values.get("dynamic_delta_rmse_rad", float("nan"))),
        dynamic_r2=float(values.get("dynamic_r2", float("nan"))),
        samples=int(values.get("samples", 0)),
        static_samples=int(values.get("static_samples", 0)),
        source=source,
    )


def dynamic_experiment(
    primary_runs: Sequence[PreparedActuatorRun],
    holdout_runs: Sequence[PreparedActuatorRun],
) -> Tuple[Dict[str, object], Dict[str, np.ndarray]]:
    joint, diagnostics = fit_steering_actuator_joint(primary_runs)
    nominal = ActuatorFit(
        1.0, 0.0, 0.08, 0.0, float("nan"), float("nan"),
        float("nan"), 0, 0, "stable_nominal",
    )
    historical = ActuatorFit(
        1.147, -0.015, 0.073, 0.090, float("nan"), float("nan"),
        float("nan"), 0, 0, "historical_candidate",
    )
    models = (nominal, historical, replace(joint, source="joint_candidate"))
    per_run = []
    individual = []
    for role, run in [*(('primary', item) for item in primary_runs),
                      *(('holdout', item) for item in holdout_runs)]:
        fitted, _ = fit_steering_actuator_joint([run])
        individual.append({
            "name": run.name, "role": role, **fitted.as_dict(),
            "effective_63pct_time_s": float(fitted.delay + fitted.tau),
        })
        comparisons = {}
        nominal_rmse = None
        for model in models:
            metrics, _ = evaluate_steering_actuator(run, model)
            comparisons[model.source] = metrics
            if model.source == "stable_nominal":
                nominal_rmse = metrics["rmse_rad"]
        for metrics in comparisons.values():
            metrics["rmse_improvement_vs_stable_percent"] = float(
                100.0 * (nominal_rmse - metrics["rmse_rad"])
                / max(nominal_rmse, 1.0e-12)
            )
        per_run.append({
            "name": run.name, "role": role,
            "speed_mps": {
                "p05": float(np.quantile(run.vx[run.dynamic_mask], 0.05)),
                "median": float(np.median(run.vx[run.dynamic_mask])),
                "p95": float(np.quantile(run.vx[run.dynamic_mask], 0.95)),
            },
            "models": comparisons,
        })

    leave_one_out = []
    for held_index, held in enumerate(primary_runs):
        training = [run for index, run in enumerate(primary_runs)
                    if index != held_index]
        fitted, _ = fit_steering_actuator_joint(training)
        metrics, _ = evaluate_steering_actuator(held, fitted)
        leave_one_out.append({
            "held_out": held.name,
            "fit": fitted.as_dict(),
            "metrics": metrics,
            "strict_rmse_pass": bool(metrics["rmse_rad"] < 0.020),
            "r2_pass": bool(metrics["r2"] > 0.90),
        })

    delay_profile = []
    for delay in np.arange(0.055, 0.1501, 0.005):
        fitted, _ = fit_steering_actuator_joint(
            primary_runs, delay_grid=np.asarray([delay])
        )
        delay_profile.append({
            "dead_time_s": float(delay), "time_constant_s": fitted.tau,
            "gain": fitted.gain, "bias_rad": fitted.bias,
            "rmse_rad": fitted.dynamic_rmse,
            "effective_63pct_time_s": float(delay + fitted.tau),
        })
    best_score = min(item["rmse_rad"] for item in delay_profile)
    for item in delay_profile:
        item["relative_rmse_increase_percent"] = float(
            100.0 * (item["rmse_rad"] / best_score - 1.0)
        )
    near_optimal = [item for item in delay_profile
                    if item["relative_rmse_increase_percent"] <= 1.0]

    rate_runs = []
    for run in primary_runs:
        dt = float(np.median(np.diff(run.times)))
        filtered = smooth(run.delta_proxy, dt, 0.08)
        rate = np.gradient(filtered, dt)
        target = np.clip(joint.gain * run.command + joint.bias, -0.55, 0.55)
        driven = run.dynamic_mask & (np.abs(target - filtered) >= 0.02)
        positive = rate[driven & (target > filtered) & (rate > 0.0)]
        negative = -rate[driven & (target < filtered) & (rate < 0.0)]
        rate_runs.append({
            "name": run.name,
            "driven_samples": int(np.sum(driven)),
            "positive_effective_rate_radps": percentile_summary(positive),
            "negative_effective_rate_radps": percentile_summary(negative),
            "maximum_target_error_rad": float(np.max(np.abs(target - filtered))),
        })

    primary_parameters = [item for item in individual if item["role"] == "primary"]
    parameter_spread = {}
    for name in ("static_gain", "offset_rad", "dead_time_s", "time_constant_s",
                 "effective_63pct_time_s"):
        values = np.asarray([float(item[name]) for item in primary_parameters])
        parameter_spread[name] = {
            "minimum": float(np.min(values)), "maximum": float(np.max(values)),
            "range": float(np.ptp(values)), "median": float(np.median(values)),
            "relative_range_fraction": float(
                np.ptp(values) / max(abs(np.median(values)), 1.0e-12)
            ),
        }
    consistency = {
        "gain_range_le_5pct": bool(
            parameter_spread["static_gain"]["relative_range_fraction"] <= 0.05
        ),
        "bias_range_le_0p01rad": bool(
            parameter_spread["offset_rad"]["range"] <= 0.01
        ),
        "delay_range_le_0p03s": bool(
            parameter_spread["dead_time_s"]["range"] <= 0.03
        ),
        "tau_range_le_25pct": bool(
            parameter_spread["time_constant_s"]["relative_range_fraction"] <= 0.25
        ),
        "effective_time_range_le_10pct": bool(
            parameter_spread["effective_63pct_time_s"]["relative_range_fraction"] <= 0.10
        ),
    }
    consistency["all_primary_parameter_gates"] = bool(all(consistency.values()))
    return {
        "joint_candidate": joint.as_dict(),
        "individual_fits": individual,
        "parameter_spread_primary": parameter_spread,
        "parameter_consistency_gates": consistency,
        "per_run_model_comparison": per_run,
        "leave_one_primary_bag_out": leave_one_out,
        "delay_tau_profile": delay_profile,
        "near_optimal_delay_tau_ridge": near_optimal,
        "observed_effective_steering_rate": rate_runs,
        "rate_limit_identifiable": False,
        "rate_limit_note": (
            "Small PRBS amplitudes identify lag but do not repeatedly drive the servo into a hard rate limit; reported rates are lower-bound effective motion rates."
        ),
    }, {
        "joint_predictions": diagnostics["predictions"],
    }


def split_gain_response(
    run: PreparedActuatorRun,
    gain_negative: float,
    gain_positive: float,
    bias: float,
    tau: float,
    delay: float,
) -> Tuple[np.ndarray, np.ndarray]:
    delayed = zoh(run.times - delay, run.times, run.command)
    gain = np.where(delayed < 0.0, gain_negative, gain_positive)
    target = gain * delayed + bias
    prediction = first_order_response(
        run.times, run.times, target, 1.0, tau, 0.0, 0.0,
        run.delta_proxy[0],
    )
    return prediction, delayed


def fit_directional_split_gain(
    runs: Sequence[PreparedActuatorRun],
    delay: float,
    seed: Sequence[float] = (1.05, 1.11, -0.011, 0.067),
) -> Dict[str, float]:
    runs = list(runs)
    reference_count = float(np.mean([np.sum(run.dynamic_mask) for run in runs]))

    def residual(parameters: np.ndarray) -> np.ndarray:
        gain_negative, gain_positive, bias, tau = parameters
        parts = []
        for run in runs:
            prediction, _ = split_gain_response(
                run, gain_negative, gain_positive, bias, tau, delay
            )
            values = prediction[run.dynamic_mask] - run.delta_proxy[run.dynamic_mask]
            parts.append(values * math.sqrt(
                reference_count / max(int(np.sum(run.dynamic_mask)), 1)
            ))
        # Weak symmetry prior prevents the smaller negative subset from
        # claiming a large directional effect unsupported by motion data.
        parts.append(np.asarray([(gain_negative - gain_positive) / 0.30]))
        return np.concatenate(parts)

    fit = least_squares(
        residual, np.asarray(seed, dtype=float),
        bounds=(np.asarray([0.60, 0.60, -0.08, 0.020]),
                np.asarray([1.50, 1.50, 0.05, 0.160])),
        loss="soft_l1", f_scale=0.02, max_nfev=80,
    )
    values = residual(fit.x)
    return {
        "gain_negative": float(fit.x[0]),
        "gain_positive": float(fit.x[1]),
        "offset_rad": float(fit.x[2]),
        "time_constant_s": float(fit.x[3]),
        "dead_time_s": float(delay),
        "gain_asymmetry_fraction": float(
            (fit.x[1] - fit.x[0])
            / max(0.5 * (fit.x[1] + fit.x[0]), 1.0e-12)
        ),
        "group_balanced_objective_rmse_rad": float(
            np.sqrt(np.mean(np.square(values[:-1])))
        ),
        "optimizer_evaluations": int(fit.nfev),
    }


def directional_mapping_experiment(
    primary_runs: Sequence[PreparedActuatorRun],
    holdout_runs: Sequence[PreparedActuatorRun],
    single_candidate: Dict[str, object],
) -> Dict[str, object]:
    delay = float(single_candidate["dead_time_s"])
    split = fit_directional_split_gain(primary_runs, delay)
    single = ActuatorFit(
        gain=float(single_candidate["static_gain"]),
        bias=float(single_candidate["offset_rad"]),
        tau=float(single_candidate["time_constant_s"]),
        delay=delay, static_rmse=float("nan"), dynamic_rmse=float("nan"),
        dynamic_r2=float("nan"), samples=0, static_samples=0,
        source="multistep_single_gain",
    )
    per_run = []
    for role, run in [
        *(("primary", item) for item in primary_runs),
        *(("holdout", item) for item in holdout_runs),
    ]:
        single_prediction = first_order_response(
            run.times, run.times, run.command, single.gain, single.tau,
            single.delay, single.bias, run.delta_proxy[0],
        )
        split_prediction, delayed = split_gain_response(
            run, split["gain_negative"], split["gain_positive"],
            split["offset_rad"], split["time_constant_s"], delay,
        )
        model_metrics = {}
        for name, prediction in (
            ("single_gain", single_prediction), ("split_gain", split_prediction)
        ):
            by_direction = {}
            for direction, direction_mask in (
                ("negative", delayed < -0.02),
                ("positive", delayed > 0.02),
                ("all", np.ones(len(delayed), dtype=bool)),
            ):
                mask = run.dynamic_mask & direction_mask
                by_direction[direction] = rollout_metric(
                    prediction[mask] - run.delta_proxy[mask]
                )
            model_metrics[name] = by_direction
        per_run.append({
            "name": run.name, "role": role,
            "negative_dynamic_fraction": float(np.mean(
                delayed[run.dynamic_mask] < -0.02
            )),
            "models": model_metrics,
        })

    leave_one_out = []
    for held_index, held in enumerate(primary_runs):
        training = [run for index, run in enumerate(primary_runs)
                    if index != held_index]
        fitted = fit_directional_split_gain(training, delay)
        prediction, delayed = split_gain_response(
            held, fitted["gain_negative"], fitted["gain_positive"],
            fitted["offset_rad"], fitted["time_constant_s"], delay,
        )
        mask = held.dynamic_mask
        leave_one_out.append({
            "held_out": held.name,
            "fit": fitted,
            "all_rmse_rad": rollout_metric(
                prediction[mask] - held.delta_proxy[mask]
            )["rmse"],
            "negative_rmse_rad": rollout_metric(
                prediction[mask & (delayed < -0.02)]
                - held.delta_proxy[mask & (delayed < -0.02)]
            )["rmse"],
        })

    primary_improvements, holdout_improvements = [], []
    for item in per_run:
        single_rmse = item["models"]["single_gain"]["all"]["rmse"]
        split_rmse = item["models"]["split_gain"]["all"]["rmse"]
        improvement = 100.0 * (single_rmse - split_rmse) / max(single_rmse, 1.0e-12)
        (primary_improvements if item["role"] == "primary" else holdout_improvements).append(
            improvement
        )
    return {
        "method": (
            "exploratory sign-dependent gain fit at the multistep candidate delay; all gains are inferred from IMU/kinematic equivalent angle"
        ),
        "split_candidate": split,
        "per_run": per_run,
        "leave_one_primary_out": leave_one_out,
        "primary_mean_rmse_improvement_percent": float(np.mean(primary_improvements)),
        "holdout_mean_rmse_improvement_percent": float(np.mean(holdout_improvements))
        if holdout_improvements else float("nan"),
        "deployable": False,
        "non_deployment_reason": (
            "No physical steering-angle sensor and no independent current-campaign negative static plateaus; this only tests whether a split map is worth collecting dedicated data for."
        ),
    }


@dataclass(frozen=True)
class RolloutActuatorModel:
    name: str
    gain: float
    bias: float
    tau: float
    delay: float
    rate_limit: float = 5.5
    pipeline_dt: float = 0.0
    pipeline_steps: int = 0


@dataclass(frozen=True)
class RolloutContext:
    name: str
    times: np.ndarray
    command: np.ndarray
    vx: np.ndarray
    delta_proxy: np.ndarray
    yaw_rate_filtered: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    starts: np.ndarray
    dt: float
    horizon_steps: Dict[float, int]


def synchronized_pose(sync: Dict[str, np.ndarray], auxiliary: Dict[str, object]) -> Dict[str, np.ndarray]:
    pose = np.asarray(auxiliary["pose"], dtype=float)
    if len(pose) < 20:
        raise ValueError("multi-step rollout requires localization pose")
    order = np.argsort(pose[:, 0], kind="stable")
    pose = pose[order]
    keep = np.r_[True, np.diff(pose[:, 0]) > 1.0e-6]
    pose = pose[keep]
    times = sync["times"]
    return {
        "x": np.interp(times, pose[:, 0], pose[:, 2]),
        "y": np.interp(times, pose[:, 0], pose[:, 3]),
        "yaw": np.interp(times, pose[:, 0], np.unwrap(pose[:, 4])),
    }


def rollout_metric(values: Sequence[float], absolute: bool = False) -> Dict[str, float]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if len(array) == 0:
        return {"samples": 0, "rmse": float("nan"), "mean": float("nan"),
                "p95_abs": float("nan"), "maximum_abs": float("nan")}
    magnitude = array if absolute else np.abs(array)
    return {
        "samples": int(len(array)),
        "rmse": float(np.sqrt(np.mean(np.square(array)))),
        "mean": float(np.mean(array)),
        "p95_abs": float(np.quantile(magnitude, 0.95)),
        "maximum_abs": float(np.max(magnitude)),
    }


def delayed_command_for_model(
    times: np.ndarray, command: np.ndarray, model: RolloutActuatorModel
) -> np.ndarray:
    if model.pipeline_steps > 0:
        if model.pipeline_dt <= 0.0:
            raise ValueError("pipeline model requires a positive grid interval")
        stage_time = np.floor((times - times[0]) / model.pipeline_dt) * model.pipeline_dt
        query = times[0] + stage_time - model.pipeline_steps * model.pipeline_dt
    else:
        query = times - model.delay
    return zoh(query, times, command)


def make_rollout_context(
    run: PreparedActuatorRun,
    sync: Dict[str, np.ndarray],
    auxiliary: Dict[str, object],
    horizons: Sequence[float] = (0.10, 0.20, 0.50, 1.00),
    stride_s: float = 0.05,
) -> RolloutContext:
    times = run.times
    dt = float(np.median(np.diff(times)))
    horizon_steps = {float(value): int(round(float(value) / dt)) for value in horizons}
    maximum_steps = max(horizon_steps.values())
    pose = synchronized_pose(sync, auxiliary)
    valid_sample = (
        np.isfinite(run.vx) & (run.vx >= 0.45) & (run.vx <= 2.10)
        & np.isfinite(pose["x"]) & np.isfinite(pose["y"])
        & np.isfinite(pose["yaw"])
    )
    invalid_prefix = np.r_[0, np.cumsum((~valid_sample).astype(np.int64))]
    start_margin = int(math.ceil(0.20 / dt))
    stride = max(1, int(round(stride_s / dt)))
    starts = []
    for start in range(start_margin, len(times) - maximum_steps - 1, stride):
        end = start + maximum_steps + 1
        if invalid_prefix[end] - invalid_prefix[start] == 0:
            starts.append(start)
    return RolloutContext(
        name=run.name, times=times, command=run.command, vx=run.vx,
        delta_proxy=run.delta_proxy,
        yaw_rate_filtered=smooth(run.yaw_rate, dt, 0.12),
        x=pose["x"], y=pose["y"], yaw=pose["yaw"],
        starts=np.asarray(starts, dtype=np.int64), dt=dt,
        horizon_steps=horizon_steps,
    )


def vectorized_multistep_residual(
    parameters: np.ndarray,
    delay: float,
    contexts: Sequence[RolloutContext],
    wheelbase: float = 0.320,
    rate_limit: float = 5.5,
) -> np.ndarray:
    """Robustly scaled yaw/heading/lateral residual used only on training bags."""
    gain, bias, tau = (float(value) for value in parameters)
    horizon_weights = {0.10: 0.50, 0.20: 1.50, 0.50: 1.50, 1.00: 0.75}
    parts = []
    valid_contexts = [context for context in contexts if len(context.starts)]
    reference_count = float(np.mean([len(context.starts) for context in valid_contexts]))
    for context in valid_contexts:
        delayed = zoh(context.times - delay, context.times, context.command)
        starts = context.starts
        delta = context.delta_proxy[starts].copy()
        predicted_x = context.x[starts].copy()
        predicted_y = context.y[starts].copy()
        predicted_yaw = context.yaw[starts].copy()
        maximum_steps = max(context.horizon_steps.values())
        scale_run = math.sqrt(reference_count / max(len(starts), 1))
        for relative in range(1, maximum_steps + 1):
            index = starts + relative - 1
            target = np.clip(gain * delayed[index] + bias, -0.55, 0.55)
            rate = np.clip((target - delta) / tau, -rate_limit, rate_limit)
            delta = np.clip(delta + rate * context.dt, -0.55, 0.55)
            vx = context.vx[index]
            predicted_yaw_rate = vx * np.tan(delta) / wheelbase
            predicted_yaw = predicted_yaw + predicted_yaw_rate * context.dt
            predicted_x = predicted_x + vx * np.cos(predicted_yaw) * context.dt
            predicted_y = predicted_y + vx * np.sin(predicted_yaw) * context.dt
            for horizon, steps in context.horizon_steps.items():
                if relative != steps:
                    continue
                endpoint = starts + steps
                actual_yaw = context.yaw[endpoint]
                dx = predicted_x - context.x[endpoint]
                dy = predicted_y - context.y[endpoint]
                heading = np.arctan2(
                    np.sin(predicted_yaw - actual_yaw),
                    np.cos(predicted_yaw - actual_yaw),
                )
                lateral = -np.sin(actual_yaw) * dx + np.cos(actual_yaw) * dy
                yaw_rate_error = predicted_yaw_rate - context.yaw_rate_filtered[endpoint]
                weight = horizon_weights[float(horizon)] * scale_run
                # Scales put the three physical outputs on comparable robust
                # loss magnitudes.  0.2--0.5 s carries the highest weight.
                parts.extend([
                    weight * yaw_rate_error / 0.15,
                    weight * heading / 0.05,
                    weight * lateral / 0.06,
                ])
    if not parts:
        raise ValueError("no valid multi-step rollout windows")
    # Weak priors prevent one-sided steering data from trading gain against
    # offset without overpowering the measured multi-step outputs.
    parts.extend([
        np.asarray([(gain - 1.11) / 0.20]),
        np.asarray([(bias + 0.013) / 0.05]),
    ])
    return np.concatenate(parts)


def fit_multistep_actuator(
    contexts: Sequence[RolloutContext],
    seed: Dict[str, object],
) -> Dict[str, object]:
    delay_grid = np.arange(0.065, 0.1301, 0.005)
    profile = []
    best = None
    for delay in delay_grid:
        result = least_squares(
            lambda p: vectorized_multistep_residual(p, float(delay), contexts),
            np.asarray([
                float(seed["static_gain"]), float(seed["offset_rad"]),
                float(seed["time_constant_s"]),
            ]),
            bounds=(np.asarray([0.80, -0.06, 0.020]),
                    np.asarray([1.40, 0.03, 0.160])),
            loss="soft_l1", f_scale=1.0, max_nfev=60,
        )
        residual = vectorized_multistep_residual(result.x, float(delay), contexts)
        score = float(np.sqrt(np.mean(np.square(residual))))
        item = {
            "dead_time_s": float(delay), "static_gain": float(result.x[0]),
            "offset_rad": float(result.x[1]),
            "time_constant_s": float(result.x[2]),
            "effective_63pct_time_s": float(delay + result.x[2]),
            "normalized_objective_rmse": score,
            "optimizer_evaluations": int(result.nfev),
        }
        profile.append(item)
        if best is None or score < best[0]:
            best = (score, item)
    assert best is not None
    for item in profile:
        item["relative_objective_increase_percent"] = float(
            100.0 * (item["normalized_objective_rmse"] / best[0] - 1.0)
        )
    return {
        "method": "group-balanced robust multi-horizon output-error fit",
        "training_runs": [context.name for context in contexts],
        "candidate": best[1],
        "delay_profile": profile,
        "loss_horizon_weights": {"0.10s": 0.50, "0.20s": 1.50,
                                 "0.50s": 1.50, "1.00s": 0.75},
        "loss_scales": {"yaw_rate_radps": 0.15, "heading_rad": 0.05,
                        "lateral_m": 0.06},
        "holdout_used_for_fit": False,
    }


def rollout_run(
    run: PreparedActuatorRun,
    sync: Dict[str, np.ndarray],
    auxiliary: Dict[str, object],
    models: Sequence[RolloutActuatorModel],
    horizons: Sequence[float] = (0.10, 0.20, 0.50, 1.00),
    wheelbase: float = 0.320,
) -> Dict[str, object]:
    times = run.times
    dt = float(np.median(np.diff(times)))
    horizon_steps = {float(value): int(round(float(value) / dt)) for value in horizons}
    maximum_steps = max(horizon_steps.values())
    pose = synchronized_pose(sync, auxiliary)
    yaw_rate_filtered = smooth(run.yaw_rate, dt, 0.12)
    valid_sample = (
        np.isfinite(run.vx) & (run.vx >= 0.45) & (run.vx <= 2.10)
        & np.isfinite(pose["x"]) & np.isfinite(pose["y"]) & np.isfinite(pose["yaw"])
    )
    invalid_prefix = np.r_[0, np.cumsum((~valid_sample).astype(np.int64))]
    start_margin = int(math.ceil(0.20 / dt))
    stride = max(1, int(round(0.05 / dt)))
    starts = []
    for start in range(start_margin, len(times) - maximum_steps - 1, stride):
        end = start + maximum_steps + 1
        if invalid_prefix[end] - invalid_prefix[start] == 0:
            starts.append(start)
    result = {"name": run.name, "windows": int(len(starts)), "models": {}}
    for model in models:
        effective_command = delayed_command_for_model(times, run.command, model)
        errors = {
            horizon: defaultdict(list) for horizon in horizon_steps
        }
        for start in starts:
            delta = float(run.delta_proxy[start])
            predicted_x = float(pose["x"][start])
            predicted_y = float(pose["y"][start])
            predicted_yaw = float(pose["yaw"][start])
            for relative in range(1, maximum_steps + 1):
                index = start + relative - 1
                target = float(np.clip(
                    model.gain * effective_command[index] + model.bias,
                    -0.55, 0.55,
                ))
                unrestricted_rate = (target - delta) / model.tau
                if math.isfinite(model.rate_limit):
                    rate = float(np.clip(
                        unrestricted_rate, -model.rate_limit, model.rate_limit
                    ))
                    delta += rate * dt
                else:
                    decay = math.exp(-dt / model.tau)
                    delta = target + (delta - target) * decay
                delta = float(np.clip(delta, -0.55, 0.55))
                vx = float(run.vx[index])
                predicted_yaw_rate = vx * math.tan(delta) / wheelbase
                predicted_yaw += predicted_yaw_rate * dt
                predicted_x += vx * math.cos(predicted_yaw) * dt
                predicted_y += vx * math.sin(predicted_yaw) * dt
                endpoint_step = relative
                for horizon, steps in horizon_steps.items():
                    if endpoint_step != steps:
                        continue
                    endpoint = start + steps
                    actual_yaw = float(pose["yaw"][endpoint])
                    dx = predicted_x - float(pose["x"][endpoint])
                    dy = predicted_y - float(pose["y"][endpoint])
                    heading_error = math.atan2(
                        math.sin(predicted_yaw - actual_yaw),
                        math.cos(predicted_yaw - actual_yaw),
                    )
                    lateral_error = -math.sin(actual_yaw) * dx + math.cos(actual_yaw) * dy
                    errors[horizon]["delta_rad"].append(
                        delta - float(run.delta_proxy[endpoint])
                    )
                    errors[horizon]["yaw_rate_radps"].append(
                        predicted_yaw_rate - float(yaw_rate_filtered[endpoint])
                    )
                    errors[horizon]["heading_rad"].append(heading_error)
                    errors[horizon]["lateral_m"].append(lateral_error)
                    errors[horizon]["position_m"].append(math.hypot(dx, dy))
        result["models"][model.name] = {
            f"{horizon:.2f}s": {
                "delta_error_rad": rollout_metric(values["delta_rad"]),
                "yaw_rate_error_radps": rollout_metric(values["yaw_rate_radps"]),
                "heading_error_rad": rollout_metric(values["heading_rad"]),
                "lateral_error_m": rollout_metric(values["lateral_m"]),
                "position_error_m": rollout_metric(values["position_m"], absolute=True),
            }
            for horizon, values in errors.items()
        }
    return result


def multi_step_experiment(
    primary_runs: Sequence[PreparedActuatorRun],
    holdout_runs: Sequence[PreparedActuatorRun],
    syncs: Sequence[Dict[str, np.ndarray]],
    auxiliaries: Sequence[Dict[str, object]],
    dynamic_report: Dict[str, object],
) -> Dict[str, object]:
    candidate = dynamic_report["joint_candidate"]
    training_contexts = [
        make_rollout_context(run, sync, auxiliary, stride_s=0.10)
        for run, sync, auxiliary in zip(
            primary_runs,
            syncs[:len(primary_runs)],
            auxiliaries[:len(primary_runs)],
        )
    ]
    multistep_fit = fit_multistep_actuator(training_contexts, candidate)
    multistep_candidate = multistep_fit["candidate"]
    ridge = min(
        dynamic_report["delay_tau_profile"],
        key=lambda item: abs(item["dead_time_s"] - 0.095),
    )
    models = [
        RolloutActuatorModel("stable_nominal", 1.0, 0.0, 0.08, 0.0, 5.5),
        RolloutActuatorModel("historical_candidate", 1.147, -0.015, 0.073, 0.090, 5.5),
        RolloutActuatorModel(
            "joint_fopdt", float(candidate["static_gain"]),
            float(candidate["offset_rad"]), float(candidate["time_constant_s"]),
            float(candidate["dead_time_s"]), 5.5,
        ),
        RolloutActuatorModel(
            "ridge_td095", float(ridge["gain"]), float(ridge["bias_rad"]),
            float(ridge["time_constant_s"]), float(ridge["dead_time_s"]), 5.5,
        ),
        RolloutActuatorModel(
            "joint_rate_1p5", float(candidate["static_gain"]),
            float(candidate["offset_rad"]), float(candidate["time_constant_s"]),
            float(candidate["dead_time_s"]), 1.5,
        ),
        RolloutActuatorModel(
            "joint_rate_3p4", float(candidate["static_gain"]),
            float(candidate["offset_rad"]), float(candidate["time_constant_s"]),
            float(candidate["dead_time_s"]), 3.4,
        ),
        RolloutActuatorModel(
            "two_stage_050", float(candidate["static_gain"]),
            float(candidate["offset_rad"]), float(candidate["time_constant_s"]),
            0.10, 5.5, pipeline_dt=0.05, pipeline_steps=2,
        ),
        RolloutActuatorModel(
            "multistep_fopdt", float(multistep_candidate["static_gain"]),
            float(multistep_candidate["offset_rad"]),
            float(multistep_candidate["time_constant_s"]),
            float(multistep_candidate["dead_time_s"]), 5.5,
        ),
    ]
    runs = list(primary_runs) + list(holdout_runs)
    per_run = [
        rollout_run(run, sync, auxiliary, models)
        for run, sync, auxiliary in zip(runs, syncs, auxiliaries)
    ]
    role_by_name = {
        run.name: "primary" for run in primary_runs
    }
    role_by_name.update({run.name: "holdout" for run in holdout_runs})
    for item in per_run:
        item["role"] = role_by_name[item["name"]]

    # Acceptance is based on each untouched holdout independently. If no
    # holdout exists, report the metric but do not pass the gate.
    holdout_acceptance = []
    for item in per_run:
        if item["role"] != "holdout":
            continue
        stable = item["models"]["stable_nominal"]
        for model_name, metrics in item["models"].items():
            yaw_stable = stable["0.20s"]["yaw_rate_error_radps"]["rmse"]
            yaw_model = metrics["0.20s"]["yaw_rate_error_radps"]["rmse"]
            lateral_p95 = metrics["0.50s"]["lateral_error_m"]["p95_abs"]
            holdout_acceptance.append({
                "run": item["name"], "model": model_name,
                "yaw_rate_0p2s_improvement_percent": float(
                    100.0 * (yaw_stable - yaw_model) / max(yaw_stable, 1.0e-12)
                ),
                "yaw_rate_improvement_pass": bool(
                    yaw_model <= 0.70 * yaw_stable
                ),
                "lateral_0p5s_p95_m": float(lateral_p95),
                "lateral_p95_pass": bool(lateral_p95 < 0.08),
            })
    candidate_acceptance = [
        item for item in holdout_acceptance if item["model"] == "joint_fopdt"
    ]
    multistep_acceptance = [
        item for item in holdout_acceptance
        if item["model"] == "multistep_fopdt"
    ]
    return {
        "method": (
            "measured-vx actuator-isolated rollout; delta initialized from the current equivalent-angle observation; future steering commands are known only for offline scoring"
        ),
        "horizons_s": [0.10, 0.20, 0.50, 1.00],
        "models": [vars(model) for model in models],
        "multistep_fit": multistep_fit,
        "per_run": per_run,
        "holdout_acceptance": holdout_acceptance,
        "joint_candidate_all_holdout_gates_pass": bool(candidate_acceptance) and all(
            item["yaw_rate_improvement_pass"] and item["lateral_p95_pass"]
            for item in candidate_acceptance
        ),
        "multistep_candidate_all_holdout_gates_pass": bool(multistep_acceptance) and all(
            item["yaw_rate_improvement_pass"] and item["lateral_p95_pass"]
            for item in multistep_acceptance
        ),
        "limitations": [
            "Measured future vx is used to isolate steering response; this is not a full autonomous vehicle-model rollout.",
            "No future vy is injected, so lateral error also contains sideslip-model mismatch.",
            "The 0.05 s two-stage model quantizes the recorded command to the OCP grid.",
        ],
    }


def observer_shadow_experiment(
    runs: Sequence[PreparedActuatorRun],
    syncs: Sequence[Dict[str, np.ndarray]],
    primary_count: int,
    models: Sequence[RolloutActuatorModel],
    vehicle_path: Path,
    controller_path: Path,
) -> Dict[str, object]:
    """Replay the deployed no-sensor observer without changing configuration."""
    from f1tenth_dynamic_mpcc.steering_actuator_model import (
        SteeringActuatorModel,
        SteeringActuatorParameters,
    )
    from f1tenth_dynamic_mpcc.steering_state_observer import (
        SteeringObserverParameters,
        SteeringStateObserver,
    )
    from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters

    vehicle_cfg = yaml.safe_load(vehicle_path.read_text(encoding="utf-8"))
    controller_cfg = yaml.safe_load(controller_path.read_text(encoding="utf-8"))
    vehicle = VehicleParameters.from_yaml(vehicle_path, controller_cfg)
    observer_parameters = SteeringObserverParameters.from_config(vehicle_cfg)
    selected = [
        model for model in models
        if model.name in ("stable_nominal", "historical_candidate", "multistep_fopdt")
    ]
    per_run = []
    for run_index, (run, sync) in enumerate(zip(runs, syncs)):
        role = "primary" if run_index < primary_count else "holdout"
        model_results = {}
        for model in selected:
            actuator_parameters = SteeringActuatorParameters(
                identified=model.name != "stable_nominal",
                map_type="linear", gain=model.gain, bias=model.bias,
                tau=model.tau, delay=model.delay,
                delta_min=-0.55, delta_max=0.55,
            )
            actuator_parameters.validate()
            observer = SteeringStateObserver(
                SteeringActuatorModel(actuator_parameters), vehicle,
                observer_parameters,
            )
            estimate, prediction, innovation, confidence, gain = [], [], [], [], []
            modes = []
            for timestamp, command, vx, vy, yaw_rate in zip(
                run.times, run.command, run.vx, sync["vy"], sync["odom_yaw_rate"]
            ):
                observer.push_command(float(timestamp), float(command))
                value = observer.update(
                    float(timestamp), float(vx), float(vy), float(yaw_rate)
                )
                estimate.append(value.delta_hat)
                prediction.append(value.delta_pred)
                innovation.append(value.innovation)
                confidence.append(value.confidence)
                gain.append(value.correction_gain)
                modes.append(value.mode)
            estimate = np.asarray(estimate, dtype=float)
            prediction = np.asarray(prediction, dtype=float)
            innovation = np.asarray(innovation, dtype=float)
            confidence = np.asarray(confidence, dtype=float)
            gain = np.asarray(gain, dtype=float)
            modes = np.asarray(modes)
            valid = (
                run.dynamic_mask
                & (run.times >= run.times[0] + 0.25)
                & np.isfinite(run.delta_proxy)
            )
            mode_counts = {
                name: int(np.sum(modes[valid] == name))
                for name in ("MODEL_ONLY", "KINEMATIC_CORRECTION",
                             "DYNAMIC_CORRECTION", "INVALID")
            }
            sample_count = max(int(np.sum(valid)), 1)
            estimate_error = estimate[valid] - run.delta_proxy[valid]
            prediction_error = prediction[valid] - run.delta_proxy[valid]
            model_results[model.name] = {
                "samples": int(np.sum(valid)),
                "delta_hat_error_rad": rollout_metric(estimate_error),
                "pre_correction_prediction_error_rad": rollout_metric(prediction_error),
                "observer_correction_rmse_improvement_percent": float(
                    100.0 * (
                        rollout_metric(prediction_error)["rmse"]
                        - rollout_metric(estimate_error)["rmse"]
                    ) / max(rollout_metric(prediction_error)["rmse"], 1.0e-12)
                ),
                "innovation_rad": rollout_metric(innovation[valid]),
                "innovation_clipped_fraction": float(np.mean(
                    np.abs(innovation[valid])
                    >= observer_parameters.innovation_limit_rad - 1.0e-6
                )),
                "confidence": percentile_summary(confidence[valid]),
                "correction_gain": percentile_summary(gain[valid]),
                "mode_counts": mode_counts,
                "mode_fractions": {
                    name: float(count / sample_count)
                    for name, count in mode_counts.items()
                },
            }
        per_run.append({"name": run.name, "role": role, "models": model_results})

    holdout_comparison = []
    for item in per_run:
        if item["role"] != "holdout":
            continue
        stable_rmse = item["models"]["stable_nominal"]["delta_hat_error_rad"]["rmse"]
        for name, metrics in item["models"].items():
            rmse = metrics["delta_hat_error_rad"]["rmse"]
            holdout_comparison.append({
                "run": item["name"], "model": name,
                "delta_hat_rmse_rad": float(rmse),
                "improvement_vs_stable_percent": float(
                    100.0 * (stable_rmse - rmse) / max(stable_rmse, 1.0e-12)
                ),
            })

    candidate_model = next(
        model for model in selected if model.name == "multistep_fopdt"
    )

    def candidate_observer_rmse(
        run: PreparedActuatorRun,
        sync: Dict[str, np.ndarray],
        parameters,
    ) -> float:
        actuator_parameters = SteeringActuatorParameters(
            identified=True, map_type="linear",
            gain=candidate_model.gain, bias=candidate_model.bias,
            tau=candidate_model.tau, delay=candidate_model.delay,
            delta_min=-0.55, delta_max=0.55,
        )
        observer = SteeringStateObserver(
            SteeringActuatorModel(actuator_parameters), vehicle, parameters
        )
        estimates = []
        for timestamp, command, vx, vy, yaw_rate in zip(
            run.times, run.command, run.vx, sync["vy"], sync["odom_yaw_rate"]
        ):
            observer.push_command(float(timestamp), float(command))
            estimates.append(observer.update(
                float(timestamp), float(vx), float(vy), float(yaw_rate)
            ).delta_hat)
        estimates = np.asarray(estimates, dtype=float)
        valid = (
            run.dynamic_mask
            & (run.times >= run.times[0] + 0.25)
            & np.isfinite(run.delta_proxy)
        )
        return float(np.sqrt(np.mean(np.square(
            estimates[valid] - run.delta_proxy[valid]
        ))))

    gain_grid = []
    for kinematic_gain in (0.0, 0.05, 0.10, 0.15):
        for dynamic_gain in (0.0, 0.05, 0.10, 0.15):
            parameters = replace(
                observer_parameters,
                kinematic_gain_max=kinematic_gain,
                dynamic_gain_max=dynamic_gain,
            )
            primary_rmse = [
                candidate_observer_rmse(run, sync, parameters)
                for run, sync in zip(runs[:primary_count], syncs[:primary_count])
            ]
            holdout_rmse = [
                candidate_observer_rmse(run, sync, parameters)
                for run, sync in zip(runs[primary_count:], syncs[primary_count:])
            ]
            gain_grid.append({
                "kinematic_gain_max": float(kinematic_gain),
                "dynamic_gain_max": float(dynamic_gain),
                "primary_group_balanced_rmse_rad": float(
                    np.sqrt(np.mean(np.square(primary_rmse)))
                ),
                "primary_per_run_rmse_rad": primary_rmse,
                "holdout_per_run_rmse_rad": holdout_rmse,
                "holdout_group_balanced_rmse_rad": float(
                    np.sqrt(np.mean(np.square(holdout_rmse)))
                ) if holdout_rmse else float("nan"),
            })
    selected_gain = min(
        gain_grid, key=lambda item: item["primary_group_balanced_rmse_rad"]
    )
    return {
        "method": (
            "exact replay of the deployed command-history observer; odometry vx/vy/yaw-rate drive the correction and IMU-derived equivalent steering is used only for offline scoring"
        ),
        "observer_parameters": vars(observer_parameters),
        "per_run": per_run,
        "holdout_comparison": holdout_comparison,
        "candidate_observer_gain_grid": gain_grid,
        "candidate_observer_gain_selected_on_primary_only": selected_gain,
        "gain_grid_holdout_used_for_selection": False,
        "limitations": [
            "The scoring angle is an IMU/kinematic equivalent angle, not a measured front-wheel angle.",
            "Observer correction and scoring share vehicle motion, so this experiment checks consistency rather than independent sensor truth.",
            "The deployed observer has no explicit hard steering-rate saturation.",
        ],
    }


@dataclass(frozen=True)
class ResidualModelV1:
    feature_names: Tuple[str, ...]
    mean: np.ndarray
    scale: np.ndarray
    coefficients: np.ndarray
    limits: np.ndarray
    envelope: np.ndarray
    fade_ratio: float

    @classmethod
    def load(cls, path: Path) -> "ResidualModelV1":
        values = yaml.safe_load(path.read_text(encoding="utf-8"))
        if int(values.get("schema_version", 1)) != 1:
            raise ValueError("full dynamic replay currently requires schema V1")
        expected = (
            "bias", "vx", "vy", "yaw_rate", "delta", "speed_cmd",
            "steer_cmd", "speed_error", "steer_error", "vx_yaw_rate",
            "vy_yaw_rate", "abs_vx_vy", "abs_vx_yaw_rate",
            "vx_tan_delta", "vx_steer_cmd", "vx2_steer_cmd",
        )
        names = tuple(values["feature_names"])
        if names != expected:
            raise ValueError("unexpected V1 residual feature ordering")
        return cls(
            names,
            np.asarray(values["feature_mean"], dtype=float),
            np.asarray(values["feature_scale"], dtype=float),
            np.asarray(values["coefficients_feature_by_output"], dtype=float),
            np.asarray(values["output_limits"], dtype=float),
            np.asarray(values["feature_abs_z_limit"], dtype=float),
            float(values.get("ood_fade_ratio", 1.5)),
        )

    def predict(
        self,
        state: np.ndarray,
        speed_command: np.ndarray,
        steering_command: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        vx, vy, yaw_rate, delta = (
            state[:, 3], state[:, 4], state[:, 5], state[:, 6]
        )
        features = np.column_stack((
            np.ones(len(state)), vx, vy, yaw_rate, delta,
            speed_command, steering_command, speed_command - vx,
            steering_command - delta, vx * yaw_rate, vy * yaw_rate,
            np.abs(vx) * vy, np.abs(vx) * yaw_rate,
            vx * np.tan(np.clip(delta, -0.7, 0.7)),
            vx * steering_command, vx * vx * steering_command,
        ))
        normalized = (features - self.mean) / self.scale
        raw = normalized @ self.coefficients
        bounded = np.clip(raw, -self.limits, self.limits)
        ratio = np.max(np.abs(normalized) / self.envelope, axis=1)
        confidence = np.clip(
            (self.fade_ratio - ratio) / max(self.fade_ratio - 1.0, 1.0e-9),
            0.0, 1.0,
        )
        return confidence[:, None] * bounded, confidence


@dataclass(frozen=True)
class DynamicReplayScenario:
    name: str
    actuator: RolloutActuatorModel
    residual_enabled: bool


def dynamic_derivatives(
    state: np.ndarray,
    speed_drive: np.ndarray,
    steering_drive: np.ndarray,
    speed_feature: np.ndarray,
    steering_feature: np.ndarray,
    actuator: RolloutActuatorModel,
    vehicle,
    residual: Optional[ResidualModelV1],
) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorized seven-state vehicle derivative used by bag replay."""
    yaw, vx, vy, yaw_rate, delta = (
        state[:, 2], state[:, 3], state[:, 4], state[:, 5], state[:, 6]
    )
    safe_vx = np.hypot(vx, vehicle.vx_regularization)
    alpha_front = delta - np.arctan2(vy + vehicle.lf * yaw_rate, safe_vx)
    alpha_rear = -np.arctan2(vy - vehicle.lr * yaw_rate, safe_vx)
    force_front = vehicle.cf * alpha_front
    force_rear = vehicle.cr * alpha_rear
    vx_dot = np.clip(
        (vehicle.speed_gain * speed_drive - vx) / vehicle.speed_tau,
        -vehicle.max_decel, vehicle.max_accel,
    )
    vy_dynamic = (force_front + force_rear) / vehicle.mass - vx * yaw_rate
    yaw_dynamic = (
        vehicle.lf * force_front - vehicle.lr * force_rear
    ) / vehicle.yaw_inertia
    blend = 0.5 * (np.tanh((vx - 0.45) / 0.08) + 1.0)
    yaw_kinematic = vx * np.tan(delta) / vehicle.wheelbase
    vy_dot = blend * vy_dynamic + (1.0 - blend) * (-vy / 0.12)
    yaw_rate_dot = (
        blend * yaw_dynamic
        + (1.0 - blend) * ((yaw_kinematic - yaw_rate) / 0.10)
    )
    target = np.clip(
        actuator.gain * steering_drive + actuator.bias,
        -vehicle.max_steer, vehicle.max_steer,
    )
    delta_dot = (target - delta) / actuator.tau
    if math.isfinite(actuator.rate_limit):
        delta_dot = np.clip(
            delta_dot, -actuator.rate_limit, actuator.rate_limit
        )
    if residual is not None:
        correction, confidence = residual.predict(
            state, speed_feature, steering_feature
        )
        vx_dot = vx_dot + correction[:, 0]
        vy_dot = vy_dot + correction[:, 1]
        yaw_rate_dot = yaw_rate_dot + correction[:, 2]
    else:
        confidence = np.full(len(state), np.nan)
    derivative = np.column_stack((
        vx * np.cos(yaw) - vy * np.sin(yaw),
        vx * np.sin(yaw) + vy * np.cos(yaw),
        yaw_rate, vx_dot, vy_dot, yaw_rate_dot, delta_dot,
    ))
    return derivative, confidence


def dynamic_rk4_step(
    state: np.ndarray,
    dt: float,
    speed_drive: np.ndarray,
    steering_drive: np.ndarray,
    speed_feature: np.ndarray,
    steering_feature: np.ndarray,
    scenario: DynamicReplayScenario,
    vehicle,
    residual: Optional[ResidualModelV1],
) -> Tuple[np.ndarray, np.ndarray]:
    """Two-substep RK4, matching the stiff low-speed model safely at 100 Hz."""
    output = state.copy()
    confidence = np.full(len(state), np.nan)
    sub_dt = dt / 2.0
    active_residual = residual if scenario.residual_enabled else None
    for _ in range(2):
        k1, confidence = dynamic_derivatives(
            output, speed_drive, steering_drive, speed_feature,
            steering_feature, scenario.actuator, vehicle, active_residual,
        )
        k2, _ = dynamic_derivatives(
            output + 0.5 * sub_dt * k1, speed_drive, steering_drive,
            speed_feature, steering_feature, scenario.actuator, vehicle,
            active_residual,
        )
        k3, _ = dynamic_derivatives(
            output + 0.5 * sub_dt * k2, speed_drive, steering_drive,
            speed_feature, steering_feature, scenario.actuator, vehicle,
            active_residual,
        )
        k4, _ = dynamic_derivatives(
            output + sub_dt * k3, speed_drive, steering_drive,
            speed_feature, steering_feature, scenario.actuator, vehicle,
            active_residual,
        )
        output += sub_dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        output[:, 3] = np.maximum(output[:, 3], 0.0)
        output[:, 6] = np.clip(
            output[:, 6], -vehicle.max_steer, vehicle.max_steer
        )
    return output, confidence


def full_dynamic_rollout_run(
    run: PreparedActuatorRun,
    sync: Dict[str, np.ndarray],
    auxiliary: Dict[str, object],
    scenarios: Sequence[DynamicReplayScenario],
    vehicle,
    residual: ResidualModelV1,
    horizons: Sequence[float] = (0.10, 0.20, 0.50, 1.00),
) -> Dict[str, object]:
    times = run.times
    dt = float(np.median(np.diff(times)))
    horizon_steps = {float(value): int(round(float(value) / dt)) for value in horizons}
    maximum_steps = max(horizon_steps.values())
    pose = synchronized_pose(sync, auxiliary)
    measured = np.column_stack((
        pose["x"], pose["y"], pose["yaw"], run.vx, sync["vy"],
        sync["odom_yaw_rate"], run.delta_proxy,
    ))
    valid_sample = (
        np.all(np.isfinite(measured), axis=1)
        & (run.vx >= 0.45) & (run.vx <= 3.80)
        & (np.abs(sync["vy"]) <= 1.0)
        & (np.abs(sync["odom_yaw_rate"]) <= 6.0)
    )
    invalid_prefix = np.r_[0, np.cumsum((~valid_sample).astype(np.int64))]
    stride = max(1, int(round(0.10 / dt)))
    start_margin = int(math.ceil(0.25 / dt))
    starts = np.asarray([
        start
        for start in range(start_margin, len(times) - maximum_steps - 1, stride)
        if invalid_prefix[start + maximum_steps + 1] - invalid_prefix[start] == 0
    ], dtype=np.int64)
    speed_delayed = zoh(
        times - vehicle.speed_dead_time, times, sync["speed_command"]
    )
    result = {"name": run.name, "windows": int(len(starts)), "scenarios": {}}
    for scenario in scenarios:
        steering_delayed = delayed_command_for_model(
            times, run.command, scenario.actuator
        )
        state = measured[starts].copy()
        errors = {horizon: defaultdict(list) for horizon in horizon_steps}
        residual_confidence = []
        residual_corrections = [[], [], []]
        for relative in range(1, maximum_steps + 1):
            index = starts + relative - 1
            state, _ = dynamic_rk4_step(
                state, dt, speed_delayed[index], steering_delayed[index],
                sync["speed_command"][index], run.command[index],
                scenario, vehicle, residual,
            )
            if scenario.residual_enabled:
                correction, confidence = residual.predict(
                    state, sync["speed_command"][index], run.command[index]
                )
                residual_confidence.extend(confidence.tolist())
                for axis in range(3):
                    residual_corrections[axis].extend(correction[:, axis].tolist())
            for horizon, steps in horizon_steps.items():
                if relative != steps:
                    continue
                endpoint = starts + steps
                actual = measured[endpoint]
                dx = state[:, 0] - actual[:, 0]
                dy = state[:, 1] - actual[:, 1]
                heading = np.arctan2(
                    np.sin(state[:, 2] - actual[:, 2]),
                    np.cos(state[:, 2] - actual[:, 2]),
                )
                lateral = -np.sin(actual[:, 2]) * dx + np.cos(actual[:, 2]) * dy
                errors[horizon]["vx_mps"].extend((state[:, 3] - actual[:, 3]).tolist())
                errors[horizon]["vy_mps"].extend((state[:, 4] - actual[:, 4]).tolist())
                errors[horizon]["yaw_rate_radps"].extend((state[:, 5] - actual[:, 5]).tolist())
                errors[horizon]["delta_rad"].extend((state[:, 6] - actual[:, 6]).tolist())
                errors[horizon]["heading_rad"].extend(heading.tolist())
                errors[horizon]["lateral_m"].extend(lateral.tolist())
                errors[horizon]["position_m"].extend(np.hypot(dx, dy).tolist())
        scenario_result = {
            f"{horizon:.2f}s": {
                "vx_error_mps": rollout_metric(values["vx_mps"]),
                "vy_error_mps": rollout_metric(values["vy_mps"]),
                "yaw_rate_error_radps": rollout_metric(values["yaw_rate_radps"]),
                "delta_error_rad": rollout_metric(values["delta_rad"]),
                "heading_error_rad": rollout_metric(values["heading_rad"]),
                "lateral_error_m": rollout_metric(values["lateral_m"]),
                "position_error_m": rollout_metric(values["position_m"], absolute=True),
            }
            for horizon, values in errors.items()
        }
        if scenario.residual_enabled:
            confidence = np.asarray(residual_confidence, dtype=float)
            scenario_result["residual_confidence"] = {
                **percentile_summary(confidence),
                "active_fraction": float(np.mean(confidence > 0.0)),
                "zero_fraction": float(np.mean(confidence <= 0.0)),
            }
            scenario_result["residual_correction"] = {
                name: rollout_metric(values)
                for name, values in zip(
                    ("vx_accel_mps2", "vy_accel_mps2", "yaw_accel_radps2"),
                    residual_corrections,
                )
            }
        result["scenarios"][scenario.name] = scenario_result
    return result


def full_dynamic_experiment(
    runs: Sequence[PreparedActuatorRun],
    syncs: Sequence[Dict[str, np.ndarray]],
    auxiliaries: Sequence[Dict[str, object]],
    primary_count: int,
    actuator_models: Sequence[RolloutActuatorModel],
    vehicle_path: Path,
    controller_path: Path,
    residual_path: Path,
) -> Dict[str, object]:
    from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters

    controller_cfg = yaml.safe_load(controller_path.read_text(encoding="utf-8"))
    vehicle = VehicleParameters.from_yaml(vehicle_path, controller_cfg)
    residual = ResidualModelV1.load(residual_path)
    by_name = {model.name: model for model in actuator_models}
    scenarios = [
        DynamicReplayScenario("stable_physics", by_name["stable_nominal"], False),
        DynamicReplayScenario("stable_plus_v1_residual", by_name["stable_nominal"], True),
        DynamicReplayScenario("multistep_physics", by_name["multistep_fopdt"], False),
        DynamicReplayScenario("multistep_plus_old_v1_residual", by_name["multistep_fopdt"], True),
        DynamicReplayScenario("historical_physics", by_name["historical_candidate"], False),
    ]
    per_run = [
        full_dynamic_rollout_run(
            run, sync, auxiliary, scenarios, vehicle, residual
        )
        for run, sync, auxiliary in zip(runs, syncs, auxiliaries)
    ]
    for index, item in enumerate(per_run):
        item["role"] = "primary" if index < primary_count else "holdout"
    holdout_comparison = []
    for item in per_run:
        if item["role"] != "holdout":
            continue
        stable = item["scenarios"]["stable_physics"]
        stable_yaw = stable["0.20s"]["yaw_rate_error_radps"]["rmse"]
        stable_lateral = stable["0.50s"]["lateral_error_m"]["p95_abs"]
        for name, metrics in item["scenarios"].items():
            yaw = metrics["0.20s"]["yaw_rate_error_radps"]["rmse"]
            lateral = metrics["0.50s"]["lateral_error_m"]["p95_abs"]
            holdout_comparison.append({
                "run": item["name"], "scenario": name,
                "yaw_rate_0p2s_rmse_radps": float(yaw),
                "yaw_rate_improvement_vs_stable_percent": float(
                    100.0 * (stable_yaw - yaw) / max(stable_yaw, 1.0e-12)
                ),
                "lateral_0p5s_p95_m": float(lateral),
                "lateral_improvement_vs_stable_percent": float(
                    100.0 * (stable_lateral - lateral)
                    / max(stable_lateral, 1.0e-12)
                ),
                "position_1p0s_p95_m": float(
                    metrics["1.00s"]["position_error_m"]["p95_abs"]
                ),
            })
    return {
        "method": (
            "seven-state dynamic-bicycle open-loop bag replay with 100 Hz commands, two-substep RK4, measured initial state only, 0.13 s longitudinal dead time and each scenario's steering FOPDT"
        ),
        "residual_model": str(residual_path.resolve()),
        "scenarios": [
            {"name": item.name, "actuator": vars(item.actuator),
             "residual_enabled": item.residual_enabled}
            for item in scenarios
        ],
        "per_run": per_run,
        "holdout_comparison": holdout_comparison,
        "limitations": [
            "The full replay evaluates the existing nominal tire model; Cf/Cr and lateral velocity are not independently identified by these low-speed bags.",
            "The old V1 residual was trained with the stable steering model, so candidate-plus-V1 is explicitly a compatibility test, not a newly trained residual.",
            "Future recorded commands are available to offline replay; closed-loop MPCC does not know them and must represent delay causally.",
        ],
    }


def tire_identifiability_experiment(
    paths: Sequence[Path],
    syncs: Sequence[Dict[str, np.ndarray]],
    primary_count: int,
    actuator_candidate: Dict[str, object],
) -> Dict[str, object]:
    """Try the existing axle-force fit and explicitly retain failed fits."""
    actuator = ActuatorFit(
        gain=float(actuator_candidate["static_gain"]),
        bias=float(actuator_candidate["offset_rad"]),
        tau=float(actuator_candidate["time_constant_s"]),
        delay=float(actuator_candidate["dead_time_s"]),
        static_rmse=float("nan"), dynamic_rmse=float("nan"),
        dynamic_r2=float("nan"), samples=0, static_samples=0,
        source="multistep_fopdt",
    )
    results = []
    accepted = []
    for index, (path, sync) in enumerate(zip(paths, syncs)):
        item = {
            "name": path.stem,
            "role": "primary" if index < primary_count else "holdout",
        }
        try:
            fit, _ = fit_tire_stiffness(
                sync["times"], sync["times"], sync["steering_command"],
                sync["vx"], sync["vy"], sync["odom_yaw_rate"], actuator,
            )
            values = fit.as_dict()
            valid = bool(
                values["Cf_N_per_rad"] > 5.05
                and values["Cr_N_per_rad"] > 5.05
                and values["front_force_r2"] > 0.0
                and values["rear_force_r2"] > 0.0
            )
            item.update(values)
            item["accepted"] = valid
            if not valid:
                item["rejection_reason"] = (
                    "stiffness reached/approached the lower bound or axle-force R2 is non-positive"
                )
            else:
                accepted.append(item)
        except ValueError as error:
            item.update({
                "accepted": False,
                "rejection_reason": str(error),
            })
        results.append(item)
    return {
        "method": (
            "existing force-balance Cf/Cr fit using candidate steering FOPDT, Point-LIO vx/vy/yaw-rate and smoothed derivatives"
        ),
        "per_run": results,
        "accepted_fit_count": int(len(accepted)),
        "identifiable_from_current_bags": bool(len(accepted) >= 2),
        "conclusion": (
            "当前转向辨识 bag 不能独立辨识前后轴侧偏刚度；Cf/Cr 保持不变，后续需采集专用的更高速横向激励，并改善横向状态可观测性。"
            if len(accepted) < 2 else
            "已有多份数据通过基本力拟合门槛，但部署前仍需验证跨 bag 一致性。"
        ),
    }


def lag_alignment(
    times: np.ndarray,
    reference: np.ndarray,
    observed: np.ndarray,
    mask: np.ndarray,
    maximum_lag_s: float = 0.25,
) -> Dict[str, float]:
    """Find observed(t) ~= a * reference(t-lag) + b.

    A positive lag means the observed stream follows the reference stream.
    An affine fit removes scale/offset while selecting temporal alignment.
    """
    times = np.asarray(times, dtype=float)
    reference = np.asarray(reference, dtype=float)
    observed = np.asarray(observed, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    grid = np.arange(-maximum_lag_s, maximum_lag_s + 0.0001, 0.005)
    candidates = []
    for lag in grid:
        valid = (
            mask
            & (times - lag >= times[0])
            & (times - lag <= times[-1])
            & np.isfinite(reference) & np.isfinite(observed)
        )
        shifted = np.interp(times[valid] - lag, times, reference)
        target = observed[valid]
        if len(target) < 100 or np.std(shifted) < 1.0e-4 or np.std(target) < 1.0e-4:
            continue
        design = np.column_stack((shifted, np.ones(len(shifted))))
        gain, offset = np.linalg.lstsq(design, target, rcond=None)[0]
        prediction = gain * shifted + offset
        correlation = float(np.corrcoef(shifted, target)[0, 1])
        candidates.append({
            "lag_s": float(lag), "gain": float(gain),
            "offset": float(offset), "correlation": correlation,
            "affine_rmse": float(np.sqrt(np.mean(np.square(prediction - target)))),
            "samples": int(len(target)),
        })
    if not candidates:
        return {
            "lag_s": float("nan"), "gain": float("nan"),
            "offset": float("nan"), "correlation": float("nan"),
            "affine_rmse": float("nan"), "samples": 0,
        }
    return max(candidates, key=lambda item: item["correlation"])


def sensor_alignment_experiment(
    runs: Sequence[PreparedActuatorRun],
    syncs: Sequence[Dict[str, np.ndarray]],
    auxiliaries: Sequence[Dict[str, object]],
    primary_count: int,
) -> Dict[str, object]:
    per_run = []
    for index, (run, sync, auxiliary) in enumerate(zip(runs, syncs, auxiliaries)):
        dt = float(np.median(np.diff(run.times)))
        moving = np.isfinite(run.vx) & (run.vx >= 0.45)
        imu_yaw = smooth(sync["yaw_rate"], dt, 0.10)
        pointlio_yaw = smooth(sync["odom_yaw_rate"], dt, 0.10)
        yaw_alignment = lag_alignment(
            run.times, imu_yaw, pointlio_yaw,
            moving & (np.abs(imu_yaw) >= 0.05),
        )
        wheel = np.asarray(auxiliary["wheel"], dtype=float)
        if len(wheel) >= 20:
            order = np.argsort(wheel[:, 0], kind="stable")
            wheel = wheel[order]
            keep = np.r_[True, np.diff(wheel[:, 0]) > 1.0e-6]
            wheel = wheel[keep]
            wheel_vx = np.interp(run.times, wheel[:, 0], wheel[:, 2])
            speed_alignment = lag_alignment(
                run.times, smooth(wheel_vx, dt, 0.16),
                smooth(run.vx, dt, 0.16), moving,
            )
        else:
            speed_alignment = lag_alignment(
                run.times, run.vx, run.vx, np.zeros(len(run.times), dtype=bool)
            )
        per_run.append({
            "name": run.name,
            "role": "primary" if index < primary_count else "holdout",
            "pointlio_yaw_rate_vs_imu": yaw_alignment,
            "pointlio_vx_vs_wheel_odom": speed_alignment,
        })
    yaw_lags = np.asarray([
        item["pointlio_yaw_rate_vs_imu"]["lag_s"] for item in per_run
    ], dtype=float)
    speed_lags = np.asarray([
        item["pointlio_vx_vs_wheel_odom"]["lag_s"] for item in per_run
    ], dtype=float)
    return {
        "lag_sign_convention": (
            "positive lag means the Point-LIO-derived observed stream follows the reference stream in header-stamp time"
        ),
        "per_run": per_run,
        "median_pointlio_yaw_rate_lag_vs_imu_s": float(np.nanmedian(yaw_lags)),
        "median_pointlio_vx_lag_vs_wheel_odom_s": float(np.nanmedian(speed_lags)),
        "interpretation": (
            "This is signal-shape alignment after header timestamps and affine scale removal; it is distinct from ROS receive age and must not be added blindly to actuator dead time."
        ),
    }


def constant_circle_validation(
    paths: Sequence[Path],
    actuator_candidate: Dict[str, object],
    directional_candidate: Optional[Dict[str, object]] = None,
    wheelbase: float = 0.320,
) -> Dict[str, object]:
    """Use historical fixed circles as steady-curvature diagnostics only."""
    gain = float(actuator_candidate["static_gain"])
    bias = float(actuator_candidate["offset_rad"])
    per_bag = []
    for path in paths:
        dataset = read_bag(path)
        sync = synchronized(dataset)
        times = sync["times"]
        dt = float(np.median(np.diff(times)))
        yaw_rate = smooth(sync["yaw_rate"], dt, 0.10)
        active = (
            (sync["speed_command"] >= 0.20)
            & (sync["vx"] >= 0.50)
            & (np.abs(yaw_rate) >= 0.05)
        )
        indices = np.flatnonzero(active)
        if len(indices) < 100:
            per_bag.append({
                "name": path.stem, "usable": False,
                "reason": "not enough steady-circle samples",
            })
            continue
        active &= times >= times[indices[0]] + 2.0
        active &= times <= times[indices[-1]] - 0.50
        equivalent = np.arctan(
            wheelbase * yaw_rate[active]
            / np.maximum(np.abs(sync["vx"][active]), 0.10)
        )
        command = sync["steering_command"][active]
        candidate = gain * command + bias
        directional = None
        if directional_candidate is not None:
            directional_gain = np.where(
                command < 0.0,
                float(directional_candidate["gain_negative"]),
                float(directional_candidate["gain_positive"]),
            )
            directional = (
                directional_gain * command
                + float(directional_candidate["offset_rad"])
            )
        radius = np.abs(sync["vx"][active] / yaw_rate[active])
        stable_error = command - equivalent
        candidate_error = candidate - equivalent
        item = {
            "name": path.stem, "path": str(path.resolve()), "usable": True,
            "samples": int(np.sum(active)),
            "median_speed_mps": float(np.median(sync["vx"][active])),
            "median_command_rad": float(np.median(command)),
            "median_equivalent_angle_rad": float(np.median(equivalent)),
            "median_radius_m": float(np.median(radius)),
            "median_lateral_accel_mps2": float(np.median(
                np.abs(sync["vx"][active] * yaw_rate[active])
            )),
            "stable_error_rad": rollout_metric(stable_error),
            "candidate_error_rad": rollout_metric(candidate_error),
            "candidate_rmse_improvement_vs_stable_percent": float(
                100.0 * (
                    rollout_metric(stable_error)["rmse"]
                    - rollout_metric(candidate_error)["rmse"]
                ) / max(rollout_metric(stable_error)["rmse"], 1.0e-12)
            ),
        }
        if directional is not None:
            directional_error = directional - equivalent
            item["directional_error_rad"] = rollout_metric(directional_error)
            item["directional_rmse_improvement_vs_stable_percent"] = float(
                100.0 * (
                    rollout_metric(stable_error)["rmse"]
                    - rollout_metric(directional_error)["rmse"]
                ) / max(rollout_metric(stable_error)["rmse"], 1.0e-12)
            )
        per_bag.append(item)
    usable = [item for item in per_bag if item.get("usable", False)]
    return {
        "method": (
            "steady-circle equivalent angle atan(L*r/vx); excluded from FOPDT fitting because a fixed command contains no persistent delay excitation"
        ),
        "per_bag": per_bag,
        "usable_bags": int(len(usable)),
        "candidate_better_than_stable_count": int(sum(
            item["candidate_error_rad"]["rmse"]
            < item["stable_error_rad"]["rmse"] for item in usable
        )),
        "directional_better_than_stable_count": int(sum(
            item.get("directional_error_rad", {}).get("rmse", float("inf"))
            < item["stable_error_rad"]["rmse"] for item in usable
        )),
        "limitations": [
            "Equivalent curvature angle includes tire sideslip and is not a front-wheel sensor measurement.",
            "These bags predate the current collection campaign and are historical diagnostics, not final acceptance data.",
        ],
    }


def write_markdown(path: Path, report: Dict[str, object]) -> None:
    dynamic = report["dynamic_actuator"]
    one_step = dynamic["joint_candidate"]
    static = report["static_mapping"]
    rollout = report["multi_step_rollout"]
    multistep = rollout["multistep_fit"]["candidate"]
    observer = report["observer_shadow_replay"]
    full = report["full_dynamic_shadow_replay"]
    tire = report["tire_stiffness_identifiability"]
    alignment = report["sensor_time_alignment"]
    circle = report["historical_constant_circle_validation"]
    directional = report["directional_gain_experiment"]
    solver_baseline = report["stable_cpp_solver_baseline"]
    lines = [
        "# TianRacer 转向执行器本地离线实验", "",
        f"生成时间：`{report['generated_at']}`", "",
        f"状态：`{report['status']}`", "",
        "本报告完成当前本地 bag 能支持的辨识和影子回放，只生成候选；没有修改 stable、vehicle.yaml、controller.yaml 或实车配置。", "",
        "## 数据划分", "",
        "- 4 个 8 月 16 日 completed/最佳里程 bag 作为 primary，只用于拟合与模型选择。",
        f"- {len(report['holdout_bags'])} 个 bag 作为 holdout：1 个 8 月 16 日暂停 bag，加 {max(len(report['holdout_bags']) - 1, 0)} 个外部历史双向转向 bag；均未参与当前候选拟合。",
        "- 暂停区间由连续运动窗口过滤，不会作为车辆响应样本。", "",
        "## 数据审计", "",
    ]
    for item in report["data_audit"]:
        topics = item["topics"]
        command_topic = max(
            ("/f1tenth_identification/command_stamped",
             "/f1tenth_mpcc/real/ackermann_cmd_stamped"),
            key=lambda name: topics.get(name, {"samples": 0})["samples"],
        )
        imu_topic = max(
            ("/livox/imu", "/tianracer/imu"),
            key=lambda name: topics.get(name, {"samples": 0})["samples"],
        )
        lines.append(
            f"- `{Path(item['bag']).name}`：completed={item['completed']}，"
            f"有效运动 {item['moving_duration_s']:.2f} s，最长暂停 "
            f"{item['longest_stopped_interval_s']:.2f} s，"
            f"cmd/IMU/vehicle_odom="
            f"{topics[command_topic]['rate_hz']:.1f}/"
            f"{topics[imu_topic]['rate_hz']:.1f}/"
            f"{topics['/localization/vehicle_odom']['rate_hz']:.1f} Hz，"
            f"Point-LIO age P95={item['pointlio_receive_age_s']['p95']:.3f} s，"
            f"位移/航向跳变={item['pose_quality']['translation_jump_events']}/"
            f"{item['pose_quality']['yaw_jump_events']}"
        )
    lines.extend([
        "", "## 传感器时间对齐（header stamp 域）", "",
        f"- Point-LIO yaw-rate 相对 IMU 的跨 bag 中位信号滞后："
        f"`{alignment['median_pointlio_yaw_rate_lag_vs_imu_s']:.3f} s`。",
        f"- Point-LIO vx 相对轮式里程计的跨 bag 中位信号滞后："
        f"`{alignment['median_pointlio_vx_lag_vs_wheel_odom_s']:.3f} s`。",
        "- 这是去除仿射比例后的信号形状对齐，不是 ROS 接收 age；不能与舵机 Td 或 Point-LIO 网络 age 直接相加。转向辨识使用 IMU yaw-rate 的 header stamp，因此没有把约 0.14 s 的 Point-LIO 接收 age 当成舵机延迟。",
        "", "## 静态映射", "",
        f"- 平台数：`{static['plateau_count']}`",
        f"- 命令覆盖：`{static['command_range_rad'][0]:.4f} .. "
        f"{static['command_range_rad'][1]:.4f} rad`",
        f"- 推荐映射：`{static['recommended_map_type']}`",
        f"- 方向覆盖：`{static['identifiability']}`",
        f"- 回差代理：`{static['hysteresis_proxy_rad']:.5f} rad`",
        "- 结论：本批总转角命令几乎只有正向平台，因此不能据此辨识左右不对称、死区或真实机械回差。",
        "", "## 历史固定圆负向转向检查", "",
    ])
    for item in circle["per_bag"]:
        if not item.get("usable", False):
            lines.append(f"- `{item['name']}`：不可用，{item['reason']}")
            continue
        lines.append(
            f"- `{item['name']}`：vx=`{item['median_speed_mps']:.2f} m/s`，"
            f"cmd=`{item['median_command_rad']:.3f} rad`，等效角="
            f"`{item['median_equivalent_angle_rad']:.3f} rad`，半径="
            f"`{item['median_radius_m']:.3f} m`；stable/candidate RMSE="
            f"`{item['stable_error_rad']['rmse']:.4f}/{item['candidate_error_rad']['rmse']:.4f} rad`，"
            f"split=`{item.get('directional_error_rad', {}).get('rmse', float('nan')):.4f} rad`"
        )
    lines.extend([
        f"- 固定圆中 candidate 优于 stable 的数量：`{circle['candidate_better_than_stable_count']}/{circle['usable_bags']}`。",
        f"- 固定圆中 exploratory split 优于 stable 的数量：`{circle['directional_better_than_stable_count']}/{circle['usable_bags']}`。",
        "- 固定圆无持续动态激励，且等效角含轮胎侧偏；它不能估计 Td/tau，但能暴露当前单一正负对称仿射增益在负向稳态上的风险。",
        "", "## 探索性正负分段增益", "",
        f"- K_neg/K_pos：`{directional['split_candidate']['gain_negative']:.5f}/"
        f"{directional['split_candidate']['gain_positive']:.5f}`，bias="
        f"`{directional['split_candidate']['offset_rad']:.5f} rad`，tau="
        f"`{directional['split_candidate']['time_constant_s']:.5f} s`。",
        f"- primary/holdout 平均动态 RMSE 改善："
        f"`{directional['primary_mean_rmse_improvement_percent']:.2f}%/"
        f"{directional['holdout_mean_rmse_improvement_percent']:.2f}%`。",
        "- 这是决定下一轮数据采集是否值得做的探索，不可部署：仍缺当前配置下独立的正负静态平台和真实前轮角。",
        "", "## 单步联合 FOPDT", "",
        f"- K：`{one_step['static_gain']:.6f}`",
        f"- b：`{one_step['offset_rad']:.6f} rad`",
        f"- Td：`{one_step['dead_time_s']:.6f} s`",
        f"- tau：`{one_step['time_constant_s']:.6f} s`",
        f"- Td + tau：`{one_step['dead_time_s'] + one_step['time_constant_s']:.6f} s`",
        f"- 动态 RMSE/R2：`{one_step['dynamic_delta_rmse_rad']:.6f} rad / {one_step['dynamic_r2']:.5f}`",
        "- 4 个 primary 的 K、b、Td、tau、Td+tau 跨 bag 一致性门槛全部通过；严格 leave-one-bag RMSE <0.020 rad 仅差约 0.0004–0.0018 rad，未严格通过。",
        "- `Td` 与 `tau` 存在明显等效脊，最可靠的是总响应时间约 0.16–0.17 s，不是二者唯一拆分。",
        "", "## 多时域输出误差候选（推荐继续验证）", "",
        f"- K：`{multistep['static_gain']:.6f}`",
        f"- b：`{multistep['offset_rad']:.6f} rad`",
        f"- Td：`{multistep['dead_time_s']:.6f} s`",
        f"- tau：`{multistep['time_constant_s']:.6f} s`",
        f"- Td + tau：`{multistep['effective_63pct_time_s']:.6f} s`",
        "- 该候选只由 4 个 primary 的 0.1/0.2/0.5/1.0 s yaw-rate、heading、lateral 输出误差拟合；holdout 未参与。",
        "", "### 舵机隔离 rollout 的 holdout 结果", "",
    ])
    for item in rollout["holdout_acceptance"]:
        if item["model"] not in (
            "stable_nominal", "historical_candidate", "joint_fopdt",
            "multistep_fopdt", "two_stage_050",
        ):
            continue
        lines.append(
            f"- `{item['model']}` on `{item['run']}`：0.2 s yaw-rate 相对 stable 改善 "
            f"`{item['yaw_rate_0p2s_improvement_percent']:.1f}%`，"
            f"0.5 s lateral P95=`{item['lateral_0p5s_p95_m']:.4f} m`"
        )
    lines.append(
        f"- multistep 所有 holdout 门槛通过："
        f"`{rollout['multistep_candidate_all_holdout_gates_pass']}`；"
        "高速度闭环 bag 未达到 yaw-rate 改善 ≥30% 门槛。"
    )
    selected_gain = observer["candidate_observer_gain_selected_on_primary_only"]
    lines.extend([
        "", "## 无转角传感器观测器影子回放", "",
    ])
    for item in observer["holdout_comparison"]:
        lines.append(
            f"- `{item['model']}` on `{item['run']}`：delta_hat RMSE=`{item['delta_hat_rmse_rad']:.5f} rad`，"
            f"相对 stable 改善=`{item['improvement_vs_stable_percent']:.1f}%`"
        )
    lines.extend([
        f"- 只按 primary 选择的增益网格结果：kinematic=`{selected_gain['kinematic_gain_max']:.2f}`，"
        f"dynamic=`{selected_gain['dynamic_gain_max']:.2f}`；其 holdout RMSE="
        f"`{selected_gain['holdout_group_balanced_rmse_rad']:.5f} rad`。",
        "- 当前修正对新候选只带来约 1% 量级变化；1.5 m/s completed bag 上动态修正还略微恶化。因此不能把运动学伪测量当作真实转角反馈。",
        "", "## 整车动力学开环影子回放", "",
    ])
    for item in full["holdout_comparison"]:
        lines.append(
            f"- `{item['scenario']}` on `{item['run']}`：0.2 s yaw-rate RMSE="
            f"`{item['yaw_rate_0p2s_rmse_radps']:.4f} rad/s`，0.5 s lateral P95="
            f"`{item['lateral_0p5s_p95_m']:.4f} m`，1.0 s position P95="
            f"`{item['position_1p0s_p95_m']:.4f} m`"
        )
    lines.extend([
        "- 3 个专用转向 holdout 上，新舵机模型单独使用时 yaw-rate 改善约 33–48%，lateral 改善约 9–22%。",
        "- 两个约 2–3.15 m/s 的历史闭环 holdout 上，新舵机模型单独使用时 yaw-rate 反而恶化约 7–9%，lateral 恶化约 1–6%；当前候选没有高速泛化证据。",
        "- 旧 V1 残差能缓解高速度影子误差，但在多份 0.6 m/s 数据上把 1 s position P95 放大到约 0.25–0.34 m；说明其速度域兼容性不成立，不能直接与新舵机模型绑定部署。",
        "", "## 当前 stable C++/acados 闭环基线", "",
    ])
    if solver_baseline["results"]:
        for item in solver_baseline["results"]:
            lines.append(
                f"- `{item['variant']}`：solves={item['solves']}，failures={item['failures']}，"
                f"status4={item['status4']}，laps={item['completed_laps']:.3f}，"
                f"minimum margin={item['minimum_track_margin_m']:.3f} m，"
                f"solve P95/max={item['solve_p95_ms']:.3f}/{item['solve_max_ms']:.3f} ms"
            )
    else:
        lines.append("- 未找到 machine-readable solver 基线；请先运行 `run_cpp_delay_matrix.sh <output.jsonl>`。")
    lines.extend([
        "- 该矩阵只验证当前 stable OCP；新候选约 95 ms 的纯延迟尚未写入 OCP，因此 `candidate_delay_model_tested=false`。",
        "", "## 轮胎侧偏刚度可辨识性", "",
        f"- 通过力平衡基本门槛的 bag：`{tire['accepted_fit_count']}`。",
    ])
    for item in tire["per_run"]:
        detail = item.get("rejection_reason", "accepted")
        if "Cf_N_per_rad" in item:
            detail += (
                f"；Cf={item['Cf_N_per_rad']:.2f}, Cr={item['Cr_N_per_rad']:.2f} N/rad，"
                f"R2f/R2r={item['front_force_r2']:.3f}/{item['rear_force_r2']:.3f}"
            )
        lines.append(f"- `{item['name']}`：{detail}")
    lines.extend([
        f"- 结论：{tire['conclusion']}",
        "", "## 最终判定与部署门槛", "",
        "- 当前本地证据仅支持把 `K=1.11323, b=-0.01117 rad, Td=0.095 s, tau=0.06666 s` 保留为 0.6–1.5 m/s 的瞬态候选；高速度门槛失败，禁止直接部署到 MPCC。",
        "- 当前证据不支持修改 Cf/Cr，也不支持宣称已辨识 1.5、3.4 或 5.5 rad/s 的机械硬转角速率。",
        "- 不自动部署候选：除 OCP 需因果表示约 95 ms 纯延迟外，还必须先解决高速横向动力学/Cf-Cr 不可辨识和旧残差低速失配。",
        "- 下一轮至少需要：当前配置下独立左右静态平台、2–3 m/s 安全 PRBS、全新未见高速 final-test bag、candidate solver 闭环仿真回归，以及 stable 一键回退。",
        "", "## 复现实验", "",
        "```bash",
        "cd /home/ros/f1tenth_residual_controller_ws",
        "./scripts/run_steering_offline_experiments.sh",
        "```",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_report(path: Path, report: Dict[str, object]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    static = report["static_mapping"]
    dynamic = report["dynamic_actuator"]
    figure, axes = plt.subplots(2, 2, figsize=(13, 9))
    command = np.asarray(static["plateau_command_rad"], dtype=float)
    target = np.asarray(static["plateau_equivalent_angle_rad"], dtype=float)
    grid = np.linspace(np.min(command), np.max(command), 200)
    axes[0, 0].scatter(command, target, s=28, label="plateau medians")
    for model in static["models"]:
        parameters = model["parameters"]
        if model["kind"] == "linear":
            vector = np.asarray([parameters["gain"], parameters["bias_rad"]])
        elif model["kind"] == "split_gain":
            vector = np.asarray([parameters["gain_negative"], parameters["gain_positive"], parameters["bias_rad"]])
        else:
            vector = np.asarray([parameters["gain"], parameters["bias_rad"], parameters["deadband_rad"]])
        axes[0, 0].plot(grid, static_model_value(model["kind"], vector, grid),
                        label=model["kind"])
    axes[0, 0].set_title("Static-map hypotheses")
    axes[0, 0].set_xlabel("command [rad]")
    axes[0, 0].set_ylabel("equivalent angle [rad]")
    axes[0, 0].grid(True); axes[0, 0].legend()

    delay = np.asarray([item["dead_time_s"] for item in dynamic["delay_tau_profile"]])
    score = np.asarray([item["relative_rmse_increase_percent"] for item in dynamic["delay_tau_profile"]])
    tau = np.asarray([item["time_constant_s"] for item in dynamic["delay_tau_profile"]])
    axes[0, 1].plot(delay, score, marker="o")
    axes[0, 1].set_title("Delay profile")
    axes[0, 1].set_xlabel("Td [s]"); axes[0, 1].set_ylabel("RMSE increase [%]")
    axes[0, 1].grid(True)
    axes[1, 0].plot(delay, tau, marker="o", label="tau")
    axes[1, 0].plot(delay, delay + tau, marker="o", label="Td + tau")
    axes[1, 0].set_title("Delay/time-constant ridge")
    axes[1, 0].set_xlabel("Td [s]"); axes[1, 0].set_ylabel("time [s]")
    axes[1, 0].grid(True); axes[1, 0].legend()

    names, stable_rmse, candidate_rmse = [], [], []
    for item in dynamic["per_run_model_comparison"]:
        names.append(item["name"].replace("track_", ""))
        stable_rmse.append(item["models"]["stable_nominal"]["rmse_rad"])
        candidate_rmse.append(item["models"]["joint_candidate"]["rmse_rad"])
    x = np.arange(len(names)); width = 0.38
    axes[1, 1].bar(x - width / 2, stable_rmse, width, label="stable")
    axes[1, 1].bar(x + width / 2, candidate_rmse, width, label="candidate")
    axes[1, 1].set_xticks(x); axes[1, 1].set_xticklabels(names, rotation=25, ha="right")
    axes[1, 1].set_ylabel("equivalent-angle RMSE [rad]")
    axes[1, 1].set_title("Per-bag generalization")
    axes[1, 1].grid(True, axis="y"); axes[1, 1].legend()
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)

    rollout = report.get("multi_step_rollout")
    if not rollout:
        return
    holdouts = [item for item in rollout["per_run"] if item["role"] == "holdout"]
    if not holdouts:
        return
    item = holdouts[0]
    model_names = list(item["models"])
    horizons = ["0.10s", "0.20s", "0.50s", "1.00s"]
    figure, axes = plt.subplots(2, 2, figsize=(13, 8))
    metrics = [
        ("yaw_rate_error_radps", "rmse", "yaw-rate RMSE [rad/s]"),
        ("heading_error_rad", "rmse", "heading RMSE [rad]"),
        ("lateral_error_m", "p95_abs", "lateral P95 [m]"),
        ("position_error_m", "p95_abs", "position P95 [m]"),
    ]
    x = np.arange(len(horizons))
    for axis, (metric, field, label) in zip(axes.flat, metrics):
        for model_name in model_names:
            values = [item["models"][model_name][horizon][metric][field]
                      for horizon in horizons]
            axis.plot(x, values, marker="o", label=model_name)
        axis.set_xticks(x); axis.set_xticklabels(horizons)
        axis.set_ylabel(label); axis.grid(True)
    axes[0, 0].legend(fontsize=7, ncol=2)
    figure.suptitle("Held-out multi-step rollout: " + item["name"])
    figure.tight_layout()
    figure.savefig(path.with_name(path.stem + "_rollout.png"), dpi=150)
    plt.close(figure)

    full_holdouts = [
        item for item in report["full_dynamic_shadow_replay"]["per_run"]
        if item["role"] == "holdout"
    ]
    if not full_holdouts:
        return
    item = full_holdouts[0]
    scenario_names = list(item["scenarios"])
    figure, axes = plt.subplots(2, 2, figsize=(13, 8))
    full_metrics = [
        ("yaw_rate_error_radps", "rmse", "yaw-rate RMSE [rad/s]"),
        ("lateral_error_m", "p95_abs", "lateral P95 [m]"),
        ("position_error_m", "p95_abs", "position P95 [m]"),
    ]
    for axis, (metric, field, label) in zip(axes.flat[:3], full_metrics):
        for scenario_name in scenario_names:
            values = [
                item["scenarios"][scenario_name][horizon][metric][field]
                for horizon in horizons
            ]
            axis.plot(x, values, marker="o", label=scenario_name)
        axis.set_xticks(x); axis.set_xticklabels(horizons)
        axis.set_ylabel(label); axis.grid(True)
    axes[0, 0].legend(fontsize=7)

    grid = report["observer_shadow_replay"]["candidate_observer_gain_grid"]
    kinematic_values = sorted({item["kinematic_gain_max"] for item in grid})
    dynamic_values = sorted({item["dynamic_gain_max"] for item in grid})
    heat = np.full((len(dynamic_values), len(kinematic_values)), np.nan)
    for value in grid:
        row = dynamic_values.index(value["dynamic_gain_max"])
        column = kinematic_values.index(value["kinematic_gain_max"])
        heat[row, column] = value["primary_group_balanced_rmse_rad"]
    image = axes[1, 1].imshow(heat, origin="lower", aspect="auto")
    axes[1, 1].set_xticks(np.arange(len(kinematic_values)))
    axes[1, 1].set_xticklabels([f"{value:.2f}" for value in kinematic_values])
    axes[1, 1].set_yticks(np.arange(len(dynamic_values)))
    axes[1, 1].set_yticklabels([f"{value:.2f}" for value in dynamic_values])
    axes[1, 1].set_xlabel("kinematic gain max")
    axes[1, 1].set_ylabel("dynamic gain max")
    axes[1, 1].set_title("Observer primary RMSE [rad]")
    figure.colorbar(image, ax=axes[1, 1])
    figure.suptitle("Held-out full dynamics and primary-only observer tuning")
    figure.tight_layout()
    figure.savefig(path.with_name(path.stem + "_shadow.png"), dpi=150)
    plt.close(figure)

    high_speed = [
        value for value in full_holdouts
        if "mpcc" in value["name"]
    ]
    if not high_speed:
        return
    figure, axes = plt.subplots(2, 2, figsize=(14, 9))
    selected_scenarios = (
        "stable_physics", "multistep_physics",
        "multistep_plus_old_v1_residual",
    )
    for axis, (metric, field, label) in zip(axes.flat[:3], full_metrics):
        for value in high_speed:
            short_name = value["name"].replace("lateral_mpcc_20260814_", "mpcc_")
            for scenario_name in selected_scenarios:
                points = [
                    value["scenarios"][scenario_name][horizon][metric][field]
                    for horizon in horizons
                ]
                axis.plot(
                    x, points, marker="o",
                    label=f"{short_name}:{scenario_name}",
                )
        axis.set_xticks(x); axis.set_xticklabels(horizons)
        axis.set_ylabel(label); axis.grid(True)
    axes[0, 0].legend(fontsize=6, ncol=2)

    comparison = [
        value for value in report["full_dynamic_shadow_replay"]["holdout_comparison"]
        if "mpcc" in value["run"] and value["scenario"] == "multistep_physics"
    ]
    labels = [value["run"].replace("lateral_mpcc_20260814_", "mpcc_")
              for value in comparison]
    positions = np.arange(len(comparison))
    width = 0.36
    axes[1, 1].bar(
        positions - width / 2,
        [value["yaw_rate_improvement_vs_stable_percent"] for value in comparison],
        width, label="yaw-rate improvement",
    )
    axes[1, 1].bar(
        positions + width / 2,
        [value["lateral_improvement_vs_stable_percent"] for value in comparison],
        width, label="lateral improvement",
    )
    axes[1, 1].axhline(0.0, color="black", linewidth=1)
    axes[1, 1].set_xticks(positions); axes[1, 1].set_xticklabels(labels, rotation=20)
    axes[1, 1].set_ylabel("improvement vs stable [%]")
    axes[1, 1].set_title("Candidate high-speed generalization")
    axes[1, 1].grid(True, axis="y"); axes[1, 1].legend(fontsize=7)
    figure.suptitle("Historical 2--3.15 m/s full-dynamics shadow replay")
    figure.tight_layout()
    figure.savefig(path.with_name(path.stem + "_high_speed_shadow.png"), dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--external-bag", type=Path, action="append", default=[])
    parser.add_argument("--circle-bag", type=Path, action="append", default=[])
    args = parser.parse_args()
    data_dir = args.data_dir.expanduser().resolve()
    output = args.output.expanduser().resolve()
    primary_paths, holdout_paths, selection = select_bags(data_dir)
    selected_paths = {path.resolve() for path in primary_paths + holdout_paths}
    external_paths = []
    for path in args.external_bag:
        resolved = path.expanduser().resolve()
        if not resolved.exists():
            raise FileNotFoundError(resolved)
        if resolved not in selected_paths:
            external_paths.append(resolved)
            selected_paths.add(resolved)
    holdout_paths = holdout_paths + external_paths
    selection["external_holdouts"] = [str(path) for path in external_paths]
    all_paths = primary_paths + holdout_paths
    datasets = [read_bag(path) for path in all_paths]
    syncs = [synchronized(dataset) for dataset in datasets]
    auxiliaries = [read_auxiliary(path) for path in all_paths]
    runs = [
        prepare_actuator_run(
            sync["times"], sync["steering_command"], sync["vx"], sync["yaw_rate"],
            name=path.stem, source=sync["yaw_rate_source"],
            platform_signal=sync["identification_excitation"],
        )
        for path, sync in zip(all_paths, syncs)
    ]
    primary_runs = runs[:len(primary_paths)]
    holdout_runs = runs[len(primary_paths):]
    static_run = primary_runs[PRIMARY_PROFILES.index("track_static_060")]
    static_report = static_mapping_experiment(static_run)
    dynamic_report, _ = dynamic_experiment(primary_runs, holdout_runs)
    rollout_report = multi_step_experiment(
        primary_runs, holdout_runs, syncs, auxiliaries, dynamic_report
    )
    directional_report = directional_mapping_experiment(
        primary_runs, holdout_runs,
        rollout_report["multistep_fit"]["candidate"],
    )
    workspace_root = Path(__file__).resolve().parents[1]
    package_root = workspace_root / "src/f1tenth_dynamic_mpcc"
    actuator_models = [
        RolloutActuatorModel(**values) for values in rollout_report["models"]
    ]
    observer_report = observer_shadow_experiment(
        runs, syncs, len(primary_runs), actuator_models,
        package_root / "config/vehicle.yaml",
        package_root / "config/controller.yaml",
    )
    full_dynamic_report = full_dynamic_experiment(
        runs, syncs, auxiliaries, len(primary_runs), actuator_models,
        package_root / "config/vehicle.yaml",
        package_root / "config/controller.yaml",
        package_root / "config/residual/residual_markov_v1.yaml",
    )
    tire_report = tire_identifiability_experiment(
        all_paths, syncs, len(primary_runs),
        rollout_report["multistep_fit"]["candidate"],
    )
    alignment_report = sensor_alignment_experiment(
        runs, syncs, auxiliaries, len(primary_runs)
    )
    circle_paths = [path.expanduser().resolve() for path in args.circle_bag]
    missing_circle = [path for path in circle_paths if not path.exists()]
    if missing_circle:
        raise FileNotFoundError(missing_circle[0])
    circle_report = constant_circle_validation(
        circle_paths, rollout_report["multistep_fit"]["candidate"],
        directional_report["split_candidate"],
    )
    solver_baseline_path = data_dir / "cpp_stable_delay_matrix.jsonl"
    status = (
        "offline_stage_5_all_local_data_complete_candidate_low_speed_only_high_speed_gate_failed_not_applied"
        if not rollout_report["multistep_candidate_all_holdout_gates_pass"]
        else "offline_stage_5_all_local_data_complete_candidate_not_applied"
    )
    report = finite({
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "status": status,
        "data_directory": str(data_dir),
        "selection": selection,
        "primary_bags": [str(path.resolve()) for path in primary_paths],
        "holdout_bags": [str(path.resolve()) for path in holdout_paths],
        "data_audit": [
            audit_bag(path, sync, auxiliary)
            for path, sync, auxiliary in zip(all_paths, syncs, auxiliaries)
        ],
        "static_mapping": static_report,
        "dynamic_actuator": dynamic_report,
        "multi_step_rollout": rollout_report,
        "directional_gain_experiment": directional_report,
        "observer_shadow_replay": observer_report,
        "full_dynamic_shadow_replay": full_dynamic_report,
        "tire_stiffness_identifiability": tire_report,
        "sensor_time_alignment": alignment_report,
        "historical_constant_circle_validation": circle_report,
        "stable_cpp_solver_baseline": {
            "path": str(solver_baseline_path),
            "results": load_json_lines(solver_baseline_path),
            "candidate_delay_model_tested": False,
        },
        "limitations": [
            "No physical steering-angle sensor is present.",
            "Point-LIO pose is used only with message header time; receive time is audited separately.",
            "No controller configuration is modified by this tool.",
        ],
    })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(output.with_suffix(".md"), report)
    plot_report(output.with_suffix(".png"), report)
    print(json.dumps({
        "status": report["status"],
        "output": str(output),
        "primary_bags": report["primary_bags"],
        "holdout_bags": report["holdout_bags"],
        "joint_candidate": report["dynamic_actuator"]["joint_candidate"],
        "multistep_candidate": report["multi_step_rollout"]["multistep_fit"]["candidate"],
        "multistep_holdout_gates_pass": report["multi_step_rollout"]["multistep_candidate_all_holdout_gates_pass"],
        "static_map": report["static_mapping"]["recommended_map_type"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
