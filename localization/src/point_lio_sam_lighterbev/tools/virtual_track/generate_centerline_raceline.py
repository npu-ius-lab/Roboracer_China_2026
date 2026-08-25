#!/usr/bin/env python3
"""Build a centerline and NMPC-compatible raceline from paired boundaries."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize

from virtual_track_common import (
    RACELINE_FIELDS,
    closed_arclength,
    heading_curvature_closed,
    left_normals,
    load_ros_map,
    periodic_resample,
    portable_path,
    read_numeric_csv,
    world_to_local,
    write_csv,
)


def optimize_raceline(
    center_x: np.ndarray,
    center_y: np.ndarray,
    w_left: np.ndarray,
    w_right: np.ndarray,
    vehicle_width: float,
    track_margin: float,
    curvature_weight: float,
    smooth_weight: float,
    magnitude_weight: float,
    max_iterations: int,
    control_points: int,
    minimum_tracker_margin: float,
    enabled: bool,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    _, center_length = closed_arclength(center_x, center_y)
    psi_center, _ = heading_curvature_closed(center_x, center_y, center_length)
    normal_x, normal_y = left_normals(psi_center)
    half_vehicle = 0.5 * vehicle_width
    lower = -w_right + half_vehicle + track_margin
    upper = w_left - half_vehicle - track_margin
    if np.any(lower >= upper):
        bad = int(np.argmax(lower >= upper))
        raise ValueError(
            f"Track is too narrow after vehicle and margin shrink at sample {bad}."
        )

    control_points = max(8, min(int(control_points), len(center_x)))
    control_phase = np.linspace(0.0, 1.0, control_points, endpoint=False)
    sample_phase = np.linspace(0.0, 1.0, len(center_x), endpoint=False)
    numerical_guard_m = 0.002
    global_lower = float(np.max(lower)) + minimum_tracker_margin + numerical_guard_m
    global_upper = float(np.min(upper)) - minimum_tracker_margin - numerical_guard_m
    if global_lower >= global_upper:
        raise ValueError(
            "Track is too narrow for the requested vehicle, planning buffer, and "
            "minimum tracker boundary margin."
        )

    def controls_to_offsets(controls: np.ndarray) -> np.ndarray:
        offsets = CubicSpline(
            np.r_[control_phase, 1.0],
            np.r_[controls, controls[0]],
            bc_type="periodic",
        )(sample_phase)
        return np.clip(offsets, global_lower, global_upper)

    def evaluate(controls: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        offsets = controls_to_offsets(controls)
        x = center_x + offsets * normal_x
        y = center_y + offsets * normal_y
        _, length = closed_arclength(x, y)
        _, kappa = heading_curvature_closed(x, y, length)
        return x, y, kappa, offsets

    def objective(controls: np.ndarray) -> float:
        _, _, kappa, offsets = evaluate(controls)
        first_difference = offsets - np.roll(offsets, 1)
        return float(
            curvature_weight * np.mean(kappa * kappa)
            + smooth_weight * np.mean(first_difference * first_difference)
            + magnitude_weight * np.mean(offsets * offsets)
        )

    if enabled:
        result = minimize(
            objective,
            x0=np.zeros(control_points, dtype=float),
            method="L-BFGS-B",
            bounds=[(global_lower, global_upper)] * control_points,
            options={
                "maxiter": int(max_iterations),
                "maxfun": max(10000, 500 * control_points),
                "ftol": 1.0e-10,
                "maxls": 40,
            },
        )
        controls = np.asarray(result.x, dtype=float)
        optimizer = {
            "enabled": True,
            "control_points": control_points,
            "minimum_tracker_boundary_margin_m": minimum_tracker_margin,
            "numerical_guard_m": numerical_guard_m,
            "success": bool(result.success),
            "status": int(result.status),
            "iterations": int(result.nit),
            "objective": float(result.fun),
            "message": str(result.message),
        }
    else:
        controls = np.zeros(control_points, dtype=float)
        optimizer = {
            "enabled": False,
            "control_points": control_points,
            "minimum_tracker_boundary_margin_m": minimum_tracker_margin,
            "numerical_guard_m": numerical_guard_m,
            "success": True,
            "status": 0,
            "iterations": 0,
            "objective": objective(controls),
            "message": "centerline used as raceline by request",
        }

    race_x, race_y, _, offsets = evaluate(controls)
    return {
        "x_m": race_x,
        "y_m": race_y,
        "offset_m": offsets,
        "w_left_m": w_left - offsets,
        "w_right_m": w_right + offsets,
    }, optimizer


def steering_rate_speed_limit(
    kappa: np.ndarray,
    length: float,
    wheelbase: float,
    max_steer_rate: float,
    min_speed: float,
    max_speed: float,
) -> np.ndarray:
    ds = length / len(kappa)
    steer = np.arctan(wheelbase * kappa)
    steer_slope = np.abs(np.roll(steer, -1) - np.roll(steer, 1)) / (2.0 * ds)
    return np.clip(
        max_steer_rate / np.maximum(steer_slope, 1.0e-9),
        min_speed,
        max_speed,
    )


def build_speed_profile(
    kappa: np.ndarray,
    length: float,
    wheelbase: float,
    min_speed: float,
    max_speed: float,
    max_steer: float,
    max_steer_rate: float,
    max_accel: float,
    max_decel: float,
    lateral_accel_limit: float,
) -> np.ndarray:
    curvature_speed = np.sqrt(
        lateral_accel_limit / np.maximum(np.abs(kappa), 1.0e-4)
    )
    implied_steer = np.abs(np.arctan(wheelbase * kappa))
    steer_speed = np.where(implied_steer <= 0.90 * max_steer, max_speed, min_speed)
    rate_speed = steering_rate_speed_limit(
        kappa,
        length,
        wheelbase,
        max_steer_rate,
        min_speed,
        max_speed,
    )
    speed = np.clip(
        np.minimum.reduce([curvature_speed, steer_speed, rate_speed]),
        min_speed,
        max_speed,
    )
    ds = length / len(speed)
    # Forward/backward passes are circular because the trajectory is closed.
    for _ in range(max(8, 2 * len(speed))):
        previous = speed.copy()
        for i in range(len(speed)):
            j = (i + 1) % len(speed)
            speed[j] = min(
                speed[j], math.sqrt(speed[i] * speed[i] + 2.0 * max_accel * ds)
            )
        for i in range(len(speed) - 1, -1, -1):
            j = (i - 1) % len(speed)
            speed[j] = min(
                speed[j], math.sqrt(speed[i] * speed[i] + 2.0 * max_decel * ds)
            )
        if float(np.max(np.abs(speed - previous))) < 1.0e-5:
            break
    return np.clip(speed, min_speed, max_speed)


def load_vehicle_defaults(path: Path | None) -> dict[str, float | int]:
    if path is None:
        return {}
    resolved = path.expanduser().resolve()
    with resolved.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or int(config.get("schema_version", 0)) != 1:
        raise ValueError("vehicle config must be a schema_version=1 mapping")
    for section in ("vehicle", "limits", "dynamic_model", "planning"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"vehicle config is missing mapping section: {section}")
    vehicle = config["vehicle"]
    limits = config["limits"]
    dynamic_model = config["dynamic_model"]
    planning = config["planning"]
    return {
        "raceline_points": int(planning["raceline_points"]),
        "wheelbase_m": float(vehicle["wheelbase"]),
        "vehicle_width_m": float(vehicle["ego_width"]),
        "track_margin_m": float(limits["track_margin"]),
        "min_speed_mps": float(limits["min_speed"]),
        "max_speed_mps": float(limits["max_speed"]),
        "max_steer_rad": float(limits["max_steer"]),
        "max_steer_rate_radps": float(limits["max_steer_rate"]),
        "max_accel_mps2": float(limits["max_accel"]),
        "max_decel_mps2": float(limits["max_decel"]),
        "lateral_accel_limit_mps2": float(limits["lateral_accel_limit"]),
        "friction_coefficient": float(dynamic_model["friction_coefficient"]),
        "gravity_mps2": float(dynamic_model["gravity"]),
        "friction_usage_fraction": float(planning["friction_usage_fraction"]),
        "min_tracker_boundary_margin_m": float(
            planning["minimum_tracker_boundary_margin"]
        ),
    }


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    map_dir = script_dir.parents[1] / "point_lio" / "localization_map"
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--vehicle-config", type=Path, default=None)
    config_args, _ = config_parser.parse_known_args()
    vehicle_defaults = load_vehicle_defaults(config_args.vehicle_config)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle-config", type=Path, default=config_args.vehicle_config)
    parser.add_argument(
        "--corridor-csv",
        type=Path,
        default=map_dir / "virtual_track" / "virtual_track_corridor.csv",
    )
    parser.add_argument("--map-yaml", type=Path, default=map_dir / "point_lio_map_2d.yaml")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--raceline-points",
        type=int,
        default=vehicle_defaults.get("raceline_points", 260),
    )
    parser.add_argument("--centerline-only", action="store_true")
    parser.add_argument(
        "--wheelbase-m", type=float, default=vehicle_defaults.get("wheelbase_m", 0.265)
    )
    parser.add_argument(
        "--vehicle-width-m",
        type=float,
        default=vehicle_defaults.get("vehicle_width_m", 0.22),
    )
    parser.add_argument(
        "--track-margin-m",
        type=float,
        default=vehicle_defaults.get("track_margin_m", 0.08),
    )
    parser.add_argument(
        "--min-speed-mps",
        type=float,
        default=vehicle_defaults.get("min_speed_mps", 0.20),
    )
    parser.add_argument(
        "--max-speed-mps",
        type=float,
        default=vehicle_defaults.get("max_speed_mps", 1.85),
    )
    parser.add_argument(
        "--max-steer-rad",
        type=float,
        default=vehicle_defaults.get("max_steer_rad", 0.55),
    )
    parser.add_argument(
        "--max-steer-rate-radps",
        type=float,
        default=vehicle_defaults.get("max_steer_rate_radps", 5.5),
    )
    parser.add_argument(
        "--max-accel-mps2",
        type=float,
        default=vehicle_defaults.get("max_accel_mps2", 2.4),
    )
    parser.add_argument(
        "--max-decel-mps2",
        type=float,
        default=vehicle_defaults.get("max_decel_mps2", 4.0),
    )
    parser.add_argument(
        "--lateral-accel-limit-mps2",
        type=float,
        default=vehicle_defaults.get("lateral_accel_limit_mps2", 3.2),
    )
    parser.add_argument(
        "--friction-coefficient",
        type=float,
        default=vehicle_defaults.get("friction_coefficient", 0.90),
    )
    parser.add_argument(
        "--gravity-mps2",
        type=float,
        default=vehicle_defaults.get("gravity_mps2", 9.81),
    )
    parser.add_argument(
        "--friction-usage-fraction",
        type=float,
        default=vehicle_defaults.get("friction_usage_fraction", 0.40),
    )
    parser.add_argument("--curvature-weight", type=float, default=1.0)
    parser.add_argument("--offset-smooth-weight", type=float, default=0.12)
    parser.add_argument("--offset-magnitude-weight", type=float, default=0.06)
    parser.add_argument("--max-optimizer-iterations", type=int, default=260)
    parser.add_argument("--optimizer-control-points", type=int, default=24)
    parser.add_argument(
        "--min-tracker-boundary-margin-m",
        "--optimizer-extra-margin-m",
        dest="min_tracker_boundary_margin_m",
        type=float,
        default=vehicle_defaults.get("min_tracker_boundary_margin_m", 0.25),
        help=(
            "remaining clearance required after half vehicle width and track margin; "
            "0.25 m satisfies the current NMPC simulator and Tianracer prechecks"
        ),
    )
    args = parser.parse_args()
    if args.output_dir is None:
        args.output_dir = args.corridor_csv.parent
    if args.raceline_points < 40:
        parser.error("--raceline-points must be at least 40 for stable curvature.")
    if not 0.05 <= args.friction_coefficient <= 2.0:
        parser.error("--friction-coefficient must be in [0.05, 2.0].")
    if args.gravity_mps2 <= 0.0:
        parser.error("--gravity-mps2 must be positive.")
    if not 0.0 < args.friction_usage_fraction <= 1.0:
        parser.error("--friction-usage-fraction must be in (0, 1].")
    return args


def main() -> int:
    args = parse_args()
    friction_lateral_limit = (
        args.friction_coefficient * args.gravity_mps2 * args.friction_usage_fraction
    )
    effective_lateral_accel_limit = min(
        args.lateral_accel_limit_mps2, friction_lateral_limit
    )
    corridor_path = args.corridor_csv.expanduser().resolve()
    corridor = read_numeric_csv(corridor_path)
    required = {
        "center_x_m",
        "center_y_m",
        "w_left_m",
        "w_right_m",
        "left_boundary_x_m",
        "left_boundary_y_m",
        "right_boundary_x_m",
        "right_boundary_y_m",
    }
    missing = sorted(required - set(corridor))
    if missing:
        raise ValueError(f"Corridor CSV is missing columns: {', '.join(missing)}")

    s_center, center_x, center_y, extras, center_length = periodic_resample(
        corridor["center_x_m"],
        corridor["center_y_m"],
        args.raceline_points,
        corridor["w_left_m"],
        corridor["w_right_m"],
        corridor["left_boundary_x_m"],
        corridor["left_boundary_y_m"],
        corridor["right_boundary_x_m"],
        corridor["right_boundary_y_m"],
    )
    (
        w_left,
        w_right,
        left_x,
        left_y,
        right_x,
        right_y,
    ) = extras
    psi_center, kappa_center = heading_curvature_closed(
        center_x, center_y, center_length
    )

    raceline_seed, optimizer = optimize_raceline(
        center_x,
        center_y,
        w_left,
        w_right,
        args.vehicle_width_m,
        args.track_margin_m,
        args.curvature_weight,
        args.offset_smooth_weight,
        args.offset_magnitude_weight,
        args.max_optimizer_iterations,
        args.optimizer_control_points,
        args.min_tracker_boundary_margin_m,
        enabled=not args.centerline_only,
    )
    s_race, race_x, race_y, race_extras, race_length = periodic_resample(
        raceline_seed["x_m"],
        raceline_seed["y_m"],
        args.raceline_points,
        raceline_seed["w_left_m"],
        raceline_seed["w_right_m"],
        raceline_seed["offset_m"],
    )
    race_w_left, race_w_right, race_offset = race_extras
    psi, kappa = heading_curvature_closed(race_x, race_y, race_length)
    speed = build_speed_profile(
        kappa,
        race_length,
        args.wheelbase_m,
        args.min_speed_mps,
        args.max_speed_mps,
        args.max_steer_rad,
        args.max_steer_rate_radps,
        args.max_accel_mps2,
        args.max_decel_mps2,
        effective_lateral_accel_limit,
    )
    ds = race_length / len(speed)
    accel = (np.roll(speed, -1) ** 2 - speed**2) / (2.0 * ds)
    steer = np.arctan(args.wheelbase_m * kappa)
    steer_rate = (
        (np.roll(steer, -1) - np.roll(steer, 1)) / (2.0 * ds) * speed
    )

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    centerline_csv = output_dir / "centerline.csv"
    centerline_fields = [
        "s_m",
        "x_m",
        "y_m",
        "psi_rad",
        "kappa_radpm",
        "w_tr_right_m",
        "w_tr_left_m",
        "right_boundary_x_m",
        "right_boundary_y_m",
        "left_boundary_x_m",
        "left_boundary_y_m",
    ]
    write_csv(
        centerline_csv,
        centerline_fields,
        (
            {
                "s_m": f"{s_center[i]:.9f}",
                "x_m": f"{center_x[i]:.9f}",
                "y_m": f"{center_y[i]:.9f}",
                "psi_rad": f"{psi_center[i]:.9f}",
                "kappa_radpm": f"{kappa_center[i]:.9f}",
                "w_tr_right_m": f"{w_right[i]:.9f}",
                "w_tr_left_m": f"{w_left[i]:.9f}",
                "right_boundary_x_m": f"{right_x[i]:.9f}",
                "right_boundary_y_m": f"{right_y[i]:.9f}",
                "left_boundary_x_m": f"{left_x[i]:.9f}",
                "left_boundary_y_m": f"{left_y[i]:.9f}",
            }
            for i in range(len(center_x))
        ),
    )
    raceline_csv = output_dir / "raceline.csv"
    write_csv(
        raceline_csv,
        RACELINE_FIELDS,
        (
            {
                "s_m": f"{s_race[i]:.9f}",
                "x_m": f"{race_x[i]:.9f}",
                "y_m": f"{race_y[i]:.9f}",
                "psi_rad": f"{psi[i]:.9f}",
                "kappa_radpm": f"{kappa[i]:.9f}",
                "vx_mps": f"{speed[i]:.9f}",
                "ax_mps2": f"{accel[i]:.9f}",
                "w_tr_right_m": f"{race_w_right[i]:.9f}",
                "w_tr_left_m": f"{race_w_left[i]:.9f}",
            }
            for i in range(len(race_x))
        ),
    )

    min_margin = float(
        np.min(np.minimum(race_w_left, race_w_right))
        - 0.5 * args.vehicle_width_m
        - args.track_margin_m
    )
    max_abs_steer = float(np.max(np.abs(steer)))
    max_abs_steer_rate = float(np.max(np.abs(steer_rate)))
    feasible = bool(
        min_margin >= args.min_tracker_boundary_margin_m - 1.0e-8
        and max_abs_steer <= args.max_steer_rad + 1.0e-8
        and max_abs_steer_rate <= args.max_steer_rate_radps * 1.01
        and float(np.min(speed)) >= args.min_speed_mps - 1.0e-8
        and float(np.max(speed)) <= args.max_speed_mps + 1.0e-8
    )
    planning_status = (
        "feasible_converged"
        if feasible and optimizer["success"]
        else "feasible_unconverged_seed"
        if feasible
        else "rejected"
    )

    image, meta, _ = load_ros_map(args.map_yaml.expanduser().resolve())
    resolution = float(meta["resolution"])
    center_x_l, center_y_l = world_to_local(center_x, center_y, meta)
    race_x_l, race_y_l = world_to_local(race_x, race_y, meta)
    left_x_l, left_y_l = world_to_local(left_x, left_y, meta)
    right_x_l, right_y_l = world_to_local(right_x, right_y, meta)
    debug_png = output_dir / "centerline_raceline_debug.png"
    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    extent = [0.0, image.shape[1] * resolution, 0.0, image.shape[0] * resolution]
    ax.imshow(np.flipud(image), cmap="gray", origin="lower", extent=extent)
    ax.plot(left_x_l, left_y_l, color="#16a34a", linewidth=1.6, label="left boundary")
    ax.plot(right_x_l, right_y_l, color="#7c3aed", linewidth=1.6, label="right boundary")
    ax.plot(center_x_l, center_y_l, color="#f97316", linestyle="--", linewidth=1.2, label="centerline")
    ax.plot(race_x_l, race_y_l, color="#2563eb", linewidth=1.8, label="NMPC raceline")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map-local x [m]")
    ax.set_ylabel("map-local y [m]")
    ax.set_title(f"Virtual track and NMPC raceline ({planning_status})")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.savefig(debug_png, dpi=180)
    plt.close(fig)

    summary = {
        "planning_status": planning_status,
        "planner_feasible": feasible,
        "source_corridor_csv": portable_path(corridor_path, output_dir),
        "source_map_yaml": portable_path(args.map_yaml, output_dir),
        "source_vehicle_config": (
            portable_path(args.vehicle_config, output_dir)
            if args.vehicle_config is not None
            else None
        ),
        "centerline_length_m": center_length,
        "raceline_length_m": race_length,
        "num_points": len(race_x),
        "optimizer": optimizer,
        "maximum_abs_offset_m": float(np.max(np.abs(race_offset))),
        "speed_min_mps": float(np.min(speed)),
        "speed_max_mps": float(np.max(speed)),
        "maximum_abs_curvature_radpm": float(np.max(np.abs(kappa))),
        "maximum_abs_implied_steer_rad": max_abs_steer,
        "maximum_abs_implied_steer_rate_radps": max_abs_steer_rate,
        "minimum_margin_after_vehicle_and_buffer_m": min_margin,
        "vehicle": {
            "wheelbase_m": args.wheelbase_m,
            "width_m": args.vehicle_width_m,
        },
        "limits": {
            "track_margin_m": args.track_margin_m,
            "minimum_tracker_boundary_margin_m": args.min_tracker_boundary_margin_m,
            "min_speed_mps": args.min_speed_mps,
            "max_speed_mps": args.max_speed_mps,
            "max_steer_rad": args.max_steer_rad,
            "max_steer_rate_radps": args.max_steer_rate_radps,
            "max_accel_mps2": args.max_accel_mps2,
            "max_decel_mps2": args.max_decel_mps2,
            "lateral_accel_limit_mps2": args.lateral_accel_limit_mps2,
            "friction_coefficient": args.friction_coefficient,
            "friction_usage_fraction": args.friction_usage_fraction,
            "friction_lateral_limit_mps2": friction_lateral_limit,
            "effective_lateral_accel_limit_mps2": effective_lateral_accel_limit,
        },
        "outputs": {
            "centerline_csv": portable_path(centerline_csv, output_dir),
            "raceline_csv": portable_path(raceline_csv, output_dir),
            "debug_png": portable_path(debug_png, output_dir),
        },
    }
    summary_path = output_dir / "raceline_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if feasible else 2


if __name__ == "__main__":
    raise SystemExit(main())
