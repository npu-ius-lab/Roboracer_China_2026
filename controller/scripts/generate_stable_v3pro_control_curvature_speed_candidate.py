#!/usr/bin/env python3
"""Generate a bag-guided, control-aware raceline candidate for V3PRO.

The frozen V3PRO controller, vehicle and residual model remain unchanged.  The
candidate optimizes lateral path placement using curvature, feed-forward
steering-rate and lap-time proxies, then raises selected standard-zone limits
only where the clean V3PRO bag shows adequate tracking quality and margin.
Raw speed limits are never reduced relative to racelinev3.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import minimize
import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
sys.path.insert(0, str(PACKAGE / "python"))

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle


CANDIDATE = "stable_v3pro_control_curvature_speed_candidate"
TRACK_NAME = "racelinev3_stable_v3pro_control_curvature_speed_candidate"
SOURCE_TRACK = PACKAGE / "data/tracks/racelinev3/raceline.csv"
SOURCE_ZONES = PACKAGE / "data/tracks/racelinev3/speed_zones.csv"
SOURCE_CONTROLLER = PACKAGE / "config/candidates/stable_v3pro/controller.yaml"
SOURCE_VEHICLE = PACKAGE / "config/vehicle_stable_v3_racelineV3.yaml"
GEOMETRY_SEED = (
    PACKAGE
    / "data/tracks/racelinev3_stable_v5_global_curvature_race_candidate/raceline.csv"
)
SOURCE_BAG = (
    ROOT
    / "bags/stable_v3pro_20260822_latest/stable_v3pro_20260822_015637.bag"
)
TRACK_DIR = PACKAGE / "data/tracks" / TRACK_NAME
TRACK_CSV = TRACK_DIR / "raceline.csv"
ANALYSIS = ROOT / "analysis/stable_v3pro_control_curvature_speed_20260822"

SOURCE_HASHES = {
    SOURCE_TRACK: "3bc7c472ed534331c0798960520294e346a9c931a5aa1adbaaea068a753371f9",
    SOURCE_ZONES: "2efa8b5fdf7e4433ee408e9b339f5747140669eb20d79b087c20dbb8fbaabd09",
    SOURCE_CONTROLLER: "df8f558111e09db665a70bafaf4baacf238bbc37095e116e6fc992e88498bfe2",
    SOURCE_VEHICLE: "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6",
    GEOMETRY_SEED: "1b69e51d9dfd59f64b7371d29c20c642eea3fc0e4db7847dbc8393c7bbd9be1a",
    SOURCE_BAG: "637c1cb0b461af9eefc45ca446b662f0400e5a7afacaaad56be0cb95979f543b",
}

FIELDS = [
    "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps",
    "ax_mps2", "speed_limit_mps", "speed_zone", "w_tr_right_m",
    "w_tr_left_m",
]
KNOT_SPACING_M = 2.5
MAX_ABS_SHIFT_M = 0.16
MIN_CENTER_CLEARANCE_M = 0.34
OUTPUT_FACTOR = 4
RUNNING_START_S = 19.5
RUNNING_END_S = 137.5


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_sources() -> None:
    for path, expected in SOURCE_HASHES.items():
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"pinned input changed: {path}: {actual} != {expected}")


def planning_config() -> dict:
    controller = yaml.safe_load(SOURCE_CONTROLLER.read_text(encoding="utf-8"))
    vehicle = yaml.safe_load(SOURCE_VEHICLE.read_text(encoding="utf-8"))
    planning = dict(controller["speed_planning"])
    planning.update(
        wheelbase_m=vehicle["vehicle"]["wheelbase"],
        max_steer_rad=vehicle["limits"]["max_steer"],
        max_steer_rate_radps=vehicle["limits"]["max_steer_rate"],
        max_accel_mps2=vehicle["limits"]["max_accel"],
        max_decel_mps2=vehicle["limits"]["max_decel"],
        lateral_accel_limit_mps2=vehicle["limits"]["lateral_accel_limit"],
    )
    return planning


def quaternion_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def bag_tracking_profile(track: PeriodicTrack) -> dict[str, np.ndarray | dict]:
    """Return robust 0.5 m tracking statistics from the clean RUNNING window."""
    bins = max(8, int(math.ceil(track.length / 0.5)))
    samples: dict[int, list[tuple[float, ...]]] = defaultdict(list)
    last_s = None
    with rosbag.Bag(str(SOURCE_BAG)) as bag:
        t0 = bag.get_start_time()
        for topic, message, stamp in bag.read_messages(
            topics=["/localization/vehicle_odom"]
        ):
            elapsed = stamp.to_sec() - t0
            if elapsed < RUNNING_START_S or elapsed > RUNNING_END_S:
                continue
            speed = abs(float(message.twist.twist.linear.x))
            if speed < 0.8:
                continue
            position = message.pose.pose.position
            projection = track.project(position.x, position.y, last_s)
            last_s = projection.s
            wrapped = projection.s % track.length
            heading_error = float(
                wrap_angle(quaternion_yaw(message.pose.pose.orientation) - projection.psi_ref)
            )
            physical_margin = min(
                float(track.width_left(wrapped)) - 0.12 - projection.e_contour,
                float(track.width_right(wrapped)) - 0.12 + projection.e_contour,
            )
            index = min(int(wrapped / track.length * bins), bins - 1)
            samples[index].append(
                (projection.e_contour, heading_error, physical_margin, speed)
            )

    centers = (np.arange(bins) + 0.5) * track.length / bins
    median_error = np.zeros(bins)
    error_p95 = np.full(bins, 0.25)
    heading_p95 = np.full(bins, 0.30)
    margin_p05 = np.full(bins, 0.20)
    speed_p50 = np.zeros(bins)
    counts = np.zeros(bins, dtype=int)
    for index in range(bins):
        values = np.asarray(samples.get(index, []), dtype=float)
        if len(values) < 5:
            continue
        counts[index] = len(values)
        median_error[index] = float(np.median(values[:, 0]))
        error_p95[index] = float(np.quantile(np.abs(values[:, 0]), 0.95))
        heading_p95[index] = float(np.quantile(np.abs(values[:, 1]), 0.95))
        margin_p05[index] = float(np.quantile(values[:, 2], 0.05))
        speed_p50[index] = float(np.median(values[:, 3]))

    # Circular smoothing preserves the seam and prevents one noisy half-metre
    # bin from changing either path placement or a speed-zone boundary.
    median_error = gaussian_filter1d(median_error, 1.5, mode="wrap")
    error_p95 = gaussian_filter1d(error_p95, 1.0, mode="wrap")
    heading_p95 = gaussian_filter1d(heading_p95, 1.0, mode="wrap")
    margin_p05 = gaussian_filter1d(margin_p05, 1.0, mode="wrap")
    return {
        "s": centers,
        "median_error": median_error,
        "error_p95": error_p95,
        "heading_p95": heading_p95,
        "margin_p05": margin_p05,
        "speed_p50": speed_p50,
        "counts": counts,
        "summary": {
            "samples": int(np.sum(counts)),
            "running_window_s": [RUNNING_START_S, RUNNING_END_S],
            "median_abs_contour_m": float(np.median(np.abs(median_error))),
            "maximum_smoothed_error_p95_m": float(np.max(error_p95)),
            "maximum_smoothed_heading_p95_rad": float(np.max(heading_p95)),
            "minimum_smoothed_margin_p05_m": float(np.min(margin_p05)),
        },
    }


def periodic_interp(query, nodes, values, length):
    nodes = np.asarray(nodes)
    values = np.asarray(values)
    return np.interp(
        np.mod(query, length),
        np.r_[nodes[-1] - length, nodes, nodes[0] + length],
        np.r_[values[-1], values, values[0]],
    )


def seed_offsets(source: PeriodicTrack) -> tuple[np.ndarray, np.ndarray]:
    seed = PeriodicTrack(GEOMETRY_SEED)
    source_progress = []
    offset = []
    guess = None
    for value_s in np.linspace(0.0, seed.length, 2400, endpoint=False):
        x, y = seed.position(value_s)
        projection = source.project(x, y, guess)
        guess = projection.s
        source_progress.append(projection.s % source.length)
        offset.append(projection.e_contour)
    order = np.argsort(source_progress)
    return np.asarray(source_progress)[order], np.asarray(offset)[order]


def shifted_geometry(track, s, offset):
    x, y = track.position(s)
    heading = np.asarray(track.tangent(s))
    x = np.asarray(x) - offset * np.sin(heading)
    y = np.asarray(y) + offset * np.cos(heading)
    dx = np.gradient(x, s, edge_order=2)
    dy = np.gradient(y, s, edge_order=2)
    ddx = np.gradient(dx, s, edge_order=2)
    ddy = np.gradient(dy, s, edge_order=2)
    curvature = (dx * ddy - dy * ddx) / np.maximum(
        (dx * dx + dy * dy) ** 1.5, 1e-9
    )
    return x, y, curvature, np.hypot(dx, dy)


def optimize_offset(source: PeriodicTrack, bag_profile: dict):
    count = int(math.ceil(source.length / KNOT_SPACING_M))
    knots = np.linspace(0.0, source.length, count, endpoint=False)
    dense_s = np.linspace(0.0, source.length, 4001, endpoint=False)
    spacing = source.length / len(dense_s)
    seed_s, seed_value = seed_offsets(source)
    seed = periodic_interp(knots, seed_s, seed_value, source.length)
    tracking_target = np.clip(
        periodic_interp(
            knots,
            bag_profile["s"],
            bag_profile["median_error"],
            source.length,
        ),
        -0.12,
        0.12,
    )
    start = 0.88 * seed + 0.12 * tracking_target
    lower = np.maximum(
        -MAX_ABS_SHIFT_M,
        -np.asarray(source.width_right(knots)) + MIN_CENTER_CLEARANCE_M,
    )
    upper = np.minimum(
        MAX_ABS_SHIFT_M,
        np.asarray(source.width_left(knots)) - MIN_CENTER_CLEARANCE_M,
    )
    # Preserve the already clean start straight and A3-A6 high-speed arc.
    # Geometry optimization is concentrated on the measured high-stress bends;
    # the periodic spline provides a short, smooth transition at either end.
    fixed_high_speed = (knots <= 6.0) | (knots >= 71.5)
    lower[fixed_high_speed] = 0.0
    upper[fixed_high_speed] = 0.0
    arc_transition = (knots > 65.5) & (knots < 71.5)
    taper_phase = np.clip((71.5 - knots[arc_transition]) / 6.0, 0.0, 1.0)
    taper_limit = MAX_ABS_SHIFT_M * np.sin(0.5 * np.pi * taper_phase) ** 2
    lower[arc_transition] = np.maximum(lower[arc_transition], -taper_limit)
    upper[arc_transition] = np.minimum(upper[arc_transition], taper_limit)
    start = np.clip(start, lower, upper)

    base_speed = np.asarray(source.speed_prior(dense_s))
    base_offset = np.zeros_like(dense_s)
    _, _, base_curvature, base_scale = shifted_geometry(source, dense_s, base_offset)
    base_delta = np.arctan(0.32 * base_curvature)
    base_delta_s = np.gradient(base_delta, dense_s)
    denominators = {
        "curvature": max(float(np.mean(base_curvature**2 + 0.7 * base_curvature**4)), 1e-6),
        "gradient": max(float(np.mean(np.gradient(base_curvature, dense_s) ** 2)), 1e-6),
        "control": max(float(np.mean((base_delta_s * base_speed) ** 2)), 1e-6),
        "lap": float(np.trapz(base_scale / np.maximum(base_speed, 0.2), dx=spacing)),
    }

    dense_target = periodic_interp(
        dense_s,
        bag_profile["s"],
        bag_profile["median_error"],
        source.length,
    )
    dense_left = np.asarray(source.width_left(dense_s))
    dense_right = np.asarray(source.width_right(dense_s))

    def decode(values):
        spline = CubicSpline(
            np.r_[knots, source.length],
            np.r_[values, values[0]],
            bc_type="periodic",
        )
        return np.asarray(spline(dense_s))

    def components(values):
        offset = decode(values)
        _, _, curvature, ds_scale = shifted_geometry(source, dense_s, offset)
        curvature_gradient = np.gradient(curvature, dense_s)
        delta = np.arctan(0.32 * curvature)
        delta_s = np.gradient(delta, dense_s)
        curvature_speed = np.sqrt(3.2 / np.maximum(np.abs(curvature), 1e-4))
        steering_rate_speed = 3.4 / np.maximum(np.abs(delta_s), 1e-4)
        proxy_speed = np.minimum.reduce(
            [base_speed, curvature_speed, steering_rate_speed, np.full_like(base_speed, 4.0)]
        )
        lap = float(np.trapz(ds_scale / np.maximum(proxy_speed, 0.2), dx=spacing))
        left_violation = np.maximum(offset - (dense_left - MIN_CENTER_CLEARANCE_M), 0.0)
        right_violation = np.maximum(
            (-dense_right + MIN_CENTER_CLEARANCE_M) - offset, 0.0
        )
        shift_violation = np.maximum(np.abs(offset) - MAX_ABS_SHIFT_M, 0.0)
        boundary = float(np.mean(
            left_violation**2 + right_violation**2 + shift_violation**2
        ))
        return {
            "curvature": float(np.mean(curvature**2 + 0.7 * curvature**4)),
            "gradient": float(np.mean(curvature_gradient**2)),
            "control": float(np.mean((delta_s * base_speed) ** 2)),
            "lap": lap,
            "smooth": float(np.mean(np.gradient(np.gradient(offset, dense_s), dense_s) ** 2)),
            "tracking": float(np.mean((offset - dense_target) ** 2)),
            "length": float(np.trapz(ds_scale, dx=spacing)),
            "boundary": boundary,
        }

    def objective(values):
        c = components(values)
        return (
            0.30 * c["curvature"] / denominators["curvature"]
            + 0.16 * c["gradient"] / denominators["gradient"]
            + 0.24 * c["control"] / denominators["control"]
            + 1.20 * c["lap"] / denominators["lap"]
            + 0.020 * c["smooth"]
            + 0.030 * c["tracking"]
            + 0.30 * c["length"] / source.length
            + 2.0e6 * c["boundary"]
        )

    result = minimize(
        objective,
        start,
        method="L-BFGS-B",
        bounds=list(zip(lower, upper)),
        options={
            "maxiter": 220,
            "maxfun": 18000,
            "ftol": 1e-8,
            "gtol": 1e-4,
            "maxls": 30,
        },
    )
    # Pull the solution inward slightly to cover spline overshoot between knots.
    controls = np.asarray(result.x) * 0.997
    return knots, controls, result, components(start), components(controls)


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: row[key] if key == "speed_zone" else f"{float(row[key]):.9f}"
                for key in FIELDS
            })


def choose_speed_limit(source_limit, source_zone, curvature, source_s, bag_profile):
    """Never lower a raw limit; selectively lift well-observed standard zones."""
    if source_zone != "standard":
        return source_limit, source_zone
    error = float(periodic_interp(
        source_s, bag_profile["s"], bag_profile["error_p95"], 87.75971526991167
    ))
    heading = float(periodic_interp(
        source_s, bag_profile["s"], bag_profile["heading_p95"], 87.75971526991167
    ))
    margin = float(periodic_interp(
        source_s, bag_profile["s"], bag_profile["margin_p05"], 87.75971526991167
    ))
    if abs(curvature) <= 0.18 and error <= 0.24 and heading <= 0.22 and margin >= 0.22:
        return max(source_limit, 3.5), "standard_control_clear_3p5"
    if abs(curvature) <= 0.35 and error <= 0.30 and heading <= 0.30 and margin >= 0.15:
        return max(source_limit, 3.3), "standard_control_clear_3p3"
    return source_limit, source_zone


def consolidate_speed_recommendations(rows, spacing_m):
    """Turn point-wise bag recommendations into stable, driveable regions.

    Isolated recommendations are useful diagnostics but poor runtime commands:
    they make the raw cap chatter between 3.0/3.3/3.5 m/s.  Keep only standard
    road windows that remain clear for at least 1.5 m.  Within such a window,
    3.3 m/s is the conservative regional floor and 3.5 m/s is retained only
    when its strongest evidence is continuous for at least 0.95 m.
    """
    enhanced = [
        index for index, row in enumerate(rows)
        if row["speed_limit_mps"] > row["source_limit_mps"] + 1e-9
    ]
    groups = []
    for index in enhanced:
        if not groups or index != groups[-1][-1] + 1:
            groups.append([index])
        else:
            groups[-1].append(index)

    for group in groups:
        length_m = len(group) * spacing_m
        if length_m < 1.5:
            for index in group:
                rows[index]["speed_limit_mps"] = rows[index]["source_limit_mps"]
                rows[index]["speed_zone"] = rows[index]["source_zone"]
            continue

        for index in group:
            rows[index]["speed_limit_mps"] = max(rows[index]["source_limit_mps"], 3.3)
            rows[index]["speed_zone"] = "standard_control_clear_3p3"

        strong = [index for index in group if rows[index]["pointwise_limit_mps"] >= 3.5 - 1e-9]
        strong_groups = []
        for index in strong:
            if not strong_groups or index != strong_groups[-1][-1] + 1:
                strong_groups.append([index])
            else:
                strong_groups[-1].append(index)
        for strong_group in strong_groups:
            if len(strong_group) * spacing_m < 0.95:
                continue
            for index in strong_group:
                rows[index]["speed_limit_mps"] = max(rows[index]["source_limit_mps"], 3.5)
                rows[index]["speed_zone"] = "standard_control_clear_3p5"

    boosts = defaultdict(int)
    for row in rows:
        if row["speed_limit_mps"] > row["source_limit_mps"] + 1e-9:
            boosts[row["speed_zone"]] += 1
    return dict(boosts)


def build_track(source, source_rows, knots, controls, bag_profile):
    old_s = np.asarray([float(row["s_m"]) for row in source_rows])
    old_x = np.asarray([float(row["x_m"]) for row in source_rows])
    old_y = np.asarray([float(row["y_m"]) for row in source_rows])
    old_left = np.asarray([float(row["w_tr_left_m"]) for row in source_rows])
    old_right = np.asarray([float(row["w_tr_right_m"]) for row in source_rows])
    offset_spline = CubicSpline(
        np.r_[knots, source.length],
        np.r_[controls, controls[0]],
        bc_type="periodic",
    )
    offset = np.asarray(offset_spline(old_s))
    heading = np.asarray(source.tangent(old_s))
    new_x = old_x - offset * np.sin(heading)
    new_y = old_y + offset * np.cos(heading)
    new_left = old_left - offset
    new_right = old_right + offset
    if min(float(np.min(new_left)), float(np.min(new_right))) < MIN_CENTER_CLEARANCE_M - 2e-3:
        raise RuntimeError("candidate violates center-clearance constraint")

    new_s = np.r_[0.0, np.cumsum(np.hypot(np.diff(new_x), np.diff(new_y)))]
    length = float(new_s[-1] + math.hypot(new_x[0] - new_x[-1], new_y[0] - new_y[-1]))
    periodic_s = np.r_[new_s, length]
    sx = CubicSpline(periodic_s, np.r_[new_x, new_x[0]], bc_type="periodic")
    sy = CubicSpline(periodic_s, np.r_[new_y, new_y[0]], bc_type="periodic")
    sl = CubicSpline(periodic_s, np.r_[new_left, new_left[0]], bc_type="periodic")
    sr = CubicSpline(periodic_s, np.r_[new_right, new_right[0]], bc_type="periodic")
    output_s = np.linspace(0.0, length, len(source_rows) * OUTPUT_FACTOR, endpoint=False)
    source_s = np.interp(
        output_s, np.r_[new_s, length], np.r_[old_s, source.length]
    )
    dx, dy = sx(output_s, 1), sy(output_s, 1)
    ddx, ddy = sx(output_s, 2), sy(output_s, 2)
    curvature = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    source_index = np.minimum(
        np.searchsorted(old_s, source_s, side="right") - 1,
        len(source_rows) - 1,
    )
    source_index = np.maximum(source_index, 0)
    rows = []
    for index, value_s in enumerate(output_s):
        source_row = source_rows[source_index[index]]
        source_limit = float(source_row["speed_limit_mps"])
        limit, zone = choose_speed_limit(
            source_limit,
            source_row["speed_zone"],
            curvature[index],
            source_s[index],
            bag_profile,
        )
        rows.append({
            "s_m": value_s,
            "x_m": sx(value_s),
            "y_m": sy(value_s),
            "psi_rad": math.atan2(dy[index], dx[index]),
            "kappa_radpm": curvature[index],
            "vx_mps": limit,
            "ax_mps2": 0.0,
            "speed_limit_mps": limit,
            "speed_zone": zone,
            "w_tr_right_m": sr(value_s),
            "w_tr_left_m": sl(value_s),
            "source_s_m": source_s[index],
            "source_limit_mps": source_limit,
            "source_zone": source_row["speed_zone"],
            "pointwise_limit_mps": limit,
        })
    boosts = consolidate_speed_recommendations(rows, length / len(output_s))
    write_rows(TRACK_CSV, rows)
    runtime = PeriodicTrack(TRACK_CSV, speed_planning=planning_config())
    for index, row in enumerate(rows):
        row["vx_mps"] = float(runtime.speed_nodes[index])
        row["ax_mps2"] = float(runtime.accel_nodes[index])
    write_rows(TRACK_CSV, rows)
    reproduced = PeriodicTrack(TRACK_CSV, speed_planning=planning_config())
    if np.max(np.abs(reproduced.speed_nodes - runtime.speed_nodes)) > 2e-8:
        raise RuntimeError("runtime speed envelope is not reproducible")
    if min(row["speed_limit_mps"] - row["source_limit_mps"] for row in rows) < -1e-9:
        raise RuntimeError("candidate lowered a source raw speed limit")
    return reproduced, new_s, offset, source_s, rows, dict(boosts)


def dense_audit(track: PeriodicTrack) -> dict:
    s = np.linspace(0.0, track.length, 50001, endpoint=False)
    curvature = np.asarray(track.curvature(s))
    speed = np.asarray(track.speed_prior(s))
    delta = np.arctan(0.32 * curvature)
    steering_rate = np.abs(np.gradient(delta, s)) * speed
    return {
        "length_m": float(track.length),
        "maximum_abs_curvature_1pm": float(np.max(np.abs(curvature))),
        "curvature_rms_1pm": float(np.sqrt(np.mean(curvature**2))),
        "maximum_abs_curvature_gradient_1pm2": float(np.max(np.abs(np.gradient(curvature, s)))),
        "feedforward_steering_rate_p95_radps": float(np.quantile(steering_rate, 0.95)),
        "feedforward_steering_rate_max_radps": float(np.max(steering_rate)),
        "minimum_speed_mps": float(np.min(speed)),
        "mean_speed_mps": float(np.mean(speed)),
        "maximum_speed_mps": float(np.max(speed)),
        "ideal_profile_lap_s": float(np.trapz(1.0 / np.maximum(speed, 0.2), s)),
        "minimum_left_width_m": float(np.min(track.width_left(s))),
        "minimum_right_width_m": float(np.min(track.width_right(s))),
    }


REGIONS = {
    "C0": (7.0, 12.0),
    "C1": (14.0, 25.0),
    "C2": (26.0, 33.0),
    "C3": (34.0, 42.0),
    "C4": (43.0, 56.0),
    "C5": (59.0, 65.5),
    "C6": (65.5, 70.5),
    "A3_A6": (70.5, 83.5),
}


def region_audit(source, candidate, candidate_source_s):
    result = {}
    candidate_nodes = candidate.s_nodes
    for name, (start, end) in REGIONS.items():
        old_s = np.linspace(start, end, 1600)
        mask = (candidate_source_s >= start) & (candidate_source_s <= end)
        new_s = candidate_nodes[mask]
        if len(new_s) < 3:
            continue
        old_k = np.abs(source.curvature(old_s))
        new_k = np.abs(candidate.curvature(new_s))
        old_v = source.speed_prior(old_s)
        new_v = candidate.speed_prior(new_s)
        result[name] = {
            "source_s_m": [start, end],
            "source_max_abs_curvature_1pm": float(np.max(old_k)),
            "candidate_max_abs_curvature_1pm": float(np.max(new_k)),
            "source_minimum_speed_mps": float(np.min(old_v)),
            "candidate_minimum_speed_mps": float(np.min(new_v)),
            "source_mean_speed_mps": float(np.mean(old_v)),
            "candidate_mean_speed_mps": float(np.mean(new_v)),
        }
    return result


def write_speed_zones(rows):
    runs = []
    begin = 0
    for index in range(1, len(rows) + 1):
        if index == len(rows) or rows[index]["speed_zone"] != rows[begin]["speed_zone"]:
            runs.append((begin, index - 1))
            begin = index
    with TRACK_DIR.joinpath("speed_zones.csv").open("w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "zone_name", "zone_name_cn", "start_s_m", "end_s_m",
            "wraps_seam", "maximum_speed_mps", "speed_scale",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for run_index, (start, end) in enumerate(runs):
            zone = rows[start]["speed_zone"]
            writer.writerow({
                "zone_name": f"{zone}_{run_index:02d}",
                "zone_name_cn": zone,
                "start_s_m": f"{rows[start]['s_m']:.9f}",
                "end_s_m": f"{rows[end]['s_m']:.9f}",
                "wraps_seam": "false",
                "maximum_speed_mps": f"{max(row['speed_limit_mps'] for row in rows[start:end+1]):.9f}",
                "speed_scale": "1.000000000",
            })


def make_plot(source, candidate, candidate_source_s, offset, bag_profile):
    old_s = np.linspace(0.0, source.length, 18000, endpoint=False)
    new_s = np.linspace(0.0, candidate.length, 18000, endpoint=False)
    source_map = np.interp(new_s, candidate.s_nodes, candidate_source_s)
    ox, oy = source.position(old_s)
    nx, ny = candidate.position(new_s)
    fig, axes = plt.subplots(3, 2, figsize=(18, 16), constrained_layout=True)
    axes[0, 0].plot(ox, oy, color="0.65", lw=2.8, label="V3PRO racelinev3")
    axes[0, 0].plot(nx, ny, color="#0969da", lw=1.5, label="candidate")
    left = source.width_left(old_s)
    right = source.width_right(old_s)
    yaw = source.tangent(old_s)
    axes[0, 0].plot(ox - np.sin(yaw) * left, oy + np.cos(yaw) * left, "k--", lw=.6)
    axes[0, 0].plot(ox + np.sin(yaw) * right, oy - np.cos(yaw) * right, "k--", lw=.6)
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].set_title("Raceline and physical corridor")
    axes[0, 0].legend()
    axes[0, 1].plot(old_s, source.curvature(old_s), color="0.65", label="V3PRO")
    axes[0, 1].plot(source_map, candidate.curvature(new_s), color="#0969da", label="candidate")
    axes[0, 1].set_title("Curvature")
    axes[1, 0].plot(old_s, source.speed_prior(old_s), color="0.65", label="V3PRO")
    axes[1, 0].plot(source_map, candidate.speed_prior(new_s), color="#18864b", label="candidate")
    axes[1, 0].set_title("Runtime planned speed")
    axes[1, 0].legend()
    axes[1, 1].plot(source.s_nodes, offset, color="#7a3db8")
    axes[1, 1].axhline(0.0, color="black", lw=.7)
    axes[1, 1].set_title("Lateral shift, left positive")
    axes[2, 0].plot(bag_profile["s"], bag_profile["error_p95"], label="|e| p95")
    axes[2, 0].plot(bag_profile["s"], bag_profile["heading_p95"], label="|heading| p95")
    axes[2, 0].plot(bag_profile["s"], bag_profile["margin_p05"], label="margin p05")
    axes[2, 0].set_title("Clean V3PRO bag tracking quality")
    axes[2, 0].legend()
    old_delta = np.arctan(.32 * source.curvature(old_s))
    new_delta = np.arctan(.32 * candidate.curvature(new_s))
    axes[2, 1].plot(
        old_s,
        np.abs(np.gradient(old_delta, old_s)) * source.speed_prior(old_s),
        color="0.65",
        label="V3PRO",
    )
    axes[2, 1].plot(
        source_map,
        np.abs(np.gradient(new_delta, new_s)) * candidate.speed_prior(new_s),
        color="#d97706",
        label="candidate",
    )
    axes[2, 1].set_title("Feed-forward steering-rate proxy")
    axes[2, 1].legend()
    for axis in axes.flat:
        axis.grid(alpha=.25)
        if axis is not axes[0, 0]:
            axis.set_xlabel("source progress s [m]")
    ANALYSIS.mkdir(parents=True, exist_ok=True)
    fig.savefig(ANALYSIS / "v3pro_control_curvature_speed_comparison.png", dpi=190)
    plt.close(fig)


def main() -> None:
    validate_sources()
    source = PeriodicTrack(SOURCE_TRACK, speed_planning=planning_config())
    with SOURCE_TRACK.open(newline="", encoding="utf-8") as stream:
        source_rows = list(csv.DictReader(stream))
    bag_profile = bag_tracking_profile(source)
    knots, controls, optimizer, initial_components, final_components = optimize_offset(
        source, bag_profile
    )
    candidate, new_s, offset, candidate_source_s, rows, boosts = build_track(
        source, source_rows, knots, controls, bag_profile
    )
    write_speed_zones(rows)
    source_metrics = dense_audit(source)
    candidate_metrics = dense_audit(candidate)
    if candidate_metrics["ideal_profile_lap_s"] > source_metrics["ideal_profile_lap_s"] + 1e-6:
        raise RuntimeError("candidate ideal profile is slower than V3PRO")
    if candidate_metrics["mean_speed_mps"] + 1e-6 < source_metrics["mean_speed_mps"]:
        raise RuntimeError("candidate mean planned speed is lower than V3PRO")
    if candidate_metrics["minimum_speed_mps"] + 0.02 < source_metrics["minimum_speed_mps"]:
        raise RuntimeError("candidate minimum planned speed regressed by more than 0.02 m/s")
    make_plot(source, candidate, candidate_source_s, offset, bag_profile)
    summary = {
        "candidate": CANDIDATE,
        "track": TRACK_NAME,
        "status": "offline_validated_candidate_not_hardware_approved",
        "method": "V3PRO bag-guided curvature + steering-rate + speed optimization",
        "source_hashes": {
            str(path.relative_to(ROOT)): value for path, value in SOURCE_HASHES.items()
        },
        "frozen_controller": str(SOURCE_CONTROLLER.relative_to(ROOT)),
        "optimizer": {
            "success": bool(optimizer.success),
            # L-BFGS may exhaust its evaluation budget after already reaching
            # a feasible plateau. Acceptance is decided by the explicit dense
            # geometry/speed gates above, not by the optimizer status alone.
            "accepted_feasible": True,
            "message": str(optimizer.message),
            "iterations": int(optimizer.nit),
            "evaluations": int(optimizer.nfev),
            "objective": float(optimizer.fun),
            "initial_components": initial_components,
            "final_components": final_components,
        },
        "geometry_constraints": {
            "maximum_abs_shift_m": float(np.max(np.abs(offset))),
            "limit_m": MAX_ABS_SHIFT_M,
            "minimum_center_clearance_m": MIN_CENTER_CLEARANCE_M,
        },
        "speed_policy": {
            "raw_speed_limits_never_lowered": True,
            "standard_clear_limits_mps": [3.3, 3.5],
            "global_runtime_cap_mps": 4.0,
            "boosted_output_nodes": boosts,
        },
        "bag_profile": bag_profile["summary"],
        "v3pro": source_metrics,
        "candidate_metrics": candidate_metrics,
        "regions": region_audit(source, candidate, candidate_source_s),
        "output_hashes": {
            str(TRACK_CSV.relative_to(ROOT)): sha256(TRACK_CSV),
            str(TRACK_DIR.joinpath("speed_zones.csv").relative_to(ROOT)): sha256(
                TRACK_DIR / "speed_zones.csv"
            ),
        },
    }
    ANALYSIS.mkdir(parents=True, exist_ok=True)
    text = json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    (ANALYSIS / "generation_summary.json").write_text(text, encoding="utf-8")
    (TRACK_DIR / "MANIFEST.json").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
