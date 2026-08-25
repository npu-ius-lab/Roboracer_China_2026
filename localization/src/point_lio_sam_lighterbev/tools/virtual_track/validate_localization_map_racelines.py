#!/usr/bin/env python3
"""Validate the three localization-map racelines before controller integration."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from virtual_track_common import load_ros_map, world_to_local


TRACK_NAMES = ("virtual_track", "virtual_track_u_turn", "virtual_track_s_u")
EXPECTED_FIELDS = (
    "s_m",
    "x_m",
    "y_m",
    "psi_rad",
    "kappa_radpm",
    "vx_mps",
    "ax_mps2",
    "w_tr_right_m",
    "w_tr_left_m",
)


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_map_dir = script_dir.parents[1] / "point_lio" / "localization_map"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, default=default_map_dir)
    parser.add_argument("--expected-points", type=int, default=260)
    return parser.parse_args()


def load_csv(path: Path) -> tuple[list[str], dict[str, np.ndarray]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = list(reader.fieldnames or [])
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    values = {
        field: np.asarray([float(row[field]) for row in rows], dtype=float)
        for field in fields
    }
    return fields, values


def validate_track(
    map_dir: Path,
    track_name: str,
    expected_points: int,
    image_shape: tuple[int, int],
    map_meta: dict,
) -> list[str]:
    errors: list[str] = []
    track_dir = map_dir / track_name
    csv_path = track_dir / "raceline.csv"
    summary_path = track_dir / "raceline_summary.json"
    corridor_path = track_dir / "virtual_track_corridor.csv"
    if not corridor_path.is_file():
        return [f"{track_name}: missing source corridor {corridor_path}"]
    if not csv_path.is_file() or not summary_path.is_file():
        return [f"{track_name}: missing raceline.csv or raceline_summary.json"]

    fields, data = load_csv(csv_path)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if tuple(fields) != EXPECTED_FIELDS:
        errors.append(f"{track_name}: unexpected CSV schema {fields}")
    point_count = len(data["s_m"])
    if point_count != expected_points:
        errors.append(
            f"{track_name}: expected {expected_points} points, got {point_count}"
        )
    if not all(np.all(np.isfinite(values)) for values in data.values()):
        errors.append(f"{track_name}: CSV contains NaN or Inf")
    if not np.all(np.diff(data["s_m"]) > 0.0):
        errors.append(f"{track_name}: s_m is not strictly increasing")

    x = data["x_m"]
    y = data["y_m"]
    segment = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)
    median_segment = float(np.median(segment))
    if np.any(segment <= 0.5 * median_segment) or np.any(
        segment >= 1.5 * median_segment
    ):
        errors.append(f"{track_name}: closed-loop point spacing is discontinuous")

    limits = summary["limits"]
    vehicle = summary["vehicle"]
    speed = data["vx_mps"]
    curvature = data["kappa_radpm"]
    accel = data["ax_mps2"]
    lateral_accel = speed * speed * np.abs(curvature)
    body_margin = (
        np.minimum(data["w_tr_left_m"], data["w_tr_right_m"])
        - 0.5 * float(vehicle["width_m"])
        - float(limits["track_margin_m"])
    )
    required_margin = float(limits["minimum_tracker_boundary_margin_m"])
    if float(np.min(body_margin)) < required_margin - 1.0e-6:
        errors.append(f"{track_name}: vehicle/body boundary margin is too small")
    if float(np.max(lateral_accel)) > float(limits["lateral_accel_limit_mps2"]) + 1.0e-5:
        errors.append(f"{track_name}: lateral acceleration limit is exceeded")
    if float(np.min(speed)) < float(limits["min_speed_mps"]) - 1.0e-5:
        errors.append(f"{track_name}: minimum speed limit is violated")
    if float(np.max(speed)) > float(limits["max_speed_mps"]) + 1.0e-5:
        errors.append(f"{track_name}: maximum speed limit is exceeded")
    if float(np.max(accel)) > float(limits["max_accel_mps2"]) + 1.0e-5:
        errors.append(f"{track_name}: longitudinal acceleration limit is exceeded")
    if float(np.min(accel)) < -float(limits["max_decel_mps2"]) - 1.0e-5:
        errors.append(f"{track_name}: longitudinal deceleration limit is exceeded")

    steer = np.arctan(float(vehicle["wheelbase_m"]) * curvature)
    if float(np.max(np.abs(steer))) > float(limits["max_steer_rad"]) + 1.0e-5:
        errors.append(f"{track_name}: steering-angle limit is exceeded")
    ds = float(summary["raceline_length_m"]) / point_count
    steer_rate = (np.roll(steer, -1) - np.roll(steer, 1)) / (2.0 * ds) * speed
    if float(np.max(np.abs(steer_rate))) > float(limits["max_steer_rate_radps"]) * 1.01:
        errors.append(f"{track_name}: steering-rate limit is exceeded")

    local_x, local_y = world_to_local(x, y, map_meta)
    resolution = float(map_meta["resolution"])
    height, width = image_shape
    inside = (
        (local_x >= 0.0)
        & (local_x < width * resolution)
        & (local_y >= 0.0)
        & (local_y < height * resolution)
    )
    if not bool(np.all(inside)):
        errors.append(f"{track_name}: raceline leaves the ROS map extent")
    if not bool(summary.get("planner_feasible")):
        errors.append(f"{track_name}: summary marks planner_feasible=false")
    if not bool(summary.get("optimizer", {}).get("success")):
        errors.append(f"{track_name}: optimizer did not converge")

    result_label = "PASS" if not errors else "CHECK"
    print(
        f"[{result_label}] {track_name}: points={point_count}, "
        f"length={float(summary['raceline_length_m']):.3f} m, "
        f"speed={float(np.min(speed)):.3f}..{float(np.max(speed)):.3f} m/s, "
        f"ay_max={float(np.max(lateral_accel)):.3f} m/s^2, "
        f"body_margin_min={float(np.min(body_margin)):.3f} m"
    )
    return errors


def main() -> int:
    args = parse_args()
    map_dir = args.map_dir.expanduser().resolve()
    image, meta, _ = load_ros_map(map_dir / "point_lio_map_2d.yaml")
    errors: list[str] = []
    for track_name in TRACK_NAMES:
        errors.extend(
            validate_track(
                map_dir,
                track_name,
                args.expected_points,
                image.shape,
                meta,
            )
        )
    if errors:
        for error in errors:
            print(f"[FAIL] {error}")
        return 2
    print("[OK] All localization-map racelines passed static validation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
