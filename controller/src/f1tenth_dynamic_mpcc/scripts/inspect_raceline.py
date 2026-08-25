#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle
from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect the immutable periodic raceline")
    parser.add_argument("csv", type=Path)
    parser.add_argument(
        "--planned-speed",
        action="store_true",
        help="show the runtime curvature/acceleration/local-limit profile",
    )
    parser.add_argument("--controller-config", default="controller.yaml")
    parser.add_argument("--vehicle-config", default="vehicle.yaml")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    controller = load_yaml(root / "config" / args.controller_config)
    vehicle = VehicleParameters.from_yaml(
        root / "config" / args.vehicle_config, controller
    )
    speed_planning = None
    if args.planned_speed:
        speed_planning = dict(controller["speed_planning"])
        speed_planning.update(
            wheelbase_m=vehicle.wheelbase,
            max_steer_rad=vehicle.max_steer,
            max_steer_rate_radps=vehicle.max_steer_rate,
            max_accel_mps2=vehicle.max_accel,
            max_decel_mps2=vehicle.max_decel,
            lateral_accel_limit_mps2=vehicle.lateral_accel_limit,
        )
    track = PeriodicTrack(args.csv, speed_planning=speed_planning)
    payload = args.csv.read_bytes()
    seam_position = np.linalg.norm(
        np.asarray(track.position(0.0)) - np.asarray(track.position(track.length))
    )
    seam_tangent = abs(float(wrap_angle(track.tangent(track.length) - track.tangent(0.0))))
    samples = np.linspace(0.0, track.length, 4096, endpoint=False)
    curvature_error = np.abs(track.curvature(track.s_nodes) - track.csv_curvature(track.s_nodes))
    print(f"path: {args.csv.resolve()}")
    print(f"sha256: {hashlib.sha256(payload).hexdigest()}")
    print(f"points: {len(track.s_nodes)}")
    print(f"closed_length_m: {track.length:.9f}")
    print(f"seam_position_error_m: {seam_position:.3e}")
    print(f"seam_tangent_error_rad: {seam_tangent:.3e}")
    print(f"width_left_range_m: {track.width_left(samples).min():.6f} .. {track.width_left(samples).max():.6f}")
    print(f"width_right_range_m: {track.width_right(samples).min():.6f} .. {track.width_right(samples).max():.6f}")
    print(f"spline_curvature_abs_max_radpm: {np.max(np.abs(track.curvature(samples))):.6f}")
    print(f"csv_vs_spline_curvature_p95_radpm: {np.quantile(curvature_error, 0.95):.6f}")
    print(f"speed_profile_source: {track.speed_profile_source}")
    print(f"speed_range_mps: {np.min(track.speed_nodes):.6f} .. {np.max(track.speed_nodes):.6f}")
    print(f"longitudinal_accel_range_mps2: {np.min(track.accel_nodes):.6f} .. {np.max(track.accel_nodes):.6f}")
    lateral = track.speed_nodes**2 * np.abs(track.curvature(track.s_nodes))
    print(f"lateral_accel_abs_max_mps2: {np.max(lateral):.6f}")


if __name__ == "__main__":
    main()
