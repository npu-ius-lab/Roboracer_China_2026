#!/usr/bin/env python3
"""Fit preliminary Cf/Cr with delta_hat primary and delta_cmd comparison.

Input is the optimized hardware JSONL telemetry. This read-only diagnostic does
not edit vehicle.yaml because steering and tire parameters remain coupled until
the observer has been validated with independent vehicle data.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def robust_slope(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    keep = np.isfinite(x) & np.isfinite(y) & (np.abs(x) > 0.01)
    x, y = x[keep], y[keep]
    if len(x) < 30:
        raise ValueError("not enough informative tire samples")
    slope = float(np.dot(x, y) / np.dot(x, x))
    for _ in range(6):
        residual = y - slope * x
        scale = max(float(np.median(np.abs(residual))) / 0.6745, 1.0e-6)
        weight = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residual), 1.0e-9))
        slope = float(np.dot(weight * x, y) / np.dot(weight * x, x))
    rmse = float(np.sqrt(np.mean(np.square(y - slope * x))))
    return slope, rmse


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry", type=Path)
    parser.add_argument("--mass", type=float, default=3.80)
    parser.add_argument("--yaw-inertia", type=float, default=0.060)
    parser.add_argument("--lf", type=float, default=0.195)
    parser.add_argument("--lr", type=float, default=0.125)
    parser.add_argument("--min-speed", type=float, default=0.8)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.telemetry.open() if line.strip()]
    samples = []
    previous = None
    for row in rows:
        state = row.get("state_measured")
        actuator = row.get("actuator", {})
        if state is None or actuator.get("steering_estimate") is None:
            continue
        timestamp = float(row["measurement_timestamp"])
        vx, vy, yaw_rate = map(float, state[3:6])
        if previous is not None:
            dt = timestamp - previous[0]
            if 0.01 <= dt <= 0.15 and vx >= args.min_speed:
                vy_dot = (vy - previous[2]) / dt
                r_dot = (yaw_rate - previous[3]) / dt
                ay = vy_dot + vx * yaw_rate
                length = args.lf + args.lr
                fyf = (args.lr * args.mass * ay + args.yaw_inertia * r_dot) / length
                fyr = (args.lf * args.mass * ay - args.yaw_inertia * r_dot) / length
                kinematic = np.arctan2(vy + args.lf * yaw_rate, max(vx, 0.2))
                alpha_r = -np.arctan2(vy - args.lr * yaw_rate, max(vx, 0.2))
                samples.append((
                    float(actuator["steering_command"]) - kinematic,
                    float(actuator["steering_estimate"]) - kinematic,
                    alpha_r, fyf, fyr,
                ))
        previous = (timestamp, vx, vy, yaw_rate)
    data = np.asarray(samples, dtype=float)
    if len(data) < 30:
        raise SystemExit("telemetry lacks enough dynamic samples")
    cf_command, cf_command_rmse = robust_slope(data[:, 0], data[:, 3])
    cf_hat, cf_hat_rmse = robust_slope(data[:, 1], data[:, 3])
    cr_hat, cr_hat_rmse = robust_slope(data[:, 2], data[:, 4])
    print(json.dumps({
        "status": "candidate_not_applied",
        "samples": len(data),
        "fit_using_delta_cmd_comparison": {
            "Cf_N_per_rad": cf_command,
            "force_rmse_N": cf_command_rmse,
        },
        "fit_using_delta_hat_primary": {
            "Cf_N_per_rad": cf_hat,
            "Cr_N_per_rad": cr_hat,
            "front_force_rmse_N": cf_hat_rmse,
            "rear_force_rmse_N": cr_hat_rmse,
        },
        "warning": "Do not apply until steering observer and derivatives are validated.",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
