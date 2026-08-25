#!/usr/bin/env python3
"""Closed-loop Dynamic MPCC simulation and acceptance metrics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.acados_solver import AcadosDynamicMPCC
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle
from f1tenth_dynamic_mpcc.steering_actuator_model import SteeringActuatorModel
from f1tenth_dynamic_mpcc.steering_state_observer import SteeringStateObserver
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--track", default="virtual_track")
    parser.add_argument("--formulation", choices=("baseline", "mpcc"), default="mpcc")
    parser.add_argument("--mode", choices=("debug", "race"), default="debug")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--initial-lateral-offset", type=float, default=0.0)
    parser.add_argument("--initial-heading-offset", type=float, default=0.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.laps < 1:
        parser.error("--laps must be positive")
    controller = load_yaml(root / "config/controller.yaml")
    controller["mode"] = args.mode
    track_cfg = load_yaml(root / "config/track.yaml")
    vehicle = VehicleParameters.from_yaml(root / "config/vehicle.yaml", controller)
    speed_planning = dict(controller.get("speed_planning", {}))
    speed_planning.update(
        wheelbase_m=vehicle.wheelbase,
        max_steer_rad=vehicle.max_steer,
        max_steer_rate_radps=vehicle.max_steer_rate,
        max_accel_mps2=vehicle.max_accel,
        max_decel_mps2=vehicle.max_decel,
        lateral_accel_limit_mps2=vehicle.lateral_accel_limit,
    )
    track = PeriodicTrack(
        root / track_cfg["tracks"][args.track], speed_planning=speed_planning
    )
    model = DynamicBicycleModel(vehicle)
    steering_actuator = SteeringActuatorModel.from_config(
        load_yaml(root / "config/vehicle.yaml")
    )
    steering_observer = SteeringStateObserver.from_config(
        load_yaml(root / "config/vehicle.yaml"), steering_actuator, vehicle
    )
    solver = AcadosDynamicMPCC(
        track,
        vehicle,
        controller,
        root / "generated/acados",
        formulation=args.formulation,
        build=True,
    )
    x0, y0 = track.position(0.0)
    yaw0 = float(track.tangent(0.0))
    x0 -= math.sin(yaw0) * args.initial_lateral_offset
    y0 += math.cos(yaw0) * args.initial_lateral_offset
    # Plant-only state. Its delta_true is deliberately never passed to the
    # controller; the controller constructs x0[6] from the observer.
    plant_state = np.asarray(
        [x0, y0, yaw0 + args.initial_heading_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    dt = 1.0 / float(controller["timing"]["control_rate_hz"])
    rows = []
    failures = 0
    last_control = np.zeros(3)
    for tick in range(int(args.duration / dt)):
        timestamp = tick * dt
        observed = steering_observer.update(
            timestamp, plant_state[3], plant_state[4], plant_state[5]
        )
        projection = track.project(plant_state[0], plant_state[1], plant_state[8])
        controller_state = np.r_[
            plant_state[:6], observed.delta_hat, plant_state[7], projection.s
        ]
        result = solver.solve(controller_state)
        if result.success:
            control = result.controls[0]
        else:
            failures += 1
            control = np.asarray(
                [max(last_control[0] - vehicle.max_decel * dt, 0.0), last_control[1], 0.0]
            )
        steering_observer.push_command(timestamp, plant_state[7])
        plant_state = model.step(plant_state, control, dt)
        last_control = control.copy()
        alpha_f, alpha_r = model.slip_angles(plant_state)
        projection = track.project(plant_state[0], plant_state[1], plant_state[8])
        plant_state[8] = projection.s
        margin = min(
            float(track.width_left(plant_state[8])) - vehicle.body_width / 2.0 - projection.e_contour,
            float(track.width_right(plant_state[8])) - vehicle.body_width / 2.0 + projection.e_contour,
        )
        rows.append(
            {
                "time_s": tick * dt,
                "state": plant_state.tolist(),
                "delta_true": float(plant_state[6]),
                "delta_cmd": float(plant_state[7]),
                "delta_cmd_rate": float(control[1]),
                "delta_target": float(steering_actuator.target(plant_state[7])),
                "delta_hat": float(observed.delta_hat),
                "observer_mode": observed.mode,
                "observer_innovation": observed.innovation,
                "control": control.tolist(),
                "e_contour": projection.e_contour,
                "e_lag": projection.e_lag,
                "e_heading": float(wrap_angle(plant_state[2] - projection.psi_ref)),
                "track_margin": margin,
                "track_slack": result.track_slack_max,
                "tire_slack": result.tire_slack_max,
                "alpha_f": alpha_f,
                "alpha_r": alpha_r,
                "beta": math.atan2(plant_state[4], max(plant_state[3], vehicle.vx_regularization)),
                "solver_status": result.status,
                "solver_success": bool(result.success),
                "solve_time_s": result.solve_time_s,
            }
        )
        if projection.s >= args.laps * track.length:
            break
    elapsed = max(rows[-1]["time_s"], dt)
    solve_times = [row["solve_time_s"] for row in rows]
    metrics = {
        "requested_laps": args.laps,
        "completed_laps": float(rows[-1]["state"][8] / track.length),
        "completed_lap": bool(rows[-1]["state"][8] >= args.laps * track.length),
        "lap_time_s": elapsed / args.laps
        if rows[-1]["state"][8] >= args.laps * track.length
        else None,
        "mean_speed_mps": statistics.fmean(row["state"][3] for row in rows),
        "max_speed_mps": max(row["state"][3] for row in rows),
        "max_abs_contour_error_m": max(abs(row["e_contour"]) for row in rows),
        "minimum_track_margin_m": min(row["track_margin"] for row in rows),
        "max_track_slack_m": max(row["track_slack"] for row in rows),
        "max_tire_slack_rad": max(row["tire_slack"] for row in rows),
        "max_abs_alpha_f_rad": max(abs(row["alpha_f"]) for row in rows),
        "max_abs_alpha_r_rad": max(abs(row["alpha_r"]) for row in rows),
        "max_abs_beta_rad": max(abs(row["beta"]) for row in rows),
        "solver_time_mean_ms": 1000.0 * statistics.fmean(solve_times),
        "solver_time_p95_ms": 1000.0 * float(np.quantile(solve_times, 0.95)),
        "solver_time_max_ms": 1000.0 * max(solve_times),
        "solver_failure_count": failures,
        "steering_observer_rmse_rad": float(np.sqrt(np.mean([
            (row["delta_hat"] - row["delta_true"]) ** 2 for row in rows
        ]))),
        "ticks": len(rows),
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
