#!/usr/bin/env python3
"""Generate the baseline and MPCC C solvers for the current interface."""

from __future__ import annotations

import argparse
from pathlib import Path

from f1tenth_dynamic_mpcc.acados_solver import NP, AcadosDynamicMPCC
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack
from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=Path.home() / ".cache/f1tenth_dynamic_mpcc/acados",
    )
    parser.add_argument("--track", default="virtual_track")
    parser.add_argument("--controller-config", default="controller.yaml")
    parser.add_argument("--vehicle-config", default="vehicle.yaml")
    parser.add_argument("--residual-model")
    parser.add_argument("--residual-feature-set")
    parser.add_argument("--cost-contour", type=float)
    parser.add_argument("--cost-heading-race", type=float)
    parser.add_argument("--cost-steering-command-rate", type=float)
    args = parser.parse_args()
    generated = args.generated_dir.expanduser().resolve()
    controller_path = root / "config" / args.controller_config
    if not controller_path.is_file():
        parser.error(f"controller config not found: {controller_path}")
    controller = load_yaml(controller_path)
    if bool(args.residual_model) != bool(args.residual_feature_set):
        parser.error("--residual-model and --residual-feature-set must be provided together")
    if args.residual_model:
        controller["residual_dynamics"]["enabled"] = True
        controller["residual_dynamics"]["model_path"] = args.residual_model
        controller["residual_dynamics"]["required_feature_set"] = args.residual_feature_set
    if args.cost_contour is not None:
        if args.cost_contour <= 0.0:
            parser.error("--cost-contour must be positive")
        controller["cost"]["contour"] = args.cost_contour
    if args.cost_heading_race is not None:
        if args.cost_heading_race <= 0.0:
            parser.error("--cost-heading-race must be positive")
        controller["cost"]["heading_race"] = args.cost_heading_race
    if args.cost_steering_command_rate is not None:
        if args.cost_steering_command_rate <= 0.0:
            parser.error("--cost-steering-command-rate must be positive")
        controller["cost"]["steering_command_rate"] = (
            args.cost_steering_command_rate
        )
    residual = controller.get("residual_dynamics", {})
    if residual.get("enabled", False):
        model_path = Path(residual["model_path"])
        if not model_path.is_absolute():
            model_path = root / model_path
        residual["model_path"] = str(model_path.resolve())
        controller["residual_dynamics"] = residual
    controller["mode"] = "race"
    vehicle_path = root / "config" / args.vehicle_config
    if not vehicle_path.is_file():
        parser.error(f"vehicle config not found: {vehicle_path}")
    vehicle = VehicleParameters.from_yaml(vehicle_path, controller)
    speed = dict(controller["speed_planning"])
    speed.update(
        wheelbase_m=vehicle.wheelbase,
        max_steer_rad=vehicle.max_steer,
        max_steer_rate_radps=vehicle.max_steer_rate,
        max_accel_mps2=vehicle.max_accel,
        max_decel_mps2=vehicle.max_decel,
        lateral_accel_limit_mps2=vehicle.lateral_accel_limit,
    )
    track_csv = root / "data/tracks" / args.track / "raceline.csv"
    if not track_csv.is_file():
        parser.error(f"track not found: {track_csv}")
    track = PeriodicTrack(track_csv, speed_planning=speed)
    for formulation in ("baseline", "mpcc"):
        print(f"Generating {formulation} solver (np={NP})", flush=True)
        AcadosDynamicMPCC(
            track, vehicle, controller, generated,
            formulation=formulation, build=True,
        )
    (generated / "interface_np.txt").write_text(f"{NP}\n", encoding="ascii")


if __name__ == "__main__":
    main()
