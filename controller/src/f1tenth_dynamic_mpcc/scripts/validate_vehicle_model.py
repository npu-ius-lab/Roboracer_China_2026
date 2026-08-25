#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=Path, default=root / "config/vehicle.yaml")
    parser.add_argument("--controller", type=Path, default=root / "config/controller.yaml")
    args = parser.parse_args()
    controller = load_yaml(args.controller)
    parameters = VehicleParameters.from_yaml(args.vehicle, controller)
    model = DynamicBicycleModel(parameters)
    state = np.zeros(9)
    for _ in range(400):
        state = model.step(state, np.asarray([1.0, 0.0, 1.0]), 0.01)
    if abs(state[1]) > 1.0e-6 or abs(state[2]) > 1.0e-6:
        raise SystemExit("straight-line validation failed")
    print("[OK] vehicle schema and straight-line dynamics are valid")
    print(
        f"wheelbase={parameters.wheelbase:.3f} "
        f"body_width={parameters.body_width:.3f} mass={parameters.mass:.3f} "
        f"lf={parameters.lf:.3f} lr={parameters.lr:.3f}"
    )
    print("[WARN] actuator, inertia and tire values remain nominal/assumed, not identified")


if __name__ == "__main__":
    main()
