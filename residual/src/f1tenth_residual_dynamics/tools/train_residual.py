#!/usr/bin/env python3
"""Train a bounded robust residual derivative model from processed runs."""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from residual_dynamics.dataset import feature_names, load_runs, prepare_run
from residual_dynamics.nominal_model import NominalBicycleModel, load_vehicle_config
from residual_dynamics.residual_model import OUTPUT_NAMES, ResidualModel


def rmse(errors):
    return np.sqrt(np.mean(np.asarray(errors) ** 2, axis=0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("datasets", nargs="+", type=Path)
    parser.add_argument("--vehicle", type=Path,
                        default=PACKAGE / "config/vehicle_nominal.yaml")
    parser.add_argument("--config", type=Path,
                        default=PACKAGE / "config/training.yaml")
    parser.add_argument(
        "--deployment-scale", type=float, default=1.0,
        help="multiply learned residual coefficients before validation/save")
    parser.add_argument(
        "--deployment-output-scales", type=float, nargs=3,
        metavar=("VX", "VY", "YAW"),
        help=("additional per-output coefficient scales for vx, vy and yaw; "
              "use VX=0 for a lateral-only residual model"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 0.0 < args.deployment_scale <= 1.0:
        raise SystemExit("--deployment-scale must be in (0, 1]")
    output_scales = np.ones(3, dtype=float)
    if args.deployment_output_scales is not None:
        output_scales = np.asarray(args.deployment_output_scales, dtype=float)
        if np.any(output_scales < 0.0) or np.any(output_scales > 1.0):
            raise SystemExit("--deployment-output-scales values must be within [0, 1]")
    vehicle_config = load_vehicle_config(args.vehicle)
    with args.config.open("r") as stream:
        training_config = yaml.safe_load(stream)
    nominal = NominalBicycleModel.from_config(vehicle_config)
    runs = load_runs(args.datasets)
    prepared = [prepare_run(run, nominal, training_config) for run in runs]
    history = training_config["dataset"]["command_history_s"]
    feature_set = training_config["dataset"].get("feature_set", "history_v1")
    names = feature_names(history, feature_set)
    fraction = float(training_config["training"]["validation_fraction"])

    train_x, train_y, valid_parts = [], [], []
    for part in prepared:
        count = len(part["features"])
        split = int(np.clip(round(count * (1.0 - fraction)), 1, count - 1))
        train_x.append(part["features"][:split])
        train_y.append(part["targets"][:split])
        valid_parts.append((part, split))
    train_x, train_y = np.vstack(train_x), np.vstack(train_y)
    if len(train_x) < 100:
        raise SystemExit("need at least 100 valid training transitions; got %d" % len(train_x))
    cfg = training_config["training"]
    metadata = {
        "created_at": __import__("datetime").datetime.now().isoformat(),
        "dataset_paths": [run["source_path"] for run in runs],
        "training_samples": int(len(train_x)),
        "command_history_s": [float(value) for value in history],
        "feature_set": feature_set,
        "nominal_vehicle_config": vehicle_config,
        "training_config": training_config,
    }
    model = ResidualModel.fit(
        train_x, train_y, names, alpha=cfg["ridge_alpha"],
        robust_iterations=cfg["robust_iterations"], huber_delta=cfg["huber_delta"],
        output_limits=cfg["residual_accel_limits"], metadata=metadata)
    model.coefficients *= args.deployment_scale * output_scales[None, :]
    model.metadata["deployment_scale"] = float(args.deployment_scale)
    model.metadata["deployment_output_scales"] = output_scales.tolist()

    validation_errors_nominal, validation_errors_corrected = [], []
    for part, split in valid_parts:
        x = part["features"][split:]
        dt = part["dt"][split:, None]
        measured = part["measured_next"][split:]
        baseline = part["baseline_next"][split:]
        validation_errors_nominal.append(measured - baseline)
        validation_errors_corrected.append(measured - (baseline + dt * model.predict(x)))
    nominal_errors = np.vstack(validation_errors_nominal)
    corrected_errors = np.vstack(validation_errors_corrected)
    metrics = {
        "validation_samples": int(len(nominal_errors)),
        "outputs": list(OUTPUT_NAMES),
        "nominal_one_step_rmse": rmse(nominal_errors).tolist(),
        "corrected_one_step_rmse": rmse(corrected_errors).tolist(),
        "improvement_percent": (100.0 * (1.0 - rmse(corrected_errors) /
                                           np.maximum(rmse(nominal_errors), 1.0e-9))).tolist(),
    }
    model.metadata["validation_metrics"] = metrics
    output = args.output.expanduser().resolve()
    model.save(output)
    report_path = output.with_suffix(".training.json")
    with report_path.open("w") as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
    print(json.dumps({"model": str(output), "report": str(report_path), **metrics},
                     indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
