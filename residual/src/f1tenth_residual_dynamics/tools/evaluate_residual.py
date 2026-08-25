#!/usr/bin/env python3
"""Evaluate one-step and open-loop trajectory prediction of a residual model."""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from residual_dynamics.dataset import (command_histories, feature_names, load_runs,
                                       make_features, prepare_run, zoh)
from residual_dynamics.nominal_model import NominalBicycleModel, load_vehicle_config
from residual_dynamics.residual_model import ResidualModel


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def pose_step(pose, body_before, body_after, dt):
    yaw_mid = pose[2] + 0.5 * dt * body_before[2]
    velocity = 0.5 * (body_before[:2] + body_after[:2])
    pose[0] += dt * (velocity[0] * np.cos(yaw_mid) - velocity[1] * np.sin(yaw_mid))
    pose[1] += dt * (velocity[0] * np.sin(yaw_mid) + velocity[1] * np.cos(yaw_mid))
    pose[2] = wrap(pose[2] + 0.5 * dt * (body_before[2] + body_after[2]))
    return pose


def rollout(run, part, nominal, model, config, start_row, horizon_s):
    indices = part["row_indices"]
    start = int(indices[start_row])
    times = run["t"]
    history_s = config["dataset"]["command_history_s"]
    feature_set = config["dataset"].get("feature_set", "history_v1")
    nominal_state = np.asarray([run["vx"][start], run["vy"][start],
                                run["yaw_rate"][start], part["steering"][start]])
    corrected_state = nominal_state.copy()
    initial_pose = np.asarray([run["x"][start], run["y"][start], run["yaw"][start]])
    nominal_pose, corrected_pose = initial_pose.copy(), initial_pose.copy()
    actual_poses, nominal_poses, corrected_poses = [], [], []
    body_nominal, body_corrected = [], []
    end_time = times[start] + horizon_s
    for index in range(start, len(times) - 1):
        if times[index + 1] > end_time:
            break
        dt = times[index + 1] - times[index]
        if not config["dataset"]["minimum_dt_s"] <= dt <= config["dataset"]["maximum_dt_s"]:
            break
        histories = command_histories(run, np.asarray([times[index]]), history_s)[0]
        delayed_speed = zoh(run["command_t"], run["command_speed"],
                            np.asarray([times[index] - nominal.speed_dead_time]))[0]
        delayed_steering = zoh(
            run.get("command_t", times), run.get("command_steer", run["cmd_steer"]),
            np.asarray([times[index] - nominal.steering_dead_time]))[0]
        command = np.asarray([delayed_speed, delayed_steering])
        if not np.all(np.isfinite(histories)) or not np.all(np.isfinite(command)):
            break
        nominal_next = nominal.step(nominal_state, command, dt)
        corrected_nominal = nominal.step(corrected_state, command, dt)
        features = make_features(*corrected_state, histories, feature_set)
        corrected_next = corrected_nominal.copy()
        corrected_next[:3] += dt * model.predict(features)
        nominal_pose = pose_step(nominal_pose, nominal_state, nominal_next, dt)
        corrected_pose = pose_step(corrected_pose, corrected_state, corrected_next, dt)
        nominal_state, corrected_state = nominal_next, corrected_next
        actual_poses.append([run["x"][index + 1], run["y"][index + 1], run["yaw"][index + 1]])
        nominal_poses.append(nominal_pose.copy())
        corrected_poses.append(corrected_pose.copy())
        body_nominal.append(nominal_state[:3].copy())
        body_corrected.append(corrected_state[:3].copy())
    return tuple(np.asarray(value) for value in
                 (actual_poses, nominal_poses, corrected_poses, body_nominal, body_corrected))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("datasets", nargs="+", type=Path)
    parser.add_argument("--vehicle", type=Path,
                        default=PACKAGE / "config/vehicle_nominal.yaml")
    parser.add_argument("--config", type=Path,
                        default=PACKAGE / "config/training.yaml")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--tail-fraction", type=float, default=0.0,
        help="evaluate only the final fraction of each run (0 disables slicing)")
    args = parser.parse_args()
    with args.config.open("r") as stream:
        config = yaml.safe_load(stream)
    nominal = NominalBicycleModel.from_config(load_vehicle_config(args.vehicle))
    model = ResidualModel.load(args.model)
    expected_names = feature_names(
        config["dataset"]["command_history_s"],
        config["dataset"].get("feature_set", "history_v1"))
    if model.feature_names != expected_names:
        raise SystemExit("training configuration feature order does not match model")
    runs = load_runs(args.datasets)
    if args.tail_fraction:
        if not 0.0 < args.tail_fraction < 1.0:
            raise SystemExit("--tail-fraction must be between 0 and 1")
        sliced = []
        for run in runs:
            count = len(run["t"])
            start = int(np.floor(count * (1.0 - args.tail_fraction)))
            subset = {}
            for key, value in run.items():
                if isinstance(value, np.ndarray) and value.ndim >= 1 and len(value) == count:
                    subset[key] = value[start:]
                else:
                    subset[key] = value
            subset["source_path"] = run["source_path"] + "[tail=%.3f]" % args.tail_fraction
            sliced.append(subset)
        runs = sliced
    parts = [prepare_run(run, nominal, config) for run in runs]

    errors_nominal, errors_corrected = [], []
    for part in parts:
        baseline = part["baseline_next"]
        measured = part["measured_next"]
        corrected = baseline + part["dt"][:, None] * model.predict(part["features"])
        errors_nominal.append(measured - baseline)
        errors_corrected.append(measured - corrected)
    errors_nominal, errors_corrected = np.vstack(errors_nominal), np.vstack(errors_corrected)
    rmse_nominal = np.sqrt(np.mean(errors_nominal ** 2, axis=0))
    rmse_corrected = np.sqrt(np.mean(errors_corrected ** 2, axis=0))

    horizon = float(config["evaluation"]["rollout_horizon_s"])
    final_nominal, final_corrected = [], []
    for run, part in zip(runs, parts):
        stride = max(1, int(round(0.25 / np.median(part["dt"]))))
        for start in range(0, len(part["row_indices"]), stride):
            actual, nominal_path, corrected_path, _, _ = rollout(
                run, part, nominal, model, config, start, horizon)
            if len(actual) < 5:
                continue
            final_nominal.append(np.linalg.norm(actual[-1, :2] - nominal_path[-1, :2]))
            final_corrected.append(np.linalg.norm(actual[-1, :2] - corrected_path[-1, :2]))

    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    run, part = runs[0], parts[0]
    plot_horizon = float(config["evaluation"]["plot_duration_s"])
    actual, nominal_path, corrected_path, body_nominal, body_corrected = rollout(
        run, part, nominal, model, config, 0, plot_horizon)
    if len(actual):
        fig, axis = plt.subplots(figsize=(8, 5))
        axis.plot(actual[:, 0], actual[:, 1], "k", label="measured")
        axis.plot(nominal_path[:, 0], nominal_path[:, 1], "--", label="nominal")
        axis.plot(corrected_path[:, 0], corrected_path[:, 1], "-.", label="residual corrected")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("map x [m]")
        axis.set_ylabel("map y [m]")
        axis.grid(True)
        axis.legend()
        fig.tight_layout()
        fig.savefig(str(output / "trajectory_prediction.png"), dpi=160)
        plt.close(fig)

    labels = ("vx [m/s]", "vy [m/s]", "yaw rate [rad/s]")
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for index, axis in enumerate(axes):
        axis.hist(errors_nominal[:, index], bins=60, alpha=0.5, label="nominal error")
        axis.hist(errors_corrected[:, index], bins=60, alpha=0.5, label="corrected error")
        axis.set_ylabel(labels[index])
        axis.grid(True)
    axes[0].legend()
    axes[-1].set_xlabel("one-step prediction error")
    fig.tight_layout()
    fig.savefig(str(output / "one_step_error_histograms.png"), dpi=160)
    plt.close(fig)

    report = {
        "model": str(args.model.resolve()),
        "datasets": [run["source_path"] for run in runs],
        "samples": int(len(errors_nominal)),
        "tail_fraction": float(args.tail_fraction),
        "one_step_rmse_nominal": rmse_nominal.tolist(),
        "one_step_rmse_corrected": rmse_corrected.tolist(),
        "one_step_improvement_percent":
            (100.0 * (1.0 - rmse_corrected / np.maximum(rmse_nominal, 1.0e-9))).tolist(),
        "rollout_horizon_s": horizon,
        "rollout_windows": len(final_nominal),
        "rollout_final_position_rmse_nominal_m":
            float(np.sqrt(np.mean(np.square(final_nominal)))) if final_nominal else None,
        "rollout_final_position_rmse_corrected_m":
            float(np.sqrt(np.mean(np.square(final_corrected)))) if final_corrected else None,
    }
    rollout_nominal = report["rollout_final_position_rmse_nominal_m"]
    rollout_corrected = report["rollout_final_position_rmse_corrected_m"]
    acceptance_checks = {
        "finite_metrics": bool(np.all(np.isfinite(rmse_corrected))),
        "vy_one_step_improved": bool(rmse_corrected[1] < rmse_nominal[1]),
        "yaw_rate_one_step_improved": bool(rmse_corrected[2] < rmse_nominal[2]),
        "rollout_position_improved": bool(
            rollout_nominal is not None and rollout_corrected is not None and
            rollout_corrected < rollout_nominal),
    }
    report["acceptance_checks"] = acceptance_checks
    report["accepted_for_integration"] = all(acceptance_checks.values())
    with (output / "evaluation.json").open("w") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
