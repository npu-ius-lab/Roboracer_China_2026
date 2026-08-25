#!/usr/bin/env python3
"""Jointly identify the effective steering actuator from complete ROS bags.

The tool is read-only with respect to controller configuration. It keeps bags
as independent groups, reports leave-one-run-out validation, and emits a
bootstrap parameter interval instead of selecting whichever single run happens
to have the smallest training error.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, List

import numpy as np

from analyze_bag import read_bag, synchronized
from fit_core import (
    ActuatorFit,
    PreparedActuatorRun,
    evaluate_steering_actuator,
    fit_steering_actuator_joint,
    prepare_actuator_run,
)


def fit_dict(fit: ActuatorFit) -> Dict[str, float]:
    return fit.as_dict()


def confidence_interval(values: List[float]) -> Dict[str, float]:
    array = np.asarray(values, dtype=float)
    return {
        "median": float(np.median(array)),
        "lower_2p5": float(np.quantile(array, 0.025)),
        "upper_97p5": float(np.quantile(array, 0.975)),
    }


def bootstrap_parameters(
    runs: List[PreparedActuatorRun],
    reference: ActuatorFit,
    iterations: int,
    seed: int,
) -> Dict[str, object]:
    if iterations <= 0:
        return {"iterations": 0, "warning": "bootstrap disabled"}
    rng = np.random.default_rng(seed)
    low = max(0.0, reference.delay - 0.035)
    high = min(0.25, reference.delay + 0.035)
    delay_grid = np.arange(low, high + 1.0e-9, 0.005)
    samples = {"gain": [], "bias_rad": [], "time_constant_s": [], "dead_time_s": []}
    failures = []
    for index in range(iterations):
        selected = [runs[item] for item in rng.integers(0, len(runs), len(runs))]
        try:
            fit, _ = fit_steering_actuator_joint(selected, delay_grid=delay_grid)
        except Exception as error:  # Keep a partial uncertainty report usable.
            failures.append({"iteration": index, "error": str(error)})
            continue
        samples["gain"].append(fit.gain)
        samples["bias_rad"].append(fit.bias)
        samples["time_constant_s"].append(fit.tau)
        samples["dead_time_s"].append(fit.delay)
    result: Dict[str, object] = {
        "iterations": iterations,
        "successful": len(samples["gain"]),
        "group_resampling": "complete_bags_with_replacement",
        "failures": failures,
    }
    if samples["gain"]:
        result["interval_95"] = {
            name: confidence_interval(values) for name, values in samples.items()
        }
    if len(runs) < 3:
        result["warning"] = (
            "fewer than three independent bags: the group-bootstrap interval is preliminary"
        )
    return result


def leave_one_run_out(runs: List[PreparedActuatorRun]) -> List[Dict[str, object]]:
    if len(runs) < 2:
        return []
    results = []
    for held_index, held in enumerate(runs):
        training = [run for index, run in enumerate(runs) if index != held_index]
        fit, _ = fit_steering_actuator_joint(training)
        metrics, _ = evaluate_steering_actuator(held, fit)
        results.append({
            "held_out": held.name,
            "trained_on": [run.name for run in training],
            "fit": fit_dict(fit),
            "metrics": metrics,
        })
    return results


def finite(value):
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite(item) for item in value]
    return value


def write_markdown(path: Path, report: Dict[str, object]) -> None:
    fit = report["joint_candidate"]
    lines = [
        "# TianRacer 转向执行器联合辨识", "",
        f"生成时间：{report['generated_at']}", "",
        "该结果是无舵角传感器条件下的等效命令到前轮角模型，不会自动修改实车配置。",
        "", "## 联合候选", "",
        f"- 增益 K：`{fit['static_gain']:.6f}`",
        f"- 偏置 b：`{fit['offset_rad']:.6f} rad`",
        f"- 纯延迟 Td：`{fit['dead_time_s']:.6f} s`",
        f"- 时间常数 tau：`{fit['time_constant_s']:.6f} s`",
        f"- 动态 RMSE：`{fit['dynamic_delta_rmse_rad']:.6f} rad`",
        f"- 动态 R2：`{fit['dynamic_r2']:.5f}`",
        "", "## 每个完整 bag", "",
    ]
    for item in report["per_run_metrics"]:
        lines.append(
            f"- `{item['name']}`: RMSE={item['rmse_rad']:.5f} rad, "
            f"R2={item['r2']:.4f}, optimistic-P95="
            f"{item['p95_optimistic_abs_delta_rad']:.5f} rad"
        )
    lines.extend(["", "## 留一验证", ""])
    for item in report["leave_one_run_out"]:
        metrics = item["metrics"]
        lines.append(
            f"- 留出 `{item['held_out']}`: RMSE={metrics['rmse_rad']:.5f} rad, "
            f"R2={metrics['r2']:.4f}"
        )
    lines.extend([
        "", "## 状态", "",
        f"`{report['status']}`", "",
        "需要新的多速度 PRBS bag 和完全未激励 stable 圈作为最终独立验收。", "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_runs(path: Path, runs: List[PreparedActuatorRun], predictions: List[np.ndarray]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    figure, axes = plt.subplots(len(runs), 1, figsize=(12, max(4, 3.4 * len(runs))), squeeze=False)
    for axis, run, prediction in zip(axes[:, 0], runs, predictions):
        time = run.times - run.times[0]
        axis.plot(time, run.command, linewidth=0.8, label="command")
        axis.plot(time, run.delta_proxy, linewidth=0.8, alpha=0.75, label="equivalent angle")
        axis.plot(time, prediction, linewidth=1.0, label="joint fit")
        axis.set_title(run.name)
        axis.set_ylabel("rad")
        axis.grid(True)
        axis.legend(loc="upper right")
    axes[-1, 0].set_xlabel("time [s]")
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--wheelbase", type=float, default=0.320)
    parser.add_argument("--bootstrap", type=int, default=32)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    if args.bootstrap < 0 or args.bootstrap > 500:
        raise SystemExit("--bootstrap must be within [0, 500]")

    datasets = [read_bag(path.expanduser().resolve()) for path in args.bags]
    synchronized_runs = [synchronized(dataset) for dataset in datasets]
    runs = []
    for dataset, sync in zip(datasets, synchronized_runs):
        runs.append(prepare_actuator_run(
            sync["times"], sync["steering_command"], sync["vx"], sync["yaw_rate"],
            wheelbase=args.wheelbase, source=sync["yaw_rate_source"],
            name=Path(dataset["path"]).stem,
            platform_signal=sync["identification_excitation"],
        ))

    fit, diagnostics = fit_steering_actuator_joint(runs)
    loo = leave_one_run_out(runs)
    bootstrap = bootstrap_parameters(runs, fit, args.bootstrap, args.seed)
    speed_ranges = [{
        "name": run.name,
        "median_mps": float(np.median(run.vx[run.dynamic_mask])),
        "p05_mps": float(np.quantile(run.vx[run.dynamic_mask], 0.05)),
        "p95_mps": float(np.quantile(run.vx[run.dynamic_mask], 0.95)),
    } for run in runs]
    leave_pass = bool(loo) and all(
        item["metrics"]["rmse_rad"] < 0.02 and item["metrics"]["r2"] > 0.90
        for item in loo
    )
    report = finite({
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "status": "candidate_needs_new_independent_bags",
        "bags": [str(dataset["path"]) for dataset in datasets],
        "method": "joint_grouped_fopdt_equivalent_steering_angle",
        "joint_candidate": fit_dict(fit),
        "per_run_metrics": diagnostics["per_run"],
        "leave_one_run_out": loo,
        "bootstrap": bootstrap,
        "speed_coverage": speed_ranges,
        "preliminary_thresholds": {
            "leave_one_run_out_pass": leave_pass,
            "minimum_independent_bags_pass": len(runs) >= 3,
            "multiple_speed_groups_pass": (
                max(item["median_mps"] for item in speed_ranges)
                - min(item["median_mps"] for item in speed_ranges) >= 0.35
            ),
        },
        "warnings": [
            "No steering-angle sensor: this is an effective command-to-motion model.",
            "Do not write the candidate into controller YAML without untouched-bag rollout validation.",
            "Fit tire parameters only after freezing the actuator model.",
        ],
    })
    if args.output:
        output = args.output.expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = args.bags[0].expanduser().resolve().parent / f"joint_actuator_{stamp}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(output.with_suffix(".md"), report)
    plot_runs(output.with_suffix(".png"), runs, diagnostics["predictions"])
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"[REPORT] {output}")


if __name__ == "__main__":
    main()
