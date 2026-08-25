#!/usr/bin/env python3
"""Offline replay of unified versus direction-dependent steering actuator models."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

from analyze_bag import read_bag, synchronized
from fit_core import (
    ActuatorFit,
    first_order_response,
    prepare_actuator_run,
    evaluate_steering_actuator,
)


def as_fit(values):
    return ActuatorFit(
        gain=float(values["gain"]), bias=float(values["bias_rad"]),
        tau=float(values["time_constant_s"]), delay=float(values["dead_time_s"]),
        static_rmse=float("nan"), dynamic_rmse=float("nan"),
        dynamic_r2=float("nan"), samples=0, static_samples=0,
        source="candidate",
    )


def finite(value):
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite(item) for item in value]
    return value


def evaluate(run, fit):
    metrics, prediction = evaluate_steering_actuator(run, fit)
    return metrics, prediction


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", nargs="+")
    args = parser.parse_args()
    candidate_data = json.loads(args.candidates.read_text(encoding="utf-8"))
    models = {
        name: as_fit(values)
        for name, values in candidate_data["candidates"].items()
    }
    reports = []
    all_errors = {"unified": [], "directional": []}
    for bag_name in args.bags:
        path = Path(bag_name).expanduser().resolve()
        dataset = read_bag(path)
        sync = synchronized(dataset)
        run = prepare_actuator_run(
            sync["times"], sync["steering_command"], sync["vx"],
            sync["yaw_rate"], wheelbase=0.320,
            source=sync["yaw_rate_source"], name=path.stem,
            platform_signal=sync["identification_excitation"],
        )
        unified, _ = evaluate(run, models["unified"])
        sign = float(np.sign(np.median(run.command[run.dynamic_mask])))
        direction_name = "left" if sign >= 0.0 else "right"
        directional, _ = evaluate(run, models[direction_name])
        for key, result in (("unified", unified), ("directional", directional)):
            all_errors[key].extend(
                [result["rmse_rad"]] * max(1, result["samples"])
            )
        reports.append({
            "bag": str(path),
            "direction_selected": direction_name,
            "median_speed_mps": float(np.median(run.vx[run.dynamic_mask])),
            "unified": unified,
            "directional": directional,
            "rmse_improvement_percent": float(
                100.0 * (unified["rmse_rad"] - directional["rmse_rad"])
                / max(unified["rmse_rad"], 1.0e-9)
            ),
        })
    summary = {}
    for key, values in all_errors.items():
        summary[key + "_sample_weighted_rmse_rad"] = float(
            np.average(values)
        ) if values else None
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    report = finite({
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "method": "offline_command_replay_against_effective_angle_proxy",
        "candidate_file": str(args.candidates.resolve()),
        "bags": reports,
        "summary": summary,
        "decision": (
            "directional_candidate_better_on_replay"
            if summary["directional_sample_weighted_rmse_rad"]
            < summary["unified_sample_weighted_rmse_rad"]
            else "unified_candidate_better_on_replay"
        ),
        "deployment_status": "experimental_only_stable_untouched",
        "warnings": [
            "Equivalent-angle proxy comes from yaw rate and contains tire/road effects.",
            "This replay validates command-to-motion prediction, not a front-wheel sensor measurement.",
        ],
    })
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md = output.with_suffix(".md")
    lines = [
        "# Steering actuator directional replay", "",
        f"Generated: {report['generated_at']}", "",
        f"Decision: `{report['decision']}`", "",
        "| bag | direction | speed | unified RMSE | directional RMSE | improvement |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for item in reports:
        lines.append(
            f"| `{Path(item['bag']).name}` | {item['direction_selected']} | "
            f"{item['median_speed_mps']:.3f} | {item['unified']['rmse_rad']:.5f} | "
            f"{item['directional']['rmse_rad']:.5f} | "
            f"{item['rmse_improvement_percent']:.1f}% |"
        )
    lines.extend(["", "Stable controller files were not modified."])
    md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"[REPORT] {output}")


if __name__ == "__main__":
    main()
