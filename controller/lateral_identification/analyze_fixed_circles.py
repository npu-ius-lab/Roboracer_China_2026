#!/usr/bin/env python3
"""Analyze paired, constant-radius circle bags without over-claiming identifiability.

Constant-circle data are useful for the static steering map, curvature gain,
understeer trend, lateral-velocity bias and residual-model validation.  They do
not contain a moving steering transition, so actuator delay/time constant and
independent front/rear cornering stiffnesses cannot be identified from these
bags alone.  This tool reports that distinction explicitly.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares, minimize_scalar

from analyze_bag import read_bag, synchronized


def _finite(value):
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(item) for item in value]
    return value


def _metadata(path: Path) -> Dict[str, object]:
    candidate = path.with_suffix(".json")
    if not candidate.exists():
        raise ValueError(f"missing metadata beside bag: {candidate}")
    return json.loads(candidate.read_text(encoding="utf-8"))


def _median_and_mad(values: np.ndarray) -> Tuple[float, float]:
    median = float(np.median(values))
    mad = float(1.4826 * np.median(np.abs(values - median)))
    return median, mad


def _stable_mask(sync: Dict[str, np.ndarray], metadata: Dict[str, object],
                 settle_s: float, end_margin_s: float) -> np.ndarray:
    times = sync["times"]
    target_speed = float(metadata["speed_command_mps"])
    target_steering = float(metadata["base_command_angle_rad"])
    active = (
        (np.abs(sync["speed_command"] - target_speed) <= max(0.15, 0.12 * target_speed))
        & (np.abs(sync["steering_command"] - target_steering) <= 0.012)
        & (sync["vx"] >= max(0.45, 0.65 * target_speed))
    )
    indices = np.flatnonzero(active)
    if len(indices) < 200:
        raise ValueError("too few active constant-circle samples")
    begin = times[indices[0]] + settle_s
    end = times[indices[-1]] - end_margin_s
    mask = active & (times >= begin) & (times <= end)
    if np.count_nonzero(mask) < 300:
        raise ValueError("too few steady samples after settling margins")
    return mask


def summarize_bag(path: Path, wheelbase: float, settle_s: float,
                  end_margin_s: float) -> Tuple[Dict[str, object], Dict[str, np.ndarray]]:
    metadata = _metadata(path)
    dataset = read_bag(path)
    sync = synchronized(dataset)
    mask = _stable_mask(sync, metadata, settle_s, end_margin_s)
    vx = sync["vx"][mask]
    vy = sync["vy"][mask]
    yaw = sync["odom_yaw_rate"][mask]
    imu_yaw = sync["yaw_rate"][mask]
    command = sync["steering_command"][mask]
    speed = np.hypot(vx, vy)
    kappa = yaw / np.maximum(speed, 0.2)
    ay = speed * yaw
    beta = np.arctan2(vy, np.maximum(vx, 0.2))
    delta_kinematic = np.arctan(wheelbase * kappa)
    radius = np.divide(1.0, np.abs(kappa), out=np.full_like(kappa, np.nan),
                       where=np.abs(kappa) > 1.0e-5)

    vx_median, vx_mad = _median_and_mad(vx)
    vy_median, vy_mad = _median_and_mad(vy)
    yaw_median, yaw_mad = _median_and_mad(yaw)
    row = {
        "bag": str(path.resolve()),
        "name": path.stem,
        "direction": str(metadata["direction"]),
        "speed_command_mps": float(metadata["speed_command_mps"]),
        "radius_command_m": float(metadata["radius_m"]),
        "steering_command_rad": float(np.median(command)),
        "samples": int(np.count_nonzero(mask)),
        "steady_duration_s": float(sync["times"][mask][-1] - sync["times"][mask][0]),
        "vx_mps": vx_median,
        "vx_mad_mps": vx_mad,
        "vy_mps": vy_median,
        "vy_mad_mps": vy_mad,
        "yaw_rate_radps": yaw_median,
        "yaw_rate_mad_radps": yaw_mad,
        "imu_yaw_rate_radps": float(np.median(imu_yaw)),
        "speed_mps": float(np.median(speed)),
        "curvature_per_m": float(np.median(kappa)),
        "radius_from_twist_m": float(np.median(radius)),
        "kinematic_delta_rad": float(np.median(delta_kinematic)),
        "lateral_accel_mps2": float(np.median(ay)),
        "sideslip_proxy_rad": float(np.median(beta)),
        "pointlio_age_median_s": float(np.median(sync["pointlio_age"])),
        "pointlio_age_p95_s": float(np.quantile(sync["pointlio_age"], 0.95)),
        "yaw_rate_imu_odom_rmse_radps": float(np.sqrt(np.mean((imu_yaw - yaw) ** 2))),
    }
    raw = {
        "times": sync["times"][mask], "vx": vx, "vy": vy, "yaw_rate": yaw,
        "command": command, "delta_kinematic": delta_kinematic, "ay": ay,
    }
    return row, raw


def _robust_linear(matrix: np.ndarray, target: np.ndarray,
                   initial: Sequence[float]) -> np.ndarray:
    ordinary = np.linalg.lstsq(matrix, target, rcond=None)[0]
    start = ordinary if np.all(np.isfinite(ordinary)) else np.asarray(initial, dtype=float)
    scale = max(1.0e-4, 1.4826 * np.median(np.abs(target - matrix @ start)))
    result = least_squares(lambda values: (matrix @ values - target) / scale,
                           start, loss="soft_l1", f_scale=1.0, max_nfev=3000)
    return result.x


def _ci(values: Iterable[float]) -> Dict[str, float]:
    array = np.asarray(list(values), dtype=float)
    return {
        "median": float(np.median(array)),
        "lower_2p5": float(np.quantile(array, 0.025)),
        "upper_97p5": float(np.quantile(array, 0.975)),
    }


def fit_static_map(rows: List[Dict[str, object]], low_speed_max: float,
                   bootstrap: int, rng: np.random.Generator) -> Dict[str, object]:
    selected = [row for row in rows if float(row["speed_command_mps"]) <= low_speed_max]
    matrix = np.asarray([[row["steering_command_rad"], 1.0] for row in selected], dtype=float)
    target = np.asarray([row["kinematic_delta_rad"] for row in selected], dtype=float)
    fit = _robust_linear(matrix, target, [1.0, 0.0])
    prediction = matrix @ fit
    samples = []
    for _ in range(bootstrap):
        indices = rng.integers(0, len(selected), len(selected))
        if np.linalg.matrix_rank(matrix[indices]) < 2:
            continue
        samples.append(_robust_linear(matrix[indices], target[indices], fit))
    return {
        "method": "robust low-speed kinematic equivalent-angle fit",
        "selected_bags": [row["name"] for row in selected],
        "gain": float(fit[0]),
        "bias_rad": float(fit[1]),
        "rmse_rad": float(np.sqrt(np.mean((prediction - target) ** 2))),
        "max_abs_error_rad": float(np.max(np.abs(prediction - target))),
        "bootstrap_successful": len(samples),
        "interval_95": {
            "gain": _ci(sample[0] for sample in samples),
            "bias_rad": _ci(sample[1] for sample in samples),
        } if samples else {},
        "identifiability": "static effective map only; tau and dead time are not observable",
    }


def fit_understeer(rows: List[Dict[str, object]], gain: float, bias: float,
                   mass: float, wheelbase: float, lf: float, lr: float,
                   reference_cf: float, reference_cr: float,
                   bootstrap: int, rng: np.random.Generator) -> Dict[str, object]:
    command = np.asarray([row["steering_command_rad"] for row in rows], dtype=float)
    kinematic = np.asarray([row["kinematic_delta_rad"] for row in rows], dtype=float)
    ay = np.asarray([row["lateral_accel_mps2"] for row in rows], dtype=float)
    effective = gain * command + bias
    residual = effective - kinematic

    def solve(indices: np.ndarray) -> float:
        x = ay[indices]
        y = residual[indices]
        scale = max(2.0e-4, 1.4826 * np.median(np.abs(y - np.median(y))))
        result = least_squares(lambda value: (value[0] * x - y) / scale,
                               [0.0], loss="soft_l1", f_scale=1.0)
        return float(result.x[0])

    all_indices = np.arange(len(rows))
    kus = solve(all_indices)
    samples = [solve(rng.integers(0, len(rows), len(rows))) for _ in range(bootstrap)]
    prediction = kinematic + kus * ay
    rmse = float(np.sqrt(np.mean((effective - prediction) ** 2)))
    denominator_cf = kus * wheelbase / mass + lf / reference_cr
    cf_if_cr_reference = lr / denominator_cf if denominator_cf > 0.0 else float("nan")
    denominator_cr = lr / reference_cf - kus * wheelbase / mass
    cr_if_cf_reference = lf / denominator_cr if denominator_cr > 0.0 else float("nan")
    reference_kus = mass / wheelbase * (lr / reference_cf - lf / reference_cr)

    # A single steady-state K_us cannot uniquely identify both axle
    # stiffnesses.  This is the closest positive pair to the trusted prior in
    # log-parameter space, subject to the measured K_us constraint.  Keeping
    # this explicit avoids presenting an arbitrary Cf/Cr split as raw data.
    def prior_distance(log_cf: float) -> float:
        cf = math.exp(float(log_cf))
        denominator = lr / cf - kus * wheelbase / mass
        if denominator <= 0.0:
            return 1.0e12
        cr = lf / denominator
        return math.log(cf / reference_cf) ** 2 + math.log(cr / reference_cr) ** 2

    regularized = minimize_scalar(
        prior_distance, bounds=(math.log(10.0), math.log(500.0)), method="bounded")
    regularized_cf = math.exp(float(regularized.x))
    regularized_cr = lf / (
        lr / regularized_cf - kus * wheelbase / mass)
    return {
        "method": "delta = atan(L*kappa) + K_us*a_y, robust zero-intercept fit",
        "understeer_gradient_rad_per_mps2": kus,
        "interval_95": _ci(samples),
        "effective_delta_fit_rmse_rad": rmse,
        "effective_delta_max_abs_error_rad": float(np.max(np.abs(effective - prediction))),
        "reference_pair": {
            "Cf_N_per_rad": reference_cf,
            "Cr_N_per_rad": reference_cr,
            "implied_understeer_gradient_rad_per_mps2": reference_kus,
        },
        "stiffness_family_examples": {
            f"Cf_if_Cr_fixed_at_{reference_cr:g}": cf_if_cr_reference,
            f"Cr_if_Cf_fixed_at_{reference_cf:g}": cr_if_cf_reference,
        },
        "prior_regularized_candidate": {
            "Cf_N_per_rad": regularized_cf,
            "Cr_N_per_rad": regularized_cr,
            "prior_Cf_N_per_rad": reference_cf,
            "prior_Cr_N_per_rad": reference_cr,
            "method": "closest log-space pair to prior subject to measured K_us",
            "warning": "regularized candidate, not two independently observed stiffnesses",
        },
        "identifiability": (
            "K_us supplies one constraint on Cf and Cr; independent Cf/Cr require "
            "transient lateral excitation or a trusted sideslip measurement"
        ),
    }


def paired_sideslip(rows: List[Dict[str, object]]) -> Dict[str, object]:
    speeds = sorted(set(float(row["speed_command_mps"]) for row in rows))
    pairs = []
    for speed in speeds:
        left = next(row for row in rows if row["direction"] == "left"
                    and float(row["speed_command_mps"]) == speed)
        right = next(row for row in rows if row["direction"] == "right"
                     and float(row["speed_command_mps"]) == speed)
        beta_left = float(left["sideslip_proxy_rad"])
        beta_right = float(right["sideslip_proxy_rad"])
        pairs.append({
            "speed_command_mps": speed,
            "common_mode_beta_rad": 0.5 * (beta_left + beta_right),
            "turn_antisymmetric_beta_rad": 0.5 * (beta_left - beta_right),
        })
    common = np.asarray([item["common_mode_beta_rad"] for item in pairs])
    return {
        "paired_values": pairs,
        "estimated_common_sideslip_bias_rad": float(np.median(common)),
        "common_bias_mad_rad": float(1.4826 * np.median(np.abs(common - np.median(common)))),
        "warning": "vehicle_odom vy contains a clear common-mode bias; do not fit Cf/Cr directly from raw vy",
    }


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    keys = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(path: Path, report: Dict[str, object]) -> None:
    static = report["static_steering_map"]
    understeer = report["steady_state_lateral_model"]
    friction = report["friction_observation"]
    inherited = report["inherited_dynamic_parameters"]
    lines = [
        "# 固定半径左右转数据辨识报告", "",
        f"生成时间：{report['generated_at']}", "",
        "## 可直接使用的结论", "",
        f"- 低速等效转向映射：`delta = {static['gain']:.6f} * u "
        f"{static['bias_rad']:+.6f}` rad。",
        f"- 欠转向梯度：`{understeer['understeer_gradient_rad_per_mps2']:.6g} rad/(m/s^2)`。",
        f"- 已观测最大横向加速度：`{friction['max_observed_abs_ay_mps2']:.3f} m/s^2`，"
        f"只说明 `mu >= {friction['minimum_observed_mu']:.3f}`。",
        f"- odom 横向速度公共偏置约：`{report['paired_sideslip']['estimated_common_sideslip_bias_rad']:.5f} rad`。",
        "", "## 不能由这批 bag 单独辨识的参数", "",
        f"- 舵机时间常数沿用动态试验值：`{inherited['time_constant_s']:.6f} s`。",
        f"- 舵机纯延迟沿用动态试验值：`{inherited['dead_time_s']:.6f} s`。",
        "- Cf/Cr 只有一个欠转向梯度约束，不能把两个数都当成新识别真值。",
        "", "## 每个 bag 的稳态摘要", "",
        "| direction | v_cmd | vx | yaw rate | radius | ay | delta_kin | odom age |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in report["runs"]:
        lines.append(
            f"| {row['direction']} | {row['speed_command_mps']:.1f} | {row['vx_mps']:.3f} | "
            f"{row['yaw_rate_radps']:.3f} | {row['radius_from_twist_m']:.3f} | "
            f"{row['lateral_accel_mps2']:.3f} | {row['kinematic_delta_rad']:.4f} | "
            f"{row['pointlio_age_median_s']:.3f} |"
        )
    lines.extend(["", "完整数值、置信区间和刚度参数族见同名 JSON。", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_summary(path: Path, rows: List[Dict[str, object]], gain: float,
                 bias: float, kus: float, wheelbase: float) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    figure, axes = plt.subplots(2, 2, figsize=(11, 8))
    for direction, marker in (("left", "o"), ("right", "s")):
        selected = [row for row in rows if row["direction"] == direction]
        speed = [row["speed_command_mps"] for row in selected]
        axes[0, 0].plot(speed, [row["radius_from_twist_m"] for row in selected], marker, label=direction)
        axes[0, 1].plot(speed, [abs(row["lateral_accel_mps2"]) for row in selected], marker, label=direction)
    command = np.asarray([row["steering_command_rad"] for row in rows])
    ay = np.asarray([row["lateral_accel_mps2"] for row in rows])
    delta_kin = np.asarray([row["kinematic_delta_rad"] for row in rows])
    effective = gain * command + bias
    axes[1, 0].scatter(ay, effective - delta_kin)
    grid = np.linspace(np.min(ay), np.max(ay), 100)
    axes[1, 0].plot(grid, kus * grid)
    axes[1, 1].scatter(command, delta_kin)
    command_grid = np.linspace(np.min(command), np.max(command), 100)
    axes[1, 1].plot(command_grid, gain * command_grid + bias)
    axes[0, 0].axhline(2.5, color="k", linestyle="--", linewidth=.8)
    axes[0, 0].set(title="Measured radius", xlabel="command speed [m/s]", ylabel="m")
    axes[0, 1].set(title="Lateral acceleration", xlabel="command speed [m/s]", ylabel="m/s²")
    axes[1, 0].set(title="Understeer relation", xlabel="signed ay [m/s²]", ylabel="delta - atan(L*kappa) [rad]")
    axes[1, 1].set(title="Low-speed static steering map", xlabel="command u [rad]", ylabel="kinematic delta [rad]")
    for axis in axes.flat:
        axis.grid(True); axis.legend(loc="best") if axis is axes[0, 0] or axis is axes[0, 1] else None
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--wheelbase", type=float, default=0.320)
    parser.add_argument("--mass", type=float, default=3.8)
    parser.add_argument("--lf", type=float, default=0.195)
    parser.add_argument("--lr", type=float, default=0.125)
    parser.add_argument("--reference-cf", type=float, default=70.0)
    parser.add_argument("--reference-cr", type=float, default=110.0)
    parser.add_argument("--low-speed-max", type=float, default=1.5)
    parser.add_argument("--settle", type=float, default=5.0)
    parser.add_argument("--end-margin", type=float, default=2.0)
    parser.add_argument("--bootstrap", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=20260820)
    parser.add_argument("--inherited-tau", type=float, default=0.062671845)
    parser.add_argument("--inherited-dead-time", type=float, default=0.105)
    args = parser.parse_args()

    if len(args.bags) < 4:
        raise SystemExit("at least two left/right speed pairs are required")
    rows, raw = [], []
    for bag in args.bags:
        row, signals = summarize_bag(bag.expanduser().resolve(), args.wheelbase,
                                     args.settle, args.end_margin)
        rows.append(row); raw.append(signals)
    rows.sort(key=lambda item: (float(item["speed_command_mps"]), str(item["direction"])))
    rng = np.random.default_rng(args.seed)
    static = fit_static_map(rows, args.low_speed_max, args.bootstrap, rng)
    understeer = fit_understeer(
        rows, static["gain"], static["bias_rad"], args.mass, args.wheelbase,
        args.lf, args.lr, args.reference_cf, args.reference_cr, args.bootstrap, rng,
    )
    max_ay = max(abs(float(row["lateral_accel_mps2"])) for row in rows)
    report = _finite({
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "status": "identified_candidate_not_deployed",
        "method": "paired_constant_circle_steady_state",
        "runs": rows,
        "static_steering_map": static,
        "inherited_dynamic_parameters": {
            "time_constant_s": args.inherited_tau,
            "dead_time_s": args.inherited_dead_time,
            "source": "previous moving-PRBS identification; not observable in these fixed-command bags",
        },
        "steady_state_lateral_model": understeer,
        "paired_sideslip": paired_sideslip(rows),
        "friction_observation": {
            "max_observed_abs_ay_mps2": max_ay,
            "minimum_observed_mu": max_ay / 9.80665,
            "identifiability": "lower bound only; no saturation event was recorded",
        },
        "reference_geometry": {
            "mass_kg": args.mass, "wheelbase_m": args.wheelbase,
            "lf_m": args.lf, "lr_m": args.lr,
        },
        "warnings": [
            "The collector used an older inverse steering map, so the achieved radius is not exactly 2.5 m.",
            "No moving steering transitions: do not refit actuator tau or dead time from this dataset.",
            "Raw odom vy has common-mode bias: direct per-bag Cf/Cr fits are rejected.",
            "A fixed-circle dataset alone is insufficient to validate a residual model during fast steering transients.",
        ],
    })
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")
    write_csv(output.with_suffix(".csv"), rows)
    write_markdown(output.with_suffix(".md"), report)
    plot_summary(output.with_suffix(".png"), rows, static["gain"], static["bias_rad"],
                 understeer["understeer_gradient_rad_per_mps2"], args.wheelbase)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    print(f"[REPORT] {output}")


if __name__ == "__main__":
    main()
