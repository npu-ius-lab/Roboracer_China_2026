#!/usr/bin/env python3
"""Analyze one or more lateral-identification bags without editing controller YAML."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

try:
    from .fit_core import (
        ActuatorFit,
        fit_steering_actuator,
        fit_tire_stiffness,
        zoh,
    )
except ImportError:  # Direct execution from lateral_identification/.
    from fit_core import ActuatorFit, fit_steering_actuator, fit_tire_stiffness, zoh


COMMAND_STAMPED = "/f1tenth_identification/command_stamped"
COMMAND_MPCC_STAMPED = "/f1tenth_mpcc/real/ackermann_cmd_stamped"
COMMAND_RAW = "/tianracer/ackermann_cmd"
ODOM = "/localization/vehicle_odom"
IMU = "/livox/imu"
IMU_FALLBACK = "/tianracer/imu"
IDENTIFICATION_DEBUG = "/f1tenth_identification/debug"


def message_stamp(message, receipt) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    value = stamp.to_sec() if stamp is not None else 0.0
    return value if value > 0.0 else receipt.to_sec()


def sorted_unique(data: List[tuple]) -> np.ndarray:
    array = np.asarray(sorted(data, key=lambda row: row[0]), dtype=float)
    if len(array) < 2:
        return array
    keep = np.r_[True, np.diff(array[:, 0]) > 1.0e-6]
    return array[keep]


def read_bag(path: Path) -> Dict[str, object]:
    try:
        import rosbag
    except ImportError as error:
        raise SystemExit(f"ROS rosbag Python module is required: {error}")
    stamped, mpcc_stamped, raw, odom = [], [], [], []
    imu, fallback_imu, identification_debug = [], [], []
    with rosbag.Bag(str(path)) as bag:
        for topic, message, receipt in bag.read_messages(
            topics=[COMMAND_STAMPED, COMMAND_MPCC_STAMPED, COMMAND_RAW,
                    ODOM, IMU, IMU_FALLBACK, IDENTIFICATION_DEBUG]
        ):
            if topic == COMMAND_STAMPED:
                stamped.append((message_stamp(message, receipt), message.drive.speed,
                                message.drive.steering_angle))
            elif topic == COMMAND_MPCC_STAMPED:
                mpcc_stamped.append((message_stamp(message, receipt), message.drive.speed,
                                     message.drive.steering_angle))
            elif topic == COMMAND_RAW:
                raw.append((receipt.to_sec(), message.speed, message.steering_angle))
            elif topic == ODOM:
                twist = message.twist.twist
                odom.append((message_stamp(message, receipt), receipt.to_sec(),
                             twist.linear.x, twist.linear.y, twist.angular.z))
            elif topic == IMU:
                imu.append((message_stamp(message, receipt), receipt.to_sec(),
                            message.angular_velocity.z,
                            message.linear_acceleration.x,
                            message.linear_acceleration.y,
                            message.linear_acceleration.z))
            elif topic == IMU_FALLBACK:
                fallback_imu.append((message_stamp(message, receipt), receipt.to_sec(),
                                     message.angular_velocity.z,
                                     message.linear_acceleration.x,
                                     message.linear_acceleration.y,
                                     message.linear_acceleration.z))
            elif topic == IDENTIFICATION_DEBUG and len(message.data) >= 4:
                identification_debug.append((
                    receipt.to_sec(), message.data[0], message.data[1],
                    message.data[2], message.data[3],
                ))
    if len(stamped) >= 20:
        command_values, command_source = stamped, COMMAND_STAMPED
    elif len(mpcc_stamped) >= 20:
        command_values, command_source = mpcc_stamped, COMMAND_MPCC_STAMPED
    else:
        command_values, command_source = raw, COMMAND_RAW
    command = sorted_unique(command_values)
    odometry = sorted_unique(odom)
    inertial_source = IMU if len(imu) >= 100 else IMU_FALLBACK
    inertial = sorted_unique(imu if len(imu) >= 100 else fallback_imu)
    debug = sorted_unique(identification_debug)
    if len(command) < 50 or len(odometry) < 50:
        raise ValueError(f"{path} lacks command or vehicle odometry samples")
    return {
        "path": path.resolve(), "command": command, "odom": odometry,
        "imu": inertial, "identification_debug": debug,
        "command_source": command_source, "imu_source": inertial_source,
    }


def synchronized(dataset: Dict[str, object]) -> Dict[str, np.ndarray]:
    command = dataset["command"]
    odom = dataset["odom"]
    imu = dataset["imu"]
    identification_debug = dataset["identification_debug"]
    use_imu = len(imu) >= 100
    begin = max(command[0, 0], odom[0, 0], imu[0, 0] if use_imu else odom[0, 0])
    end = min(command[-1, 0], odom[-1, 0], imu[-1, 0] if use_imu else odom[-1, 0])
    if end - begin < 8.0:
        raise ValueError("bag has less than 8 s of overlapping command and state data")
    dt = 0.01 if use_imu else 0.02
    times = np.arange(begin, end, dt)
    speed_command = zoh(times, command[:, 0], command[:, 1])
    steering_command = zoh(times, command[:, 0], command[:, 2])
    vx = np.interp(times, odom[:, 0], odom[:, 2])
    vy = np.interp(times, odom[:, 0], odom[:, 3])
    odom_yaw_rate = np.interp(times, odom[:, 0], odom[:, 4])
    yaw_rate = np.interp(times, imu[:, 0], imu[:, 2]) if use_imu else odom_yaw_rate
    acceleration_norm_median = (
        float(np.median(np.linalg.norm(imu[:, 3:6], axis=1))) if use_imu else float("nan")
    )
    if len(identification_debug) >= 20:
        identification_base_steering = zoh(
            times, identification_debug[:, 0], identification_debug[:, 1]
        )
        identification_excitation = zoh(
            times, identification_debug[:, 0], identification_debug[:, 2]
        )
    else:
        identification_base_steering = np.full(len(times), np.nan)
        identification_excitation = np.full(len(times), np.nan)
    return {
        "times": times, "speed_command": speed_command,
        "steering_command": steering_command, "vx": vx, "vy": vy,
        "yaw_rate": yaw_rate, "odom_yaw_rate": odom_yaw_rate,
        "yaw_rate_source": dataset.get("imu_source", IMU) if use_imu else "vehicle_odom",
        "imu_acceleration_norm_median_raw": acceleration_norm_median,
        "pointlio_age": odom[:, 1] - odom[:, 0],
        "identification_base_steering": identification_base_steering,
        "identification_excitation": identification_excitation,
    }


def metadata_for(path: Path) -> Dict[str, object]:
    candidate = path.with_suffix(".metadata.json")
    return json.loads(candidate.read_text()) if candidate.exists() else {}


def actuator_from_report(path: Path) -> ActuatorFit:
    report = json.loads(path.read_text())
    values = report["steering_actuator_candidate"]
    return ActuatorFit(
        gain=float(values["static_gain"]), bias=float(values["offset_rad"]),
        tau=float(values["time_constant_s"]), delay=float(values["dead_time_s"]),
        static_rmse=float(values.get("static_delta_rmse_rad", float("nan"))),
        dynamic_rmse=float(values.get("dynamic_delta_rmse_rad", float("nan"))),
        dynamic_r2=float(values.get("dynamic_r2", float("nan"))),
        samples=int(values.get("samples", 0)), static_samples=int(values.get("static_samples", 0)),
        source=str(values.get("yaw_rate_source", "saved_result")),
    )


def nominal_actuator(root: Path) -> ActuatorFit:
    controller = yaml.safe_load((root / "src/f1tenth_dynamic_mpcc/config/controller.yaml").read_text())
    actuator = controller["actuator"]
    return ActuatorFit(
        gain=float(actuator["steering_static_gain"]), bias=0.0,
        tau=float(actuator["steering_time_constant_s"]),
        delay=float(actuator["steering_dead_time_s"]),
        static_rmse=float("nan"), dynamic_rmse=float("nan"), dynamic_r2=float("nan"),
        samples=0, static_samples=0, source="nominal_not_identified",
    )


def find_actuator_result(directory: Path) -> Optional[Path]:
    candidates = []
    for path in directory.glob("*.identification.json"):
        try:
            report = json.loads(path.read_text())
            if "steering_actuator_candidate" in report:
                candidates.append(path)
        except Exception:
            continue
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def finite(value):
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite(item) for item in value]
    return value


def write_markdown(path: Path, report: Dict[str, object]) -> None:
    lines = [
        "# TianRacer 横向参数辨识报告", "",
        f"生成时间：{report['generated_at']}", "",
        "本报告只产生候选参数，不会自动修改实车配置。", "",
    ]
    if "steering_actuator_candidate" in report:
        a = report["steering_actuator_candidate"]
        lines.extend([
            "## 转向执行器候选", "",
            f"- 静态增益 K：`{a['static_gain']:.6f}`",
            f"- 中位偏置：`{a['offset_rad']:.6f} rad`",
            f"- 时间常数 τ：`{a['time_constant_s']:.6f} s`",
            f"- 纯延迟 Td：`{a['dead_time_s']:.6f} s`",
            f"- 动态拟合 RMSE：`{a['dynamic_delta_rmse_rad']:.6f} rad`",
            f"- 动态拟合 R²：`{a['dynamic_r2']:.4f}`", "",
        ])
    if "tire_stiffness_candidate" in report:
        t = report["tire_stiffness_candidate"]
        lines.extend([
            "## 轮胎侧偏刚度候选", "",
            f"- 前轴 Cf：`{t['Cf_N_per_rad']:.3f} N/rad`",
            f"- 后轴 Cr：`{t['Cr_N_per_rad']:.3f} N/rad`",
            f"- 前轴拟合 R²：`{t['front_force_r2']:.4f}`",
            f"- 后轴拟合 R²：`{t['rear_force_r2']:.4f}`", "",
        ])
    lines.extend(["## 验收要求", "", *[f"- {item}" for item in report["warnings"]], ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_report(path: Path, sync: Dict[str, np.ndarray], actuator_diagnostics=None,
                tire_diagnostics=None) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    times = sync["times"] - sync["times"][0]
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(times, sync["steering_command"], label="steering command")
    if actuator_diagnostics is not None:
        axes[0].plot(times, actuator_diagnostics["delta_proxy"], alpha=.7, label="kinematic proxy")
        axes[0].plot(times, actuator_diagnostics["delta_prediction"], label="actuator fit")
    axes[0].set_ylabel("steering [rad]"); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(times, sync["vx"], label="vx")
    axes[1].plot(times, sync["yaw_rate"], label="yaw rate")
    axes[1].set_ylabel("state"); axes[1].legend(); axes[1].grid(True)
    if tire_diagnostics is not None:
        front = tire_diagnostics["front_mask"]; rear = tire_diagnostics["rear_mask"]
        axes[2].scatter(tire_diagnostics["alpha_f"][front], tire_diagnostics["fyf"][front], s=4, label="front")
        axes[2].scatter(tire_diagnostics["alpha_r"][rear], tire_diagnostics["fyr"][rear], s=4, label="rear")
        axes[2].set_xlabel("slip angle [rad]"); axes[2].set_ylabel("lateral force [N]")
    else:
        axes[2].plot(times, sync["vy"], label="vy")
        axes[2].set_xlabel("time [s]"); axes[2].set_ylabel("vy [m/s]")
    axes[2].legend(); axes[2].grid(True)
    figure.tight_layout(); figure.savefig(path, dpi=140); plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--actuator-result", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    datasets = [read_bag(path.expanduser().resolve()) for path in args.bags]
    synchronized_sets = [synchronized(dataset) for dataset in datasets]
    metadata = [metadata_for(dataset["path"]) for dataset in datasets]
    profiles = [str(item.get("profile", "unknown")) for item in metadata]
    output = (args.output or args.bags[0].with_suffix(".identification.json")).resolve()
    warnings = [
        "必须使用独立 bag 验证，不能直接把候选参数写入 vehicle.yaml。",
        "没有真实前轮角传感器，转向拟合仍是 IMU/运动学等效角估计。",
        "Cf/Cr 与 Iz、质心位置和转向模型耦合，应检查拟合 R² 和跨速度一致性。",
    ]
    report: Dict[str, object] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "bags": [str(dataset["path"]) for dataset in datasets],
        "profiles": profiles, "status": "candidate_not_applied", "warnings": warnings,
        "signals": [],
    }
    for dataset, sync in zip(datasets, synchronized_sets):
        ages = sync["pointlio_age"]
        report["signals"].append({
            "bag": str(dataset["path"]), "command_source": dataset["command_source"],
            "yaw_rate_source": sync["yaw_rate_source"], "samples": len(sync["times"]),
            "imu_acceleration_norm_median_raw": sync["imu_acceleration_norm_median_raw"],
            "pointlio_age_s": {"median": float(np.median(ages)),
                                  "p95": float(np.quantile(ages, .95)),
                                  "max": float(np.max(ages))},
        })

    actuator = None
    actuator_diagnostics = None
    actuator_candidates = []
    for profile, sync in zip(profiles, synchronized_sets):
        if profile in ("actuator", "track_actuator") \
                or profile.startswith(("track_static_", "track_prbs_")) or (
            profile == "unknown" and np.nanmax(sync["vx"]) <= 1.3
        ):
            try:
                candidate, diagnostics = fit_steering_actuator(
                    sync["times"], sync["steering_command"], sync["vx"],
                    sync["yaw_rate"], source=sync["yaw_rate_source"],
                    platform_signal=sync["identification_excitation"],
                )
                actuator_candidates.append(candidate)
                if actuator_diagnostics is None:
                    actuator_diagnostics = diagnostics
            except ValueError as error:
                warnings.append(f"转向执行器拟合跳过：{error}")
    if actuator_candidates:
        actuator = min(actuator_candidates, key=lambda item: item.dynamic_rmse)
        report["steering_actuator_candidate"] = actuator.as_dict()
    else:
        selected = args.actuator_result
        if selected is None:
            selected = find_actuator_result(args.bags[0].resolve().parent)
        if selected is not None:
            actuator = actuator_from_report(selected.resolve())
            report["actuator_result_used"] = str(selected.resolve())
        else:
            actuator = nominal_actuator(root)
            report["actuator_result_used"] = "nominal configuration; NOT IDENTIFIED"
            warnings.append("未找到 actuator 辨识结果，本次 Cf/Cr 只能作为低可信候选。")

    tire_candidates = []
    tire_diagnostics = None
    tire_details = []
    rejected_tire_details = []
    for profile, sync in zip(profiles, synchronized_sets):
        if profile.startswith(("tire_", "track_tire_")) or (
            profile == "unknown" and np.nanmax(sync["vx"]) > 1.0
        ):
            try:
                candidate, diagnostics = fit_tire_stiffness(
                    sync["times"], sync["times"], sync["steering_command"],
                    sync["vx"], sync["vy"], sync["odom_yaw_rate"], actuator,
                )
                details = {"profile": profile, **candidate.as_dict()}
                invalid = (
                    candidate.cf <= 5.05 or candidate.cr <= 5.05
                    or candidate.front_r2 <= 0.0 or candidate.rear_r2 <= 0.0
                )
                if invalid:
                    details["rejection_reason"] = (
                        "fit reached stiffness lower bound or force R2 is non-positive"
                    )
                    rejected_tire_details.append(details)
                    warnings.append(
                        f"{profile} Cf/Cr 拟合无效：参数贴下界或力拟合 R² 非正。"
                    )
                else:
                    tire_candidates.append(candidate)
                    tire_details.append(details)
                    if tire_diagnostics is None:
                        tire_diagnostics = diagnostics
            except ValueError as error:
                warnings.append(f"{profile} 轮胎拟合跳过：{error}")
    if tire_candidates:
        front_weights = np.asarray([item.samples_front for item in tire_candidates], dtype=float)
        rear_weights = np.asarray([item.samples_rear for item in tire_candidates], dtype=float)
        aggregate = {
            "Cf_N_per_rad": float(np.average([item.cf for item in tire_candidates], weights=front_weights)),
            "Cr_N_per_rad": float(np.average([item.cr for item in tire_candidates], weights=rear_weights)),
            "front_force_rmse_N": float(np.average([item.front_rmse for item in tire_candidates], weights=front_weights)),
            "rear_force_rmse_N": float(np.average([item.rear_rmse for item in tire_candidates], weights=rear_weights)),
            "front_force_r2": float(np.average([item.front_r2 for item in tire_candidates], weights=front_weights)),
            "rear_force_r2": float(np.average([item.rear_r2 for item in tire_candidates], weights=rear_weights)),
            "front_samples": int(np.sum(front_weights)), "rear_samples": int(np.sum(rear_weights)),
            "per_bag": tire_details,
        }
        report["tire_stiffness_candidate"] = aggregate
    if rejected_tire_details:
        report["rejected_tire_fits"] = rejected_tire_details

    report = finite(report)
    output.write_text(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown = output.with_suffix(".md")
    write_markdown(markdown, report)
    plot_report(output.with_suffix(".png"), synchronized_sets[0], actuator_diagnostics, tire_diagnostics)
    print(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False))
    print(f"[REPORT] {output}")
    print(f"[REPORT] {markdown}")


if __name__ == "__main__":
    main()
