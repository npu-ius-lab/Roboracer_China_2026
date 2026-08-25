#!/usr/bin/env python3
"""Closed-loop comparison of racelines with the frozen stableV3 MPCC model.

The stableV3 YAML/model/raceline files are read-only inputs. Runtime launch
overrides are applied to in-memory dictionaries only.
"""

from __future__ import annotations

import copy
import json
import math
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from f1tenth_dynamic_mpcc.acados_solver import AcadosDynamicMPCC
from f1tenth_dynamic_mpcc.config import apply_runtime_speed_cap, load_yaml
from f1tenth_dynamic_mpcc.steering_actuator_model import SteeringActuatorModel
from f1tenth_dynamic_mpcc.steering_state_observer import SteeringStateObserver
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters
from f1tenth_dynamic_mpcc.warm_start import WarmStart


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
OUTPUT = Path(__file__).resolve().parent
GENERATED = Path.home() / ".cache/f1tenth_residual_mpcc/acados_stable_v3_sim_exact"
CONTROLLER_PATH = PACKAGE / "config/controller_stable_v3_racelineV3.yaml"
VEHICLE_PATH = PACKAGE / "config/vehicle_stable_v3_racelineV3.yaml"
RESIDUAL_PATH = (
    PACKAGE / "config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
)

TRACKS = {
    "stableV3 / V3": PACKAGE / "data/tracks/racelinev3/raceline.csv",
    "stableV3 / original V5": (
        PACKAGE / "data/tracks/racelinev5_candidate_min_curvature/raceline.csv"
    ),
    "stableV3 / V5 OIO": (
        PACKAGE / "data/tracks/racelinev5_candidate_outer_inner_outer/raceline.csv"
    ),
}
SCENARIOS = {
    "nominal": (0.0, 0.0),
    "left_offset": (0.10, 0.08),
    "right_offset": (-0.10, -0.08),
}
ZONES = {
    "C1": (14.0, 24.5),
    "C2": (26.0, 32.5),
    "C3": (34.0, 41.5),
    "C4": (43.0, 52.0),
}


def make_controller() -> dict:
    controller = load_yaml(CONTROLLER_PATH)
    controller["mode"] = "race"
    controller["residual_dynamics"]["enabled"] = True
    controller["residual_dynamics"]["model_path"] = str(RESIDUAL_PATH)
    controller["residual_dynamics"]["required_feature_set"] = "markov_v1"
    controller["cost"]["contour"] = 45.0
    controller["cost"]["heading_race"] = 1.0
    controller["cost"]["steering_command_rate"] = 0.60
    controller["speed_planning"]["profile_max_decel_mps2"] = 3.0
    apply_runtime_speed_cap(controller, 4.0)
    return controller


def make_track(path: Path, controller: dict, vehicle: VehicleParameters) -> PeriodicTrack:
    speed = dict(controller["speed_planning"])
    speed.update(
        wheelbase_m=vehicle.wheelbase,
        max_steer_rad=vehicle.max_steer,
        max_steer_rate_radps=vehicle.max_steer_rate,
        max_accel_mps2=vehicle.max_accel,
        max_decel_mps2=vehicle.max_decel,
        lateral_accel_limit_mps2=vehicle.lateral_accel_limit,
    )
    return PeriodicTrack(path, speed_planning=speed)


def reset_solver(solver: AcadosDynamicMPCC, track: PeriodicTrack) -> None:
    solver.track = track
    solver.warm = WarmStart(solver.N, solver.NX, solver.NU)
    solver.previous_control = np.zeros(solver.NU)
    solver._last_runtime_bounds = None


def run_case(
    solver: AcadosDynamicMPCC,
    track: PeriodicTrack,
    vehicle: VehicleParameters,
    vehicle_cfg: dict,
    lateral_offset: float,
    heading_offset: float,
    runtime_speed_cap_mps: float = 4.0,
    laps: float = 1.0,
    maximum_duration_s: float = 65.0,
) -> tuple[dict, dict[str, np.ndarray]]:
    reset_solver(solver, track)
    model = DynamicBicycleModel(vehicle)
    actuator = SteeringActuatorModel.from_config(vehicle_cfg)
    observer = SteeringStateObserver.from_config(vehicle_cfg, actuator, vehicle)
    dt = 1.0 / 40.0
    start_s = 5.0
    target_s = start_s + laps * track.length
    x0, y0 = track.position(start_s)
    yaw0 = float(track.tangent(start_s))
    x0 -= math.sin(yaw0) * lateral_offset
    y0 += math.cos(yaw0) * lateral_offset
    plant = np.asarray(
        [x0, y0, yaw0 + heading_offset, 0.60, 0.0, 0.0, 0.0, 0.0, start_s],
        dtype=float,
    )
    failures = 0
    last_control = np.asarray([0.60, 0.0, 0.60], dtype=float)
    rows: dict[str, list] = {
        key: []
        for key in (
            "time", "x", "y", "s", "speed", "e_contour", "e_heading",
            "margin", "delta", "delta_cmd", "delta_rate", "alpha_f",
            "alpha_r", "solve_ms", "solver_success", "speed_prior",
            "runtime_speed_cap",
        )
    }
    for tick in range(int(maximum_duration_s / dt)):
        timestamp = tick * dt
        observed = observer.update(timestamp, plant[3], plant[4], plant[5])
        projection = track.project(plant[0], plant[1], plant[8])
        controller_state = np.r_[
            plant[:6], observed.delta_hat, plant[7], projection.s
        ]
        # Match mpcc_node.cpp: the runtime MPCC cap is never allowed to exceed
        # the current raceline speed envelope. The dynamic feasibility guard
        # inside solve() can keep the input command temporarily above this cap
        # while the plant decelerates.
        runtime_speed_cap = min(
            runtime_speed_cap_mps, float(track.speed_prior(projection.s))
        )
        result = solver.solve(
            controller_state,
            speed_cap=runtime_speed_cap,
            steering_rate_cap=3.4,
        )
        if result.success:
            control = result.controls[0].copy()
        else:
            failures += 1
            control = np.asarray(
                [max(last_control[0] - vehicle.max_decel * dt, 0.0),
                 last_control[1], 0.0],
                dtype=float,
            )
        observer.push_command(timestamp, plant[7])
        plant = model.step(plant, control, dt)
        last_control = control.copy()
        projection = track.project(plant[0], plant[1], plant[8])
        plant[8] = projection.s
        alpha_f, alpha_r = model.slip_angles(plant)
        margin = min(
            float(track.width_left(projection.s))
            - vehicle.body_width / 2.0
            - projection.e_contour,
            float(track.width_right(projection.s))
            - vehicle.body_width / 2.0
            + projection.e_contour,
        )
        values = {
            "time": timestamp,
            "x": plant[0],
            "y": plant[1],
            "s": projection.s,
            "speed": plant[3],
            "e_contour": projection.e_contour,
            "e_heading": float(wrap_angle(plant[2] - projection.psi_ref)),
            "margin": margin,
            "delta": plant[6],
            "delta_cmd": plant[7],
            "delta_rate": control[1],
            "alpha_f": alpha_f,
            "alpha_r": alpha_r,
            "solve_ms": result.solve_time_s * 1000.0,
            "solver_success": float(result.success),
            "speed_prior": float(track.speed_prior(projection.s)),
            "runtime_speed_cap": runtime_speed_cap,
        }
        for key, value in values.items():
            rows[key].append(value)
        if projection.s >= target_s:
            break
    arrays = {key: np.asarray(value) for key, value in rows.items()}
    completed = bool(arrays["s"][-1] >= target_s)
    zone_metrics = {}
    s_wrapped = np.mod(arrays["s"], track.length)
    for zone, (lo, hi) in ZONES.items():
        mask = (s_wrapped >= lo) & (s_wrapped < hi)
        if np.any(mask):
            zone_metrics[zone] = {
                "mean_speed_mps": float(np.mean(arrays["speed"][mask])),
                "minimum_speed_mps": float(np.min(arrays["speed"][mask])),
                "rms_contour_error_m": float(
                    np.sqrt(np.mean(arrays["e_contour"][mask] ** 2))
                ),
                "maximum_abs_contour_error_m": float(
                    np.max(np.abs(arrays["e_contour"][mask]))
                ),
                "minimum_physical_margin_m": float(np.min(arrays["margin"][mask])),
            }
    metrics = {
        "completed_lap": completed,
        "lap_time_s": float(arrays["time"][-1]) if completed else None,
        "progress_laps": float((arrays["s"][-1] - start_s) / track.length),
        "mean_speed_mps": float(np.mean(arrays["speed"])),
        "maximum_speed_mps": float(np.max(arrays["speed"])),
        "minimum_speed_mps": float(np.min(arrays["speed"])),
        "rms_contour_error_m": float(np.sqrt(np.mean(arrays["e_contour"] ** 2))),
        "maximum_abs_contour_error_m": float(np.max(np.abs(arrays["e_contour"]))),
        "rms_heading_error_rad": float(np.sqrt(np.mean(arrays["e_heading"] ** 2))),
        "maximum_abs_heading_error_rad": float(np.max(np.abs(arrays["e_heading"]))),
        "minimum_physical_margin_m": float(np.min(arrays["margin"])),
        "maximum_abs_steering_rad": float(np.max(np.abs(arrays["delta"]))),
        "maximum_abs_steering_command_rate_radps": float(
            np.max(np.abs(arrays["delta_rate"]))
        ),
        "maximum_abs_front_slip_rad": float(np.max(np.abs(arrays["alpha_f"]))),
        "maximum_abs_rear_slip_rad": float(np.max(np.abs(arrays["alpha_r"]))),
        "solver_failure_count": int(failures),
        "solver_success_rate": float(np.mean(arrays["solver_success"])),
        "solver_time_mean_ms": float(np.mean(arrays["solve_ms"])),
        "solver_time_p95_ms": float(np.quantile(arrays["solve_ms"], 0.95)),
        "zone_metrics": zone_metrics,
    }
    return metrics, arrays


def plot_results(tracks: dict[str, PeriodicTrack], runs: dict) -> None:
    colors = {
        "stableV3 / V3": "#e41a1c",
        "stableV3 / original V5": "#888888",
        "stableV3 / V5 OIO": "#009e73",
    }
    fig, axes = plt.subplots(1, 3, figsize=(19, 6))
    for name, track in tracks.items():
        s = np.linspace(0.0, track.length, 1800, endpoint=False)
        xr, yr = track.position(s)
        psi = track.tangent(s)
        left = track.width_left(s)
        right = track.width_right(s)
        axes[0].plot(xr - np.sin(psi) * right, yr + np.cos(psi) * right,
                     color="#2c7fb8", lw=0.5, alpha=0.25)
        axes[0].plot(xr + np.sin(psi) * left, yr - np.cos(psi) * left,
                     color="#d7301f", lw=0.5, alpha=0.25)
        row = runs[name]["nominal"]["arrays"]
        axes[0].plot(row["x"], row["y"], color=colors[name], lw=1.7, label=name)
        progress = np.mod(row["s"], track.length) / track.length
        axes[1].plot(progress, row["speed"], color=colors[name], lw=1.3, label=name)
        axes[2].plot(progress, row["e_contour"], color=colors[name], lw=1.2, label=name)
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_title("Nominal closed-loop trajectories")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[1].set_title("Speed over one lap")
    axes[1].set_xlabel("lap progress")
    axes[1].set_ylabel("speed [m/s]")
    axes[2].set_title("Contour error over one lap")
    axes[2].set_xlabel("lap progress")
    axes[2].set_ylabel("contour error [m]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    fig.suptitle("Frozen stableV3 MPCC closed-loop raceline comparison")
    fig.tight_layout()
    fig.savefig(OUTPUT / "stable_v3_raceline_closed_loop_comparison.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    names = list(TRACKS)
    x = np.arange(len(names))
    width = 0.24
    for scenario_index, scenario in enumerate(SCENARIOS):
        shift = (scenario_index - 1) * width
        rms = [runs[name][scenario]["metrics"]["rms_contour_error_m"] for name in names]
        margins = [runs[name][scenario]["metrics"]["minimum_physical_margin_m"] for name in names]
        times = [runs[name][scenario]["metrics"]["lap_time_s"] or 65.0 for name in names]
        axes[0].bar(x + shift, rms, width, label=scenario)
        axes[1].bar(x + shift, margins, width, label=scenario)
        axes[2].bar(x + shift, times, width, label=scenario)
    labels = ["V3", "V5", "V5 OIO"]
    for axis, title, ylabel in zip(
        axes,
        ("RMS contour error", "Minimum physical margin", "Lap time"),
        ("m", "m", "s"),
    ):
        axis.set_xticks(x)
        axis.set_xticklabels(labels)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.25)
        axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUTPUT / "stable_v3_raceline_perturbation_bars.png", dpi=180)
    plt.close(fig)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    controller = make_controller()
    vehicle_cfg = load_yaml(VEHICLE_PATH)
    vehicle = VehicleParameters.from_yaml(VEHICLE_PATH, controller)
    tracks = {
        name: make_track(path, controller, vehicle) for name, path in TRACKS.items()
    }
    first_track = next(iter(tracks.values()))
    solver = AcadosDynamicMPCC(
        first_track, vehicle, controller, GENERATED, formulation="mpcc", build=False
    )
    runs = {}
    serializable = {
        "status": "offline_closed_loop_simulation_not_hardware_approval",
        "stable_v3_runtime_overrides": {
            "speed_cap_mps": 4.0,
            "cost_contour": 45.0,
            "cost_heading_race": 1.0,
            "cost_steering_command_rate": 0.60,
            "profile_max_decel_mps2": 3.0,
            "steering_rate_cap_radps": 3.4,
            "residual": "stable_v3_racelinev3_h3_scale030 / markov_v1",
        },
        "scenarios": {},
    }
    for track_name, track in tracks.items():
        runs[track_name] = {}
        serializable["scenarios"][track_name] = {}
        for scenario, (lateral, heading) in SCENARIOS.items():
            metrics, arrays = run_case(
                solver, track, vehicle, vehicle_cfg, lateral, heading
            )
            runs[track_name][scenario] = {"metrics": metrics, "arrays": arrays}
            serializable["scenarios"][track_name][scenario] = metrics
            np.savez_compressed(
                OUTPUT / f"{track_name.replace(' / ', '_').replace(' ', '_')}_{scenario}.npz",
                **arrays,
            )
            print(track_name, scenario, json.dumps(metrics, sort_keys=True))
    (OUTPUT / "comparison_metrics.json").write_text(
        json.dumps(serializable, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    plot_results(tracks, runs)

    nominal = {
        name: runs[name]["nominal"]["metrics"] for name in tracks
    }
    report = [
        "# stableV3 MPCC raceline closed-loop simulation",
        "",
        "Status: offline simulation only; stableV3 files were not modified.",
        "",
        "| Raceline | Lap | lap time (s) | RMS contour (m) | max contour (m) | min physical margin (m) | solver success |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, metrics in nominal.items():
        report.append(
            f"| {name} | {metrics['completed_lap']} | "
            f"{metrics['lap_time_s'] if metrics['lap_time_s'] is not None else float('nan'):.3f} | "
            f"{metrics['rms_contour_error_m']:.4f} | "
            f"{metrics['maximum_abs_contour_error_m']:.4f} | "
            f"{metrics['minimum_physical_margin_m']:.4f} | "
            f"{100.0 * metrics['solver_success_rate']:.2f}% |"
        )
    report.extend([
        "",
        "The plant is the identified stableV3 nominal dynamic bicycle model; the MPCC",
        "prediction model includes the frozen H3/0.30 residual. Localization delay, tire",
        "surface variation, command transport delay and collisions are not represented, so",
        "this comparison is a geometry/controller screen rather than hardware approval.",
        "",
    ])
    (OUTPUT / "SIMULATION_REPORT.md").write_text(
        "\n".join(report), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
