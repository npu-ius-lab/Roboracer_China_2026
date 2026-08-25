#!/usr/bin/env python3
"""Closed-loop screen for the isolated RoboRacer 4.5/5.0 candidates."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
sys.path.insert(0, str(PACKAGE / "python"))

from f1tenth_dynamic_mpcc.acados_solver import AcadosDynamicMPCC
from f1tenth_dynamic_mpcc.config import apply_runtime_speed_cap, load_yaml
from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters


COMPARISON = ROOT / "analysis/stable_v3_raceline_closed_loop_sim_20260821/run_comparison.py"
spec = importlib.util.spec_from_file_location("stable_v3_comparison", COMPARISON)
comparison = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(comparison)

VARIANTS = {
    "4p5": {
        "cap": 4.5,
        "candidate": "stable_roboracer_arc_straight_4p5_candidate",
        "track": "raceline_smooth_roboracer_arc_straight_4p5_candidate",
    },
    "5p0": {
        "cap": 5.0,
        "candidate": "stable_roboracer_arc_straight_5p0_candidate",
        "track": "raceline_smooth_roboracer_arc_straight_5p0_candidate",
    },
}
SCENARIOS = {
    "nominal": (0.0, 0.0),
    "left_offset": (0.10, 0.08),
    "right_offset": (-0.10, -0.08),
}
RESIDUAL = PACKAGE / "config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"


def speed_zone_metrics(track_path: Path, track, arrays: dict[str, np.ndarray]) -> dict:
    with track_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    node_s = np.asarray([float(row["s_m"]) for row in rows])
    node_zone = np.asarray([row["speed_zone"] for row in rows], dtype=object)
    indexes = np.searchsorted(node_s, np.mod(arrays["s"], track.length), side="right") - 1
    indexes = np.maximum(indexes, 0)
    sample_zone = node_zone[indexes]
    output = {}
    for zone in sorted(set(node_zone)):
        mask = sample_zone == zone
        output[zone] = {
            "maximum_actual_speed_mps": float(np.max(arrays["speed"][mask])),
            "mean_actual_speed_mps": float(np.mean(arrays["speed"][mask])),
            "maximum_runtime_cap_mps": float(np.max(arrays["runtime_speed_cap"][mask])),
            "rms_contour_error_m": float(np.sqrt(np.mean(arrays["e_contour"][mask] ** 2))),
            "minimum_physical_margin_m": float(np.min(arrays["margin"][mask])),
        }
    return output


def run_variant(key: str, values: dict) -> dict:
    cap = float(values["cap"])
    candidate = values["candidate"]
    controller_path = PACKAGE / "config/candidates" / candidate / "controller.yaml"
    vehicle_path = PACKAGE / "config/candidates" / candidate / "vehicle.yaml"
    track_path = PACKAGE / "data/tracks" / values["track"] / "raceline.csv"
    generated = Path.home() / ".cache/f1tenth_residual_mpcc" / f"acados_{candidate}"

    controller = load_yaml(controller_path)
    controller["mode"] = "race"
    controller["residual_dynamics"]["enabled"] = True
    controller["residual_dynamics"]["model_path"] = str(RESIDUAL)
    controller["residual_dynamics"]["required_feature_set"] = "markov_v1"
    controller["cost"]["contour"] = 45.0
    controller["cost"]["heading_race"] = 1.0
    controller["cost"]["steering_command_rate"] = 0.60
    controller["speed_planning"]["profile_max_decel_mps2"] = 3.0
    apply_runtime_speed_cap(controller, cap)

    vehicle_cfg = load_yaml(vehicle_path)
    vehicle = VehicleParameters.from_yaml(vehicle_path, controller)
    track = comparison.make_track(track_path, controller, vehicle)
    solver = AcadosDynamicMPCC(track, vehicle, controller, generated, formulation="mpcc", build=False)

    report = {"cap_mps": cap, "candidate": candidate, "scenarios": {}}
    for name, (lateral, heading) in SCENARIOS.items():
        metrics, arrays = comparison.run_case(
            solver,
            track,
            vehicle,
            vehicle_cfg,
            lateral,
            heading,
            runtime_speed_cap_mps=cap,
        )
        metrics["speed_zone_metrics"] = speed_zone_metrics(track_path, track, arrays)
        report["scenarios"][name] = metrics
        print(key, name, json.dumps(metrics, sort_keys=True))
    return report


def main() -> None:
    report = {
        "status": "offline_closed_loop_screen_not_hardware_approval",
        "source_release": "stable_roboracer",
        "variants": {key: run_variant(key, values) for key, values in VARIANTS.items()},
    }
    (HERE / "closed_loop_metrics.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
