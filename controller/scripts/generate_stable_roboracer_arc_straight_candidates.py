#!/usr/bin/env python3
"""Materialize isolated RoboRacer 4.5/5.0 m/s dual-decel candidates."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
sys.path.insert(0, str(PACKAGE / "python"))

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


BASE_CONTROLLER = PACKAGE / "config/candidates/stable_roboracer/controller.yaml"
BASE_VEHICLE = PACKAGE / "config/vehicle_stable_v3_racelineV3.yaml"
BASE_TRACK = PACKAGE / "data/tracks/raceline_smooth/raceline.csv"
CANDIDATE_LAUNCH = (
    PACKAGE / "launch/hardware_mpcc_stable_roboracer_arc_straight_dual_decel_candidate.launch"
)
CANDIDATE_NODE = PACKAGE / "src/mpcc_node_auto_relaunch_dual_decel.cpp"
CORE_HEADER = PACKAGE / "include/f1tenth_dynamic_mpcc/core.hpp"
CORE_SOURCE = PACKAGE / "src/core.cpp"
PURE_PURSUIT_SOURCE = PACKAGE / "src/pure_pursuit_candidate.cpp"
CMAKE = PACKAGE / "CMakeLists.txt"

EXPECTED_BASE_HASHES = {
    BASE_CONTROLLER: "df8f558111e09db665a70bafaf4baacf238bbc37095e116e6fc992e88498bfe2",
    BASE_VEHICLE: "0188d1ea37e2f113168fc31c1056912eca8ab37966f35132c8f879910a9d54a6",
    BASE_TRACK: "1f7b00d996f67c19181094799621dcd4b684a0bf7a1e7be4f22c779ade060476",
}

FAST_ZONES = {
    "dynamic_arc_transition",
    "round_arc_transition",
    "straight_acceleration",
}
STANDARD_ZONES = {
    "standard",
    "tight_bend_1_standard",
    "tight_bend_2_standard",
}
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


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_frozen_inputs() -> None:
    for path, expected in EXPECTED_BASE_HASHES.items():
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"frozen RoboRacer input changed: {path}: {actual}")


def planning_config(controller_path: Path, vehicle_path: Path) -> dict:
    controller = yaml.safe_load(controller_path.read_text(encoding="utf-8"))
    vehicle = yaml.safe_load(vehicle_path.read_text(encoding="utf-8"))
    planning = dict(controller["speed_planning"])
    planning.update(
        wheelbase_m=vehicle["vehicle"]["wheelbase"],
        max_steer_rad=vehicle["limits"]["max_steer"],
        max_steer_rate_radps=vehicle["limits"]["max_steer_rate"],
        max_accel_mps2=vehicle["limits"]["max_accel"],
        max_decel_mps2=vehicle["limits"]["max_decel"],
        lateral_accel_limit_mps2=vehicle["limits"]["lateral_accel_limit"],
    )
    return planning


def write_controller(output: Path, cap: float) -> None:
    config = yaml.safe_load(BASE_CONTROLLER.read_text(encoding="utf-8"))
    config["speed_planning"]["max_speed_mps"] = cap
    config["publisher"]["deceleration_limit_mps2"] = 2.2
    config["publisher"]["emergency_deceleration_limit_mps2"] = 4.0
    config["pure_pursuit_candidate"][
        "emergency_deceleration_limit_mps2"
    ] = 4.0
    for key in (
        "global_speed_max_mps",
        "baseline_speed_max_mps",
        "debug_speed_max_mps",
    ):
        config["safety"][key] = cap
    output.write_text(
        "# Generated from frozen Stable RoboRacer: candidate speed ceilings and dual-rate deceleration.\n"
        + yaml.safe_dump(config, sort_keys=False),
        encoding="utf-8",
    )


def write_vehicle(output: Path, cap: float) -> None:
    config = yaml.safe_load(BASE_VEHICLE.read_text(encoding="utf-8"))
    config["limits"]["max_speed"] = max(
        float(config["limits"]["max_speed"]), cap
    )
    output.write_text(
        "# Generated from frozen Stable V3 vehicle; only max_speed may differ.\n"
        + yaml.safe_dump(config, sort_keys=False),
        encoding="utf-8",
    )


def save_rows(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_track(
    output: Path, controller_path: Path, vehicle_path: Path, cap: float
) -> dict:
    with BASE_TRACK.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError("raceline header is missing")
        fieldnames = list(reader.fieldnames)
        rows = list(reader)

    for row in rows:
        zone = row["speed_zone"]
        if zone in FAST_ZONES:
            row["speed_limit_mps"] = f"{cap:.9f}"
        elif zone in STANDARD_ZONES:
            row["speed_limit_mps"] = "3.000000000"
        else:
            raise RuntimeError(f"unclassified Stable RoboRacer speed zone: {zone}")

    # The spline segment following a node blends both endpoint limits. Keeping
    # the first fast node at 3 m/s prevents the preceding standard segment from
    # being raised while allowing acceleration immediately after the boundary.
    fast_entry_rows = []
    for index, row in enumerate(rows):
        previous = rows[(index - 1) % len(rows)]["speed_zone"]
        if row["speed_zone"] in FAST_ZONES and previous in STANDARD_ZONES:
            row["speed_limit_mps"] = "3.000000000"
            fast_entry_rows.append(index)

    save_rows(output, fieldnames, rows)
    track = PeriodicTrack(
        output, speed_planning=planning_config(controller_path, vehicle_path)
    )
    for index, row in enumerate(rows):
        row["vx_mps"] = f"{float(track.speed_nodes[index]):.9f}"
        row["ax_mps2"] = f"{float(track.accel_nodes[index]):.9f}"
    save_rows(output, fieldnames, rows)

    reproduced = PeriodicTrack(
        output, speed_planning=planning_config(controller_path, vehicle_path)
    )
    zone_summary = {}
    for zone in sorted(FAST_ZONES | STANDARD_ZONES):
        indexes = [i for i, row in enumerate(rows) if row["speed_zone"] == zone]
        speeds = [float(reproduced.speed_nodes[i]) for i in indexes]
        limits = [float(rows[i]["speed_limit_mps"]) for i in indexes]
        zone_summary[zone] = {
            "nodes": len(indexes),
            "local_limit_mps": [min(limits), max(limits)],
            "planned_speed_mps": [min(speeds), max(speeds)],
        }
        expected = cap if zone in FAST_ZONES else 3.0
        if max(speeds) > expected + 1e-8:
            raise RuntimeError(f"{zone} planned speed exceeds {expected}")
        if zone in STANDARD_ZONES and any(abs(value - 3.0) > 1e-9 for value in limits):
            raise RuntimeError(f"{zone} no longer has a strict 3.0 m/s limit")

    if max(reproduced.speed_nodes) < cap - 1e-6:
        raise RuntimeError(f"candidate never reaches requested {cap:.1f} m/s cap")
    return {
        "length_m": float(reproduced.length),
        "maximum_planned_speed_mps": float(max(reproduced.speed_nodes)),
        "fast_entry_row_indexes": fast_entry_rows,
        "zones": zone_summary,
    }


def write_speed_zones(path: Path, track_path: Path) -> None:
    with track_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    runs = []
    begin = 0
    for index in range(1, len(rows) + 1):
        if index == len(rows) or rows[index]["speed_zone"] != rows[begin]["speed_zone"]:
            runs.append((begin, index - 1))
            begin = index
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ["name", "zone", "start_s_m", "end_s_m", "wraps_seam", "maximum_speed_mps", "speed_scale"]
        )
        for number, (start, end) in enumerate(runs, 1):
            zone = rows[start]["speed_zone"]
            writer.writerow(
                [
                    f"{zone}_{number:02d}",
                    zone,
                    rows[start]["s_m"],
                    rows[end]["s_m"],
                    "false",
                    f"{max(float(row['speed_limit_mps']) for row in rows[start:end + 1]):.9f}",
                    "1.000000000",
                ]
            )


def materialize(key: str, spec: dict) -> None:
    cap = float(spec["cap"])
    candidate_dir = PACKAGE / "config/candidates" / spec["candidate"]
    track_dir = PACKAGE / "data/tracks" / spec["track"]
    candidate_dir.mkdir(parents=True, exist_ok=True)
    track_dir.mkdir(parents=True, exist_ok=True)

    controller = candidate_dir / "controller.yaml"
    vehicle = candidate_dir / "vehicle.yaml"
    track = track_dir / "raceline.csv"
    zones = track_dir / "speed_zones.csv"
    write_controller(controller, cap)
    write_vehicle(vehicle, cap)
    track_summary = write_track(track, controller, vehicle, cap)
    write_speed_zones(zones, track)

    outputs = (
        controller,
        vehicle,
        track,
        zones,
        CANDIDATE_LAUNCH,
        CANDIDATE_NODE,
        CORE_HEADER,
        CORE_SOURCE,
        PURE_PURSUIT_SOURCE,
        CMAKE,
    )
    manifest = {
        "candidate": spec["candidate"],
        "status": "isolated_candidate_not_stable",
        "source_release": "stable_roboracer",
        "policy": {
            "runtime_global_cap_mps": cap,
            "fast_arc_round_arc_straight_cap_mps": cap,
            "standard_and_tight_bend_cap_mps": 3.0,
            "pure_pursuit_candidate_cap_mps": 4.0,
            "normal_command_deceleration_mps2": 2.2,
            "safety_command_deceleration_mps2": 4.0,
            "safety_deceleration_paths": [
                "predictive_risk",
                "pure_pursuit_candidate",
                "out_of_bounds",
                "checkpoint_recovery",
                "solver_or_input_failure",
            ],
            "fast_zones": sorted(FAST_ZONES),
            "standard_zones": sorted(STANDARD_ZONES),
            "all_other_roboracer_behavior_frozen": True,
        },
        "track": track_summary,
        "base_hashes": {
            str(path.relative_to(ROOT)): value
            for path, value in EXPECTED_BASE_HASHES.items()
        },
        "output_hashes": {
            str(path.relative_to(ROOT)): sha256(path) for path in outputs
        },
    }
    manifest_path = candidate_dir / "MANIFEST.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"variant": key, **manifest}, ensure_ascii=False, indent=2))


def main() -> None:
    verify_frozen_inputs()
    for key, spec in VARIANTS.items():
        materialize(key, spec)


if __name__ == "__main__":
    main()
