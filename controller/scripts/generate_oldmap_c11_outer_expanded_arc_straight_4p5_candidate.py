#!/usr/bin/env python3
"""Generate the additive old-map C11-expanded 4.5 m/s speed candidate."""

from __future__ import annotations

import csv
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
sys.path.insert(0, str(PACKAGE / "python"))

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


SOURCE_TRACK = PACKAGE / "data/tracks/raceline_smooth_c11_outer_boundary_expanded_candidate/raceline.csv"
OUTPUT_DIR = PACKAGE / "data/tracks/raceline_smooth_c11_outer_boundary_expanded_arc_straight_4p5_candidate"
OUTPUT_TRACK = OUTPUT_DIR / "raceline.csv"
OUTPUT_ZONES = OUTPUT_DIR / "speed_zones.csv"
CONTROLLER = PACKAGE / "config/candidates/stable_roboracer_c11_outer_expanded_arc_straight_4p5/controller.yaml"
VEHICLE = PACKAGE / "config/vehicle_stable_v3_racelineV3.yaml"

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


def planning_config() -> dict:
    controller = yaml.safe_load(CONTROLLER.read_text(encoding="utf-8"))
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))
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


def write_rows(fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with OUTPUT_TRACK.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    with SOURCE_TRACK.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError("source raceline has no header")
        fieldnames = list(reader.fieldnames)
        rows = list(reader)

    for row in rows:
        zone = row["speed_zone"]
        if zone in FAST_ZONES:
            row["speed_limit_mps"] = "4.500000000"
        elif zone in STANDARD_ZONES:
            row["speed_limit_mps"] = "3.000000000"
        else:
            raise RuntimeError(f"unclassified speed zone: {zone}")

    # Do not raise the spline segment immediately before a fast-zone boundary.
    for index, row in enumerate(rows):
        previous_zone = rows[(index - 1) % len(rows)]["speed_zone"]
        if row["speed_zone"] in FAST_ZONES and previous_zone in STANDARD_ZONES:
            row["speed_limit_mps"] = "3.000000000"

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    write_rows(fieldnames, rows)
    track = PeriodicTrack(OUTPUT_TRACK, speed_planning=planning_config())
    for index, row in enumerate(rows):
        row["vx_mps"] = f"{float(track.speed_nodes[index]):.9f}"
        row["ax_mps2"] = f"{float(track.accel_nodes[index]):.9f}"
    write_rows(fieldnames, rows)

    runs: list[tuple[int, int]] = []
    begin = 0
    for index in range(1, len(rows) + 1):
        if index == len(rows) or rows[index]["speed_zone"] != rows[begin]["speed_zone"]:
            runs.append((begin, index - 1))
            begin = index
    with OUTPUT_ZONES.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["name", "zone", "start_s_m", "end_s_m", "wraps_seam", "maximum_speed_mps", "speed_scale"])
        for number, (start, end) in enumerate(runs, 1):
            zone = rows[start]["speed_zone"]
            writer.writerow([
                f"{zone}_{number:02d}", zone, rows[start]["s_m"], rows[end]["s_m"],
                "false", f"{max(float(row['speed_limit_mps']) for row in rows[start:end + 1]):.9f}", "1.000000000",
            ])

    reproduced = PeriodicTrack(OUTPUT_TRACK, speed_planning=planning_config())
    if max(reproduced.speed_nodes) < 4.5 - 1.0e-6:
        raise RuntimeError("candidate does not reach 4.5 m/s")
    for row in rows:
        limit = float(row["speed_limit_mps"])
        if row["speed_zone"] in STANDARD_ZONES and abs(limit - 3.0) > 1.0e-9:
            raise RuntimeError("standard/tight-bend limit changed")
    print(f"[OK] generated {OUTPUT_TRACK}")
    print(f"[OK] points={len(rows)} length={reproduced.length:.3f} m max_speed={max(reproduced.speed_nodes):.3f} m/s")


if __name__ == "__main__":
    main()
