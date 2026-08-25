#!/usr/bin/env python3
"""Build an isolated Stable V5 candidate with a smoother speed envelope.

Geometry, controller, vehicle, residual model and hardware safety limits remain
byte-for-byte Stable V5.  Only the raceline's node-wise local speed limits are
lowered to an envelope planned with 2.0 m/s^2 acceleration and 1.8 m/s^2
deceleration.  The runtime planner still uses Stable V5's 2.4/4.0 m/s^2 hard
limits, so it reproduces the pre-shaped envelope without weakening emergency
or publisher braking authority.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
sys.path.insert(0, str(PACKAGE / "python"))

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


CANDIDATE = "stable_v5_speed_smooth_a20_d18_candidate"
SOURCE_TRACK_NAME = "racelinev3_stable_v5_fast5_std32"
CANDIDATE_TRACK_NAME = "racelinev3_stable_v5_speed_smooth_a20_d18_candidate"

SOURCE_TRACK_DIR = PACKAGE / "data/tracks" / SOURCE_TRACK_NAME
SOURCE_TRACK = SOURCE_TRACK_DIR / "raceline.csv"
SOURCE_ZONES = SOURCE_TRACK_DIR / "speed_zones.csv"
CANDIDATE_TRACK_DIR = PACKAGE / "data/tracks" / CANDIDATE_TRACK_NAME
CANDIDATE_TRACK = CANDIDATE_TRACK_DIR / "raceline.csv"
CANDIDATE_ZONES = CANDIDATE_TRACK_DIR / "speed_zones.csv"
CANDIDATE_README = CANDIDATE_TRACK_DIR / "README.md"
CANDIDATE_MANIFEST = CANDIDATE_TRACK_DIR / "MANIFEST.json"

CONTROLLER = PACKAGE / "config/candidates/stable_v5/controller.yaml"
VEHICLE = PACKAGE / "config/candidates/stable_v5/vehicle.yaml"
LAUNCH = PACKAGE / "launch/hardware_mpcc_stable_v5.launch"
RESIDUAL = PACKAGE / "config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MPCC_NODE = PACKAGE / "src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR = PACKAGE / "scripts/automatic_relaunch_supervisor.py"
MOTION_LOGIC = PACKAGE / "python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC = PACKAGE / "python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RECORDER = ROOT / "scripts/record_stable_v5_bag.sh"
TEMPLATE_LAUNCHER = ROOT / "scripts/start_mpcc_hardware_stable_v5_u_complex_std32_candidate.sh"
CANDIDATE_LAUNCHER = ROOT / f"scripts/start_mpcc_hardware_{CANDIDATE}.sh"

ANALYSIS_DIR = ROOT / "analysis/stable_v5_speed_smooth_a20_d18_20260822"
COMPARISON_CSV = ANALYSIS_DIR / "speed_profile_comparison.csv"
COMPARISON_PLOT = ANALYSIS_DIR / "speed_profile_comparison.png"
ANALYSIS_JSON = ANALYSIS_DIR / "analysis_summary.json"
ANALYSIS_REPORT = ANALYSIS_DIR / "REPORT.md"

TARGET_ACCEL_MPS2 = 2.0
TARGET_DECEL_MPS2 = 1.8

MARKED_ZONES = {
    "T3": (17.003, 20.843),
    "T5": (38.669, 41.686),
    "T6-T7": (49.091, 55.124),
}

EXPECTED_SOURCE_HASHES = {
    SOURCE_TRACK: "88ca2acf725207a68d0ea20aa9e26d8af93dcff027afd235b9ce2045f866b82a",
    SOURCE_ZONES: "3ace10062380356525c99d02e07d61f125c971b476594bdd4c26c2da6f250132",
    CONTROLLER: "350b532ca033aba4c7fd03a63cb7170cfd34989673705767752ab95491c685cb",
    VEHICLE: "fd5f9d91bc1fe967d7429cb12f42d982e8527d0d8e559f41dbefd8f2cf24366d",
    LAUNCH: "604c80574833135f460b7057bf16be0887a5f8391151621e79475f0a96b63d45",
    RESIDUAL: "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44",
    MPCC_NODE: "344bc7003ff491ff082351a2fb6979a608b9ead2df0238a0cee6c0d1b911bde0",
    SUPERVISOR: "f9996adfb6c59f66a2ccaf7522f40825e2c74a9e3370c7f8ffc5e4fa1a22545a",
    MOTION_LOGIC: "43760fcbb6a965da5c7d8e68b7683a2a520d2129da30dcc8d4eff8db2521a8d6",
    PLACEMENT_LOGIC: "21217b67871accf841c1bef76f9e6c7f874526cfd1bd99c497706b989cb7b178",
    RECORDER: "1ebd7a7a5b4b5531058206b3a64377b5bb17ff03643d97bbb5dc294d695302ec",
    TEMPLATE_LAUNCHER: "1ae105a29e04bf4bf9e784aa2c53c9b08b49c369f8307633d1e7ce054bf1e91b",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def validate_sources() -> None:
    for path, expected in EXPECTED_SOURCE_HASHES.items():
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(
                f"Stable V5 source changed: {relative(path)}: expected={expected} actual={actual}"
            )


def planning_config(*, accel: float, decel: float) -> dict:
    controller = yaml.safe_load(CONTROLLER.read_text(encoding="utf-8"))
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))
    planning = dict(controller["speed_planning"])
    planning.update(
        wheelbase_m=vehicle["vehicle"]["wheelbase"],
        max_steer_rad=vehicle["limits"]["max_steer"],
        max_steer_rate_radps=vehicle["limits"]["max_steer_rate"],
        max_accel_mps2=accel,
        max_decel_mps2=vehicle["limits"]["max_decel"],
        profile_max_decel_mps2=decel,
        lateral_accel_limit_mps2=vehicle["limits"]["lateral_accel_limit"],
    )
    return planning


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError(f"missing CSV header: {path}")
        return list(reader.fieldnames), list(reader)


def write_rows(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def build_track() -> tuple[PeriodicTrack, PeriodicTrack]:
    source_runtime = PeriodicTrack(
        SOURCE_TRACK,
        speed_planning=planning_config(accel=2.4, decel=4.0),
    )
    target_planner = PeriodicTrack(
        SOURCE_TRACK,
        speed_planning=planning_config(
            accel=TARGET_ACCEL_MPS2,
            decel=TARGET_DECEL_MPS2,
        ),
    )

    fieldnames, rows = read_rows(SOURCE_TRACK)
    geometry_columns = (
        "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm",
        "speed_zone", "w_tr_right_m", "w_tr_left_m",
    )
    original_geometry = [[row[name] for name in geometry_columns] for row in rows]
    for index, row in enumerate(rows):
        row["speed_limit_mps"] = f"{float(target_planner.speed_nodes[index]):.9f}"
        row["vx_mps"] = f"{float(target_planner.speed_nodes[index]):.9f}"
        row["ax_mps2"] = f"{float(target_planner.accel_nodes[index]):.9f}"

    CANDIDATE_TRACK_DIR.mkdir(parents=True, exist_ok=True)
    write_rows(CANDIDATE_TRACK, fieldnames, rows)
    CANDIDATE_ZONES.write_bytes(SOURCE_ZONES.read_bytes())

    _, generated_rows = read_rows(CANDIDATE_TRACK)
    generated_geometry = [
        [row[name] for name in geometry_columns] for row in generated_rows
    ]
    if generated_geometry != original_geometry:
        raise RuntimeError("candidate changed a geometry, width or zone column")

    candidate_runtime = PeriodicTrack(
        CANDIDATE_TRACK,
        speed_planning=planning_config(accel=2.4, decel=4.0),
    )
    difference = float(
        np.max(np.abs(candidate_runtime.speed_nodes - target_planner.speed_nodes))
    )
    if difference > 2.0e-8:
        raise RuntimeError(f"runtime planner does not reproduce target envelope: {difference}")
    if float(np.max(candidate_runtime.accel_nodes)) > TARGET_ACCEL_MPS2 + 1.0e-6:
        raise RuntimeError("candidate acceleration exceeds target")
    if float(np.min(candidate_runtime.accel_nodes)) < -TARGET_DECEL_MPS2 - 1.0e-6:
        raise RuntimeError("candidate deceleration exceeds target")
    return source_runtime, candidate_runtime


def dense_metrics(track: PeriodicTrack) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    s = np.linspace(0.0, track.length, 40001, endpoint=False)
    speed = np.asarray(track.speed_prior(s), dtype=float)
    slope = np.gradient(speed, s)
    summary = {
        "ideal_lap_time_s": float(np.trapz(1.0 / np.maximum(speed, 0.2), s)),
        "speed_mps": {
            "minimum": float(np.min(speed)),
            "median": float(np.median(speed)),
            "p90": float(np.percentile(speed, 90)),
            "maximum": float(np.max(speed)),
        },
        "dv_ds_mps_per_m": {
            "minimum": float(np.min(slope)),
            "maximum": float(np.max(slope)),
        },
        "zones": {},
    }
    for name, (start, end) in MARKED_ZONES.items():
        mask = (s >= start) & (s <= end)
        summary["zones"][name] = {
            "s_range_m": [start, end],
            "speed_minimum_mps": float(np.min(speed[mask])),
            "speed_maximum_mps": float(np.max(speed[mask])),
            "steepest_decrease_dv_ds": float(np.min(slope[mask])),
            "steepest_recovery_dv_ds": float(np.max(slope[mask])),
        }
    return s, speed, slope, summary


def write_analysis(source: PeriodicTrack, candidate: PeriodicTrack) -> dict:
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    s, source_speed, source_slope, source_summary = dense_metrics(source)
    _, candidate_speed, candidate_slope, candidate_summary = dense_metrics(candidate)

    with COMPARISON_CSV.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ["s_m", "stable_v5_speed_mps", "candidate_speed_mps",
             "stable_v5_dv_ds", "candidate_dv_ds"]
        )
        for values in zip(s, source_speed, candidate_speed, source_slope, candidate_slope):
            writer.writerow([f"{float(value):.9f}" for value in values])

    fig, axes = plt.subplots(4, 1, figsize=(15, 13), constrained_layout=True)
    axes[0].plot(s, source_speed, color="#d64b40", lw=1.7, label="StableV5 current")
    axes[0].plot(s, candidate_speed, color="#0969da", lw=2.0, label="smooth a2.0/d1.8")
    axes[0].set_title("StableV5 speed envelope: current vs smooth candidate")
    axes[0].set_ylabel("planned speed [m/s]")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="upper right")
    for axis, (name, (start, end)) in zip(axes[1:], MARKED_ZONES.items()):
        mask = (s >= start) & (s <= end)
        axis.plot(s[mask], source_speed[mask], color="#d64b40", lw=2.0, label="StableV5")
        axis.plot(s[mask], candidate_speed[mask], color="#0969da", lw=2.2, label="candidate")
        axis.set_title(f"{name}: s={start:.3f}..{end:.3f} m")
        axis.set_ylabel("speed [m/s]")
        axis.grid(alpha=0.25)
    axes[-1].set_xlabel("track progress s [m]")
    fig.savefig(COMPARISON_PLOT, dpi=180)
    plt.close(fig)

    summary = {
        "candidate": CANDIDATE,
        "source_track": SOURCE_TRACK_NAME,
        "candidate_track": CANDIDATE_TRACK_NAME,
        "change_scope": "speed_limit_mps/vx_mps/ax_mps2 only",
        "geometry_identical": True,
        "target_envelope": {
            "maximum_acceleration_mps2": TARGET_ACCEL_MPS2,
            "maximum_deceleration_mps2": TARGET_DECEL_MPS2,
        },
        "runtime_safety_limits_unchanged": {
            "vehicle_max_acceleration_mps2": 2.4,
            "vehicle_max_deceleration_mps2": 4.0,
            "publisher_deceleration_mps2": 4.0,
        },
        "stable_v5": source_summary,
        "candidate_summary": candidate_summary,
    }
    ANALYSIS_JSON.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    stable = summary["stable_v5"]
    smooth = summary["candidate_summary"]
    zone_lines = []
    for name in MARKED_ZONES:
        before = stable["zones"][name]
        after = smooth["zones"][name]
        reduction = 100.0 * (
            1.0
            - abs(after["steepest_decrease_dv_ds"])
            / abs(before["steepest_decrease_dv_ds"])
        )
        zone_lines.append(
            f"| {name} | {before['steepest_decrease_dv_ds']:.3f} | "
            f"{after['steepest_decrease_dv_ds']:.3f} | {reduction:.1f}% | "
            f"{after['speed_minimum_mps']:.3f} |"
        )
    ANALYSIS_REPORT.write_text(
        f"""# Stable V5 speed-smooth a2.0/d1.8 candidate

## Result

The candidate changes only the raceline speed columns. Geometry, track width,
Stable V5 controller, vehicle model, residual model, 60 Hz pipeline and
publisher/fallback braking remain unchanged.

| Zone | Stable V5 steepest dv/ds | Candidate steepest dv/ds | Magnitude reduction | Candidate minimum speed |
|---|---:|---:|---:|---:|
{chr(10).join(zone_lines)}

Units: `dv/ds` is `(m/s)/m`; speed is `m/s`.

- Planned acceleration envelope: <= {TARGET_ACCEL_MPS2:.1f} m/s^2.
- Planned deceleration envelope: <= {TARGET_DECEL_MPS2:.1f} m/s^2.
- Stable V5 publisher/fallback braking authority: unchanged at 4.0 m/s^2.
- Ideal profile-only lap time: {stable['ideal_lap_time_s']:.2f} ->
  {smooth['ideal_lap_time_s']:.2f} s (+{smooth['ideal_lap_time_s'] - stable['ideal_lap_time_s']:.2f} s).
- The three marked-zone minimum speeds are unchanged; braking starts earlier
  and the descent/recovery is spread over more distance.

## Validation

- Exactly 320/320 geometry rows retained.
- Changed columns: `vx_mps`, `ax_mps2`, `speed_limit_mps` only.
- Python speed/periodic-track tests: 13 passed.
- C++ core smoke test: passed.
- Remote hash check and isolated acados solver preparation: passed.
- No controller was launched and no hardware command was published during preparation.

## Run and rollback

```bash
cd /home/tianbot/f1tenth_residual_controller_ws
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \\
  ./scripts/start_mpcc_hardware_{CANDIDATE}.sh mpcc
```

Rollback: stop the candidate, then run unchanged Stable V5:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \\
  ./scripts/start_mpcc_hardware_stable_v5.sh mpcc
```
""",
        encoding="utf-8",
    )
    return summary


def replace_required(text: str, old: str, new: str) -> str:
    if old not in text:
        raise RuntimeError(f"launcher template fragment missing: {old}")
    return text.replace(old, new)


def write_launcher() -> None:
    text = TEMPLATE_LAUNCHER.read_text(encoding="utf-8")
    text = replace_required(text, "stable_v5_u_complex_std32_candidate", CANDIDATE)
    text = replace_required(
        text, "racelinev6_u_complex_std32_stable_v5_candidate", CANDIDATE_TRACK_NAME
    )
    text = replace_required(
        text,
        "4a8209690e9443b59cba3c23069e6abc4ee22b634069933b7b8657fcb777c989",
        sha256(CANDIDATE_TRACK),
    )
    text = replace_required(
        text,
        "910cbe2cfdea9c65f0ebbbffa34c1fd2162f45ff8eeb80acdfb3959da450c7e5",
        sha256(CANDIDATE_ZONES),
    )
    text = text.replace("Stable V5 U-complex candidate", "Stable V5 speed-smooth candidate")
    text = text.replace("isolated U-complex track", "isolated speed-envelope track")
    text = text.replace(
        "track=$TRACK; ordinary=3.2 m/s; fast/global=5.0 m/s; profile decel=4.0 m/s^2",
        "track=$TRACK; envelope accel/decel=2.0/1.8 m/s^2; hard braking remains 4.0 m/s^2",
    )
    text = text.replace(
        "Stable V5 60 Hz release + hardware-tested V6 compound U geometry",
        "Stable V5 geometry/controller + isolated smooth speed envelope",
    )
    text = text.replace(
        "ordinary=3.2 m/s; fast/global=${RESIDUAL_MPCC_SPEED_CAP} m/s; profile/publisher decel=4.0 m/s^2",
        "global=${RESIDUAL_MPCC_SPEED_CAP} m/s; envelope accel/decel=2.0/1.8 m/s^2",
    )
    text = text.replace(
        "contour=45; heading=1.0; steering-command-rate=0.60; Stable V3 H3 residual",
        "geometry/MPCC/residual unchanged; publisher/fallback braking remains 4.0 m/s^2",
    )
    CANDIDATE_LAUNCHER.write_text(text, encoding="utf-8")
    os.chmod(CANDIDATE_LAUNCHER, 0o755)


def write_docs_and_manifest(summary: dict) -> None:
    CANDIDATE_README.write_text(
        f"""# Stable V5 speed-smooth a2.0/d1.8 candidate

This candidate keeps Stable V5 geometry, controller, vehicle, residual model,
60 Hz pipeline and 4.0 m/s^2 publisher/fallback braking unchanged.

Only `speed_limit_mps`, `vx_mps` and `ax_mps2` differ from
`{SOURCE_TRACK_NAME}`.  The node-wise envelope is generated with a maximum
planned acceleration of {TARGET_ACCEL_MPS2:.1f} m/s^2 and maximum planned
deceleration of {TARGET_DECEL_MPS2:.1f} m/s^2.  Runtime planning reproduces
that envelope while retaining the stronger Stable V5 feasibility and safety
braking limits.

Run from the controller workspace:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \\
  ./scripts/start_mpcc_hardware_{CANDIDATE}.sh mpcc
```

Rollback is immediate: stop this launcher and run the unchanged
`./scripts/start_mpcc_hardware_stable_v5.sh` (or frozen StableV3 launcher).
""",
        encoding="utf-8",
    )
    files = [
        CANDIDATE_TRACK,
        CANDIDATE_ZONES,
        CANDIDATE_README,
        CANDIDATE_LAUNCHER,
        CONTROLLER,
        VEHICLE,
        LAUNCH,
        RESIDUAL,
        MPCC_NODE,
        SUPERVISOR,
        MOTION_LOGIC,
        PLACEMENT_LOGIC,
        RECORDER,
    ]
    manifest = {
        "candidate": CANDIDATE,
        "status": "offline_validated_candidate",
        "created_date": "2026-08-22",
        "source_release": "stable_v5",
        "source_track": SOURCE_TRACK_NAME,
        "candidate_track": CANDIDATE_TRACK_NAME,
        "change_scope": ["speed_limit_mps", "vx_mps", "ax_mps2"],
        "geometry_columns_identical": True,
        "target_envelope": {
            "maximum_acceleration_mps2": TARGET_ACCEL_MPS2,
            "maximum_deceleration_mps2": TARGET_DECEL_MPS2,
        },
        "analysis": summary,
        "hashes": {relative(path): sha256(path) for path in files},
    }
    CANDIDATE_MANIFEST.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    validate_sources()
    source, candidate = build_track()
    summary = write_analysis(source, candidate)
    write_launcher()
    write_docs_and_manifest(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
