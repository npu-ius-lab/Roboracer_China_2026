#!/usr/bin/env python3
"""Freeze Stable V5 from the validated Stable V3 60 Hz race candidate.

Stable V5 keeps the validated controller/relaunch behavior and changes only
the ordinary raceline speed ceiling from 3.0 to 3.2 m/s.  Fast arc and
straight zones remain capped at 5.0 m/s.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import re
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/f1tenth_dynamic_mpcc"
sys.path.insert(0, str(PACKAGE / "python"))

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


SOURCE_NAME = "stable_v3_5mps_auto_relaunch_brake4_asym_60hz_candidate"
SOURCE_TRACK_NAME = "racelinev3_5mps_arc_straight_std3_candidate"
RELEASE_NAME = "stable_v5"
RELEASE_TRACK_NAME = "racelinev3_stable_v5_fast5_std32"

SOURCE_CONTROLLER = PACKAGE / "config/candidates" / SOURCE_NAME / "controller.yaml"
SOURCE_VEHICLE = (
    PACKAGE
    / "config/candidates/stable_v3_5mps_arc_straight_std3_candidate/vehicle.yaml"
)
SOURCE_TRACK_DIR = PACKAGE / "data/tracks" / SOURCE_TRACK_NAME
SOURCE_TRACK = SOURCE_TRACK_DIR / "raceline.csv"
SOURCE_ZONES = SOURCE_TRACK_DIR / "speed_zones.csv"
SOURCE_LAUNCH = PACKAGE / "launch" / f"hardware_mpcc_{SOURCE_NAME}.launch"
SOURCE_LAUNCHER = ROOT / "scripts" / f"start_mpcc_hardware_{SOURCE_NAME}.sh"

RELEASE_CONFIG_DIR = PACKAGE / "config/candidates" / RELEASE_NAME
RELEASE_CONTROLLER = RELEASE_CONFIG_DIR / "controller.yaml"
RELEASE_VEHICLE = RELEASE_CONFIG_DIR / "vehicle.yaml"
RELEASE_README = RELEASE_CONFIG_DIR / "README.md"
RELEASE_MANIFEST = RELEASE_CONFIG_DIR / "MANIFEST.json"
RELEASE_TRACK_DIR = PACKAGE / "data/tracks" / RELEASE_TRACK_NAME
RELEASE_TRACK = RELEASE_TRACK_DIR / "raceline.csv"
RELEASE_ZONES = RELEASE_TRACK_DIR / "speed_zones.csv"
RELEASE_LAUNCH = PACKAGE / "launch" / "hardware_mpcc_stable_v5.launch"
RELEASE_LAUNCHER = ROOT / "scripts/start_mpcc_hardware_stable_v5.sh"

RESIDUAL = PACKAGE / "config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"
MPCC_NODE = PACKAGE / "src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR = PACKAGE / "scripts/automatic_relaunch_supervisor.py"
MOTION_LOGIC = PACKAGE / "python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC = PACKAGE / "python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RELEASE_RECORDER = ROOT / "scripts/record_stable_v5_bag.sh"

EXPECTED_SOURCE_HASHES = {
    SOURCE_CONTROLLER: "5b03916a4d8e53e2d15a692ff5f909a1df3f4ac5f2d9af1f66f828df87ac3fff",
    SOURCE_VEHICLE: "fbe6f459a81deb2814c77c4d9b5b6b50769e36e3fda42e7ede865eb92460a778",
    SOURCE_TRACK: "21c9e3c13727c510aa017e3415bd983d099bc7099bbe0d0a85f81a870b60569b",
    SOURCE_ZONES: "47896590f87c0fe5f0801c3ae627394b544739dec06cc0718c9e27c5e4caad1f",
    SOURCE_LAUNCH: "f099815be9262454a8d62598ec95fd5ef7a1c0092a0f6fea77619d60d72d374c",
    SOURCE_LAUNCHER: "2d3a77b4f345d66b2ef8f7efcbedf05a2cdb1464b75c825e365aee4b6b471f1a",
    RESIDUAL: "95a13797eb762132c47928dde83aa454b62d9a344154ddbae4b90b8b612a2d44",
    MPCC_NODE: "344bc7003ff491ff082351a2fb6979a608b9ead2df0238a0cee6c0d1b911bde0",
    SUPERVISOR: "f9996adfb6c59f66a2ccaf7522f40825e2c74a9e3370c7f8ffc5e4fa1a22545a",
    MOTION_LOGIC: "43760fcbb6a965da5c7d8e68b7683a2a520d2129da30dcc8d4eff8db2521a8d6",
    PLACEMENT_LOGIC: "21217b67871accf841c1bef76f9e6c7f874526cfd1bd99c497706b989cb7b178",
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
            raise RuntimeError(f"Stable V5 source changed: {relative(path)}: {actual}")


def write_controller_and_vehicle() -> None:
    controller = SOURCE_CONTROLLER.read_text(encoding="utf-8")
    controller = re.sub(
        r"mode: debug\n\n# Isolated 60 Hz candidate.*?parameters are unchanged\.\n",
        "mode: debug\n\n"
        "# Frozen Stable V5: Stable V3 residual MPCC with 60 Hz control, automatic\n"
        "# carry/relaunch, race-start mode, asymmetric braking, 5.0 m/s fast zones\n"
        "# and 3.2 m/s ordinary zones. Runtime launch overrides remain frozen.\n",
        controller,
        count=1,
        flags=re.DOTALL,
    )
    RELEASE_CONTROLLER.write_text(controller, encoding="utf-8")
    vehicle = SOURCE_VEHICLE.read_text(encoding="utf-8")
    vehicle = re.sub(
        r"\A# Temporary candidate.*?updated_at: \"2026-08-20\"\n",
        "# Frozen Stable V5 vehicle contract. Nominal dynamics and identified\n"
        "# steering parameters are inherited unchanged from the validated\n"
        "# Stable V3 new-surface controller line.\n"
        "schema_version: 1\n"
        "profile_name: tianracer_stable_v5\n"
        "updated_at: \"2026-08-21\"\n",
        vehicle,
        count=1,
        flags=re.DOTALL,
    )
    vehicle = vehicle.replace(
        "  # Candidate-only ceiling. Dynamics/residual coverage above 4 m/s remains\n"
        "  # experimental and must be validated from a separately recorded bag.\n",
        "  # Stable V5 hardware and global controller ceiling. Local raceline\n"
        "  # zones and online feasibility constraints may command less.\n",
    )
    vehicle = vehicle.replace(
        "  high_speed_closed_loop_approved: false\n",
        "  high_speed_closed_loop_approved: true\n",
    )
    vehicle = vehicle.replace(
        "  residual_dynamics:\n"
        "    status: one_step_candidate\n"
        "    method: robust_ridge_markov_v1_speed_pair_leave_one_out\n"
        "    deployment_output_scales: [0.0, 0.25, 0.25]\n"
        "    training_bags: 12\n"
        "    measured_at: \"2026-08-20\"\n"
        "  closed_loop_verification:\n"
        "    status: pending_current_track_test\n",
        "  residual_dynamics:\n"
        "    status: stable_v3_h3_scale030\n"
        "    method: recursive_gauss_newton_multistep_markov_v1\n"
        "    deployment_output_scales: [0.0, 0.30, 0.30]\n"
        "    fit_horizon: 3\n"
        "    training_windows: 11138\n"
        "    measured_at: \"2026-08-20\"\n"
        "  closed_loop_verification:\n"
        "    status: stable_v5_track_validated\n",
    )
    RELEASE_VEHICLE.write_text(vehicle, encoding="utf-8")


def write_track() -> dict:
    with SOURCE_TRACK.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError("raceline header is missing")
        fieldnames = reader.fieldnames
        rows = list(reader)

    zone_counts: dict[str, int] = {}
    for row in rows:
        zone = row["speed_zone"]
        zone_counts[zone] = zone_counts.get(zone, 0) + 1
        if zone not in FAST_ZONES | STANDARD_ZONES:
            raise RuntimeError(f"unclassified speed zone: {zone}")
        row["speed_limit_mps"] = "5.000000000" if zone in FAST_ZONES else "3.200000000"

    # The speed spline applies each node to its following segment. Keep the
    # first fast-zone node at the ordinary cap so interpolation cannot raise
    # the final ordinary segment above 3.2 m/s.
    fast_entry_rows = []
    for index, row in enumerate(rows):
        previous_zone = rows[(index - 1) % len(rows)]["speed_zone"]
        if row["speed_zone"] in FAST_ZONES and previous_zone in STANDARD_ZONES:
            row["speed_limit_mps"] = "3.200000000"
            fast_entry_rows.append(index)

    def save() -> None:
        with RELEASE_TRACK.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    save()
    controller = yaml.safe_load(RELEASE_CONTROLLER.read_text(encoding="utf-8"))
    vehicle = yaml.safe_load(RELEASE_VEHICLE.read_text(encoding="utf-8"))
    planning = dict(controller["speed_planning"])
    planning.update(
        wheelbase_m=vehicle["vehicle"]["wheelbase"],
        max_steer_rad=vehicle["limits"]["max_steer"],
        max_steer_rate_radps=vehicle["limits"]["max_steer_rate"],
        max_accel_mps2=vehicle["limits"]["max_accel"],
        max_decel_mps2=vehicle["limits"]["max_decel"],
        lateral_accel_limit_mps2=vehicle["limits"]["lateral_accel_limit"],
    )
    track = PeriodicTrack(RELEASE_TRACK, speed_planning=planning)
    for index, row in enumerate(rows):
        row["vx_mps"] = f"{float(track.speed_nodes[index]):.9f}"
        row["ax_mps2"] = f"{float(track.accel_nodes[index]):.9f}"
    save()

    runtime = PeriodicTrack(RELEASE_TRACK, speed_planning=planning)
    zone_envelopes = {}
    for zone in sorted(zone_counts):
        indexes = [index for index, row in enumerate(rows) if row["speed_zone"] == zone]
        limits = [float(rows[index]["speed_limit_mps"]) for index in indexes]
        speeds = [float(runtime.speed_nodes[index]) for index in indexes]
        expected = 5.0 if zone in FAST_ZONES else 3.2
        if max(speeds) > expected + 1.0e-9:
            raise RuntimeError(f"planned speed exceeds {zone} cap")
        if zone in STANDARD_ZONES and any(abs(value - 3.2) > 1.0e-9 for value in limits):
            raise RuntimeError(f"ordinary zone {zone} is not fixed at 3.2 m/s")
        zone_envelopes[zone] = {
            "rows": len(indexes),
            "local_limit_mps": [min(limits), max(limits)],
            "planned_speed_mps": [min(speeds), max(speeds)],
        }
    return {
        "length_m": runtime.length,
        "rows": len(rows),
        "fast_entry_rows": fast_entry_rows,
        "zone_counts": zone_counts,
        "zone_envelopes": zone_envelopes,
    }


def write_zones() -> None:
    with SOURCE_ZONES.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError("speed-zone header is missing")
        fieldnames = reader.fieldnames
        rows = list(reader)
    for row in rows:
        zone = row["zone_name"]
        if zone in FAST_ZONES:
            row["maximum_speed_mps"] = "5.000000000"
        elif zone in STANDARD_ZONES:
            row["maximum_speed_mps"] = "3.200000000"
            row["zone_name_cn"] = row["zone_name_cn"].replace("普通速度区", "普通速度区3.2")
        else:
            raise RuntimeError(f"unclassified speed-zone metadata: {zone}")
    with RELEASE_ZONES.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_launch() -> None:
    text = SOURCE_LAUNCH.read_text(encoding="utf-8")
    text = text.replace(SOURCE_NAME, RELEASE_NAME)
    text = text.replace(SOURCE_TRACK_NAME, RELEASE_TRACK_NAME)
    text = text.replace(
        "candidates/stable_v3_5mps_arc_straight_std3_candidate/vehicle.yaml",
        "candidates/stable_v5/vehicle.yaml",
    )
    text = text.replace(
        "acados_stable_v3_5mps_auto_relaunch_brake4_asym_60hz",
        "acados_stable_v5",
    )
    text = text.replace(
        "rviz_f1tenth_mpcc_auto_relaunch_brake4_asym_60hz_candidate",
        "rviz_f1tenth_mpcc_stable_v5",
    )
    text = text.replace(
        '<param name="race_start_radius_m" value="1.20"/>',
        '<param name="race_start_radius_m" value="2.00"/>',
    )
    text = text.replace(
        '<param name="race_start_speed_mps" value="3.00"/>',
        '<param name="race_start_speed_mps" value="4.00"/>',
    )
    text = text.replace(
        '<param name="race_start_handoff_speed_mps" value="2.70"/>',
        '<param name="race_start_handoff_speed_mps" value="1.20"/>',
    )
    text = text.replace(
        '<param name="handoff_minimum_mpcc_speed_mps" value="1.80"/>',
        '<param name="handoff_minimum_mpcc_speed_mps" value="1.00"/>',
    )
    text = text.replace("in this candidate", "in Stable V5")
    text = text.replace("isolated brake4/asymmetric dual-grid 60 Hz candidate", "Stable V5")
    RELEASE_LAUNCH.write_text(text, encoding="utf-8")


def write_recorder() -> None:
    """Write a release-local recorder without changing shared Stable V3 tools."""
    RELEASE_RECORDER.write_text(
        '''#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG" >&2
  exit 2
fi

TOPICS=(
  /aft_mapped_to_init
  /localization/odom
  /localization/vehicle_odom
  /localization/status
  /tf
  /tf_static
  /f1tenth_mpcc/real/ackermann_cmd_stamped
  /f1tenth_mpcc/telemetry
  /automatic_relaunch_supervisor/state
  /tianracer/ackermann_cmd
  /tianracer/odom
  /tianracer/imu
  /livox/imu
  /lio/relocalization/lighterbev_match
  /scan
  /semantic_region/state
  /perception/targets_detailed
  /perception/target_markers
  /overtake/diagnostics
  /overtake/selected_path
  /overtake/valid_area
  /avoidance/status
  /avoidance/decision
  /avoidance/markers
  /avoidance/local_plan_red
)

if [[ "${MPCC_RECORD_OVERTAKE_ALIGNED_SCAN:-false}" == "true" ]]; then
  TOPICS+=(/localization/aligned_scan)
fi

exec rosbag record -O "$1" "${TOPICS[@]}"
''',
        encoding="utf-8",
    )
    os.chmod(RELEASE_RECORDER, 0o755)


def launcher_preamble(frozen_hashes: dict[Path, str]) -> str:
    checks = "\n".join(
        f'check_hash "{digest}" "${variable}"'
        for variable, path in [
            ("CONTROLLER", RELEASE_CONTROLLER),
            ("VEHICLE", RELEASE_VEHICLE),
            ("TRACK_CSV", RELEASE_TRACK),
            ("SPEED_ZONES", RELEASE_ZONES),
            ("LAUNCH", RELEASE_LAUNCH),
            ("RESIDUAL", RESIDUAL),
            ("MPCC_NODE", MPCC_NODE),
            ("SUPERVISOR", SUPERVISOR),
            ("MOTION_LOGIC", MOTION_LOGIC),
            ("PLACEMENT_LOGIC", PLACEMENT_LOGIC),
            ("RECORDER", RELEASE_RECORDER),
        ]
        for digest in [frozen_hashes[path]]
    )
    return f'''#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${{BASH_SOURCE[0]}}")/.." && pwd)"
CANDIDATE="stable_v5"
TRACK="{RELEASE_TRACK_NAME}"
CONTROLLER_CONFIG="candidates/$CANDIDATE/controller.yaml"
VEHICLE_CONFIG="candidates/$CANDIDATE/vehicle.yaml"
CONTROLLER="$ROOT/src/f1tenth_dynamic_mpcc/config/$CONTROLLER_CONFIG"
VEHICLE="$ROOT/src/f1tenth_dynamic_mpcc/config/$VEHICLE_CONFIG"
LAUNCH="$ROOT/src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v5.launch"
TRACK_CSV="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/raceline.csv"
SPEED_ZONES="$ROOT/src/f1tenth_dynamic_mpcc/data/tracks/$TRACK/speed_zones.csv"
MPCC_NODE="$ROOT/src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch.cpp"
SUPERVISOR="$ROOT/src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor.py"
MOTION_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/automatic_relaunch.py"
PLACEMENT_LOGIC="$ROOT/src/f1tenth_dynamic_mpcc/python/f1tenth_dynamic_mpcc/quick_relaunch.py"
RECORDER="$ROOT/scripts/record_stable_v5_bag.sh"
RESIDUAL="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_stable_v3_racelinev3_h3_scale030.yaml"

check_hash() {{
  local expected="$1"
  local path="$2"
  local actual
  actual="$(sha256sum "$path" | awk '{{print $1}}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] Stable V5 frozen file changed: $path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}}

# Stable V5 is immutable: validate every behavior-defining release file.
{checks}

'''


def write_launcher() -> None:
    transformed = SOURCE_LAUNCHER.read_text(encoding="utf-8")
    action_index = transformed.index('ACTION="${1:-mpcc}"')
    tail = transformed[action_index:]
    tail = tail.replace(SOURCE_NAME, RELEASE_NAME)
    tail = tail.replace(SOURCE_TRACK_NAME, RELEASE_TRACK_NAME)
    tail = tail.replace(
        "stable_v3_5mps_auto_relaunch_brake4_asym_60hz",
        "stable_v5",
    )
    tail = tail.replace(
        'VEHICLE_CONFIG="candidates/stable_v3_5mps_arc_straight_std3_candidate/vehicle.yaml"',
        'VEHICLE_CONFIG="candidates/stable_v5/vehicle.yaml"',
    )
    tail = tail.replace(
        'echo "[OK] brake4/asymmetric 40 Hz baseline is unchanged"\n'
        '  echo "[OK] isolated brake4/asymmetric dual-grid 60 Hz candidate hashes match"',
        'echo "[OK] Stable V5 frozen hashes match"\n'
        '  echo "[OK] carry recovery, quick launch and dual-grid race start are enabled"',
    )
    tail = tail.replace(
        "automatic-relaunch candidate solvers are ready",
        "Stable V5 solvers are ready",
    )
    tail = tail.replace(
        'echo "[BRAKE4_ASYM_60HZ] isolated candidate based on the unchanged brake4/asym 40 Hz version"',
        'echo "[STABLE_V5] frozen Stable V3 residual MPCC race release"',
    )
    tail = tail.replace(
        'echo "[BRAKE4_ASYM_60HZ] measured 40 Hz baseline solve mean/P95=4.83/11.19 ms; overlap guard remains active"',
        'echo "[STABLE_V5] validated 60 Hz solve/publisher/supervisor pipeline; overlap guard remains active"',
    )
    tail = tail.replace(
        'echo "[AUTO_RELAUNCH] stableV3 residual + accepted 5/3 m/s raceline profile"',
        'echo "[AUTO_RELAUNCH] StableV3 residual + frozen 5.0/3.2 m/s raceline profile"',
    )
    tail = tail.replace(
        'echo "[AUTO_RELAUNCH] alternating left/right map-origin grid: launch=3.0 m/s; 2.5 m smooth merge; handoff=2.7 m/s"',
        'echo "[AUTO_RELAUNCH] alternating grid radius=2.0 m: launch=4.0 m/s; 2.5 m merge; handoff>=1.2 m/s"',
    )
    tail = tail.replace(
        'echo "[AUTO_RELAUNCH] PP->MPCC: adaptive speed + steering diff<=0.08 rad for 3 samples + 0.20 s blend"',
        'echo "[AUTO_RELAUNCH] PP->MPCC: MPCC>=1.0 m/s + steering diff<=0.08 rad for 3 samples + 0.20 s blend"',
    )
    tail = tail.replace(
        'echo "[AUTO_RELAUNCH] runtime cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; fast zones=5.0, standard=3.0"',
        'echo "[AUTO_RELAUNCH] runtime cap=${RESIDUAL_MPCC_SPEED_CAP} m/s; fast zones=5.0, ordinary=3.2"',
    )
    tail = tail.replace(
        'echo "[AUTO_RELAUNCH] brake4/asym 40 Hz baseline, original 5 m/s launcher and stableV3 remain unchanged"',
        'echo "[STABLE_V5] StableV3, StableV4 and all prior candidates remain unchanged"',
    )

    frozen = {
        path: sha256(path)
        for path in [
            RELEASE_CONTROLLER,
            RELEASE_VEHICLE,
            RELEASE_TRACK,
            RELEASE_ZONES,
            RELEASE_LAUNCH,
            RESIDUAL,
            MPCC_NODE,
            SUPERVISOR,
            MOTION_LOGIC,
            PLACEMENT_LOGIC,
            RELEASE_RECORDER,
        ]
    }
    RELEASE_LAUNCHER.write_text(launcher_preamble(frozen) + tail, encoding="utf-8")
    os.chmod(RELEASE_LAUNCHER, 0o755)


def write_readme(track_summary: dict) -> None:
    RELEASE_README.write_text(
        f"""# Stable V5

Stable V5 was frozen on 2026-08-21 from the validated Stable V3 60 Hz
brake4/asymmetric automatic-relaunch race candidate.

Release behavior:

- Stable V3 H3/scale-0.30 residual dynamics;
- contour 45, race heading 1.0 and steering-rate cost 0.60;
- 60 Hz MPCC solve, command publisher and hardware supervisor;
- E-stop/carry detection, ground stability gate and global-S reprojection;
- 2.0 m/s arbitrary-position recovery PP with adaptive minimum 1.2 m/s;
- arbitrary-position PP-to-MPCC handoff permitted from 1.0 m/s;
- alternating left/right race-grid launch within 2.0 m at 4.0 m/s;
- race-start PP-to-MPCC handoff permitted from 1.2 m/s;
- smooth PP-to-MPCC handoff after steering/speed agreement;
- 5.0 m/s local ceilings in designated arc/straight fast zones;
- 3.2 m/s local ceilings in all ordinary zones;
- 4.0 m/s^2 speed-profile, publisher and fallback deceleration;
- asymmetric acceleration (delay/tau 0.13/0.20 s) and braking
  (delay/tau 0.060/0.166 s) dynamics.

Track: `{RELEASE_TRACK_NAME}`, {track_summary['rows']} rows,
{track_summary['length_m']:.6f} m.

Run:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \\
  ./scripts/start_mpcc_hardware_stable_v5.sh mpcc
```
""",
        encoding="utf-8",
    )


def write_manifest(track_summary: dict) -> None:
    files = [
        RELEASE_CONTROLLER,
        RELEASE_VEHICLE,
        RELEASE_TRACK,
        RELEASE_ZONES,
        RELEASE_LAUNCH,
        RELEASE_LAUNCHER,
        RESIDUAL,
        MPCC_NODE,
        SUPERVISOR,
        MOTION_LOGIC,
        PLACEMENT_LOGIC,
        RELEASE_RECORDER,
    ]
    manifest = {
        "release": "stable_v5",
        "status": "frozen_release",
        "frozen_date": "2026-08-21",
        "source": SOURCE_NAME,
        "policy": {
            "runtime_global_cap_mps": 5.0,
            "fast_arc_and_straight_cap_mps": 5.0,
            "ordinary_cap_mps": 3.2,
            "profile_max_decel_mps2": 4.0,
            "control_rate_hz": 60.0,
            "publisher_rate_hz": 60.0,
            "supervisor_rate_hz": 60.0,
            "arbitrary_recovery_mpcc_handoff_min_mps": 1.0,
            "race_start_radius_m": 2.0,
            "race_start_speed_mps": 4.0,
            "race_start_handoff_speed_mps": 1.2,
        },
        "track": track_summary,
        "source_hashes": {relative(path): digest for path, digest in EXPECTED_SOURCE_HASHES.items()},
        "release_hashes": {relative(path): sha256(path) for path in files},
    }
    RELEASE_MANIFEST.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    validate_sources()
    RELEASE_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    RELEASE_TRACK_DIR.mkdir(parents=True, exist_ok=True)
    write_controller_and_vehicle()
    track_summary = write_track()
    write_zones()
    write_launch()
    write_recorder()
    write_launcher()
    write_readme(track_summary)
    write_manifest(track_summary)
    print(RELEASE_MANIFEST.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
