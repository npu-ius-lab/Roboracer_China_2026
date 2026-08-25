#!/usr/bin/env python3
import csv
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

from f1tenth_dynamic_mpcc.config import load_yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
SOURCE_CONTROLLER = PACKAGE / "config/candidates/stable_v3_5mps_auto_relaunch_brake4_asym_60hz_candidate/controller.yaml"
CONTROLLER = PACKAGE / "config/candidates/stable_v5/controller.yaml"
TRACK = PACKAGE / "data/tracks/racelinev3_stable_v5_fast5_std32/raceline.csv"
LAUNCH = PACKAGE / "launch/hardware_mpcc_stable_v5.launch"
LAUNCHER = WORKSPACE / "scripts/start_mpcc_hardware_stable_v5.sh"

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


class TestStableV5Release(unittest.TestCase):
    def test_controller_changes_only_tracking_gate_and_safety_margin(self):
        source = load_yaml(SOURCE_CONTROLLER)
        stable = load_yaml(CONTROLLER)

        # Stable V5 keeps the validated controller intact except for the
        # explicitly released tracking-quality speed gate and restored margin.
        stable["pure_pursuit_candidate"]["activation_current_margin_m"] = (
            source["pure_pursuit_candidate"]["activation_current_margin_m"]
        )
        stable["safety"].pop("high_speed_tracking_gate")
        stable["safety"].pop("prediction_soft_margin_m")
        self.assertEqual(source, stable)

    def test_speed_zone_policy_is_5_and_3p2(self):
        with TRACK.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 320)
        standard_limits = {
            float(row["speed_limit_mps"])
            for row in rows
            if row["speed_zone"] in STANDARD_ZONES
        }
        fast_limits = {
            float(row["speed_limit_mps"])
            for row in rows
            if row["speed_zone"] in FAST_ZONES
        }
        self.assertEqual(standard_limits, {3.2})
        # One fast-entry node deliberately keeps the preceding ordinary cap.
        self.assertEqual(fast_limits, {3.2, 5.0})
        self.assertLessEqual(
            max(float(row["vx_mps"]) for row in rows if row["speed_zone"] in STANDARD_ZONES),
            3.2,
        )

    def test_launch_freezes_relaunch_and_race_start(self):
        root = ET.parse(LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default") for arg in root.findall("arg")}
        self.assertEqual(args["track"], "racelinev3_stable_v5_fast5_std32")
        self.assertEqual(args["controller_config"], "candidates/stable_v5/controller.yaml")
        self.assertEqual(args["vehicle_config"], "candidates/stable_v5/vehicle.yaml")

        supervisor = next(
            node for node in root.findall("node")
            if node.get("name") == "automatic_relaunch_supervisor"
        )
        params = {param.get("name"): param.get("value") for param in supervisor.findall("param")}
        self.assertEqual(params["publish_rate_hz"], "60.0")
        self.assertEqual(params["probe_speed_mps"], "2.00")
        self.assertEqual(params["race_start_radius_m"], "2.00")
        self.assertEqual(params["race_start_speed_mps"], "4.00")
        self.assertEqual(params["race_start_handoff_speed_mps"], "3.00")
        self.assertEqual(params["recovery_handoff_minimum_speed_mps"], "1.00")
        self.assertEqual(params["handoff_minimum_mpcc_speed_mps"], "1.80")
        mpcc_node = next(
            node for node in root.findall("node")
            if node.get("name") == "f1tenth_dynamic_mpcc"
        )
        mpcc_params = {
            param.get("name"): param.get("value") for param in mpcc_node.findall("param")
        }
        self.assertEqual(mpcc_params["near_track_horizon_override"], "0.60")
        self.assertEqual(mpcc_params["predictive_speed_hold_override"], "0.75")
        self.assertEqual(mpcc_params["pp_current_margin_override"], "0.10")
        self.assertEqual(params["race_start_merge_lookahead_m"], "2.50")
        self.assertEqual(params["maximum_pp_recovery_angle_rad"], "1.0471975512")

        controller = load_yaml(CONTROLLER)
        gate = controller["safety"]["high_speed_tracking_gate"]
        self.assertTrue(gate["enabled"])
        self.assertEqual(gate["base_speed_mps"], 3.2)
        self.assertEqual(gate["lateral_full_speed_m"], 0.10)
        self.assertEqual(gate["lateral_base_speed_m"], 0.30)
        self.assertEqual(gate["heading_full_speed_rad"], 0.04)
        self.assertEqual(gate["heading_base_speed_rad"], 0.12)
        self.assertEqual(gate["margin_base_speed_m"], 0.12)
        self.assertEqual(gate["margin_full_speed_m"], 0.30)
        self.assertEqual(controller["safety"]["prediction_soft_margin_m"], 0.15)
        self.assertEqual(
            controller["pure_pursuit_candidate"]["activation_current_margin_m"],
            0.10,
        )

    def test_frozen_hash_gate_passes(self):
        result = subprocess.run(
            [str(LAUNCHER), "--check-only"],
            cwd=WORKSPACE,
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("Stable V5 frozen hashes match", result.stdout)


if __name__ == "__main__":
    unittest.main()
