#!/usr/bin/env python3
import csv
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.config import apply_runtime_speed_cap, load_yaml
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack
from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters


ROOT = Path(__file__).resolve().parents[1]


class TestSpeedPlanning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = load_yaml(ROOT / "config/controller.yaml")
        cls.vehicle = VehicleParameters.from_yaml(
            ROOT / "config/vehicle.yaml", cls.controller
        )
        planning = dict(cls.controller["speed_planning"])
        planning.update(
            wheelbase_m=cls.vehicle.wheelbase,
            max_steer_rad=cls.vehicle.max_steer,
            max_steer_rate_radps=cls.vehicle.max_steer_rate,
            max_accel_mps2=cls.vehicle.max_accel,
            max_decel_mps2=cls.vehicle.max_decel,
            lateral_accel_limit_mps2=cls.vehicle.lateral_accel_limit,
        )
        cls.track = PeriodicTrack(
            ROOT / "data/tracks/virtual_track/raceline.csv",
            speed_planning=planning,
        )

    def test_four_mps_is_a_cap_not_a_fixed_reference(self):
        speed = self.track.speed_nodes
        self.assertLessEqual(float(np.max(speed)), 4.0 + 1.0e-9)
        self.assertAlmostEqual(float(np.max(speed)), 4.0, places=6)
        self.assertLess(float(np.min(speed)), 2.5)
        self.assertGreater(float(np.ptp(speed)), 0.5)

    def test_lateral_and_longitudinal_constraints_hold(self):
        speed = self.track.speed_nodes
        curvature = np.abs(self.track.curvature(self.track.s_nodes))
        lateral = speed**2 * curvature
        self.assertLessEqual(
            float(np.max(lateral)), self.vehicle.lateral_accel_limit + 1.0e-6
        )
        self.assertLessEqual(
            float(np.max(self.track.accel_nodes)), self.vehicle.max_accel + 1.0e-6
        )
        self.assertGreaterEqual(
            float(np.min(self.track.accel_nodes)), -self.vehicle.max_decel - 1.0e-6
        )

    def test_all_controller_and_vehicle_caps_are_four_mps(self):
        safety = self.controller["safety"]
        self.assertEqual(float(self.controller["speed_planning"]["max_speed_mps"]), 4.0)
        self.assertEqual(float(safety["global_speed_max_mps"]), 4.0)
        self.assertEqual(float(safety["baseline_speed_max_mps"]), 4.0)
        self.assertEqual(float(safety["debug_speed_max_mps"]), 4.0)
        self.assertEqual(self.vehicle.max_speed, 4.0)

    def test_runtime_cap_clamps_python_controller_without_raising_yaml_limits(self):
        controller = deepcopy(self.controller)
        apply_runtime_speed_cap(controller, 1.5)
        self.assertEqual(float(controller["speed_planning"]["max_speed_mps"]), 1.5)
        self.assertEqual(float(controller["safety"]["global_speed_max_mps"]), 1.5)
        self.assertEqual(float(controller["safety"]["baseline_speed_max_mps"]), 1.5)
        self.assertEqual(float(controller["safety"]["debug_speed_max_mps"]), 1.5)

    def test_runtime_cap_rejects_nonpositive_value(self):
        with self.assertRaises(ValueError):
            apply_runtime_speed_cap(deepcopy(self.controller), 0.0)

    def test_profile_deceleration_can_be_more_conservative_than_vehicle_limit(self):
        planning = dict(self.controller["speed_planning"])
        planning["profile_max_decel_mps2"] = 1.25
        planning.update(
            wheelbase_m=self.vehicle.wheelbase,
            max_steer_rad=self.vehicle.max_steer,
            max_steer_rate_radps=self.vehicle.max_steer_rate,
            max_accel_mps2=self.vehicle.max_accel,
            max_decel_mps2=self.vehicle.max_decel,
            lateral_accel_limit_mps2=self.vehicle.lateral_accel_limit,
        )
        track = PeriodicTrack(
            ROOT / "data/tracks/virtual_track/raceline.csv",
            speed_planning=planning,
        )
        self.assertGreaterEqual(float(np.min(track.accel_nodes)), -1.25 - 1.0e-6)

    def test_profile_acceleration_can_match_a_slower_command_publisher(self):
        planning = dict(self.controller["speed_planning"])
        planning["profile_max_accel_mps2"] = 0.85
        planning.update(
            wheelbase_m=self.vehicle.wheelbase,
            max_steer_rad=self.vehicle.max_steer,
            max_steer_rate_radps=self.vehicle.max_steer_rate,
            max_accel_mps2=self.vehicle.max_accel,
            max_decel_mps2=self.vehicle.max_decel,
            lateral_accel_limit_mps2=self.vehicle.lateral_accel_limit,
        )
        track = PeriodicTrack(
            ROOT / "data/tracks/virtual_track/raceline.csv",
            speed_planning=planning,
        )
        self.assertLessEqual(float(np.max(track.accel_nodes)), 0.85 + 1.0e-6)

    def test_per_node_speed_limits_override_the_global_cap(self):
        source = ROOT / "data/tracks/virtual_track/raceline.csv"
        with source.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        with tempfile.TemporaryDirectory() as directory:
            limited = Path(directory) / "raceline.csv"
            fieldnames = list(rows[0]) + ["speed_limit_mps"]
            with limited.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                for row in rows:
                    writer.writerow({**row, "speed_limit_mps": "0.9"})
            planning = dict(self.controller["speed_planning"])
            planning.update(
                wheelbase_m=self.vehicle.wheelbase,
                max_steer_rad=self.vehicle.max_steer,
                max_steer_rate_radps=self.vehicle.max_steer_rate,
                max_accel_mps2=self.vehicle.max_accel,
                max_decel_mps2=self.vehicle.max_decel,
                lateral_accel_limit_mps2=self.vehicle.lateral_accel_limit,
            )
            track = PeriodicTrack(limited, speed_planning=planning)
            self.assertLessEqual(float(np.max(track.speed_nodes)), 0.9 + 1.0e-9)
            dense_s = np.linspace(0.0, track.length, 20000, endpoint=False)
            self.assertLessEqual(
                float(np.max(track.speed_prior(dense_s))), 0.9 + 1.0e-9
            )
            self.assertEqual(
                track.speed_profile_source,
                "runtime_constraints+local_speed_limits",
            )


if __name__ == "__main__":
    unittest.main()
