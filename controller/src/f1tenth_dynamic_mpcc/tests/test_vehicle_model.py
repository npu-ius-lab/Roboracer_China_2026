#!/usr/bin/env python3
import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


ROOT = Path(__file__).resolve().parents[1]


class TestVehicleModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        controller = load_yaml(ROOT / "config/controller.yaml")
        cls.params = VehicleParameters.from_yaml(ROOT / "config/vehicle.yaml", controller)
        cls.model = DynamicBicycleModel(cls.params)

    def test_geometry_contract(self):
        self.assertAlmostEqual(self.params.wheelbase, 0.320, places=9)
        self.assertAlmostEqual(self.params.body_width, 0.24, places=9)
        self.assertAlmostEqual(self.params.lf + self.params.lr, 0.320, places=9)

    def test_identified_longitudinal_time_constant_is_loaded(self):
        self.assertAlmostEqual(self.params.speed_gain, 1.0, places=9)
        self.assertAlmostEqual(self.params.speed_tau, 0.20, places=9)

    def test_straight_motion(self):
        state = np.zeros(9)
        control = np.asarray([1.0, 0.0, 1.0])
        for _ in range(400):
            state = self.model.step(state, control, 0.01)
        self.assertGreater(state[0], 2.5)
        self.assertLess(abs(state[1]), 1.0e-8)
        self.assertLess(abs(state[2]), 1.0e-8)
        self.assertAlmostEqual(state[3], 1.0, places=2)

    def test_positive_steering_turns_left(self):
        state = np.asarray([0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.12, 0.0])
        control = np.asarray([1.2, 0.0, 1.2])
        for _ in range(150):
            state = self.model.step(state, control, 0.005)
        self.assertGreater(state[2], 0.0)
        self.assertGreater(state[5], 0.0)
        self.assertGreater(state[6], 0.0)

    def test_stiff_lateral_mode_is_numerically_stable_at_control_period(self):
        state = np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.30, 0.0])
        control = np.asarray([1.0, 0.0, 1.0])
        for _ in range(100):
            state = self.model.step(state, control, 0.05)
        self.assertLess(abs(state[4]), 0.30)
        self.assertLess(abs(state[5]), 1.50)

    def test_steering_actuator_returns_to_zero(self):
        state = np.asarray([0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.3, 0.0, 0.0])
        control = np.asarray([0.2, 0.0, 0.2])
        initial = abs(state[6])
        for _ in range(80):
            state = self.model.step(state, control, 0.005)
        self.assertLess(abs(state[6]), initial * 0.15)

    def test_slip_angle_signs_are_finite(self):
        state = np.asarray([0.0, 0.0, 0.0, 1.5, 0.05, 0.2, 0.1, 0.1, 0.0])
        alpha_f, alpha_r = self.model.slip_angles(state)
        self.assertTrue(np.isfinite([alpha_f, alpha_r]).all())
        self.assertGreater(alpha_f, 0.0)

    def test_command_angle_state_obeys_rate_input(self):
        state = np.zeros(9)
        state = self.model.step(state, np.asarray([0.0, 1.5, 0.0]), 0.05)
        self.assertAlmostEqual(state[7], 0.075, places=9)


if __name__ == "__main__":
    unittest.main()
