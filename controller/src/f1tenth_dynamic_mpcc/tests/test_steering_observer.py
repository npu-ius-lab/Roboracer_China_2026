#!/usr/bin/env python3
import math
import unittest
from pathlib import Path

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.steering_actuator_model import SteeringActuatorModel
from f1tenth_dynamic_mpcc.steering_state_observer import (
    INVALID,
    KINEMATIC_CORRECTION,
    MODEL_ONLY,
    SteeringStateObserver,
)
from f1tenth_dynamic_mpcc.vehicle_model import VehicleParameters


ROOT = Path(__file__).resolve().parents[1]


class TestSteeringObserver(unittest.TestCase):
    def setUp(self):
        self.vehicle_config = load_yaml(ROOT / "config/vehicle.yaml")
        controller = load_yaml(ROOT / "config/controller.yaml")
        vehicle = VehicleParameters.from_yaml(ROOT / "config/vehicle.yaml", controller)
        actuator = SteeringActuatorModel.from_config(self.vehicle_config)
        self.observer = SteeringStateObserver.from_config(
            self.vehicle_config, actuator, vehicle
        )

    def test_zero_speed_is_model_only(self):
        self.observer.push_command(0.0, 0.2)
        self.observer.update(0.0, 0.0, 0.0, 0.0)
        result = self.observer.update(0.037, 0.0, 0.0, 0.0)
        self.assertTrue(result.valid)
        self.assertEqual(result.mode, MODEL_ONLY)
        self.assertGreater(result.delta_hat, 0.0)

    def test_dropout_preserves_model_state_then_marks_invalid_only_on_bad_data(self):
        self.observer.push_command(0.0, 0.2)
        first = self.observer.update(0.0, 1.0, 0.0, 0.0)
        dropout = self.observer.update(0.40, 1.0, 0.0, 0.0)
        self.assertTrue(dropout.valid)
        self.assertEqual(dropout.mode, MODEL_ONLY)
        self.assertGreater(dropout.delta_hat, first.delta_hat)
        invalid = self.observer.update(float("nan"), 1.0, 0.0, 0.0)
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.mode, INVALID)
        self.assertAlmostEqual(invalid.delta_hat, dropout.delta_hat)

    def test_dropout_propagates_across_command_edges(self):
        self.observer.update(0.0, 0.0, 0.0, 0.0)
        self.observer.push_command(0.05, 0.2)
        self.observer.push_command(0.15, -0.1)
        result = self.observer.update(0.30, 0.0, 0.0, 0.0)
        actuator = self.observer.actuator
        expected = actuator.propagate(0.0, 0.0, 0.05 + actuator.p.delay)
        expected = actuator.propagate(expected, 0.2, 0.10)
        expected = actuator.propagate(
            expected, -0.1, 0.15 - actuator.p.delay
        )
        self.assertAlmostEqual(result.delta_hat, expected, places=12)

    def test_kinematic_response_builds_takeover_confidence(self):
        delta_true = 0.12
        vx = 0.8
        yaw_rate = vx / self.observer.vehicle.wheelbase * math.tan(delta_true)
        result = self.observer.update(0.0, vx, 0.0, yaw_rate)
        for index in range(1, 40):
            result = self.observer.update(index * 0.02, vx, 0.0, yaw_rate)
        self.assertEqual(result.mode, KINEMATIC_CORRECTION)
        self.assertGreater(result.confidence, 0.6)
        self.assertAlmostEqual(result.delta_kinematic, delta_true, places=6)
        self.assertGreater(result.delta_hat, 0.02)
        self.assertLess(abs(delta_true - result.delta_hat), abs(delta_true))


if __name__ == "__main__":
    unittest.main()
