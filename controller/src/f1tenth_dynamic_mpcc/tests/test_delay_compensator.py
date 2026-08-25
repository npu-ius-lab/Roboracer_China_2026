#!/usr/bin/env python3
import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.delay_compensator import DelayCompensator
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


ROOT = Path(__file__).resolve().parents[1]


class TestDelayCompensator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        controller = load_yaml(ROOT / "config/controller.yaml")
        params = VehicleParameters.from_yaml(ROOT / "config/vehicle.yaml", controller)
        cls.model = DynamicBicycleModel(params)

    def test_predicts_state_to_current_time(self):
        compensator = DelayCompensator(self.model, 0.005)
        command = np.asarray([1.0, 0.0, 1.0])
        compensator.push(10.0, command)
        state = np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        predicted, age = compensator.predict(state, 10.0, 10.2, 0.3)
        self.assertAlmostEqual(age, 0.2, places=9)
        self.assertAlmostEqual(predicted[0], 0.2, places=3)
        self.assertAlmostEqual(predicted[8], 0.2, places=3)

    def test_rejects_excessively_old_state(self):
        compensator = DelayCompensator(self.model)
        with self.assertRaises(TimeoutError):
            compensator.predict(np.zeros(9), 1.0, 1.5, 0.3)


if __name__ == "__main__":
    unittest.main()
