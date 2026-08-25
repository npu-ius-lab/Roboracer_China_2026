import math
import unittest

import numpy as np

from f1tenth_dynamic_mpcc.command_history_buffer import (
    CommandHistoryBuffer, MissingCommandHistory, PublishedCommandSample,
)


class TestCommandDelayFifo(unittest.TestCase):
    def setUp(self):
        self.history = CommandHistoryBuffer(retention_s=5.0)
        self.history.push(PublishedCommandSample(0.0, 0.0, 0.0))
        self.history.push(PublishedCommandSample(1.0, 1.0, 0.2))
        self.history.push(PublishedCommandSample(2.0, 1.0, 0.2))

    def test_step_is_not_effective_before_dead_time(self):
        self.assertEqual(self.history.effective_command_at(1.129, 0.13).speed_cmd, 0.0)
        self.assertEqual(self.history.effective_command_at(1.130, 0.13).speed_cmd, 1.0)

    def test_steering_uses_its_own_20_ms_dead_time(self):
        self.assertEqual(
            self.history.effective_command_at(1.019, 0.13, 0.02).steering_cmd,
            0.0,
        )
        self.assertEqual(
            self.history.effective_command_at(1.020, 0.13, 0.02).steering_cmd,
            0.2,
        )

    def test_first_order_response_is_632_percent_one_tau_after_delayed_step(self):
        tau, delay, dt = 0.20, 0.13, 0.0005
        vx = 0.0
        for time in np.arange(1.0, 1.0 + delay + tau, dt):
            target = self.history.effective_command_at(time, delay).speed_cmd
            vx += dt * (target - vx) / tau
        self.assertAlmostEqual(vx, 1.0 - math.exp(-1.0), delta=0.003)

    def test_production_predictor_uses_delayed_published_speed(self):
        from pathlib import Path
        from f1tenth_dynamic_mpcc.config import load_yaml
        from f1tenth_dynamic_mpcc.delay_compensator import DelayCompensator
        from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters

        root = Path(__file__).resolve().parents[1]
        params = VehicleParameters.from_yaml(
            root / "config/vehicle.yaml", load_yaml(root / "config/controller.yaml")
        )
        model = DynamicBicycleModel(params)
        history = CommandHistoryBuffer(retention_s=5.0)
        history.push(PublishedCommandSample(0.0, 0.0, 0.0))
        history.push(PublishedCommandSample(1.0, 0.4, 0.0))
        history.push(PublishedCommandSample(2.0, 0.4, 0.0))
        predictor = DelayCompensator(model, 0.002, history)
        predictor.push(0.0, np.zeros(3))
        predictor.push(1.0, np.asarray([0.4, 0.0, 0.0]))
        state = np.zeros(9)
        before, _ = predictor.predict(state, 1.0, 1.129, 1.0)
        after, _ = predictor.predict(state, 1.0, 1.0 + 0.13 + 0.20, 1.0)
        self.assertAlmostEqual(before[3], 0.0, places=9)
        self.assertAlmostEqual(after[3], 0.4 * (1.0 - math.exp(-1.0)), delta=0.01)

    def test_out_of_order_insert_and_interpolation(self):
        history = CommandHistoryBuffer(5.0)
        history.push(PublishedCommandSample(2.0, 2.0, 0.2))
        history.push(PublishedCommandSample(0.0, 0.0, 0.0))
        history.push(PublishedCommandSample(1.0, 1.0, 0.1))
        self.assertEqual(history.command_at(1.5).speed_cmd, 1.0)
        self.assertAlmostEqual(history.command_at(1.5, interpolate=True).speed_cmd, 1.5)

    def test_coverage_and_missing_history(self):
        self.assertTrue(self.history.covers(0.0, 2.0))
        self.assertFalse(self.history.covers(-0.1, 2.0))
        with self.assertRaises(MissingCommandHistory):
            self.history.command_at(-0.1)


if __name__ == "__main__":
    unittest.main()
