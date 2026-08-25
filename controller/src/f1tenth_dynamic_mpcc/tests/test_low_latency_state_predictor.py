import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


ROOT = Path(__file__).resolve().parents[1]


class TestLowLatencyStatePredictor(unittest.TestCase):
    def setUp(self):
        params = VehicleParameters.from_yaml(
            ROOT / "config/vehicle.yaml", load_yaml(ROOT / "config/controller.yaml")
        )
        history = CommandHistoryBuffer(2.0)
        history.push(PublishedCommandSample(9.0, 1.0, 0.0))
        history.push(PublishedCommandSample(11.0, 1.0, 0.0))
        self.predictor = LowLatencyVehicleStatePredictor(
            DynamicBicycleModel(params), history, 0.01
        )

    def test_contract_is_measurement_stamp_to_control_now(self):
        state = np.zeros(9)
        state[3] = 1.0
        result = self.predictor.predict_measurement_to_now(state, 10.0, 10.11, 0.30)
        self.assertEqual(result.source_stamp, 10.0)
        self.assertEqual(result.output_stamp, 10.11)
        self.assertAlmostEqual(result.prediction_horizon, 0.11)
        self.assertGreater(result.state[0], 0.10)

    def test_uses_real_age_and_rejects_stale_state(self):
        with self.assertRaises(TimeoutError):
            self.predictor.predict_measurement_to_now(np.zeros(9), 10.0, 10.31, 0.30)


if __name__ == "__main__":
    unittest.main()
