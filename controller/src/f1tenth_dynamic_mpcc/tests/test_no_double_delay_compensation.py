import unittest
from pathlib import Path
import numpy as np
from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters

ROOT = Path(__file__).resolve().parents[1]

class TestNoDoubleDelayCompensation(unittest.TestCase):
    def setUp(self):
        controller = load_yaml(ROOT/'config/controller.yaml')
        self.assertEqual(float(controller['timing']['actuation_prediction_s']), 0.0)
        p = VehicleParameters.from_yaml(ROOT/'config/vehicle.yaml', controller)
        h = CommandHistoryBuffer(3.0)
        h.push(PublishedCommandSample(0.0, 0.0, 0.0))
        h.push(PublishedCommandSample(1.0, 1.0, 0.0))
        h.push(PublishedCommandSample(2.0, 1.0, 0.0))
        self.p = p
        self.predictor = LowLatencyVehicleStatePredictor(DynamicBicycleModel(p), h, 0.005)

    def test_mpcc_state_timestamp_is_now_plus_one_dead_time(self):
        out = self.predictor.predict_committed_horizon(np.zeros(9), 1.0)
        self.assertAlmostEqual(out.output_stamp, 1.0 + self.p.speed_dead_time)
        self.assertAlmostEqual(out.prediction_horizon, 0.13)
        # The step at t=1.0 only becomes effective at the interval endpoint.
        self.assertAlmostEqual(out.state[3], 0.0, places=8)

    def test_measurement_age_is_not_added_to_actuator_dead_time(self):
        state = np.zeros(9)
        now_state = self.predictor.predict_measurement_to_now(state, 0.89, 1.0, 0.3)
        committed = self.predictor.predict_committed_horizon(now_state.state, 1.0)
        self.assertAlmostEqual(now_state.prediction_horizon, 0.11)
        self.assertAlmostEqual(committed.prediction_horizon, 0.13)
        self.assertAlmostEqual(committed.output_stamp, 1.13)
        self.assertNotAlmostEqual(committed.output_stamp, 1.24)

if __name__ == '__main__': unittest.main()
