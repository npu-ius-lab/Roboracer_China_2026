import unittest
from pathlib import Path
import numpy as np
from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters

ROOT = Path(__file__).resolve().parents[1]

class TestWheelVelocityCorrector(unittest.TestCase):
    def test_wheel_corrects_vx_but_never_constructs_vy(self):
        p = VehicleParameters.from_yaml(ROOT/'config/vehicle.yaml', load_yaml(ROOT/'config/controller.yaml'))
        h = CommandHistoryBuffer(2.0)
        h.push(PublishedCommandSample(0.0, 0.0, 0.0)); h.push(PublishedCommandSample(2.0, 0.0, 0.0))
        predictor = LowLatencyVehicleStatePredictor(DynamicBicycleModel(p), h, 0.01)
        for t in np.arange(1.0, 1.11, 0.02): predictor.push_wheel(t, 1.0)
        state = np.zeros(9); state[4] = 0.12
        out = predictor.predict_measurement_to_now(state, 1.0, 1.1, 0.3)
        self.assertGreater(out.state[3], 0.2)
        self.assertNotAlmostEqual(out.state[4], 1.0)
        self.assertEqual(out.mode, 'MODEL_IMU_WHEEL')

    def test_innovation_limit_rejects_large_slip_jump(self):
        p = VehicleParameters.from_yaml(ROOT/'config/vehicle.yaml', load_yaml(ROOT/'config/controller.yaml'))
        h = CommandHistoryBuffer(2.0)
        h.push(PublishedCommandSample(0.0, 0.0, 0.0)); h.push(PublishedCommandSample(2.0, 0.0, 0.0))
        predictor = LowLatencyVehicleStatePredictor(DynamicBicycleModel(p), h, 0.01)
        predictor.push_wheel(1.0, 20.0)
        out = predictor.predict_measurement_to_now(np.zeros(9), 1.0, 1.01, 0.3)
        self.assertLess(out.state[3], 0.2)

if __name__ == '__main__': unittest.main()
