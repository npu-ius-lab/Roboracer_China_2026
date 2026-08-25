import unittest
from pathlib import Path
import numpy as np
from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters

ROOT = Path(__file__).resolve().parents[1]

class TestImuPropagation(unittest.TestCase):
    def test_fresh_gyro_overrides_model_yaw_rate_without_integrating_accel(self):
        p = VehicleParameters.from_yaml(ROOT/'config/vehicle.yaml', load_yaml(ROOT/'config/controller.yaml'))
        h = CommandHistoryBuffer(2.0)
        h.push(PublishedCommandSample(0.0, 0.0, 0.0)); h.push(PublishedCommandSample(2.0, 0.0, 0.0))
        predictor = LowLatencyVehicleStatePredictor(DynamicBicycleModel(p), h, 0.01)
        for t in np.arange(1.0, 1.11, 0.005): predictor.push_imu(t, 0.5)
        out = predictor.predict_measurement_to_now(np.zeros(9), 1.0, 1.1, 0.3)
        self.assertAlmostEqual(out.state[5], 0.5, places=6)
        self.assertAlmostEqual(out.state[2], 0.05, delta=0.01)
        self.assertEqual(out.mode, 'MODEL_IMU')

if __name__ == '__main__': unittest.main()
