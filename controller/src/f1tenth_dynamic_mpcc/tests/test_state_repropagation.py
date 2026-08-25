import unittest
from pathlib import Path
import numpy as np
from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters

ROOT = Path(__file__).resolve().parents[1]

def predictor():
    p = VehicleParameters.from_yaml(ROOT/'config/vehicle.yaml', load_yaml(ROOT/'config/controller.yaml'))
    h = CommandHistoryBuffer(2.0)
    h.push(PublishedCommandSample(0.0, 1.0, 0.0)); h.push(PublishedCommandSample(3.0, 1.0, 0.0))
    return LowLatencyVehicleStatePredictor(DynamicBicycleModel(p), h, 0.01)

class TestStateRepropagation(unittest.TestCase):
    def test_delayed_measurement_corrects_history_and_replays_to_now(self):
        p = predictor()
        initial = np.zeros(9); initial[3] = 1.0
        first = p.correct_delayed_measurement_and_repropagate(initial, 1.0, 1.20, 0.30)
        delayed = initial.copy(); delayed[0] = 0.10
        corrected = p.correct_delayed_measurement_and_repropagate(delayed, 1.10, 1.25, 0.30)
        self.assertGreater(corrected.state[0], delayed[0])
        self.assertAlmostEqual(corrected.output_stamp, 1.25)
        self.assertEqual(corrected.mode, 'REPROPAGATION')
        at_measurement = p.state_history.state_at(1.10)
        self.assertIsNotNone(at_measurement)
        self.assertAlmostEqual(at_measurement[0], 0.10, places=9)

    def test_repeated_measurement_does_not_reset_current_state(self):
        p = predictor(); state = np.zeros(9); state[3] = 1.0
        a = p.correct_delayed_measurement_and_repropagate(state, 1.0, 1.10, 0.30)
        b = p.correct_delayed_measurement_and_repropagate(state, 1.0, 1.20, 0.30)
        self.assertGreater(b.state[0], a.state[0])

    def test_stale_delayed_measurement_is_rejected(self):
        with self.assertRaises(TimeoutError):
            predictor().correct_delayed_measurement_and_repropagate(np.zeros(9), 1.0, 1.31, 0.30)

if __name__ == '__main__': unittest.main()
