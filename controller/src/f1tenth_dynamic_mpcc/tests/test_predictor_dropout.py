import unittest
from pathlib import Path
import numpy as np
from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters

ROOT=Path(__file__).resolve().parents[1]
class TestPredictorDropout(unittest.TestCase):
    def test_hard_age_rejects_unbounded_prediction(self):
        c=load_yaml(ROOT/'config/controller.yaml'); p=VehicleParameters.from_yaml(ROOT/'config/vehicle.yaml',c)
        h=CommandHistoryBuffer(2); h.push(PublishedCommandSample(0,0,0)); h.push(PublishedCommandSample(2,0,0))
        q=LowLatencyVehicleStatePredictor(DynamicBicycleModel(p),h)
        with self.assertRaises(TimeoutError): q.correct_delayed_measurement_and_repropagate(np.zeros(9),1.0,1.31,0.30)
if __name__=='__main__': unittest.main()
