#!/usr/bin/env python3
import math
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.steering_actuator_model import (
    SteeringActuatorModel,
    SteeringActuatorParameters,
)
from f1tenth_dynamic_mpcc.steering_state_observer import (
    SteeringObserverParameters,
    SteeringStateObserver,
)
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


ROOT = Path(__file__).resolve().parents[1]


class TestClosedLoopNoDeltaSensor(unittest.TestCase):
    def test_vehicle_response_correction_beats_model_only(self):
        controller = load_yaml(ROOT / "config/controller.yaml")
        nominal = VehicleParameters.from_yaml(ROOT / "config/vehicle.yaml", controller)
        # Hidden plant mismatch emulates an uncalibrated servo. The observer sees
        # only command history and Point-LIO-like vx/vy/r, never plant_state[6].
        plant = DynamicBicycleModel(
            replace(nominal, steering_gain=0.72, steering_tau=0.11)
        )
        actuator = SteeringActuatorModel(
            SteeringActuatorParameters(False, "linear", 1.0, 0.0, 0.08, 0.0, -0.55, 0.55)
        )
        observer = SteeringStateObserver(
            actuator,
            nominal,
            SteeringObserverParameters(
                True, 0.3, 1.5, 0.15, 0.10, 2.0, 2.0, 2.0, 0.20, 0.25, 0.04
            ),
        )
        state = np.asarray([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0])
        model_only = 0.0
        observer_errors, model_errors = [], []
        dt = 0.02
        for tick in range(1000):
            timestamp = tick * dt
            command = 0.22 * math.sin(0.6 * timestamp) + 0.05 * math.sin(2.2 * timestamp)
            estimate = observer.update(timestamp, state[3], state[4], state[5])
            observer.push_command(timestamp, command)
            model_only = actuator.propagate(model_only, command, dt)
            command_rate = float(np.clip((command - state[7]) / dt, -5.5, 5.5))
            state = plant.step(state, np.asarray([1.5, command_rate, 1.5]), dt)
            if tick > 100:
                observer_errors.append(estimate.delta_hat - state[6])
                model_errors.append(model_only - state[6])
        observer_rmse = float(np.sqrt(np.mean(np.square(observer_errors))))
        model_rmse = float(np.sqrt(np.mean(np.square(model_errors))))
        self.assertLess(observer_rmse, 0.75 * model_rmse)


if __name__ == "__main__":
    unittest.main()
