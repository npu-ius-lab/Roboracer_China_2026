#!/usr/bin/env python3
import math
import unittest

from f1tenth_dynamic_mpcc.steering_actuator_model import (
    SteeringActuatorModel,
    SteeringActuatorParameters,
)


class TestSteeringActuator(unittest.TestCase):
    def setUp(self):
        self.model = SteeringActuatorModel(
            SteeringActuatorParameters(False, "linear", 0.9, 0.01, 0.10, 0.0, -0.5, 0.5)
        )

    def test_exact_first_order_step_with_timestamp_jitter(self):
        estimate = 0.0
        elapsed = 0.0
        for dt in (0.013, 0.027, 0.041, 0.019, 0.10):
            estimate = self.model.propagate(estimate, 0.2, dt)
            elapsed += dt
        target = self.model.target(0.2)
        expected = target * (1.0 - math.exp(-elapsed / self.model.p.tau))
        self.assertAlmostEqual(estimate, expected, places=12)


if __name__ == "__main__":
    unittest.main()
