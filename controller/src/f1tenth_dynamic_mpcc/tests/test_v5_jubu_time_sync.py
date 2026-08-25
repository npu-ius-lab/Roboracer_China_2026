#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "python"))

from f1tenth_v5_jubu.time_sync import (
    TimedPlanarState,
    body_to_world_vector,
    body_to_world_xy,
    interpolate_timed_state,
    propagate_xy,
)


class TimeSyncTest(unittest.TestCase):
    def test_interpolates_position_velocity_and_wrapped_yaw(self):
        samples = [
            TimedPlanarState(10.0, 0.0, 1.0, math.radians(179.0), 2.0, 0.0),
            TimedPlanarState(10.2, 0.4, 1.2, math.radians(-179.0), 4.0, 2.0),
        ]
        result = interpolate_timed_state(samples, 10.1)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.x, 0.2)
        self.assertAlmostEqual(result.y, 1.1)
        self.assertAlmostEqual(result.vx, 3.0)
        self.assertAlmostEqual(abs(result.yaw), math.pi, places=6)

    def test_rejects_unbounded_time_pairing(self):
        samples = [TimedPlanarState(2.0, 1.0, 0.0, 0.0, 1.0, 0.0)]
        self.assertIsNone(interpolate_timed_state(samples, 1.7))
        self.assertIsNone(interpolate_timed_state(samples, 2.2))

    def test_short_extrapolation_uses_world_velocity(self):
        sample = TimedPlanarState(3.0, 1.0, 2.0, 0.4, 2.0, -1.0)
        result = interpolate_timed_state([sample], 3.05)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.x, 1.1)
        self.assertAlmostEqual(result.y, 1.95)

    def test_measurement_pose_then_latency_compensation(self):
        measurement_ego = TimedPlanarState(5.0, 10.0, 3.0, math.pi / 2.0)
        target_x, target_y = body_to_world_xy(2.0, -0.5, measurement_ego)
        target_vx, target_vy = body_to_world_vector(1.0, 0.0, measurement_ego.yaw)
        current_x, current_y = propagate_xy(target_x, target_y, target_vx, target_vy, 0.30)
        self.assertAlmostEqual(target_x, 10.5)
        self.assertAlmostEqual(target_y, 5.0)
        self.assertAlmostEqual(current_x, 10.5)
        self.assertAlmostEqual(current_y, 5.3)


if __name__ == "__main__":
    unittest.main()
