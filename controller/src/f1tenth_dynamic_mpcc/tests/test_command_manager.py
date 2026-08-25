#!/usr/bin/env python3
import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.command_manager import CommandManager
from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.startup_state_machine import RACE, SAFE_DECEL, StartupProfile


ROOT = Path(__file__).resolve().parents[1]


def profile(mode=RACE):
    return StartupProfile(mode, 1.0, 2.0, 1.5, 1.0, float("inf"))


class TestCommandManager(unittest.TestCase):
    def setUp(self):
        self.config = load_yaml(ROOT / "config/controller.yaml")
        self.manager = CommandManager(self.config, 0.55)

    @staticmethod
    def solution():
        states = np.zeros((4, 9))
        controls = np.zeros((3, 3))
        states[1, 7] = 0.10
        states[2, 7] = 0.12
        controls[:, 0] = [1.0, 0.9, 0.8]
        controls[:, 2] = [1.0, 0.9, 0.8]
        return states, controls

    def test_single_failure_uses_shifted_horizon_without_zero_jump(self):
        states, controls = self.solution()
        self.manager.set_solution(states, controls, 0.0)
        first = self.manager.tick(0.0, profile())
        self.manager.note_failure()
        shifted = self.manager.tick(0.05, profile())
        self.assertEqual(shifted.source, "shifted_solution")
        self.assertGreater(shifted.speed, first.speed)
        self.assertGreater(shifted.steering, 0.0)
        self.assertLessEqual(abs(shifted.steering - first.steering), 1.5 * 0.05 + 1e-12)

    def test_safe_decel_uses_configured_continuous_rates(self):
        states, controls = self.solution()
        for index in range(10):
            now = index * 0.05
            # Model a healthy 20 Hz solver so command freshness does not
            # independently trigger the timeout fallback before this test
            # explicitly enters SAFE_DECEL.
            self.manager.set_solution(states, controls, now)
            moving = self.manager.tick(now, profile())
        safe = self.manager.tick(0.50, profile(SAFE_DECEL))
        expected_drop = float(self.config["fallback"]["decel_mps2"]) * 0.05
        self.assertAlmostEqual(moving.speed - safe.speed, expected_drop, places=9)
        self.assertLessEqual(abs(safe.steering - moving.steering), 0.75 * 0.05 + 1e-12)

    def test_publisher_tick_interval_remains_fixed_without_new_solver_output(self):
        states, controls = self.solution()
        self.manager.set_solution(states, controls, 0.0)
        period = 1.0 / float(self.config["publisher"]["rate_hz"])
        intervals = [
            self.manager.tick(index * period, profile()).publish_dt
            for index in range(6)
        ]
        self.assertTrue(all(abs(value - period) < 1e-9 for value in intervals))


if __name__ == "__main__":
    unittest.main()
