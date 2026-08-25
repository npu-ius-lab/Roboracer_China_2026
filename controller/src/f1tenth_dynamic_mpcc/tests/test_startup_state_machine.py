#!/usr/bin/env python3
import unittest
from pathlib import Path

from f1tenth_dynamic_mpcc.config import load_yaml
from f1tenth_dynamic_mpcc.startup_state_machine import (
    BOOT,
    CRAWL_OBSERVE,
    MPCC_RAMP,
    RACE,
    SAFE_DECEL,
    STEER_SETTLE,
    StartupStateMachine,
)


ROOT = Path(__file__).resolve().parents[1]


class TestStartupStateMachine(unittest.TestCase):
    def setUp(self):
        self.config = load_yaml(ROOT / "config/controller.yaml")
        self.machine = StartupStateMachine(self.config, 2.0, 1.5)
        self.machine.reset(0.0)

    def test_stationary_start_cannot_enter_race_with_model_only_confidence(self):
        self.assertEqual(self.machine.mode, BOOT)
        self.machine.update_observer(0.0, True, 0.2, True)
        self.assertEqual(self.machine.mode, STEER_SETTLE)
        self.machine.update_observer(0.61, True, 0.2, True)
        self.assertEqual(self.machine.mode, CRAWL_OBSERVE)
        for index in range(100):
            self.machine.update_observer(0.62 + index * 0.02, True, 0.2, True)
        self.assertEqual(self.machine.mode, CRAWL_OBSERVE)

    def test_valid_observer_enters_smooth_ramp_then_race(self):
        self.machine.update_observer(0.0, True, 0.2, True)
        self.machine.update_observer(0.61, True, 0.2, True)
        required = int(self.config["startup"]["observer_valid_samples_required"])
        for index in range(required):
            self.machine.update_observer(0.7 + index * 0.02, True, 0.61, True)
        self.assertEqual(self.machine.mode, MPCC_RAMP)
        mid = self.machine.profile(float(self.machine.entered_at) + 0.4)
        self.assertAlmostEqual(mid.ramp_lambda, 0.5, places=6)
        self.assertGreater(mid.speed_cap, 0.6)
        self.machine.update_observer(float(self.machine.entered_at) + 0.81, True, 0.8, True)
        self.assertEqual(self.machine.mode, RACE)

    def test_failures_and_recovery_never_jump_directly_to_race(self):
        self.machine._enter(RACE, 1.0)
        self.machine.note_solver(False, 1.05)
        self.assertEqual(self.machine.mode, RACE)
        self.machine.note_solver(False, 1.10)
        self.assertEqual(self.machine.mode, SAFE_DECEL)
        required = int(self.config["startup"]["mpcc_valid_solutions_required"])
        for index in range(required):
            self.machine.note_solver(True, 1.2 + index * 0.05)
        self.assertEqual(self.machine.mode, MPCC_RAMP)


if __name__ == "__main__":
    unittest.main()
