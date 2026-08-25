#!/usr/bin/env python3
"""Deterministic tests for the V5 Chaoche longitudinal follow policy."""

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from follow_speed_controller import FollowSpeedConfig, FollowSpeedController


class FollowSpeedControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = FollowSpeedController(FollowSpeedConfig())

    def test_far_moving_target_does_not_force_slow_crawl(self) -> None:
        cap, details = self.controller.update(1.0, 1.8, 1.5, 1.5, 7, True)
        self.assertGreater(cap, 2.0)
        self.assertLessEqual(cap, 3.0)
        self.assertEqual(details["follow_reason"], "gap_and_target_speed")

    def test_close_target_is_hard_stop(self) -> None:
        self.controller.update(1.0, 1.5, 1.5, 1.2, 7, True)
        cap, details = self.controller.update(1.1, 0.30, 1.0, 1.2, 7, True)
        self.assertEqual(cap, 0.0)
        self.assertEqual(details["follow_reason"], "hard_stop_gap")

    def test_static_obstacle_is_approached_then_stopped(self) -> None:
        approach, _ = self.controller.update(1.0, 2.0, 0.6, 0.0, 1000000001, True)
        self.assertGreater(approach, 0.6)
        stop, _ = self.controller.update(1.1, 0.34, 0.7, 0.0, 1000000001, True)
        self.assertEqual(stop, 0.0)

    def test_one_dropout_decelerates_instead_of_releasing_global_speed(self) -> None:
        initial, _ = self.controller.update(1.0, 1.8, 1.5, 1.5, 9, True)
        dropped, details = self.controller.update(1.1, None, None, None, None, False)
        self.assertLess(dropped, initial)
        self.assertGreater(dropped, 0.0)
        self.assertEqual(details["follow_reason"], "target_state_stale_deceleration")


if __name__ == "__main__":
    unittest.main()
