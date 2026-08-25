#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from follow_speed_controller import FollowSpeedConfig, FollowSpeedController


class FollowTest(unittest.TestCase):
    def test_short_dropout_keeps_last_safe_cap(self):
        controller = FollowSpeedController(FollowSpeedConfig())
        before, _ = controller.update(10.0, 1.5, 1.2, 0.7, 9, True)
        during, details = controller.update(10.2, None, None, None, None, False)
        self.assertAlmostEqual(during, before)
        self.assertEqual(details["follow_reason"], "short_target_dropout_hold")

    def test_long_dropout_decelerates(self):
        controller = FollowSpeedController(FollowSpeedConfig())
        before, _ = controller.update(10.0, 1.5, 1.2, 0.7, 9, True)
        after, details = controller.update(10.8, None, None, None, None, False)
        self.assertLess(after, before)
        self.assertEqual(details["follow_reason"], "target_state_stale_deceleration")

    def test_close_slow_target_still_hard_stops(self):
        controller = FollowSpeedController(FollowSpeedConfig())
        cap, details = controller.update(10.0, 0.3, 0.8, 0.2, 9, True)
        self.assertEqual(cap, 0.0)
        self.assertEqual(details["follow_reason"], "hard_stop_gap")


if __name__ == "__main__":
    unittest.main()
