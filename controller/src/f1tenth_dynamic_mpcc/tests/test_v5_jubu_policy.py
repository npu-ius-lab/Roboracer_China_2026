#!/usr/bin/env python3

import unittest

from f1tenth_dynamic_mpcc.v5_jubu import (
    SpeedPolicy,
    TemporalTargetConfirmation,
    inputs_healthy,
)


class SpeedPolicyTest(unittest.TestCase):
    def setUp(self):
        self.policy = SpeedPolicy()

    def test_free_uses_candidate_cruise_cap(self):
        self.assertEqual(self.policy.cap("FREE", {}, True), 3.0)

    def test_follow_matches_opponent_as_gap_closes(self):
        cap = self.policy.cap(
            "FOLLOW", {"opponent_vs": 1.2, "bumper_gap": 0.75}, True
        )
        self.assertAlmostEqual(cap, 1.2)

    def test_follow_never_exceeds_cruise_or_drops_below_floor(self):
        self.assertEqual(
            self.policy.cap("FOLLOW", {"opponent_vs": 5.0, "bumper_gap": 5.0}, True),
            3.0,
        )
        self.assertEqual(
            self.policy.cap("FOLLOW", {"opponent_vs": 0.0, "bumper_gap": 0.0}, True),
            0.4,
        )

    def test_pass_is_lower_speed_and_abort_is_stop(self):
        self.assertEqual(self.policy.cap("PASS", {}, True), 2.2)
        self.assertEqual(self.policy.cap("RETURN", {}, True), 2.2)
        self.assertEqual(self.policy.cap("ABORT", {}, True), 0.0)
        self.assertEqual(self.policy.cap("FREE", {}, False), 0.0)


class InputHealthTest(unittest.TestCase):
    def test_all_streams_must_be_fresh(self):
        self.assertTrue(inputs_healthy(10.0, 9.9, 9.4, 9.8, 0.3, 0.8, 1.0))
        self.assertFalse(inputs_healthy(10.0, 9.6, 9.4, 9.8, 0.3, 0.8, 1.0))
        self.assertFalse(inputs_healthy(10.0, 9.9, None, 9.8, 0.3, 0.8, 1.0))
        self.assertFalse(inputs_healthy(10.0, 10.1, 9.9, 9.9, 0.3, 0.8, 1.0))


class TargetConfirmationTest(unittest.TestCase):
    def test_unique_samples_and_duration_are_required(self):
        confirmation = TemporalTargetConfirmation(2, 0.15, 0.8)
        self.assertFalse(confirmation.update(7, 1.0, 5.0))
        self.assertFalse(confirmation.update(7, 1.0, 5.2))
        self.assertTrue(confirmation.update(7, 1.2, 5.2))

    def test_identity_change_or_gap_restarts_confirmation(self):
        confirmation = TemporalTargetConfirmation(2, 0.1, 0.5)
        self.assertFalse(confirmation.update(7, 1.0, 5.0))
        self.assertFalse(confirmation.update(8, 1.2, 5.2))
        self.assertTrue(confirmation.update(8, 1.4, 5.4))
        self.assertFalse(confirmation.update(8, 2.0, 6.1))


if __name__ == "__main__":
    unittest.main()
