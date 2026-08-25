import math
import unittest

from lateral_identification.track_guided_core import (
    PROFILES,
    SafeExcitationGenerator,
    TrackGuidedController,
)


class _Projection:
    def __init__(self, s, e_contour, psi_ref=0.0):
        self.s = s
        self.e_contour = e_contour
        self.psi_ref = psi_ref


class _StraightTrack:
    length = 100.0

    def project(self, x, y, _guess):
        return _Projection(x, y)

    def width_left(self, _s):
        return 0.60

    def width_right(self, _s):
        return 0.60

    def position(self, s):
        return s, 0.0

    def curvature(self, _s):
        return 0.0


class SafeExcitationTest(unittest.TestCase):
    def test_clock_pauses_and_sequence_is_bounded(self):
        profile = PROFILES["track_prbs_060"]
        signs = SafeExcitationGenerator._PRBS_SIGNS
        dwell = SafeExcitationGenerator._PRBS_DWELL
        weighted_mean = sum(
            signs[index] * dwell[index % len(dwell)] for index in range(len(signs))
        ) / sum(dwell[index % len(dwell)] for index in range(len(signs)))
        self.assertAlmostEqual(weighted_mean, 0.0)
        generator = SafeExcitationGenerator(profile)
        self.assertEqual(generator.step(1.0, False), 0.0)
        self.assertEqual(generator.active_time, 0.0)
        self.assertEqual(generator.enabled_time, 0.0)

        values = [generator.step(0.02, True) for _ in range(180)]
        self.assertTrue(any(value > 0.0 for value in values))
        self.assertTrue(any(value < 0.0 for value in values))
        self.assertLessEqual(max(abs(value) for value in values), profile.excitation_rad)
        active_before_pause = generator.active_time
        self.assertEqual(generator.step(0.08, False), 0.0)
        self.assertAlmostEqual(generator.active_time, active_before_pause)

    def test_robust_rollout_is_finite_and_excitation_is_more_demanding(self):
        controller = TrackGuidedController(_StraightTrack(), PROFILES["track_static_060"])
        baseline = controller.command(0.0, 0.0, 0.0, 0.60, 0.0, None,
                                      allow_excitation=False)
        self.assertTrue(controller.excitation_allowed(baseline))
        no_excitation = controller.predicted_minimum_margin(
            0.0, 0.0, 0.0, 0.60, 0.0, 0.0, 0.0, future_excitation=0.0
        )
        with_excitation = controller.predicted_minimum_margin(
            0.0, 0.0, 0.0, 0.60, 0.0, 0.0, 0.0, future_excitation=0.14
        )
        self.assertTrue(math.isfinite(no_excitation))
        self.assertTrue(math.isfinite(with_excitation))
        self.assertLess(with_excitation, no_excitation)


if __name__ == "__main__":
    unittest.main()
