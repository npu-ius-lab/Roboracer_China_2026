import math
import unittest
from pathlib import Path

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack

from lateral_identification.safety_predictor import AsyncSafetyPredictor
from lateral_identification.track_guided_core import PROFILES, TrackGuidedController


ROOT = Path(__file__).resolve().parents[2]
TRACK = ROOT / "src/f1tenth_dynamic_mpcc/data/tracks/virtual_track/raceline.csv"


class AsyncSafetyPredictorTest(unittest.TestCase):
    def test_spawned_prediction_returns_without_blocking_parent(self):
        track = PeriodicTrack(TRACK)
        profile = PROFILES["track_static_060"]
        x, y = track.position(1.5)
        yaw = float(track.tangent(1.5))
        predictor = AsyncSafetyPredictor(TRACK, profile)
        try:
            submitted = predictor.submit(
                float(x), float(y), yaw, 0.60, 0.0, 1.5, 0.0, 0.14
            )
            self.assertTrue(submitted)
            self.assertFalse(predictor.submit(
                float(x), float(y), yaw, 0.60, 0.0, 1.5, 0.0, -0.14
            ))
            result = predictor.wait(3.0)
            self.assertIsNotNone(result)
            self.assertEqual(result.error, "")
            self.assertTrue(math.isfinite(result.baseline_margin))
            self.assertTrue(math.isfinite(result.excitation_margin))
            self.assertGreater(result.compute_seconds, 0.0)
            controller = TrackGuidedController(track, profile)
            arguments = (float(x), float(y), yaw, 0.60, 0.0, 1.5, 0.0)
            expected_baseline = controller.predicted_minimum_margin(
                *arguments, future_excitation=0.0
            )
            expected_envelope = min(
                controller.predicted_minimum_margin(
                    *arguments, future_excitation=0.14
                ),
                controller.predicted_minimum_margin(
                    *arguments, future_excitation=-0.14
                ),
            )
            self.assertAlmostEqual(result.baseline_margin, expected_baseline)
            self.assertAlmostEqual(result.excitation_margin, expected_envelope)
        finally:
            predictor.close()

    def test_local_projection_matches_full_track_projection(self):
        track = PeriodicTrack(TRACK)
        controller = TrackGuidedController(track, PROFILES["track_prbs_100"])
        for index in range(40):
            s = index * track.length / 40.0
            x, y = track.position(s)
            yaw = float(track.tangent(s))
            offset = 0.45 * math.sin(0.71 * index)
            px = float(x) - math.sin(yaw) * offset
            py = float(y) + math.cos(yaw) * offset
            guess = s + 0.30 * math.sin(0.37 * index)
            exact = track.project(px, py, guess)
            local = controller._local_projection(px, py, guess)
            self.assertAlmostEqual(local.s, exact.s, places=8)
            self.assertAlmostEqual(local.e_contour, exact.e_contour, places=8)


if __name__ == "__main__":
    unittest.main()
