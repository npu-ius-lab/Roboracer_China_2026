#!/usr/bin/env python3
import math
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle


ROOT = Path(__file__).resolve().parents[1]
TRACK = ROOT / "data/tracks/virtual_track/raceline.csv"


class TestPeriodicTrack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = PeriodicTrack(TRACK)

    def test_position_and_tangent_are_periodic(self):
        for offset in (0.0, 0.13, 3.7):
            p0 = np.asarray(self.track.position(offset), dtype=float)
            p1 = np.asarray(self.track.position(offset + self.track.length), dtype=float)
            self.assertLess(np.linalg.norm(p0 - p1), 1.0e-9)
            tangent_error = wrap_angle(
                self.track.tangent(offset + self.track.length) - self.track.tangent(offset)
            )
            self.assertLess(abs(float(tangent_error)), 1.0e-9)

    def test_width_and_wrapping(self):
        samples = np.asarray([-0.3, 0.0, 1.0, self.track.length + 0.2])
        self.assertTrue(np.all(self.track.width_left(samples) > 0.0))
        self.assertTrue(np.all(self.track.width_right(samples) > 0.0))
        self.assertAlmostEqual(
            float(self.track.width_left(-0.3)),
            float(self.track.width_left(self.track.length - 0.3)),
            places=9,
        )

    def test_projection_of_spline_points(self):
        rng = np.random.default_rng(7)
        guess = 0.0
        for s in np.sort(rng.uniform(0.0, self.track.length, 100)):
            x, y = self.track.position(float(s))
            projection = self.track.project(float(x), float(y), guess)
            self.assertLess(projection.distance, 2.0e-6)
            self.assertLess(abs(projection.e_contour), 2.0e-6)
            guess = projection.s

    def test_projection_lifts_across_seam(self):
        s = self.track.length + 0.08
        x, y = self.track.position(s)
        projection = self.track.project(float(x), float(y), self.track.length - 0.02)
        self.assertGreater(projection.s, self.track.length)
        self.assertLess(abs(projection.s - s), 1.0e-5)

    def test_spline_curvature_matches_csv_reasonably(self):
        s = self.track.s_nodes
        difference = np.abs(self.track.curvature(s) - self.track.csv_curvature(s))
        # CSV curvature may have independent smoothing, so this is a cross-check,
        # not a byte-for-byte equality assertion.
        self.assertLess(float(np.quantile(difference, 0.95)), 0.35)
        self.assertTrue(math.isfinite(float(np.max(difference))))

    def test_race_track_does_not_require_csv_heading_curvature_speed_or_accel(self):
        minimal_columns = (
            "s_m", "x_m", "y_m", "w_tr_right_m", "w_tr_left_m"
        )
        with TRACK.open(newline="", encoding="utf-8") as source:
            rows = list(csv.DictReader(source))
        with tempfile.TemporaryDirectory() as directory:
            minimal = Path(directory) / "minimal_track.csv"
            with minimal.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=minimal_columns)
                writer.writeheader()
                writer.writerows(
                    {name: row[name] for name in minimal_columns} for row in rows
                )
            speed = {
                "enabled": True,
                "min_speed_mps": 0.2,
                "max_speed_mps": 3.0,
                "wheelbase_m": 0.32,
                "max_steer_rad": 0.55,
                "max_steer_rate_radps": 5.5,
                "max_accel_mps2": 2.4,
                "max_decel_mps2": 4.0,
                "lateral_accel_limit_mps2": 3.2,
            }
            track = PeriodicTrack(minimal, speed_planning=speed)
            self.assertTrue(np.all(np.isfinite(track.curvature(track.s_nodes))))
            self.assertTrue(np.all(np.isfinite(track.speed_nodes)))


if __name__ == "__main__":
    unittest.main()
