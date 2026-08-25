#!/usr/bin/env python3
import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.acados_solver import NP, stage_parameters
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


ROOT = Path(__file__).resolve().parents[1]


class TestStageParameters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = PeriodicTrack(ROOT / "data/tracks/virtual_track/raceline.csv")

    def test_geometry_schedule_and_wrap(self):
        theta = self.track.length + np.linspace(-0.2, 0.3, 26)
        previous = np.asarray([1.0, 0.1, 1.0])
        values = stage_parameters(
            self.track, theta, previous, 0.22, 5.0, 0.0, 3.0
        )
        self.assertEqual(values.shape, (26, NP))
        self.assertTrue(np.allclose(values[:, 7:10], previous))
        self.assertTrue(np.all(values[:, 4] > 0.0))
        self.assertTrue(np.all(values[:, 5] > 0.0))
        self.assertTrue(np.allclose(values[:, 10], 0.22))
        self.assertTrue(np.allclose(values[:, 13], 3.0))


if __name__ == "__main__":
    unittest.main()
