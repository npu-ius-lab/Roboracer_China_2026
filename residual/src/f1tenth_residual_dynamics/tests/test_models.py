import tempfile
import unittest
from pathlib import Path

import numpy as np

from residual_dynamics.nominal_model import NominalBicycleModel, load_vehicle_config
from residual_dynamics.residual_model import ResidualModel


PACKAGE = Path(__file__).resolve().parents[1]


class ModelTests(unittest.TestCase):
    def test_nominal_straight_accelerates(self):
        model = NominalBicycleModel.from_config(
            load_vehicle_config(PACKAGE / "config/vehicle_nominal.yaml"))
        state = np.zeros(4)
        next_state = model.step(state, np.asarray([1.0, 0.0]), 0.05)
        self.assertGreater(next_state[0], 0.0)
        self.assertAlmostEqual(next_state[1], 0.0, places=8)

    def test_residual_round_trip_and_bounds(self):
        rng = np.random.RandomState(4)
        x = np.column_stack((np.ones(200), rng.randn(200, 3)))
        y = np.column_stack((0.3 * x[:, 1], -0.2 * x[:, 2], 0.1 * x[:, 3]))
        model = ResidualModel.fit(x, y, ["bias", "a", "b", "c"],
                                  output_limits=[0.2, 0.2, 0.2])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.yaml"
            model.save(path)
            loaded = ResidualModel.load(path)
            prediction = loaded.predict(x[:5])
        self.assertTrue(np.all(np.abs(prediction) <= 0.2 + 1.0e-12))
        np.testing.assert_allclose(prediction, model.predict(x[:5]))


if __name__ == "__main__":
    unittest.main()
