#!/usr/bin/env python3

import unittest

import numpy as np

from lateral_identification.offline_experiments import (
    ResidualModelV1,
    RolloutActuatorModel,
    contiguous_regions,
    delayed_command_for_model,
    fit_static_model,
    leave_one_plateau_out,
    longest_true_duration,
    static_model_value,
)


class TestOfflineExperiments(unittest.TestCase):
    def test_contiguous_regions_and_longest_duration(self):
        mask = np.asarray([False, True, True, False, True, True, True, False])
        regions = contiguous_regions(mask, minimum_samples=2)
        self.assertEqual([(item.start, item.stop) for item in regions], [(1, 3), (4, 7)])
        self.assertAlmostEqual(longest_true_duration(mask, 0.02), 0.06)

    def test_linear_static_model_recovery(self):
        command = np.linspace(-0.18, 0.22, 17)
        target = 1.12 * command - 0.013
        fit = fit_static_model("linear", command, target)
        self.assertAlmostEqual(fit["parameters"]["gain"], 1.12, places=6)
        self.assertAlmostEqual(fit["parameters"]["bias_rad"], -0.013, places=6)
        self.assertLess(fit["rmse_rad"], 1.0e-8)
        self.assertLess(leave_one_plateau_out("linear", command, target), 1.0e-7)

    def test_deadband_model_is_odd_about_bias(self):
        command = np.asarray([-0.10, -0.01, 0.0, 0.01, 0.10])
        values = static_model_value(
            "deadband", np.asarray([1.2, -0.02, 0.03]), command
        )
        self.assertAlmostEqual(values[1], -0.02)
        self.assertAlmostEqual(values[2], -0.02)
        self.assertAlmostEqual(values[3], -0.02)
        self.assertAlmostEqual(values[-1] + values[0], -0.04)

    def test_exact_and_pipeline_command_delay(self):
        times = np.arange(0.0, 0.31, 0.01)
        command = np.where(times >= 0.10, 1.0, 0.0)
        exact = delayed_command_for_model(
            times, command,
            RolloutActuatorModel("exact", 1.0, 0.0, 0.08, 0.10),
        )
        pipeline = delayed_command_for_model(
            times, command,
            RolloutActuatorModel(
                "pipeline", 1.0, 0.0, 0.08, 0.10,
                pipeline_dt=0.05, pipeline_steps=2,
            ),
        )
        self.assertEqual(exact[np.searchsorted(times, 0.19)], 0.0)
        self.assertEqual(exact[np.searchsorted(times, 0.20)], 1.0)
        self.assertEqual(pipeline[np.searchsorted(times, 0.19)], 0.0)
        self.assertEqual(pipeline[np.searchsorted(times, 0.20)], 1.0)

    def test_v1_residual_confidence_gates_out_of_envelope(self):
        coefficients = np.zeros((16, 3))
        coefficients[0, 0] = 2.0
        model = ResidualModelV1(
            tuple(str(index) for index in range(16)),
            np.zeros(16), np.ones(16), coefficients,
            np.asarray([4.0, 8.0, 18.0]), np.full(16, 10.0), 1.5,
        )
        state = np.zeros((1, 7))
        correction, confidence = model.predict(
            state, np.zeros(1), np.zeros(1)
        )
        self.assertAlmostEqual(confidence[0], 1.0)
        self.assertAlmostEqual(correction[0, 0], 2.0)
        state[0, 3] = 200.0
        correction, confidence = model.predict(
            state, np.zeros(1), np.zeros(1)
        )
        self.assertAlmostEqual(confidence[0], 0.0)
        self.assertTrue(np.allclose(correction, 0.0))


if __name__ == "__main__":
    unittest.main()
