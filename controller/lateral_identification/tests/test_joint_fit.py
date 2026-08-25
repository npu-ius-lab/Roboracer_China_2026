import unittest

import numpy as np

from lateral_identification.fit_core import (
    evaluate_steering_actuator,
    first_order_response,
    fit_steering_actuator_joint,
    prepare_actuator_run,
)


def _synthetic_run(name, order, speed, seed):
    dt = 0.02
    dwell = 0.90
    cycles = 5
    levels = np.asarray(order * cycles, dtype=float)
    times = np.arange(0.0, len(levels) * dwell, dt)
    indices = np.minimum((times / dwell).astype(int), len(levels) - 1)
    command = levels[indices]
    gain, bias, tau, delay = 1.15, -0.015, 0.075, 0.090
    delta = first_order_response(
        times, times, command, gain, tau, delay, bias, bias
    )
    rng = np.random.default_rng(seed)
    vx = np.full_like(times, speed)
    yaw_rate = vx * np.tan(delta) / 0.320 + rng.normal(0.0, 0.006, len(times))
    return prepare_actuator_run(times, command, vx, yaw_rate, name=name)


class JointActuatorFitTest(unittest.TestCase):
    def test_recovers_grouped_fopdt_and_generalizes(self):
        levels_a = [0.0, 0.10, -0.04, 0.14, -0.07, 0.04, -0.14, 0.07, -0.10]
        levels_b = [0.0, -0.07, 0.14, -0.04, 0.10, -0.14, 0.04, -0.10, 0.07]
        run_a = _synthetic_run("a", levels_a, 0.60, 7)
        run_b = _synthetic_run("b", levels_b, 1.00, 11)
        self.assertTrue(run_a.static_platform_fit)
        self.assertTrue(run_b.static_platform_fit)
        fit, diagnostics = fit_steering_actuator_joint(
            [run_a, run_b], delay_grid=np.arange(0.05, 0.131, 0.01)
        )
        self.assertAlmostEqual(fit.gain, 1.15, delta=0.06)
        self.assertAlmostEqual(fit.bias, -0.015, delta=0.008)
        self.assertAlmostEqual(fit.delay, 0.090, delta=0.021)
        self.assertAlmostEqual(fit.tau, 0.075, delta=0.035)
        self.assertGreater(fit.dynamic_r2, 0.95)
        self.assertEqual(len(diagnostics["per_run"]), 2)

        held_out, _ = evaluate_steering_actuator(run_b, fit)
        self.assertLess(held_out["rmse_rad"], 0.02)
        self.assertGreater(held_out["r2"], 0.93)


if __name__ == "__main__":
    unittest.main()
