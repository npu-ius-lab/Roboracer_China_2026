#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "python"))

from f1tenth_v5_jubu import OvertakePlanner, PeriodicTrack, PlannerConfig, VehicleState


def circle_track(width_left=0.60, width_right=0.60, radius=5.0, samples=240):
    angles = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    s = radius * angles
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    yaw = angles + 0.5 * math.pi
    return PeriodicTrack(
        s=s,
        x=x,
        y=y,
        yaw=yaw,
        width_left=np.full(samples, width_left),
        width_right=np.full(samples, width_right),
        curvature=np.full(samples, 1.0 / radius),
        speed=np.full(samples, 2.0),
    )


def state_at(track, s, ey=0.0, speed=0.0, target_id=-1):
    sample = track.sample(s)
    normal = np.array([-math.sin(sample.yaw), math.cos(sample.yaw)])
    tangent = np.array([math.cos(sample.yaw), math.sin(sample.yaw)])
    position = np.array([sample.x, sample.y]) + ey * normal
    velocity = speed * tangent
    return VehicleState(
        x=float(position[0]),
        y=float(position[1]),
        yaw=sample.yaw,
        vx=float(velocity[0]),
        vy=float(velocity[1]),
        target_id=target_id,
    )


class TrackTest(unittest.TestCase):
    def test_projection_sign_and_periodic_delta(self):
        track = circle_track()
        left = state_at(track, 1.0, ey=0.25)
        projection = track.project(left.x, left.y)
        self.assertAlmostEqual(projection.ey, 0.25, places=3)
        self.assertAlmostEqual(track.forward_delta(track.length - 0.2, 0.3), 0.5, places=6)
        self.assertAlmostEqual(track.signed_delta(0.3, track.length - 0.2), -0.5, places=6)

    def test_progress_constrained_projection_avoids_parallel_lane_jump(self):
        x = np.asarray([0.0, 1.0, 2.0, 3.0, 3.0, 2.0, 1.0, 0.0])
        y = np.asarray([0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2])
        s = np.arange(len(x), dtype=float)
        yaw = np.asarray([0.0, 0.0, 0.0, math.pi / 2.0, math.pi, math.pi, math.pi, -math.pi / 2.0])
        track = PeriodicTrack(
            s=s,
            x=x,
            y=y,
            yaw=yaw,
            width_left=np.full(len(x), 0.5),
            width_right=np.full(len(x), 0.5),
        )
        global_projection = track.project(1.5, 0.11)
        constrained = track.project_near(1.5, 0.11, reference_s=1.5, maximum_progress_delta=0.8)
        self.assertGreater(global_projection.s, 4.0)
        self.assertAlmostEqual(constrained.s, 1.5, places=6)


class OvertakePlannerTest(unittest.TestCase):
    def setUp(self):
        self.track = circle_track()
        self.planner = OvertakePlanner(self.track, PlannerConfig(maximum_path_curvature=3.5))

    def test_ttc_and_preferred_left_candidate(self):
        ego = state_at(self.track, 2.0, speed=2.2)
        opponent = state_at(self.track, 4.8, speed=1.0, target_id=7)
        result = self.planner.evaluate(ego, opponent)
        self.assertTrue(result.relevant)
        self.assertTrue(result.risk)
        self.assertAlmostEqual(result.closing_speed, 1.2, places=2)
        self.assertTrue(math.isfinite(result.ttc))
        self.assertTrue(result.left.feasible, result.left.reason)
        self.assertTrue(result.right.feasible, result.right.reason)
        self.assertIs(type(result.left.feasible), bool)
        self.assertEqual(result.selected_side, "left")

    def test_candidate_is_smooth_and_returns_to_raceline(self):
        result = self.planner.evaluate(
            state_at(self.track, 1.0, ey=0.05, speed=2.0),
            state_at(self.track, 4.0, speed=0.9),
        )
        candidate = result.left
        self.assertTrue(candidate.feasible, candidate.reason)
        self.assertAlmostEqual(candidate.ey[0], result.ego.ey, places=10)
        self.assertAlmostEqual(candidate.ey[-1], 0.0, places=8)
        self.assertLess(abs(candidate.ey[1] - candidate.ey[0]), 0.01)
        self.assertLess(abs(candidate.ey[-1] - candidate.ey[-2]), 0.01)
        self.assertLessEqual(candidate.maximum_curvature, self.planner.config.maximum_path_curvature)

    def test_entry_speed_phase_delays_return_for_moving_opponent(self):
        track = circle_track(width_left=1.5, width_right=1.5)
        common = dict(
            minimum_plan_speed=2.2,
            maximum_plan_speed=4.0,
            pass_speed_advantage=1.8,
            nominal_lane_change_length=2.6,
            horizon=18.0,
            maximum_candidate_length=25.5,
            maximum_path_curvature=3.5,
        )
        no_entry_phase = OvertakePlanner(track, PlannerConfig(**common)).evaluate(
            state_at(track, 2.0, speed=2.2),
            state_at(track, 5.5, speed=2.2),
        ).left
        with_entry_phase = OvertakePlanner(
            track,
            PlannerConfig(
                **common,
                entry_plan_speed=3.0,
                entry_plan_duration=0.9,
            ),
        ).evaluate(
            state_at(track, 2.0, speed=2.2),
            state_at(track, 5.5, speed=2.2),
        ).left
        with_measured_progress_scale = OvertakePlanner(
            track,
            PlannerConfig(
                **common,
                entry_plan_speed=3.0,
                entry_plan_duration=0.9,
                longitudinal_progress_speed_scale=0.75,
            ),
        ).evaluate(
            state_at(track, 2.0, speed=2.2),
            state_at(track, 5.5, speed=2.2),
        ).left
        self.assertTrue(no_entry_phase.feasible, no_entry_phase.reason)
        self.assertTrue(with_entry_phase.feasible, with_entry_phase.reason)
        self.assertTrue(
            with_measured_progress_scale.feasible,
            with_measured_progress_scale.reason,
        )
        no_entry_hold_end = np.flatnonzero(
            np.isclose(no_entry_phase.ey, no_entry_phase.target_offset, atol=1e-8)
        )[-1]
        with_entry_hold_end = np.flatnonzero(
            np.isclose(with_entry_phase.ey, with_entry_phase.target_offset, atol=1e-8)
        )[-1]
        measured_progress_hold_end = np.flatnonzero(
            np.isclose(
                with_measured_progress_scale.ey,
                with_measured_progress_scale.target_offset,
                atol=1e-8,
            )
        )[-1]
        self.assertGreater(with_entry_hold_end, no_entry_hold_end)
        self.assertGreater(measured_progress_hold_end, with_entry_hold_end)

    def test_prediction_speed_floor_handles_startup_zero_twist(self):
        track = circle_track(width_left=1.5, width_right=1.5)
        common = dict(
            minimum_plan_speed=2.2,
            maximum_plan_speed=4.5,
            pass_speed_advantage=2.3,
            nominal_lane_change_length=2.2,
            horizon=18.0,
            maximum_candidate_length=25.5,
            maximum_path_curvature=3.5,
            entry_plan_speed=3.5,
            entry_plan_duration=0.65,
        )
        measured_zero = OvertakePlanner(track, PlannerConfig(**common)).evaluate(
            state_at(track, 2.0, speed=2.2),
            state_at(track, 5.0, speed=0.0),
        ).left
        startup_safe = OvertakePlanner(
            track,
            PlannerConfig(**common, minimum_opponent_prediction_speed=2.2),
        ).evaluate(
            state_at(track, 2.0, speed=2.2),
            state_at(track, 5.0, speed=0.0),
        ).left
        self.assertTrue(measured_zero.feasible, measured_zero.reason)
        self.assertTrue(startup_safe.feasible, startup_safe.reason)
        measured_zero_hold_end = np.flatnonzero(
            np.isclose(measured_zero.ey, measured_zero.target_offset, atol=1e-8)
        )[-1]
        startup_safe_hold_end = np.flatnonzero(
            np.isclose(startup_safe.ey, startup_safe.target_offset, atol=1e-8)
        )[-1]
        self.assertGreater(startup_safe_hold_end, measured_zero_hold_end)

    def test_narrow_corridor_rejects_both_sides(self):
        track = circle_track(width_left=0.34, width_right=0.34)
        planner = OvertakePlanner(track, PlannerConfig(maximum_path_curvature=4.0))
        result = planner.evaluate(state_at(track, 2.0, speed=2.0), state_at(track, 4.7, speed=0.8))
        self.assertFalse(result.left.feasible)
        self.assertFalse(result.right.feasible)
        self.assertEqual(result.left.reason, "track_corridor_violation")
        self.assertEqual(result.selected_side, "none")

    def test_close_opponent_rejected_before_lane_change(self):
        result = self.planner.evaluate(
            state_at(self.track, 2.0, speed=2.5),
            state_at(self.track, 2.9, speed=0.5),
        )
        self.assertEqual(result.left.reason, "insufficient_lane_change_distance")
        self.assertEqual(result.right.reason, "insufficient_lane_change_distance")
        self.assertEqual(result.selected_side, "none")

    def test_rear_opponent_is_not_relevant(self):
        result = self.planner.evaluate(
            state_at(self.track, 5.0, speed=2.0),
            state_at(self.track, 4.0, speed=2.5),
        )
        self.assertFalse(result.relevant)
        self.assertEqual(result.selected_side, "none")

    def test_no_trigger_does_not_select_candidate(self):
        config = PlannerConfig(trigger_distance=1.0, trigger_ttc=1.0, maximum_path_curvature=3.5)
        planner = OvertakePlanner(self.track, config)
        result = planner.evaluate(
            state_at(self.track, 2.0, speed=1.0),
            state_at(self.track, 6.0, speed=1.0),
        )
        self.assertTrue(result.relevant)
        self.assertFalse(result.risk)
        self.assertEqual(result.reason, "no_overtake_trigger")
        self.assertEqual(result.selected_side, "none")

    def test_speed_limited_pass_is_rejected(self):
        config = PlannerConfig(
            maximum_plan_speed=2.0,
            minimum_closing_speed=0.15,
            maximum_path_curvature=3.5,
        )
        planner = OvertakePlanner(self.track, config)
        result = planner.evaluate(
            state_at(self.track, 2.0, speed=2.0),
            state_at(self.track, 4.5, speed=1.95),
        )
        self.assertEqual(result.left.reason, "insufficient_pass_speed_advantage")
        self.assertEqual(result.selected_side, "none")

    def test_high_track_curvature_is_not_a_pass_zone(self):
        planner = OvertakePlanner(
            self.track,
            PlannerConfig(
                maximum_path_curvature=3.5,
                maximum_track_curvature_for_pass=0.15,
            ),
        )
        result = planner.evaluate(
            state_at(self.track, 2.0, speed=2.5),
            state_at(self.track, 4.8, speed=1.0),
        )
        self.assertEqual(result.left.reason, "track_curvature_outside_pass_zone")
        self.assertEqual(result.right.reason, "track_curvature_outside_pass_zone")
        self.assertEqual(result.selected_side, "none")

    def test_tracking_clearance_gate_rejects_marginal_corridor(self):
        planner = OvertakePlanner(
            self.track,
            PlannerConfig(
                maximum_path_curvature=3.5,
                minimum_boundary_clearance_for_pass=0.30,
            ),
        )
        result = planner.evaluate(
            state_at(self.track, 2.0, speed=2.5),
            state_at(self.track, 4.8, speed=1.0),
        )
        self.assertEqual(result.left.reason, "insufficient_tracking_clearance")
        self.assertEqual(result.right.reason, "insufficient_tracking_clearance")
        self.assertEqual(result.selected_side, "none")


if __name__ == "__main__":
    unittest.main()
