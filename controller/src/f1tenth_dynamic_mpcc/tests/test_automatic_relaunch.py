#!/usr/bin/env python3

import math
from pathlib import Path
import unittest

from f1tenth_dynamic_mpcc.automatic_relaunch import (
    CarryDetector,
    GroundStabilityDetector,
    HandoffGate,
    RelaunchMotionSample,
    WheelReleaseDetector,
    assess_forward_recovery_placement,
    assess_heading_aware_placement,
    blend_scalar,
    is_race_start_candidate,
    planned_recovery_speed,
)
from f1tenth_dynamic_mpcc.track_model import PeriodicTrack


PACKAGE = Path(__file__).resolve().parents[1]
TRACK = PACKAGE / "data/tracks/racelinev3_5mps_arc_straight_std3_candidate/raceline.csv"


def sample(t, *, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0,
           yaw=0.0, wheel=0.0, gyro=0.0):
    return RelaunchMotionSample(t, x, y, z, roll, pitch, yaw, wheel, gyro)


class AutomaticRelaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = PeriodicTrack(TRACK)

    def placement(self, s=20.0, heading=0.0, lateral=0.0, speed=0.0):
        x, y = self.track.position(s)
        yaw = float(self.track.tangent(s))
        x -= math.sin(yaw) * lateral
        y += math.cos(yaw) * lateral
        return assess_heading_aware_placement(
            self.track, 0.24, float(x), float(y), yaw + heading, speed,
            boundary_buffer_m=0.08,
            maximum_heading_error_rad=0.60,
            maximum_stationary_speed_mps=0.12,
        )

    def recovery(self, s=20.0, heading=0.0, lateral=0.0, speed=0.0):
        x, y = self.track.position(s)
        yaw = float(self.track.tangent(s))
        x -= math.sin(yaw) * lateral
        y += math.cos(yaw) * lateral
        return assess_forward_recovery_placement(
            self.track, 0.24, float(x), float(y), yaw + heading, speed,
            boundary_buffer_m=0.08,
            maximum_direct_heading_error_rad=0.60,
            maximum_pp_angle_rad=math.radians(60.0),
            maximum_stationary_speed_mps=0.12,
            lookahead_minimum_m=0.45,
            lookahead_maximum_m=1.20,
            minimum_path_margin_m=0.05,
        )

    def test_normal_driving_vertical_motion_is_not_carry(self):
        detector = CarryDetector(confirmation_samples=2)
        result = None
        for index in range(12):
            result = detector.update(sample(
                index * 0.03, x=index * 0.09,
                z=0.12 * math.sin(index / 3.0), wheel=3.0,
            ))
        self.assertFalse(result.detected)

    def test_stationary_wheels_and_lift_detect_carry(self):
        detector = CarryDetector(confirmation_samples=2)
        detected = False
        for index in range(12):
            result = detector.update(sample(
                index * 0.03,
                x=max(0, index - 6) * 0.04,
                z=max(0, index - 6) * 0.035,
                roll=math.radians(max(0, index - 6) * 2.0),
                wheel=0.0,
            ))
            detected = detected or result.detected
        self.assertTrue(detected)

    def test_ground_stability_requires_full_window(self):
        placement = self.placement()
        detector = GroundStabilityDetector(window_s=0.30)
        ready = False
        x, y = self.track.position(20.0)
        yaw = float(self.track.tangent(20.0))
        for index in range(12):
            ready, reason, _ = detector.update(sample(
                index * 0.03,
                x=float(x) + 0.002 * (index % 2),
                y=float(y), z=0.003 * (index % 2),
                roll=math.radians(1.0), pitch=math.radians(2.0),
                yaw=yaw + 0.002 * (index % 2), wheel=0.0, gyro=0.03,
            ), placement, True)
        self.assertTrue(ready, reason)

    def test_ground_stability_rejects_lifted_pose(self):
        placement = self.placement()
        detector = GroundStabilityDetector(window_s=0.30)
        ready = False
        for index in range(12):
            ready, reason, _ = detector.update(sample(
                index * 0.03, z=index * 0.02,
                roll=math.radians(10.0), wheel=0.0,
            ), placement, True)
        self.assertFalse(ready)
        self.assertIn(reason, {"vertical_motion", "vehicle_not_level"})

    def test_release_needs_three_wheel_samples(self):
        detector = WheelReleaseDetector(0.08, 3)
        self.assertFalse(detector.update(0.09))
        self.assertFalse(detector.update(0.10))
        self.assertTrue(detector.update(0.11))

    def test_heading_aware_placement_rejects_reverse(self):
        result = self.placement(28.0, heading=math.pi)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "heading_not_aligned_with_track_direction")

    def test_heading_aware_global_s_at_checkpoints(self):
        for source_s in (1.0, 18.2, 28.8, 36.2, 44.8, 70.0, 87.0):
            with self.subTest(source_s=source_s):
                result = self.placement(source_s, lateral=0.03)
                self.assertTrue(result.valid, result.reason)
                error = (
                    result.projection.s_wrapped - source_s + self.track.length / 2.0
                ) % self.track.length - self.track.length / 2.0
                self.assertLess(abs(error), 0.12)

    def test_forward_pp_accepts_recorded_hairpin_relocation(self):
        # Recorded 2026-08-21 relocation: nearest tangent differs by 39.6 deg,
        # while the car faces a safe forward raceline point almost directly.
        result = assess_forward_recovery_placement(
            self.track, 0.24,
            -2.415344167602627, 3.4712056285822555,
            math.radians(107.74649325729017), 0.0,
            boundary_buffer_m=0.08,
            maximum_direct_heading_error_rad=0.60,
            maximum_pp_angle_rad=math.radians(60.0),
            maximum_stationary_speed_mps=0.12,
            lookahead_minimum_m=0.45,
            lookahead_maximum_m=1.20,
            minimum_path_margin_m=0.05,
        )
        self.assertTrue(result.placement.valid, result.placement.reason)
        self.assertEqual(result.placement.reason, "placement_valid_forward_pp")
        self.assertTrue(result.recovery_required)
        self.assertLess(abs(result.bearing_error_rad), math.radians(20.0))
        self.assertGreater(result.path_minimum_margin_m, 0.30)

    def test_forward_pp_rejects_reverse_placement(self):
        result = self.recovery(28.0, heading=math.pi)
        self.assertFalse(result.placement.valid)
        self.assertEqual(result.placement.reason, "no_safe_forward_pp_target")

    def test_forward_pp_accepts_direct_start(self):
        result = self.recovery(18.2, heading=math.radians(8.0), lateral=0.05)
        self.assertTrue(result.placement.valid, result.placement.reason)
        self.assertEqual(result.placement.reason, "placement_valid_direct")
        self.assertFalse(result.recovery_required)

    def test_handoff_requires_consecutive_speed_and_steering_matches(self):
        gate = HandoffGate(1.8, 0.08, 3)
        self.assertFalse(gate.update(-0.10, 1.79, -0.10))
        self.assertFalse(gate.update(-0.10, 1.79999995, -0.03))
        self.assertFalse(gate.update(-0.10, 1.90, -0.04))
        self.assertTrue(gate.update(-0.10, 2.00, -0.08))
        self.assertEqual(gate.matching_samples, 3)

    def test_handoff_mismatch_resets_consecutive_count(self):
        gate = HandoffGate(1.8, 0.08, 2)
        self.assertFalse(gate.update(0.10, 2.0, 0.12))
        self.assertFalse(gate.update(0.10, 2.0, -0.10))
        self.assertEqual(gate.matching_samples, 0)
        self.assertFalse(gate.update(0.10, 2.0, 0.11))
        self.assertTrue(gate.update(0.10, 2.0, 0.12))

    def test_handoff_blend_is_clamped_and_continuous(self):
        self.assertAlmostEqual(blend_scalar(2.0, 1.8, -1.0), 2.0)
        self.assertAlmostEqual(blend_scalar(2.0, 1.8, 0.5), 1.9)
        self.assertAlmostEqual(blend_scalar(2.0, 1.8, 2.0), 1.8)

    def test_adaptive_handoff_accepts_low_speed_bend(self):
        gate = HandoffGate(1.8, 0.08, 3)
        self.assertFalse(gate.update(0.20, 1.20, 0.22, 1.10))
        self.assertFalse(gate.update(0.20, 1.20, 0.21, 1.10))
        self.assertTrue(gate.update(0.20, 1.20, 0.20, 1.10))
        self.assertAlmostEqual(gate.active_minimum_speed_mps, 1.10)

    def test_map_origin_aligned_pose_selects_race_start(self):
        plan = self.recovery(0.50, lateral=0.03)
        x, y = self.track.position(0.50)
        self.assertTrue(is_race_start_candidate(
            plan, float(x), float(y), self.track.length,
            radius_m=1.20, s_tolerance_m=1.50,
        ))

    def test_non_grid_pose_does_not_select_race_start(self):
        plan = self.recovery(55.0)
        x, y = self.track.position(55.0)
        self.assertFalse(is_race_start_candidate(
            plan, float(x), float(y), self.track.length,
            radius_m=1.20, s_tolerance_m=1.50,
        ))

    def test_recovery_speed_slows_before_recorded_hairpin(self):
        speed = planned_recovery_speed(
            self.track, 60.0, 2.0,
            minimum_speed_mps=1.20,
            preview_distance_m=3.0,
            lateral_acceleration_limit_mps2=2.0,
        )
        self.assertGreaterEqual(speed, 1.20)
        self.assertLess(speed, 1.55)

    def test_recovery_speed_reduces_large_heading_error(self):
        speed = planned_recovery_speed(
            self.track, 55.0, 2.0,
            minimum_speed_mps=1.20,
            preview_distance_m=3.0,
            lateral_acceleration_limit_mps2=2.0,
            heading_error_rad=math.radians(20.0),
        )
        self.assertLessEqual(speed, 1.61)

    def test_race_grid_straight_allows_three_mps(self):
        speed = planned_recovery_speed(
            self.track, 0.0, 3.0,
            minimum_speed_mps=1.20,
            preview_distance_m=3.0,
            lateral_acceleration_limit_mps2=2.0,
        )
        self.assertAlmostEqual(speed, 3.0, places=2)


if __name__ == "__main__":
    unittest.main()
