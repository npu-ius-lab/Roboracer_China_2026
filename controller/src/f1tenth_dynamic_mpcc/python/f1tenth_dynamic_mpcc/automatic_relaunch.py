"""Pure, testable motion logic for automatic carry-and-relaunch recovery."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Iterable

import numpy as np

from .quick_relaunch import PlacementAssessment
from .track_model import PeriodicTrack, wrap_angle


@dataclass(frozen=True)
class RelaunchMotionSample:
    stamp: float
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    wheel_speed_mps: float
    gyro_xy_radps: float = 0.0


@dataclass(frozen=True)
class MotionWindowMetrics:
    duration_s: float
    median_abs_wheel_speed_mps: float
    z_span_m: float
    xy_span_m: float
    yaw_span_rad: float
    maximum_tilt_rad: float
    maximum_gyro_xy_radps: float


@dataclass(frozen=True)
class CarryAssessment:
    detected: bool
    reason: str
    metrics: MotionWindowMetrics


@dataclass(frozen=True)
class ForwardRecoveryPlan:
    """Safe forward raceline target used to launch into pure pursuit."""

    placement: PlacementAssessment
    target_s: float
    target_x_m: float
    target_y_m: float
    bearing_error_rad: float
    target_heading_error_rad: float
    path_minimum_margin_m: float
    recovery_required: bool


class HandoffGate:
    """Require consecutive speed and steering agreement before MPCC handoff."""

    def __init__(
        self,
        minimum_speed_mps: float = 1.8,
        maximum_steering_difference_rad: float = 0.08,
        required_samples: int = 3,
    ):
        if (minimum_speed_mps <= 0.0 or
                maximum_steering_difference_rad < 0.0 or
                required_samples < 1):
            raise ValueError("invalid MPCC handoff gate configuration")
        self.minimum_speed_mps = float(minimum_speed_mps)
        self.maximum_steering_difference_rad = float(
            maximum_steering_difference_rad
        )
        self.required_samples = int(required_samples)
        self.matching_samples = 0
        self.steering_difference_rad = math.inf
        self.active_minimum_speed_mps = self.minimum_speed_mps

    def reset(self) -> None:
        self.matching_samples = 0
        self.steering_difference_rad = math.inf
        self.active_minimum_speed_mps = self.minimum_speed_mps

    def update(
        self,
        pp_steering_rad: float,
        mpcc_speed_mps: float,
        mpcc_steering_rad: float,
        minimum_speed_mps: float | None = None,
    ) -> bool:
        values = (pp_steering_rad, mpcc_speed_mps, mpcc_steering_rad)
        if not all(math.isfinite(float(value)) for value in values):
            self.reset()
            return False
        threshold = (
            self.minimum_speed_mps
            if minimum_speed_mps is None else float(minimum_speed_mps)
        )
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("MPCC handoff speed threshold must be positive")
        self.active_minimum_speed_mps = threshold
        self.steering_difference_rad = abs(
            float(mpcc_steering_rad) - float(pp_steering_rad)
        )
        # AckermannDrive uses float32 fields, so a commanded 1.80 m/s arrives
        # as approximately 1.79999995. Keep the configured physical threshold
        # while tolerating only that serialization roundoff.
        matches = (
            float(mpcc_speed_mps) + 1.0e-4 >= threshold and
            self.steering_difference_rad
            <= self.maximum_steering_difference_rad
        )
        self.matching_samples = self.matching_samples + 1 if matches else 0
        return self.matching_samples >= self.required_samples


def blend_scalar(start: float, target: float, progress: float) -> float:
    """Linearly blend a command scalar with clamped progress."""
    alpha = max(0.0, min(1.0, float(progress)))
    return (1.0 - alpha) * float(start) + alpha * float(target)


def circular_progress_distance(s_m: float, target_s_m: float, length_m: float) -> float:
    """Shortest absolute distance between two stations on a periodic track."""
    if length_m <= 0.0:
        raise ValueError("track length must be positive")
    return abs((float(s_m) - float(target_s_m) + length_m / 2.0) % length_m
               - length_m / 2.0)


def is_race_start_candidate(
    plan: ForwardRecoveryPlan,
    x_m: float,
    y_m: float,
    track_length_m: float,
    *,
    center_x_m: float = 0.0,
    center_y_m: float = 0.0,
    radius_m: float = 1.2,
    target_s_m: float = 0.0,
    s_tolerance_m: float = 1.5,
    maximum_contour_error_m: float = 0.60,
    maximum_heading_error_rad: float = math.radians(10.0),
    minimum_vehicle_margin_m: float = 0.05,
    minimum_path_margin_m: float = 0.05,
) -> bool:
    """Recognize either lateral start-grid slot near the map origin.

    Grid placement is deliberately not required to lie close to the raceline:
    alternating race starts can be on either side of it.  Footprint and PP-path
    margins remain the physical safety gates.
    """
    placement = plan.placement
    return bool(
        placement.valid and
        not plan.recovery_required and
        math.hypot(float(x_m) - center_x_m, float(y_m) - center_y_m)
        <= radius_m and
        circular_progress_distance(
            placement.projection.s_wrapped, target_s_m, track_length_m
        ) <= s_tolerance_m and
        abs(placement.projection.e_contour) <= maximum_contour_error_m and
        abs(placement.heading_error_rad) <= maximum_heading_error_rad and
        placement.vehicle_margin_m >= minimum_vehicle_margin_m and
        plan.path_minimum_margin_m >= minimum_path_margin_m
    )


def planned_recovery_speed(
    track: PeriodicTrack,
    s_m: float,
    base_speed_mps: float,
    *,
    minimum_speed_mps: float = 1.20,
    preview_distance_m: float = 3.0,
    lateral_acceleration_limit_mps2: float = 2.0,
    contour_error_m: float = 0.0,
    heading_error_rad: float = 0.0,
    contour_soft_m: float = 0.15,
    contour_hard_m: float = 0.45,
    heading_soft_rad: float = math.radians(10.0),
    heading_hard_rad: float = math.radians(30.0),
    preview_samples: int = 25,
) -> float:
    """Cap recovery PP speed using preview geometry and current tracking error."""
    values = (
        s_m, base_speed_mps, minimum_speed_mps, preview_distance_m,
        lateral_acceleration_limit_mps2, contour_error_m, heading_error_rad,
        contour_soft_m, contour_hard_m, heading_soft_rad, heading_hard_rad,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("recovery speed inputs must be finite")
    if (minimum_speed_mps <= 0.0 or base_speed_mps < minimum_speed_mps or
            preview_distance_m <= 0.0 or lateral_acceleration_limit_mps2 <= 0.0 or
            contour_soft_m < 0.0 or contour_hard_m <= contour_soft_m or
            heading_soft_rad < 0.0 or heading_hard_rad <= heading_soft_rad or
            preview_samples < 2):
        raise ValueError("invalid recovery speed configuration")
    station = np.linspace(
        float(s_m), float(s_m) + preview_distance_m, int(preview_samples)
    )
    profile_cap = float(np.min(track.speed_prior(station)))
    maximum_curvature = float(np.max(np.abs(track.curvature(station))))
    curvature_cap = math.sqrt(
        lateral_acceleration_limit_mps2 / max(maximum_curvature, 1.0e-4)
    )

    def severity(value: float, soft: float, hard: float) -> float:
        return max(0.0, min(1.0, (abs(value) - soft) / (hard - soft)))

    error_severity = max(
        severity(contour_error_m, contour_soft_m, contour_hard_m),
        severity(heading_error_rad, heading_soft_rad, heading_hard_rad),
    )
    error_cap = minimum_speed_mps + (1.0 - error_severity) * (
        base_speed_mps - minimum_speed_mps
    )
    return float(max(
        minimum_speed_mps,
        min(base_speed_mps, profile_cap, curvature_cap, error_cap),
    ))


def _maximum_xy_span(samples: Iterable[RelaunchMotionSample]) -> float:
    points = np.asarray([(sample.x_m, sample.y_m) for sample in samples], dtype=float)
    if len(points) < 2:
        return 0.0
    difference = points[:, None, :] - points[None, :, :]
    return float(np.max(np.linalg.norm(difference, axis=2)))


def motion_window_metrics(samples: Iterable[RelaunchMotionSample]) -> MotionWindowMetrics:
    values = tuple(samples)
    if not values:
        return MotionWindowMetrics(0.0, math.inf, math.inf, math.inf,
                                   math.inf, math.inf, math.inf)
    first = values[0]
    yaw_delta = np.asarray(
        [float(wrap_angle(sample.yaw_rad - first.yaw_rad)) for sample in values]
    )
    return MotionWindowMetrics(
        duration_s=max(0.0, values[-1].stamp - first.stamp),
        median_abs_wheel_speed_mps=float(np.median(
            np.abs([sample.wheel_speed_mps for sample in values])
        )),
        z_span_m=float(np.ptp([sample.z_m for sample in values])),
        xy_span_m=_maximum_xy_span(values),
        yaw_span_rad=float(np.ptp(yaw_delta)),
        maximum_tilt_rad=float(max(
            max(abs(sample.roll_rad), abs(sample.pitch_rad)) for sample in values
        )),
        maximum_gyro_xy_radps=float(max(
            abs(sample.gyro_xy_radps) for sample in values
        )),
    )


class CarryDetector:
    """Fuse wheel standstill with LiDAR-odometry lift and pose motion."""

    def __init__(
        self,
        *,
        window_s: float = 0.25,
        maximum_stationary_wheel_speed_mps: float = 0.10,
        minimum_z_span_m: float = 0.10,
        minimum_xy_span_m: float = 0.15,
        minimum_yaw_span_rad: float = 0.20,
        minimum_tilt_rad: float = math.radians(8.0),
        minimum_gyro_xy_radps: float = 0.80,
        confirmation_samples: int = 3,
    ):
        if window_s <= 0.0 or confirmation_samples < 1:
            raise ValueError("invalid carry detector window or confirmation count")
        self.window_s = float(window_s)
        self.maximum_stationary_wheel_speed_mps = float(
            maximum_stationary_wheel_speed_mps
        )
        self.minimum_z_span_m = float(minimum_z_span_m)
        self.minimum_xy_span_m = float(minimum_xy_span_m)
        self.minimum_yaw_span_rad = float(minimum_yaw_span_rad)
        self.minimum_tilt_rad = float(minimum_tilt_rad)
        self.minimum_gyro_xy_radps = float(minimum_gyro_xy_radps)
        self.confirmation_samples = int(confirmation_samples)
        self._samples: Deque[RelaunchMotionSample] = deque()
        self._confirmations = 0

    def reset(self) -> None:
        self._samples.clear()
        self._confirmations = 0

    def update(self, sample: RelaunchMotionSample) -> CarryAssessment:
        self._samples.append(sample)
        while self._samples and self._samples[0].stamp < sample.stamp - self.window_s:
            self._samples.popleft()
        metrics = motion_window_metrics(self._samples)
        wheel_still = (
            metrics.median_abs_wheel_speed_mps
            <= self.maximum_stationary_wheel_speed_mps
        )
        evidence = []
        if metrics.z_span_m >= self.minimum_z_span_m:
            evidence.append("vertical_lift")
        if metrics.xy_span_m >= self.minimum_xy_span_m:
            evidence.append("planar_motion_without_wheels")
        if metrics.yaw_span_rad >= self.minimum_yaw_span_rad:
            evidence.append("yaw_motion_without_wheels")
        if metrics.maximum_tilt_rad >= self.minimum_tilt_rad:
            evidence.append("body_tilt")
        if metrics.maximum_gyro_xy_radps >= self.minimum_gyro_xy_radps:
            evidence.append("imu_tilt_rate")
        enough_history = metrics.duration_s >= 0.75 * self.window_s
        if wheel_still and evidence and enough_history:
            self._confirmations += 1
        else:
            self._confirmations = 0
        detected = self._confirmations >= self.confirmation_samples
        reason = "+".join(evidence) if detected else (
            "waiting_for_confirmation" if evidence and wheel_still else "no_carry"
        )
        return CarryAssessment(detected, reason, metrics)


class GroundStabilityDetector:
    """Require a quiet, level, stationary placement for a short time window."""

    def __init__(
        self,
        *,
        window_s: float = 0.30,
        maximum_z_span_m: float = 0.025,
        maximum_xy_span_m: float = 0.040,
        maximum_yaw_span_rad: float = 0.060,
        maximum_tilt_rad: float = math.radians(6.0),
        maximum_gyro_xy_radps: float = 0.20,
        maximum_wheel_speed_mps: float = 0.10,
    ):
        if window_s <= 0.0:
            raise ValueError("ground stability window must be positive")
        self.window_s = float(window_s)
        self.maximum_z_span_m = float(maximum_z_span_m)
        self.maximum_xy_span_m = float(maximum_xy_span_m)
        self.maximum_yaw_span_rad = float(maximum_yaw_span_rad)
        self.maximum_tilt_rad = float(maximum_tilt_rad)
        self.maximum_gyro_xy_radps = float(maximum_gyro_xy_radps)
        self.maximum_wheel_speed_mps = float(maximum_wheel_speed_mps)
        self._samples: Deque[RelaunchMotionSample] = deque()

    def reset(self) -> None:
        self._samples.clear()

    def update(
        self,
        sample: RelaunchMotionSample,
        placement: PlacementAssessment,
        localization_healthy: bool,
    ) -> tuple[bool, str, MotionWindowMetrics]:
        self._samples.append(sample)
        while self._samples and self._samples[0].stamp < sample.stamp - self.window_s:
            self._samples.popleft()
        metrics = motion_window_metrics(self._samples)
        if metrics.duration_s < 0.90 * self.window_s:
            return False, "stability_window_filling", metrics
        checks = (
            (localization_healthy, "localization_unhealthy"),
            (placement.valid, placement.reason),
            (metrics.median_abs_wheel_speed_mps <= self.maximum_wheel_speed_mps,
             "wheel_not_stationary"),
            (metrics.z_span_m <= self.maximum_z_span_m, "vertical_motion"),
            (metrics.xy_span_m <= self.maximum_xy_span_m, "planar_motion"),
            (metrics.yaw_span_rad <= self.maximum_yaw_span_rad, "yaw_motion"),
            (metrics.maximum_tilt_rad <= self.maximum_tilt_rad, "vehicle_not_level"),
            (metrics.maximum_gyro_xy_radps <= self.maximum_gyro_xy_radps,
             "body_rotation"),
        )
        for passed, reason in checks:
            if not passed:
                return False, reason, metrics
        return True, "ground_ready", metrics


class WheelReleaseDetector:
    """Detect physical E-stop release from the first probe-induced wheel motion."""

    def __init__(self, threshold_mps: float = 0.08, required_samples: int = 3):
        if threshold_mps <= 0.0 or required_samples < 1:
            raise ValueError("invalid wheel release detector configuration")
        self.threshold_mps = float(threshold_mps)
        self.required_samples = int(required_samples)
        self._count = 0

    def reset(self) -> None:
        self._count = 0

    def update(self, wheel_speed_mps: float) -> bool:
        if abs(float(wheel_speed_mps)) >= self.threshold_mps:
            self._count += 1
        else:
            self._count = 0
        return self._count >= self.required_samples


def heading_aware_projection(
    track: PeriodicTrack,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    *,
    maximum_heading_error_rad: float = 0.70,
    heading_weight_m: float = 0.50,
    samples: int = 4096,
):
    """Globally reacquire progress using localization position and forward yaw."""
    if samples < 128 or maximum_heading_error_rad <= 0.0 or heading_weight_m < 0.0:
        raise ValueError("invalid heading-aware projection configuration")
    candidate_s = np.linspace(0.0, track.length, samples, endpoint=False)
    candidate_x, candidate_y = track.position(candidate_s)
    candidate_heading = track.tangent(candidate_s)
    heading_error = np.asarray(wrap_angle(yaw_rad - candidate_heading), dtype=float)
    forward = np.abs(heading_error) <= maximum_heading_error_rad
    if not np.any(forward):
        return track.project(x_m, y_m)
    distance2 = (candidate_x - x_m) ** 2 + (candidate_y - y_m) ** 2
    score = distance2 + (heading_weight_m * heading_error) ** 2
    score[~forward] = math.inf
    seed = float(candidate_s[int(np.argmin(score))])
    return track.project(x_m, y_m, seed)


def assess_heading_aware_placement(
    track: PeriodicTrack,
    body_width_m: float,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    speed_mps: float,
    *,
    boundary_buffer_m: float = 0.08,
    maximum_heading_error_rad: float = 0.60,
    maximum_stationary_speed_mps: float = 0.12,
) -> PlacementAssessment:
    projection = heading_aware_projection(
        track, x_m, y_m, yaw_rad,
        maximum_heading_error_rad=max(maximum_heading_error_rad, 0.70),
    )
    heading_error = float(wrap_angle(yaw_rad - projection.psi_ref))
    footprint = body_width_m / 2.0 + boundary_buffer_m
    vehicle_margin = min(
        float(track.width_left(projection.s)) - footprint - projection.e_contour,
        float(track.width_right(projection.s)) - footprint + projection.e_contour,
    )
    if abs(speed_mps) > maximum_stationary_speed_mps:
        reason = "vehicle_not_stationary"
    elif vehicle_margin < 0.0:
        reason = "vehicle_footprint_outside_track"
    elif abs(heading_error) > maximum_heading_error_rad:
        reason = "heading_not_aligned_with_track_direction"
    else:
        reason = "placement_valid"
    return PlacementAssessment(
        valid=reason == "placement_valid",
        reason=reason,
        projection=projection,
        heading_error_rad=heading_error,
        vehicle_margin_m=vehicle_margin,
        speed_mps=float(speed_mps),
    )


def assess_forward_recovery_placement(
    track: PeriodicTrack,
    body_width_m: float,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    speed_mps: float,
    *,
    boundary_buffer_m: float = 0.08,
    maximum_direct_heading_error_rad: float = 0.60,
    maximum_pp_angle_rad: float = math.radians(60.0),
    maximum_stationary_speed_mps: float = 0.12,
    lookahead_minimum_m: float = 0.45,
    lookahead_maximum_m: float = 1.20,
    minimum_path_margin_m: float = 0.05,
    target_samples: int = 32,
    path_samples: int = 20,
) -> ForwardRecoveryPlan:
    """Accept a placement when a safe forward PP target is reachable.

    A nearest-point tangent can be misleading around hairpins: a vehicle may
    have a large tangent error at the closest point while already facing a
    forward raceline point.  This assessment therefore gates recovery on the
    PP bearing to a forward target and on the swept straight-line corridor.
    """
    values = (
        body_width_m, x_m, y_m, yaw_rad, speed_mps, boundary_buffer_m,
        maximum_direct_heading_error_rad, maximum_pp_angle_rad,
        maximum_stationary_speed_mps, lookahead_minimum_m,
        lookahead_maximum_m, minimum_path_margin_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("forward recovery inputs must be finite")
    if body_width_m <= 0.0 or boundary_buffer_m < 0.0:
        raise ValueError("invalid forward recovery vehicle footprint")
    if maximum_direct_heading_error_rad <= 0.0 or maximum_pp_angle_rad <= 0.0:
        raise ValueError("forward recovery angles must be positive")
    if maximum_pp_angle_rad < maximum_direct_heading_error_rad:
        raise ValueError("PP recovery angle must cover the direct heading gate")
    if maximum_stationary_speed_mps < 0.0:
        raise ValueError("stationary speed threshold cannot be negative")
    if lookahead_minimum_m <= 0.0 or lookahead_maximum_m < lookahead_minimum_m:
        raise ValueError("invalid forward recovery lookahead interval")
    if minimum_path_margin_m < 0.0 or target_samples < 2 or path_samples < 2:
        raise ValueError("invalid forward recovery sampling configuration")

    projection = heading_aware_projection(
        track, x_m, y_m, yaw_rad,
        maximum_heading_error_rad=maximum_pp_angle_rad,
    )
    heading_error = float(wrap_angle(yaw_rad - projection.psi_ref))
    footprint = body_width_m / 2.0 + boundary_buffer_m

    def margin_at(px: float, py: float, s_guess: float):
        point = track.project(px, py, s_guess)
        margin = min(
            float(track.width_left(point.s)) - footprint - point.e_contour,
            float(track.width_right(point.s)) - footprint + point.e_contour,
        )
        return point, float(margin)

    _, vehicle_margin = margin_at(x_m, y_m, projection.s)
    invalid_reason = None
    if abs(speed_mps) > maximum_stationary_speed_mps:
        invalid_reason = "vehicle_not_stationary"
    elif vehicle_margin < 0.0:
        invalid_reason = "vehicle_footprint_outside_track"

    best = None
    if invalid_reason is None:
        for ds in np.linspace(lookahead_minimum_m, lookahead_maximum_m,
                              int(target_samples)):
            target_s = float(projection.s + ds)
            target_x, target_y = track.position(target_s)
            target_x, target_y = float(target_x), float(target_y)
            dx, dy = target_x - x_m, target_y - y_m
            distance = math.hypot(dx, dy)
            if distance <= 0.05:
                continue
            bearing_error = float(wrap_angle(math.atan2(dy, dx) - yaw_rad))
            target_heading_error = float(wrap_angle(
                yaw_rad - float(track.tangent(target_s))
            ))
            if (abs(bearing_error) > maximum_pp_angle_rad or
                    abs(target_heading_error) > maximum_pp_angle_rad):
                continue
            fractions = np.linspace(0.0, 1.0, int(path_samples))
            expected_s = projection.s + fractions * ds
            reference_x, reference_y = track.position(expected_s)
            reference_yaw = track.tangent(expected_s)
            path_x = x_m + fractions * dx
            path_y = y_m + fractions * dy
            error_x = path_x - reference_x
            error_y = path_y - reference_y
            contour = (
                -np.sin(reference_yaw) * error_x
                + np.cos(reference_yaw) * error_y
            )
            margins = np.minimum(
                track.width_left(expected_s) - footprint - contour,
                track.width_right(expected_s) - footprint + contour,
            )
            path_margin = float(np.min(margins))
            if path_margin < minimum_path_margin_m:
                continue
            # Prefer a target nearly straight ahead whose tangent already
            # agrees with the vehicle, with a mild penalty for reaching far.
            score = (
                abs(bearing_error)
                + 0.35 * abs(target_heading_error)
                + 0.08 * ds / lookahead_maximum_m
            )
            candidate = (
                score, target_s, target_x, target_y, bearing_error,
                target_heading_error, path_margin,
            )
            if best is None or candidate[0] < best[0]:
                best = candidate

    if invalid_reason is not None:
        reason = invalid_reason
    elif best is None:
        reason = "no_safe_forward_pp_target"
    elif abs(heading_error) <= maximum_direct_heading_error_rad:
        reason = "placement_valid_direct"
    else:
        reason = "placement_valid_forward_pp"
    valid = reason.startswith("placement_valid_")
    placement = PlacementAssessment(
        valid=valid,
        reason=reason,
        projection=projection,
        heading_error_rad=heading_error,
        vehicle_margin_m=vehicle_margin,
        speed_mps=float(speed_mps),
    )
    if best is None:
        return ForwardRecoveryPlan(
            placement=placement,
            target_s=float(projection.s),
            target_x_m=float(projection.x_ref),
            target_y_m=float(projection.y_ref),
            bearing_error_rad=math.inf,
            target_heading_error_rad=math.inf,
            path_minimum_margin_m=-math.inf,
            recovery_required=False,
        )
    return ForwardRecoveryPlan(
        placement=placement,
        target_s=float(best[1]),
        target_x_m=float(best[2]),
        target_y_m=float(best[3]),
        bearing_error_rad=float(best[4]),
        target_heading_error_rad=float(best[5]),
        path_minimum_margin_m=float(best[6]),
        recovery_required=abs(heading_error) > maximum_direct_heading_error_rad,
    )
