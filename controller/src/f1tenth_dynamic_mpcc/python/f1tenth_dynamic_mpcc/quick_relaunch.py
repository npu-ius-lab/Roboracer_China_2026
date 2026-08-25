"""Placement validation for competition checkpoint relaunches.

This module is deliberately independent of ROS so the geometry and debounce
logic can be tested offline. It does not mutate track or stableV3 state.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque

import numpy as np

from .track_model import PeriodicTrack, TrackProjection, wrap_angle


@dataclass(frozen=True)
class PlacementAssessment:
    valid: bool
    reason: str
    projection: TrackProjection
    heading_error_rad: float
    vehicle_margin_m: float
    speed_mps: float


def placement_relocated(
    reference_pose,
    current_pose,
    minimum_distance_m: float,
    minimum_yaw_rad: float,
) -> bool:
    """Return whether a carried vehicle has actually left its prior pose."""
    values = tuple(reference_pose) + tuple(current_pose) + (
        minimum_distance_m,
        minimum_yaw_rad,
    )
    if len(reference_pose) != 3 or len(current_pose) != 3:
        raise ValueError("placement poses must contain x, y and yaw")
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("placement relocation inputs must be finite")
    if minimum_distance_m <= 0.0 or minimum_yaw_rad <= 0.0:
        raise ValueError("placement relocation thresholds must be positive")
    x0, y0, yaw0 = (float(value) for value in reference_pose)
    x1, y1, yaw1 = (float(value) for value in current_pose)
    return (
        math.hypot(x1 - x0, y1 - y0) >= minimum_distance_m
        or abs(float(wrap_angle(yaw1 - yaw0))) >= minimum_yaw_rad
    )


def assess_placement(
    track: PeriodicTrack,
    body_width_m: float,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    speed_mps: float,
    *,
    boundary_buffer_m: float,
    maximum_heading_error_rad: float,
    maximum_stationary_speed_mps: float,
) -> PlacementAssessment:
    """Check that a globally reacquired track point is safe to launch from."""
    values = (
        body_width_m,
        x_m,
        y_m,
        yaw_rad,
        speed_mps,
        boundary_buffer_m,
        maximum_heading_error_rad,
        maximum_stationary_speed_mps,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("placement inputs must be finite")
    if body_width_m <= 0.0 or boundary_buffer_m < 0.0:
        raise ValueError("body width must be positive and boundary buffer non-negative")
    if maximum_heading_error_rad <= 0.0 or maximum_stationary_speed_mps < 0.0:
        raise ValueError("invalid placement heading or speed threshold")

    # No progress guess on purpose: this matches the candidate C++ controller's
    # checkpoint arm path and prevents a pre-carry lap index from selecting a
    # nearby but stale branch.
    projection = track.project(x_m, y_m)
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


class StablePlacementWindow:
    """Require several mutually consistent placement samples before enable."""

    def __init__(
        self,
        required_samples: int,
        maximum_position_span_m: float,
        maximum_yaw_span_rad: float,
        maximum_progress_span_m: float,
        track_length_m: float,
    ):
        if required_samples < 1:
            raise ValueError("required samples must be positive")
        if min(
            maximum_position_span_m,
            maximum_yaw_span_rad,
            maximum_progress_span_m,
            track_length_m,
        ) <= 0.0:
            raise ValueError("placement window limits must be positive")
        self.required_samples = int(required_samples)
        self.maximum_position_span_m = float(maximum_position_span_m)
        self.maximum_yaw_span_rad = float(maximum_yaw_span_rad)
        self.maximum_progress_span_m = float(maximum_progress_span_m)
        self.track_length_m = float(track_length_m)
        self._samples: Deque[tuple[float, float, float, float]] = deque(
            maxlen=self.required_samples
        )

    def reset(self) -> None:
        self._samples.clear()

    def update(
        self, assessment: PlacementAssessment, x_m: float, y_m: float, yaw_rad: float
    ) -> bool:
        if not assessment.valid:
            self.reset()
            return False
        self._samples.append(
            (float(x_m), float(y_m), float(yaw_rad), assessment.projection.s_wrapped)
        )
        samples = np.asarray(self._samples, dtype=float)
        position_span = float(np.max(np.hypot(
            samples[:, 0] - samples[0, 0], samples[:, 1] - samples[0, 1]
        )))
        yaw_span = float(np.max(np.abs(wrap_angle(samples[:, 2] - samples[0, 2]))))
        progress_delta = np.mod(
            samples[:, 3] - samples[0, 3] + self.track_length_m / 2.0,
            self.track_length_m,
        ) - self.track_length_m / 2.0
        progress_span = float(np.max(np.abs(progress_delta)))
        stable = (
            position_span <= self.maximum_position_span_m
            and yaw_span <= self.maximum_yaw_span_rad
            and progress_span <= self.maximum_progress_span_m
        )
        if not stable:
            # Retain only the latest observation so a fresh stable window can
            # form immediately after localization settles.
            latest = self._samples[-1]
            self._samples.clear()
            self._samples.append(latest)
            return False
        return len(self._samples) >= self.required_samples

    @property
    def sample_count(self) -> int:
        return len(self._samples)
