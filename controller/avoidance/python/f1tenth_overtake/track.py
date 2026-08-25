"""Periodic raceline loading, interpolation, and Cartesian/Frenet projection."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import numpy as np


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class TrackSample:
    s: float
    x: float
    y: float
    yaw: float
    curvature: float
    speed: float
    width_left: float
    width_right: float


@dataclass(frozen=True)
class FrenetProjection:
    s: float
    ey: float
    x: float
    y: float
    yaw: float
    width_left: float
    width_right: float
    distance: float
    segment_index: int


class PeriodicTrack:
    """A closed raceline with signed lateral error positive to the left."""

    REQUIRED_COLUMNS = ("s_m", "x_m", "y_m", "psi_rad", "w_tr_left_m", "w_tr_right_m")

    def __init__(
        self,
        s: Iterable[float],
        x: Iterable[float],
        y: Iterable[float],
        yaw: Iterable[float],
        width_left: Iterable[float],
        width_right: Iterable[float],
        curvature: Iterable[float] | None = None,
        speed: Iterable[float] | None = None,
    ) -> None:
        self.s = np.asarray(tuple(s), dtype=float)
        self.x = np.asarray(tuple(x), dtype=float)
        self.y = np.asarray(tuple(y), dtype=float)
        self.yaw = np.asarray(tuple(yaw), dtype=float)
        self.width_left = np.asarray(tuple(width_left), dtype=float)
        self.width_right = np.asarray(tuple(width_right), dtype=float)
        self.curvature = np.zeros_like(self.s) if curvature is None else np.asarray(tuple(curvature), dtype=float)
        self.speed = np.ones_like(self.s) if speed is None else np.asarray(tuple(speed), dtype=float)

        arrays = (
            self.s,
            self.x,
            self.y,
            self.yaw,
            self.width_left,
            self.width_right,
            self.curvature,
            self.speed,
        )
        if len(self.s) < 4 or any(len(array) != len(self.s) for array in arrays):
            raise ValueError("track arrays must have equal length and at least four samples")
        if not all(np.all(np.isfinite(array)) for array in arrays):
            raise ValueError("track contains non-finite values")
        if abs(self.s[0]) > 1e-6 or np.any(np.diff(self.s) <= 0.0):
            raise ValueError("track progress must start at zero and increase strictly")
        if np.any(self.width_left <= 0.0) or np.any(self.width_right <= 0.0):
            raise ValueError("track widths must be positive")

        closure = math.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1])
        typical_step = float(np.median(np.diff(self.s)))
        if closure < 1e-6:
            closure = typical_step
        self.length = float(self.s[-1] + closure)
        if self.length <= self.s[-1]:
            raise ValueError("invalid periodic track length")

        self._segment_dx = np.roll(self.x, -1) - self.x
        self._segment_dy = np.roll(self.y, -1) - self.y
        self._segment_norm2 = self._segment_dx**2 + self._segment_dy**2
        if np.any(self._segment_norm2 < 1e-12):
            raise ValueError("track has duplicate neighboring Cartesian samples")
        self._segment_ds = np.concatenate((np.diff(self.s), [self.length - self.s[-1]]))

    @classmethod
    def from_csv(cls, path: str | Path) -> "PeriodicTrack":
        source = Path(path)
        with source.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            columns = set(reader.fieldnames or ())
            missing = set(cls.REQUIRED_COLUMNS) - columns
            if missing:
                raise ValueError(f"raceline {source} missing columns: {sorted(missing)}")
            rows = list(reader)
        if not rows:
            raise ValueError(f"raceline {source} is empty")

        def values(name: str, default: float = 0.0) -> list[float]:
            return [float(row[name]) if name in row and row[name] not in (None, "") else default for row in rows]

        return cls(
            s=values("s_m"),
            x=values("x_m"),
            y=values("y_m"),
            yaw=values("psi_rad"),
            width_left=values("w_tr_left_m"),
            width_right=values("w_tr_right_m"),
            curvature=values("kappa_radpm"),
            speed=values("vx_mps", 1.0),
        )

    def wrap_s(self, s: float) -> float:
        return float(s % self.length)

    def forward_delta(self, start_s: float, end_s: float) -> float:
        return float((end_s - start_s) % self.length)

    def signed_delta(self, start_s: float, end_s: float) -> float:
        delta = self.forward_delta(start_s, end_s)
        return delta - self.length if delta > 0.5 * self.length else delta

    def _interpolation(self, query_s: float) -> tuple[int, int, float]:
        query = self.wrap_s(query_s)
        index = int(np.searchsorted(self.s, query, side="right") - 1)
        index = max(0, min(index, len(self.s) - 1))
        next_index = (index + 1) % len(self.s)
        local = query - self.s[index]
        if local < 0.0:
            local += self.length
        ratio = float(np.clip(local / self._segment_ds[index], 0.0, 1.0))
        return index, next_index, ratio

    def sample(self, query_s: float) -> TrackSample:
        index, next_index, ratio = self._interpolation(query_s)

        def lerp(array: np.ndarray) -> float:
            return float((1.0 - ratio) * array[index] + ratio * array[next_index])

        yaw_delta = _wrap_angle(float(self.yaw[next_index] - self.yaw[index]))
        yaw = _wrap_angle(float(self.yaw[index] + ratio * yaw_delta))
        return TrackSample(
            s=self.wrap_s(query_s),
            x=lerp(self.x),
            y=lerp(self.y),
            yaw=yaw,
            curvature=lerp(self.curvature),
            speed=lerp(self.speed),
            width_left=lerp(self.width_left),
            width_right=lerp(self.width_right),
        )

    def project(self, x: float, y: float) -> FrenetProjection:
        return self._project(x, y, None)

    def project_near(
        self, x: float, y: float, reference_s: float, maximum_progress_delta: float
    ) -> FrenetProjection:
        """Project using only track segments near a known progress estimate.

        Compact race tracks often contain spatially adjacent parallel lanes.
        A globally nearest projection can jump an opponent onto the wrong lane
        and invert ahead/behind. Radar targets are local, so constraining the
        candidate segments around ego progress removes that ambiguity without
        using target truth.
        """
        if maximum_progress_delta <= 0.0:
            raise ValueError("maximum_progress_delta must be positive")
        deltas = np.asarray(
            [abs(self.signed_delta(reference_s, value)) for value in self.s],
            dtype=float,
        )
        indices = np.flatnonzero(deltas <= maximum_progress_delta)
        if len(indices) < 2:
            return self.project(x, y)
        return self._project(x, y, indices)

    def _project(
        self, x: float, y: float, candidate_indices: np.ndarray | None
    ) -> FrenetProjection:
        px = float(x) - self.x
        py = float(y) - self.y
        ratios = np.clip((px * self._segment_dx + py * self._segment_dy) / self._segment_norm2, 0.0, 1.0)
        projected_x = self.x + ratios * self._segment_dx
        projected_y = self.y + ratios * self._segment_dy
        distance2 = (float(x) - projected_x) ** 2 + (float(y) - projected_y) ** 2
        if candidate_indices is None:
            index = int(np.argmin(distance2))
        else:
            local_index = int(np.argmin(distance2[candidate_indices]))
            index = int(candidate_indices[local_index])
        ratio = float(ratios[index])
        next_index = (index + 1) % len(self.s)
        segment_length = math.sqrt(float(self._segment_norm2[index]))
        tx = float(self._segment_dx[index]) / segment_length
        ty = float(self._segment_dy[index]) / segment_length
        rx = float(x) - float(projected_x[index])
        ry = float(y) - float(projected_y[index])
        ey = tx * ry - ty * rx
        yaw = math.atan2(ty, tx)
        progress = self.wrap_s(float(self.s[index] + ratio * self._segment_ds[index]))
        width_left = float((1.0 - ratio) * self.width_left[index] + ratio * self.width_left[next_index])
        width_right = float((1.0 - ratio) * self.width_right[index] + ratio * self.width_right[next_index])
        return FrenetProjection(
            s=progress,
            ey=ey,
            x=float(projected_x[index]),
            y=float(projected_y[index]),
            yaw=yaw,
            width_left=width_left,
            width_right=width_right,
            distance=math.sqrt(float(distance2[index])),
            segment_index=index,
        )
