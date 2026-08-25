"""Periodic continuous track geometry built from an unchanged raceline CSV."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np
from scipy.interpolate import CubicSpline


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


@dataclass(frozen=True)
class TrackProjection:
    s: float
    s_wrapped: float
    e_contour: float
    e_lag: float
    distance: float
    x_ref: float
    y_ref: float
    psi_ref: float


class PeriodicTrack:
    """C2 periodic spline and continuous nearest-point projection.

    The source CSV is never rewritten. The closing segment from its final point
    to its first point is added in memory to obtain the true loop length.
    """

    REQUIRED_COLUMNS = (
        "s_m",
        "x_m",
        "y_m",
        "w_tr_right_m",
        "w_tr_left_m",
    )
    OPTIONAL_COLUMNS = (
        "psi_rad",
        "kappa_radpm",
        "vx_mps",
        "ax_mps2",
        "speed_limit_mps",
    )

    def __init__(
        self,
        csv_path: str | Path,
        projection_samples: int = 4096,
        speed_planning: Mapping[str, float | bool] | None = None,
    ):
        self.csv_path = Path(csv_path).expanduser().resolve()
        rows = self._read_rows(self.csv_path)
        self.s_nodes = np.asarray([row["s_m"] for row in rows], dtype=float)
        self.x_nodes = np.asarray([row["x_m"] for row in rows], dtype=float)
        self.y_nodes = np.asarray([row["y_m"] for row in rows], dtype=float)
        self.psi_nodes = np.asarray([row.get("psi_rad", np.nan) for row in rows], dtype=float)
        self.kappa_nodes = np.asarray([row.get("kappa_radpm", np.nan) for row in rows], dtype=float)
        self.speed_nodes = np.asarray([row.get("vx_mps", np.nan) for row in rows], dtype=float)
        self.accel_nodes = np.asarray([row.get("ax_mps2", np.nan) for row in rows], dtype=float)
        self.speed_limit_nodes = np.asarray(
            [row.get("speed_limit_mps", np.nan) for row in rows], dtype=float
        )
        self.w_right_nodes = np.asarray([row["w_tr_right_m"] for row in rows], dtype=float)
        self.w_left_nodes = np.asarray([row["w_tr_left_m"] for row in rows], dtype=float)

        if len(rows) < 8:
            raise ValueError("raceline must contain at least eight samples")
        if not np.all(np.diff(self.s_nodes) > 0.0):
            raise ValueError("s_m must be strictly increasing")
        if abs(self.s_nodes[0]) > 1.0e-6:
            self.s_nodes = self.s_nodes - self.s_nodes[0]
        closing = math.hypot(
            self.x_nodes[0] - self.x_nodes[-1],
            self.y_nodes[0] - self.y_nodes[-1],
        )
        if closing <= 1.0e-6:
            raise ValueError("last CSV point must not duplicate the first point")
        self.length = float(self.s_nodes[-1] + closing)
        s_periodic = np.r_[self.s_nodes, self.length]

        def periodic(values: np.ndarray) -> CubicSpline:
            return CubicSpline(s_periodic, np.r_[values, values[0]], bc_type="periodic")

        self._x = periodic(self.x_nodes)
        self._y = periodic(self.y_nodes)
        self._w_left = periodic(self.w_left_nodes)
        self._w_right = periodic(self.w_right_nodes)
        self.speed_profile_source = "csv"
        has_local_speed_limits = np.all(np.isfinite(self.speed_limit_nodes))
        if np.any(np.isfinite(self.speed_limit_nodes)) and not has_local_speed_limits:
            raise ValueError("speed_limit_mps must be present on every raceline row")
        if has_local_speed_limits and np.any(self.speed_limit_nodes <= 0.0):
            raise ValueError("speed_limit_mps values must be positive")
        if speed_planning is not None and bool(speed_planning.get("enabled", False)):
            self.speed_nodes = self._plan_speed_profile(speed_planning)
            self.accel_nodes = self._profile_acceleration(self.speed_nodes)
            self.speed_profile_source = (
                "runtime_constraints+local_speed_limits"
                if has_local_speed_limits
                else "runtime_constraints"
            )
            self.speed_profile_min = float(speed_planning["min_speed_mps"])
            self.speed_profile_max = float(speed_planning["max_speed_mps"])
        elif not np.all(np.isfinite(self.speed_nodes)):
            raise ValueError("raceline needs vx_mps when runtime speed planning is disabled")
        else:
            self.speed_profile_min = float(np.min(self.speed_nodes))
            self.speed_profile_max = float(np.max(self.speed_nodes))
        # ax_csv is never an online reference; derive acceleration from the
        # one speed envelope that the controller actually uses.
        self.accel_nodes = self._profile_acceleration(self.speed_nodes)
        self._speed = periodic(self.speed_nodes)
        self._accel = periodic(self.accel_nodes)
        self._csv_kappa = periodic(self.kappa_nodes) if np.all(np.isfinite(self.kappa_nodes)) else None

        projection_samples = max(int(projection_samples), len(rows) * 4)
        self._coarse_s = np.linspace(0.0, self.length, projection_samples, endpoint=False)
        self._coarse_x = np.asarray(self._x(self._coarse_s))
        self._coarse_y = np.asarray(self._y(self._coarse_s))

    def _segment_lengths(self) -> np.ndarray:
        return np.diff(np.r_[self.s_nodes, self.length])

    def _profile_acceleration(self, speed: np.ndarray) -> np.ndarray:
        ds = self._segment_lengths()
        return (np.roll(speed, -1) ** 2 - speed**2) / (2.0 * ds)

    def _plan_speed_profile(self, config: Mapping[str, float | bool]) -> np.ndarray:
        required = (
            "min_speed_mps",
            "max_speed_mps",
            "wheelbase_m",
            "max_steer_rad",
            "max_steer_rate_radps",
            "max_accel_mps2",
            "max_decel_mps2",
            "lateral_accel_limit_mps2",
        )
        missing = [name for name in required if name not in config]
        if missing:
            raise ValueError(f"speed-planning configuration is missing: {missing}")
        minimum = float(config["min_speed_mps"])
        maximum = float(config["max_speed_mps"])
        wheelbase = float(config["wheelbase_m"])
        max_steer = float(config["max_steer_rad"])
        max_steer_rate = float(config["max_steer_rate_radps"])
        max_accel = float(config["max_accel_mps2"])
        max_decel = float(config["max_decel_mps2"])
        profile_max_accel = min(
            max_accel, float(config.get("profile_max_accel_mps2", max_accel))
        )
        profile_max_decel = min(
            max_decel, float(config.get("profile_max_decel_mps2", max_decel))
        )
        lateral_limit = float(config["lateral_accel_limit_mps2"])
        values = (
            minimum, maximum, wheelbase, max_steer, max_steer_rate,
            max_accel, max_decel, profile_max_accel, profile_max_decel,
            lateral_limit,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("speed-planning limits must be finite and positive")
        if minimum > maximum:
            raise ValueError("minimum planned speed exceeds maximum")

        if np.all(np.isfinite(self.speed_limit_nodes)):
            node_maximum = np.minimum(self.speed_limit_nodes, maximum)
            if np.any(node_maximum < minimum):
                raise ValueError("local speed limit is below minimum planned speed")
        else:
            node_maximum = np.full(len(self.s_nodes), maximum, dtype=float)

        curvature = np.asarray(self.curvature(self.s_nodes), dtype=float)
        curvature_speed = np.sqrt(
            lateral_limit / np.maximum(np.abs(curvature), 1.0e-4)
        )
        steering = np.arctan(wheelbase * curvature)
        steering_speed = np.where(
            np.abs(steering) <= 0.90 * max_steer, node_maximum, minimum
        )
        ds = self._segment_lengths()
        ds_center = np.roll(ds, 1) + ds
        steering_slope = np.abs(np.roll(steering, -1) - np.roll(steering, 1)) / np.maximum(
            ds_center, 1.0e-9
        )
        steering_rate_speed = max_steer_rate / np.maximum(steering_slope, 1.0e-9)
        speed = np.maximum(
            minimum,
            np.minimum.reduce(
                [curvature_speed, steering_speed, steering_rate_speed, node_maximum]
            ),
        )

        # Circular forward/backward passes enforce acceleration continuity on
        # the periodic trajectory, including its seam.
        for _ in range(max(8, 2 * len(speed))):
            previous = speed.copy()
            for index in range(len(speed)):
                following = (index + 1) % len(speed)
                speed[following] = min(
                    speed[following],
                    math.sqrt(
                        speed[index] ** 2
                        + 2.0 * profile_max_accel * ds[index]
                    ),
                )
            for index in range(len(speed) - 1, -1, -1):
                preceding = (index - 1) % len(speed)
                speed[preceding] = min(
                    speed[preceding],
                    math.sqrt(
                        speed[index] ** 2
                        + 2.0 * profile_max_decel * ds[preceding]
                    ),
                )
            if float(np.max(np.abs(speed - previous))) < 1.0e-6:
                break
        return np.maximum(minimum, np.minimum(speed, node_maximum))

    @classmethod
    def _read_rows(cls, path: Path) -> list[dict[str, float]]:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            missing = set(cls.REQUIRED_COLUMNS) - set(reader.fieldnames or ())
            if missing:
                raise ValueError(f"raceline is missing columns: {sorted(missing)}")
            rows = []
            for line_number, source in enumerate(reader, start=2):
                try:
                    row = {name: float(source[name]) for name in cls.REQUIRED_COLUMNS}
                    for name in cls.OPTIONAL_COLUMNS:
                        if source.get(name) not in (None, ""):
                            row[name] = float(source[name])
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"invalid numeric value at line {line_number}") from exc
                if not all(math.isfinite(value) for value in row.values()):
                    raise ValueError(f"non-finite value at line {line_number}")
                rows.append(row)
        return rows

    def wrap_s(self, s: float | np.ndarray) -> float | np.ndarray:
        return np.mod(s, self.length)

    def position(self, s: float | np.ndarray):
        sw = self.wrap_s(s)
        return self._x(sw), self._y(sw)

    def derivatives(self, s: float | np.ndarray):
        sw = self.wrap_s(s)
        return self._x(sw, 1), self._y(sw, 1), self._x(sw, 2), self._y(sw, 2)

    def tangent(self, s: float | np.ndarray):
        dx, dy, _, _ = self.derivatives(s)
        return np.arctan2(dy, dx)

    def curvature(self, s: float | np.ndarray):
        dx, dy, ddx, ddy = self.derivatives(s)
        denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1.0e-9)
        return (dx * ddy - dy * ddx) / denominator

    def csv_curvature(self, s: float | np.ndarray):
        if self._csv_kappa is None:
            raise ValueError("source raceline has no kappa_radpm diagnostic column")
        return self._csv_kappa(self.wrap_s(s))

    def width_left(self, s: float | np.ndarray):
        return np.maximum(self._w_left(self.wrap_s(s)), 0.0)

    def width_right(self, s: float | np.ndarray):
        return np.maximum(self._w_right(self.wrap_s(s)), 0.0)

    def speed_prior(self, s: float | np.ndarray):
        sw = self.wrap_s(s)
        # A cubic periodic spline can overshoot between two feasible speed
        # nodes.  Keep its smooth interpolation, but never let that numerical
        # overshoot violate the node-wise dynamic/local speed envelope.
        segment = np.searchsorted(self.s_nodes, sw, side="right") - 1
        segment = np.maximum(segment, 0)
        following = (segment + 1) % len(self.speed_nodes)
        lower = np.minimum(self.speed_nodes[segment], self.speed_nodes[following])
        upper = np.maximum(self.speed_nodes[segment], self.speed_nodes[following])
        return np.clip(
            np.clip(self._speed(sw), lower, upper),
            self.speed_profile_min, self.speed_profile_max,
        )

    def accel_prior(self, s: float | np.ndarray):
        return self._accel(self.wrap_s(s))

    def geometry(self, s: float) -> np.ndarray:
        x, y = self.position(s)
        return np.asarray(
            [
                float(x),
                float(y),
                float(self.tangent(s)),
                float(self.curvature(s)),
                float(self.width_left(s)),
                float(self.width_right(s)),
                float(self.speed_prior(s)),
            ]
        )

    def project(self, x: float, y: float, s_guess: float | None = None) -> TrackProjection:
        """Project a point with coarse initialization and Newton refinement.

        If an unwrapped ``s_guess`` is supplied, the returned ``s`` is lifted to
        the closest lap to that guess, preventing progress jumps at the seam.
        """
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("projection point must be finite")

        if s_guess is None:
            dist2 = (self._coarse_x - x) ** 2 + (self._coarse_y - y) ** 2
            s = float(self._coarse_s[int(np.argmin(dist2))])
        else:
            center = float(self.wrap_s(s_guess))
            window = max(1.5, self.length / 12.0)
            candidates = self.wrap_s(center + np.linspace(-window, window, 129))
            cx, cy = self.position(candidates)
            dist2 = (cx - x) ** 2 + (cy - y) ** 2
            s = float(candidates[int(np.argmin(dist2))])

        for _ in range(12):
            sw = float(self.wrap_s(s))
            xr, yr = float(self._x(sw)), float(self._y(sw))
            dx, dy = float(self._x(sw, 1)), float(self._y(sw, 1))
            ddx, ddy = float(self._x(sw, 2)), float(self._y(sw, 2))
            gradient = (xr - x) * dx + (yr - y) * dy
            hessian = dx * dx + dy * dy + (xr - x) * ddx + (yr - y) * ddy
            if abs(hessian) < 1.0e-9:
                break
            step = float(np.clip(gradient / hessian, -0.5, 0.5))
            s -= step
            if abs(step) < 1.0e-10:
                break

        sw = float(self.wrap_s(s))
        if s_guess is None:
            unwrapped = sw
        else:
            unwrapped = sw + round((float(s_guess) - sw) / self.length) * self.length
        xr, yr = (float(value) for value in self.position(sw))
        psi = float(self.tangent(sw))
        ex, ey = x - xr, y - yr
        e_contour = -math.sin(psi) * ex + math.cos(psi) * ey
        e_lag = math.cos(psi) * ex + math.sin(psi) * ey
        return TrackProjection(
            s=float(unwrapped),
            s_wrapped=sw,
            e_contour=e_contour,
            e_lag=e_lag,
            distance=math.hypot(ex, ey),
            x_ref=xr,
            y_ref=yr,
            psi_ref=psi,
        )

    def sample(self, count: int) -> Iterable[np.ndarray]:
        for s in np.linspace(0.0, self.length, int(count), endpoint=False):
            yield self.geometry(float(s))
