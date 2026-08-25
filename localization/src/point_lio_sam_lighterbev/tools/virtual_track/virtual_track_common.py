#!/usr/bin/env python3
"""Geometry and ROS-map helpers shared by the virtual-track generators."""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml
from PIL import Image
from scipy.interpolate import CubicSpline


RACELINE_FIELDS = [
    "s_m",
    "x_m",
    "y_m",
    "psi_rad",
    "kappa_radpm",
    "vx_mps",
    "ax_mps2",
    "w_tr_right_m",
    "w_tr_left_m",
]


def portable_path(path: Path, base_dir: Path) -> str:
    """Return a POSIX relative path suitable for checked-in metadata."""
    return Path(
        os.path.relpath(
            Path(path).expanduser().resolve(),
            Path(base_dir).expanduser().resolve(),
        )
    ).as_posix()


def load_ros_map(map_yaml: Path) -> tuple[np.ndarray, dict[str, Any], Path]:
    with map_yaml.open("r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    image_path = Path(str(meta["image"])).expanduser()
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = np.asarray(Image.open(image_path).convert("L"))
    if image.ndim != 2:
        raise ValueError(f"Expected a grayscale map image: {image_path}")
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    if len(origin) != 3:
        raise ValueError("ROS map origin must contain [x, y, yaw].")
    return image, meta, image_path


def occupied_mask(image: np.ndarray, meta: dict[str, Any]) -> np.ndarray:
    negate = int(meta.get("negate", 0))
    pixels = image.astype(float) / 255.0
    occupancy = pixels if negate else 1.0 - pixels
    return occupancy >= float(meta.get("occupied_thresh", 0.65))


def pixel_to_local(
    rows: np.ndarray, cols: np.ndarray, image_shape: tuple[int, int], resolution: float
) -> tuple[np.ndarray, np.ndarray]:
    height = image_shape[0]
    x_local = (np.asarray(cols, dtype=float) + 0.5) * resolution
    y_local = (height - np.asarray(rows, dtype=float) - 0.5) * resolution
    return x_local, y_local


def local_to_pixel(
    x_local: np.ndarray, y_local: np.ndarray, image_shape: tuple[int, int], resolution: float
) -> tuple[np.ndarray, np.ndarray]:
    height = image_shape[0]
    cols = np.asarray(x_local, dtype=float) / resolution - 0.5
    rows = height - np.asarray(y_local, dtype=float) / resolution - 0.5
    return rows, cols


def local_to_world(
    x_local: np.ndarray, y_local: np.ndarray, meta: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray]:
    origin_x, origin_y, origin_yaw = (float(v) for v in meta["origin"])
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    x_local = np.asarray(x_local, dtype=float)
    y_local = np.asarray(y_local, dtype=float)
    x_world = origin_x + cos_yaw * x_local - sin_yaw * y_local
    y_world = origin_y + sin_yaw * x_local + cos_yaw * y_local
    return x_world, y_world


def world_to_local(
    x_world: np.ndarray, y_world: np.ndarray, meta: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray]:
    origin_x, origin_y, origin_yaw = (float(v) for v in meta["origin"])
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    dx = np.asarray(x_world, dtype=float) - origin_x
    dy = np.asarray(y_world, dtype=float) - origin_y
    x_local = cos_yaw * dx + sin_yaw * dy
    y_local = -sin_yaw * dx + cos_yaw * dy
    return x_local, y_local


def closed_arclength(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 4:
        raise ValueError("A closed curve needs at least four samples.")
    segment = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)
    if np.any(segment <= 1.0e-8):
        raise ValueError("Closed curve contains duplicate consecutive points.")
    s = np.r_[0.0, np.cumsum(segment[:-1])]
    return s, float(np.sum(segment))


def periodic_resample(
    x: np.ndarray,
    y: np.ndarray,
    num_points: int,
    *extras: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray], float]:
    s, length = closed_arclength(x, y)
    s_closed = np.r_[s, length]
    target_s = np.linspace(0.0, length, int(num_points), endpoint=False)

    def interpolate(values: np.ndarray) -> np.ndarray:
        values = np.asarray(values, dtype=float)
        return CubicSpline(s_closed, np.r_[values, values[0]], bc_type="periodic")(target_s)

    x_new = interpolate(x)
    y_new = interpolate(y)
    extras_new = [interpolate(values) for values in extras]
    return target_s, x_new, y_new, extras_new, length


def heading_curvature_closed(
    x: np.ndarray, y: np.ndarray, length: float | None = None
) -> tuple[np.ndarray, np.ndarray]:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if length is None:
        _, length = closed_arclength(x, y)
    ds = float(length) / len(x)
    dx = (np.roll(x, -1) - np.roll(x, 1)) / (2.0 * ds)
    dy = (np.roll(y, -1) - np.roll(y, 1)) / (2.0 * ds)
    ddx = (np.roll(x, -1) - 2.0 * x + np.roll(x, 1)) / (ds * ds)
    ddy = (np.roll(y, -1) - 2.0 * y + np.roll(y, 1)) / (ds * ds)
    denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1.0e-9)
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / denominator
    return psi, kappa


def left_normals(psi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return -np.sin(psi), np.cos(psi)


def write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_numeric_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV contains no data rows: {path}")
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in rows[0]
        if key is not None and row_value_is_numeric(rows[0][key])
    }


def row_value_is_numeric(value: str) -> bool:
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False
