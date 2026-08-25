#!/usr/bin/env python3
"""Generate paired virtual track boundaries inside a ROS occupancy map.

The input map only needs to describe walls/obstacles; it does not need to
already contain a two-sided race track.  A deterministic coarse-to-fine search
places the longest feasible horizontal ellipse whose complete track strip is
clear of occupied pixels.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt, map_coordinates

from virtual_track_common import (
    closed_arclength,
    load_ros_map,
    local_to_pixel,
    local_to_world,
    occupied_mask,
    pixel_to_local,
    portable_path,
    write_csv,
)


@dataclass(frozen=True)
class EllipseCandidate:
    center_x_local: float
    center_y_local: float
    radius_x_m: float
    radius_y_m: float
    minimum_clearance_m: float
    perimeter_m: float


def ellipse_geometry(
    center_x: float,
    center_y: float,
    radius_x: float,
    radius_y: float,
    num_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    theta = np.linspace(0.0, 2.0 * math.pi, int(num_points), endpoint=False)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x = center_x + radius_x * cos_theta
    y = center_y + radius_y * sin_theta
    dx = -radius_x * sin_theta
    dy = radius_y * cos_theta
    tangent_norm = np.maximum(np.hypot(dx, dy), 1.0e-12)
    # Counter-clockwise curve: the left normal points toward the ellipse interior.
    normal_left_x = -dy / tangent_norm
    normal_left_y = dx / tangent_norm
    return x, y, normal_left_x, normal_left_y


def infer_wall_envelope(
    occupied: np.ndarray,
    resolution: float,
    quantile: float,
) -> tuple[float, float, float, float]:
    rows, cols = np.nonzero(occupied)
    if len(rows) < 20:
        raise ValueError("Map has too few occupied pixels to infer a wall envelope.")
    x_local, y_local = pixel_to_local(rows, cols, occupied.shape, resolution)
    low = float(quantile)
    high = 1.0 - low
    return (
        float(np.quantile(x_local, low)),
        float(np.quantile(x_local, high)),
        float(np.quantile(y_local, low)),
        float(np.quantile(y_local, high)),
    )


def ramanujan_perimeter(radius_x: float, radius_y: float) -> float:
    return math.pi * (
        3.0 * (radius_x + radius_y)
        - math.sqrt((3.0 * radius_x + radius_y) * (radius_x + 3.0 * radius_y))
    )


def candidate_clearance(
    center_x: float,
    center_y: float,
    radius_x: float,
    radius_y: float,
    track_half_width: float,
    distance_m: np.ndarray,
    resolution: float,
    sample_count: int,
) -> float:
    x, y, nx, ny = ellipse_geometry(
        center_x, center_y, radius_x, radius_y, sample_count
    )
    minimum = float("inf")
    # Checking five offsets validates the entire virtual driving strip, not
    # only its two outline curves.
    for offset in np.linspace(-track_half_width, track_half_width, 5):
        rows, cols = local_to_pixel(
            x + offset * nx,
            y + offset * ny,
            distance_m.shape,
            resolution,
        )
        if (
            float(np.min(rows)) < 0.0
            or float(np.max(rows)) > distance_m.shape[0] - 1
            or float(np.min(cols)) < 0.0
            or float(np.max(cols)) > distance_m.shape[1] - 1
        ):
            return 0.0
        sampled = map_coordinates(
            distance_m, [rows, cols], order=1, mode="constant", cval=0.0
        )
        minimum = min(minimum, float(np.min(sampled)))
    return minimum


def search_grid(
    centers_x: np.ndarray,
    centers_y: np.ndarray,
    radii_x: np.ndarray,
    radii_y: np.ndarray,
    envelope: tuple[float, float, float, float],
    envelope_margin: float,
    track_half_width: float,
    min_clearance: float,
    distance_m: np.ndarray,
    resolution: float,
    sample_count: int,
) -> EllipseCandidate | None:
    xmin, xmax, ymin, ymax = envelope
    best: EllipseCandidate | None = None
    for radius_x in radii_x:
        for radius_y in radii_y:
            perimeter = ramanujan_perimeter(float(radius_x), float(radius_y))
            for center_x in centers_x:
                if (
                    center_x - radius_x - track_half_width < xmin + envelope_margin
                    or center_x + radius_x + track_half_width > xmax - envelope_margin
                ):
                    continue
                for center_y in centers_y:
                    if (
                        center_y - radius_y - track_half_width < ymin + envelope_margin
                        or center_y + radius_y + track_half_width > ymax - envelope_margin
                    ):
                        continue
                    clearance = candidate_clearance(
                        float(center_x),
                        float(center_y),
                        float(radius_x),
                        float(radius_y),
                        track_half_width,
                        distance_m,
                        resolution,
                        sample_count,
                    )
                    if clearance + 1.0e-9 < min_clearance:
                        continue
                    candidate = EllipseCandidate(
                        center_x_local=float(center_x),
                        center_y_local=float(center_y),
                        radius_x_m=float(radius_x),
                        radius_y_m=float(radius_y),
                        minimum_clearance_m=float(clearance),
                        perimeter_m=float(perimeter),
                    )
                    if best is None or (
                        candidate.perimeter_m + 0.10 * candidate.minimum_clearance_m
                        > best.perimeter_m + 0.10 * best.minimum_clearance_m
                    ):
                        best = candidate
    return best


def auto_place_ellipse(
    occupied: np.ndarray,
    resolution: float,
    envelope: tuple[float, float, float, float],
    track_width: float,
    min_clearance: float,
    envelope_margin: float,
) -> EllipseCandidate:
    xmin, xmax, ymin, ymax = envelope
    width = xmax - xmin
    height = ymax - ymin
    half_width = 0.5 * track_width
    if width <= 2.0 * (half_width + min_clearance) or height <= 2.0 * (
        half_width + min_clearance
    ):
        raise ValueError("Inferred map enclosure is too small for the requested track width.")

    distance_m = distance_transform_edt(~occupied) * resolution
    centers_x = np.linspace(xmin + 0.28 * width, xmin + 0.72 * width, 13)
    centers_y = np.linspace(ymin + 0.28 * height, ymin + 0.72 * height, 11)
    radii_x = np.linspace(max(1.5, 0.18 * width), 0.44 * width, 13)
    radii_y = np.linspace(max(1.0, 0.15 * height), 0.42 * height, 11)
    coarse = search_grid(
        centers_x,
        centers_y,
        radii_x,
        radii_y,
        envelope,
        envelope_margin,
        half_width,
        min_clearance,
        distance_m,
        resolution,
        sample_count=300,
    )
    if coarse is None:
        raise ValueError(
            "No collision-free virtual ellipse found. Reduce --track-width-m or "
            "--min-obstacle-clearance-m, or supply manual ellipse parameters."
        )

    step_cx = float(centers_x[1] - centers_x[0])
    step_cy = float(centers_y[1] - centers_y[0])
    step_rx = float(radii_x[1] - radii_x[0])
    step_ry = float(radii_y[1] - radii_y[0])
    refined = search_grid(
        np.linspace(coarse.center_x_local - step_cx, coarse.center_x_local + step_cx, 7),
        np.linspace(coarse.center_y_local - step_cy, coarse.center_y_local + step_cy, 7),
        np.linspace(max(0.5, coarse.radius_x_m - step_rx), coarse.radius_x_m + step_rx, 7),
        np.linspace(max(0.5, coarse.radius_y_m - step_ry), coarse.radius_y_m + step_ry, 7),
        envelope,
        envelope_margin,
        half_width,
        min_clearance,
        distance_m,
        resolution,
        sample_count=600,
    )
    return refined or coarse


def manual_candidate(
    args: argparse.Namespace,
    occupied: np.ndarray,
    resolution: float,
) -> EllipseCandidate:
    distance_m = distance_transform_edt(~occupied) * resolution
    clearance = candidate_clearance(
        args.center_x_local,
        args.center_y_local,
        args.radius_x_m,
        args.radius_y_m,
        0.5 * args.track_width_m,
        distance_m,
        resolution,
        900,
    )
    if clearance < args.min_obstacle_clearance_m:
        raise ValueError(
            f"Manual ellipse clearance {clearance:.3f} m is below requested "
            f"{args.min_obstacle_clearance_m:.3f} m."
        )
    return EllipseCandidate(
        args.center_x_local,
        args.center_y_local,
        args.radius_x_m,
        args.radius_y_m,
        clearance,
        ramanujan_perimeter(args.radius_x_m, args.radius_y_m),
    )


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_map = (
        script_dir.parents[1]
        / "point_lio"
        / "localization_map"
        / "point_lio_map_2d.yaml"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, default=default_map)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--track-width-m", type=float, default=1.20)
    parser.add_argument("--min-obstacle-clearance-m", type=float, default=0.25)
    parser.add_argument("--wall-envelope-quantile", type=float, default=0.02)
    parser.add_argument("--wall-envelope-margin-m", type=float, default=0.10)
    parser.add_argument("--num-points", type=int, default=720)
    parser.add_argument("--line-width-px", type=int, default=2)
    parser.add_argument("--center-x-local", type=float, default=None)
    parser.add_argument("--center-y-local", type=float, default=None)
    parser.add_argument("--radius-x-m", type=float, default=None)
    parser.add_argument("--radius-y-m", type=float, default=None)
    args = parser.parse_args()
    if args.output_dir is None:
        args.output_dir = args.map_yaml.parent / "virtual_track"
    manual = [args.center_x_local, args.center_y_local, args.radius_x_m, args.radius_y_m]
    if any(value is not None for value in manual) and not all(value is not None for value in manual):
        parser.error("Manual placement requires center-x/y-local and radius-x/y-m together.")
    if args.track_width_m <= 0.0 or args.min_obstacle_clearance_m < 0.0:
        parser.error("Track width must be positive and obstacle clearance non-negative.")
    return args


def main() -> int:
    args = parse_args()
    image, meta, image_path = load_ros_map(args.map_yaml.expanduser().resolve())
    resolution = float(meta["resolution"])
    occupied = occupied_mask(image, meta)
    envelope = infer_wall_envelope(
        occupied, resolution, args.wall_envelope_quantile
    )
    if args.center_x_local is None:
        candidate = auto_place_ellipse(
            occupied,
            resolution,
            envelope,
            args.track_width_m,
            args.min_obstacle_clearance_m,
            args.wall_envelope_margin_m,
        )
    else:
        candidate = manual_candidate(args, occupied, resolution)

    half_width = 0.5 * args.track_width_m
    center_x_l, center_y_l, normal_x_l, normal_y_l = ellipse_geometry(
        candidate.center_x_local,
        candidate.center_y_local,
        candidate.radius_x_m,
        candidate.radius_y_m,
        args.num_points,
    )
    left_x_l = center_x_l + half_width * normal_x_l
    left_y_l = center_y_l + half_width * normal_y_l
    right_x_l = center_x_l - half_width * normal_x_l
    right_y_l = center_y_l - half_width * normal_y_l
    center_x, center_y = local_to_world(center_x_l, center_y_l, meta)
    left_x, left_y = local_to_world(left_x_l, left_y_l, meta)
    right_x, right_y = local_to_world(right_x_l, right_y_l, meta)
    s, length = closed_arclength(center_x, center_y)

    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    left_csv = args.output_dir / "virtual_left_boundary.csv"
    right_csv = args.output_dir / "virtual_right_boundary.csv"
    corridor_csv = args.output_dir / "virtual_track_corridor.csv"
    boundary_fields = ["side", "index", "s_m", "x_m", "y_m", "width_from_center_m"]
    write_csv(
        left_csv,
        boundary_fields,
        (
            {
                "side": "left",
                "index": i,
                "s_m": f"{s[i]:.9f}",
                "x_m": f"{left_x[i]:.9f}",
                "y_m": f"{left_y[i]:.9f}",
                "width_from_center_m": f"{half_width:.9f}",
            }
            for i in range(len(s))
        ),
    )
    write_csv(
        right_csv,
        boundary_fields,
        (
            {
                "side": "right",
                "index": i,
                "s_m": f"{s[i]:.9f}",
                "x_m": f"{right_x[i]:.9f}",
                "y_m": f"{right_y[i]:.9f}",
                "width_from_center_m": f"{half_width:.9f}",
            }
            for i in range(len(s))
        ),
    )
    corridor_fields = [
        "index",
        "s_m",
        "center_x_m",
        "center_y_m",
        "w_left_m",
        "w_right_m",
        "left_boundary_x_m",
        "left_boundary_y_m",
        "right_boundary_x_m",
        "right_boundary_y_m",
    ]
    write_csv(
        corridor_csv,
        corridor_fields,
        (
            {
                "index": i,
                "s_m": f"{s[i]:.9f}",
                "center_x_m": f"{center_x[i]:.9f}",
                "center_y_m": f"{center_y[i]:.9f}",
                "w_left_m": f"{half_width:.9f}",
                "w_right_m": f"{half_width:.9f}",
                "left_boundary_x_m": f"{left_x[i]:.9f}",
                "left_boundary_y_m": f"{left_y[i]:.9f}",
                "right_boundary_x_m": f"{right_x[i]:.9f}",
                "right_boundary_y_m": f"{right_y[i]:.9f}",
            }
            for i in range(len(s))
        ),
    )

    output_map = Image.fromarray(image.copy())
    draw = ImageDraw.Draw(output_map)
    for bx, by in ((left_x_l, left_y_l), (right_x_l, right_y_l)):
        rows, cols = local_to_pixel(bx, by, image.shape, resolution)
        points = [(int(round(c)), int(round(r))) for r, c in zip(rows, cols)]
        draw.line(
            points + [points[0]],
            fill=0,
            width=max(1, args.line_width_px),
            joint="curve",
        )
    virtual_map_pgm = args.output_dir / "virtual_track_map.pgm"
    output_map.save(virtual_map_pgm)
    virtual_map_yaml = args.output_dir / "virtual_track_map.yaml"
    virtual_meta = dict(meta)
    virtual_meta["image"] = virtual_map_pgm.name
    virtual_map_yaml.write_text(
        yaml.safe_dump(virtual_meta, sort_keys=False), encoding="utf-8"
    )

    debug_png = args.output_dir / "virtual_track_boundaries_debug.png"
    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    extent = [0.0, image.shape[1] * resolution, 0.0, image.shape[0] * resolution]
    ax.imshow(np.flipud(image), cmap="gray", origin="lower", extent=extent)
    ax.plot(left_x_l, left_y_l, color="#16a34a", linewidth=1.8, label="left / inner boundary")
    ax.plot(right_x_l, right_y_l, color="#7c3aed", linewidth=1.8, label="right / outer boundary")
    ax.plot(center_x_l, center_y_l, color="#f97316", linewidth=1.3, linestyle="--", label="virtual center seed")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map-local x [m]")
    ax.set_ylabel("map-local y [m]")
    ax.set_title("Collision-checked virtual track boundaries")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.savefig(debug_png, dpi=180)
    plt.close(fig)

    center_world = local_to_world(
        np.asarray([candidate.center_x_local]),
        np.asarray([candidate.center_y_local]),
        meta,
    )
    summary: dict[str, Any] = {
        "status": "feasible_virtual_track",
        "source_map_yaml": portable_path(args.map_yaml, args.output_dir),
        "source_map_image": portable_path(image_path, args.output_dir),
        "resolution_m_per_px": resolution,
        "origin": [float(v) for v in meta["origin"]],
        "wall_envelope_local_m": {
            "xmin": envelope[0],
            "xmax": envelope[1],
            "ymin": envelope[2],
            "ymax": envelope[3],
        },
        "ellipse_center_local_m": [candidate.center_x_local, candidate.center_y_local],
        "ellipse_center_world_m": [float(center_world[0][0]), float(center_world[1][0])],
        "ellipse_radius_m": [candidate.radius_x_m, candidate.radius_y_m],
        "track_width_m": args.track_width_m,
        "minimum_obstacle_clearance_m": candidate.minimum_clearance_m,
        "requested_minimum_obstacle_clearance_m": args.min_obstacle_clearance_m,
        "centerline_length_m": length,
        "num_points": len(s),
        "outputs": {
            "left_boundary_csv": portable_path(left_csv, args.output_dir),
            "right_boundary_csv": portable_path(right_csv, args.output_dir),
            "corridor_csv": portable_path(corridor_csv, args.output_dir),
            "virtual_map_pgm": portable_path(virtual_map_pgm, args.output_dir),
            "virtual_map_yaml": portable_path(virtual_map_yaml, args.output_dir),
            "debug_png": portable_path(debug_png, args.output_dir),
        },
    }
    summary_path = args.output_dir / "virtual_track_boundaries_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
