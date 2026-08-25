#!/usr/bin/env python3
"""Generate a stadium track with two straights and two 180-degree U-turns."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt, map_coordinates

from generate_track_boundaries import infer_wall_envelope
from virtual_track_common import (
    closed_arclength,
    load_ros_map,
    local_to_pixel,
    local_to_world,
    occupied_mask,
    portable_path,
    write_csv,
)


@dataclass(frozen=True)
class StadiumCandidate:
    center_x_local: float
    center_y_local: float
    half_straight_m: float
    turn_radius_m: float
    minimum_clearance_m: float
    centerline_length_m: float


def stadium_geometry(
    center_x: float,
    center_y: float,
    half_straight: float,
    turn_radius: float,
    num_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return a CCW stadium curve and its inward-pointing left normals."""
    total_length = 4.0 * half_straight + 2.0 * math.pi * turn_radius
    arc_points = max(
        24, int(round(num_points * math.pi * turn_radius / total_length))
    )
    straight_points = max(
        24, int(round(num_points * 2.0 * half_straight / total_length))
    )

    theta_right = np.linspace(
        -0.5 * math.pi, 0.5 * math.pi, arc_points, endpoint=False
    )
    right_arc = np.column_stack(
        [
            center_x + half_straight + turn_radius * np.cos(theta_right),
            center_y + turn_radius * np.sin(theta_right),
        ]
    )
    top = np.column_stack(
        [
            np.linspace(
                center_x + half_straight,
                center_x - half_straight,
                straight_points,
                endpoint=False,
            ),
            np.full(straight_points, center_y + turn_radius),
        ]
    )
    theta_left = np.linspace(
        0.5 * math.pi, 1.5 * math.pi, arc_points, endpoint=False
    )
    left_arc = np.column_stack(
        [
            center_x - half_straight + turn_radius * np.cos(theta_left),
            center_y + turn_radius * np.sin(theta_left),
        ]
    )
    bottom = np.column_stack(
        [
            np.linspace(
                center_x - half_straight,
                center_x + half_straight,
                straight_points,
                endpoint=False,
            ),
            np.full(straight_points, center_y - turn_radius),
        ]
    )
    points = np.vstack([right_arc, top, left_arc, bottom])
    dx = np.roll(points[:, 0], -1) - np.roll(points[:, 0], 1)
    dy = np.roll(points[:, 1], -1) - np.roll(points[:, 1], 1)
    norm = np.maximum(np.hypot(dx, dy), 1.0e-12)
    return points[:, 0], points[:, 1], -dy / norm, dx / norm


def candidate_clearance(
    center_x: float,
    center_y: float,
    half_straight: float,
    turn_radius: float,
    track_half_width: float,
    distance_m: np.ndarray,
    resolution: float,
    sample_count: int,
) -> float:
    x, y, nx, ny = stadium_geometry(
        center_x, center_y, half_straight, turn_radius, sample_count
    )
    minimum = float("inf")
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
        values = map_coordinates(
            distance_m, [rows, cols], order=1, mode="constant", cval=0.0
        )
        minimum = min(minimum, float(np.min(values)))
    return minimum


def search_grid(
    centers_x: np.ndarray,
    centers_y: np.ndarray,
    half_straights: np.ndarray,
    turn_radii: np.ndarray,
    envelope: tuple[float, float, float, float],
    envelope_margin: float,
    track_half_width: float,
    min_clearance: float,
    distance_m: np.ndarray,
    resolution: float,
    sample_count: int,
) -> StadiumCandidate | None:
    xmin, xmax, ymin, ymax = envelope
    best: StadiumCandidate | None = None
    best_score = -float("inf")
    for half_straight in half_straights:
        for turn_radius in turn_radii:
            length = 4.0 * half_straight + 2.0 * math.pi * turn_radius
            for center_x in centers_x:
                x_extent = half_straight + turn_radius + track_half_width
                if (
                    center_x - x_extent < xmin + envelope_margin
                    or center_x + x_extent > xmax - envelope_margin
                ):
                    continue
                for center_y in centers_y:
                    y_extent = turn_radius + track_half_width
                    if (
                        center_y - y_extent < ymin + envelope_margin
                        or center_y + y_extent > ymax - envelope_margin
                    ):
                        continue
                    clearance = candidate_clearance(
                        float(center_x),
                        float(center_y),
                        float(half_straight),
                        float(turn_radius),
                        track_half_width,
                        distance_m,
                        resolution,
                        sample_count,
                    )
                    if clearance + 1.0e-9 < min_clearance:
                        continue
                    # Reward both total lap length and genuinely long straights.
                    score = length + 0.25 * (4.0 * half_straight) + 0.05 * clearance
                    if score > best_score:
                        best_score = score
                        best = StadiumCandidate(
                            float(center_x),
                            float(center_y),
                            float(half_straight),
                            float(turn_radius),
                            float(clearance),
                            float(length),
                        )
    return best


def auto_place_stadium(
    occupied: np.ndarray,
    resolution: float,
    envelope: tuple[float, float, float, float],
    track_width: float,
    min_clearance: float,
    envelope_margin: float,
    minimum_turn_radius: float,
) -> StadiumCandidate:
    xmin, xmax, ymin, ymax = envelope
    width = xmax - xmin
    height = ymax - ymin
    distance_m = distance_transform_edt(~occupied) * resolution
    half_width = 0.5 * track_width
    centers_x = np.linspace(xmin + 0.30 * width, xmin + 0.72 * width, 15)
    centers_y = np.linspace(ymin + 0.30 * height, ymin + 0.70 * height, 13)
    half_straights = np.linspace(max(1.5, 0.10 * width), 0.24 * width, 13)
    turn_radii = np.linspace(
        max(minimum_turn_radius, 0.15 * height), 0.30 * height, 12
    )
    coarse = search_grid(
        centers_x,
        centers_y,
        half_straights,
        turn_radii,
        envelope,
        envelope_margin,
        half_width,
        min_clearance,
        distance_m,
        resolution,
        360,
    )
    if coarse is None:
        raise ValueError(
            "No collision-free stadium track found. Reduce track width, obstacle "
            "clearance, or minimum turn radius."
        )

    dcx = float(centers_x[1] - centers_x[0])
    dcy = float(centers_y[1] - centers_y[0])
    dh = float(half_straights[1] - half_straights[0])
    dr = float(turn_radii[1] - turn_radii[0])
    refined = search_grid(
        np.linspace(coarse.center_x_local - dcx, coarse.center_x_local + dcx, 7),
        np.linspace(coarse.center_y_local - dcy, coarse.center_y_local + dcy, 7),
        np.linspace(
            max(1.0, coarse.half_straight_m - dh),
            coarse.half_straight_m + dh,
            7,
        ),
        np.linspace(
            max(minimum_turn_radius, coarse.turn_radius_m - dr),
            coarse.turn_radius_m + dr,
            7,
        ),
        envelope,
        envelope_margin,
        half_width,
        min_clearance,
        distance_m,
        resolution,
        720,
    )
    return refined or coarse


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    map_dir = script_dir.parents[1] / "point_lio" / "localization_map"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, default=map_dir / "point_lio_map_2d.yaml")
    parser.add_argument("--output-dir", type=Path, default=map_dir / "virtual_track_u_turn")
    parser.add_argument("--track-width-m", type=float, default=1.20)
    parser.add_argument("--min-obstacle-clearance-m", type=float, default=0.25)
    parser.add_argument("--minimum-turn-radius-m", type=float, default=1.40)
    parser.add_argument("--wall-envelope-quantile", type=float, default=0.02)
    parser.add_argument("--wall-envelope-margin-m", type=float, default=0.10)
    parser.add_argument("--num-points", type=int, default=720)
    parser.add_argument("--line-width-px", type=int, default=2)
    args = parser.parse_args()
    if args.track_width_m < 1.20:
        parser.error("This generator enforces --track-width-m >= 1.20 m.")
    return args


def main() -> int:
    args = parse_args()
    map_yaml = args.map_yaml.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    image, meta, image_path = load_ros_map(map_yaml)
    resolution = float(meta["resolution"])
    occupied = occupied_mask(image, meta)
    envelope = infer_wall_envelope(
        occupied, resolution, args.wall_envelope_quantile
    )
    candidate = auto_place_stadium(
        occupied,
        resolution,
        envelope,
        args.track_width_m,
        args.min_obstacle_clearance_m,
        args.wall_envelope_margin_m,
        args.minimum_turn_radius_m,
    )
    half_width = 0.5 * args.track_width_m
    center_x_l, center_y_l, nx_l, ny_l = stadium_geometry(
        candidate.center_x_local,
        candidate.center_y_local,
        candidate.half_straight_m,
        candidate.turn_radius_m,
        args.num_points,
    )
    left_x_l = center_x_l + half_width * nx_l
    left_y_l = center_y_l + half_width * ny_l
    right_x_l = center_x_l - half_width * nx_l
    right_y_l = center_y_l - half_width * ny_l
    center_x, center_y = local_to_world(center_x_l, center_y_l, meta)
    left_x, left_y = local_to_world(left_x_l, left_y_l, meta)
    right_x, right_y = local_to_world(right_x_l, right_y_l, meta)
    s, measured_length = closed_arclength(center_x, center_y)

    output_dir.mkdir(parents=True, exist_ok=True)
    boundary_fields = [
        "side",
        "index",
        "s_m",
        "x_m",
        "y_m",
        "width_from_center_m",
    ]
    for side, bx, by, filename in (
        ("left", left_x, left_y, "virtual_left_boundary.csv"),
        ("right", right_x, right_y, "virtual_right_boundary.csv"),
    ):
        write_csv(
            output_dir / filename,
            boundary_fields,
            (
                {
                    "side": side,
                    "index": i,
                    "s_m": f"{s[i]:.9f}",
                    "x_m": f"{bx[i]:.9f}",
                    "y_m": f"{by[i]:.9f}",
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
    corridor_csv = output_dir / "virtual_track_corridor.csv"
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

    virtual_map = Image.fromarray(image.copy())
    draw = ImageDraw.Draw(virtual_map)
    for bx, by in ((left_x_l, left_y_l), (right_x_l, right_y_l)):
        rows, cols = local_to_pixel(bx, by, image.shape, resolution)
        pixels = [(int(round(col)), int(round(row))) for row, col in zip(rows, cols)]
        draw.line(
            pixels + [pixels[0]],
            fill=0,
            width=max(1, args.line_width_px),
            joint="curve",
        )
    virtual_map_pgm = output_dir / "virtual_track_map.pgm"
    virtual_map.save(virtual_map_pgm)
    virtual_meta = dict(meta)
    virtual_meta["image"] = virtual_map_pgm.name
    (output_dir / "virtual_track_map.yaml").write_text(
        yaml.safe_dump(virtual_meta, sort_keys=False), encoding="utf-8"
    )

    debug_png = output_dir / "virtual_track_boundaries_debug.png"
    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    extent = [0.0, image.shape[1] * resolution, 0.0, image.shape[0] * resolution]
    ax.imshow(np.flipud(image), cmap="gray", origin="lower", extent=extent)
    ax.fill(
        np.r_[right_x_l, left_x_l[::-1]],
        np.r_[right_y_l, left_y_l[::-1]],
        color="#56B4E9",
        alpha=0.16,
    )
    ax.plot(left_x_l, left_y_l, color="#009E73", linewidth=1.8, label="left / inner boundary")
    ax.plot(right_x_l, right_y_l, color="#7B61FF", linewidth=1.8, label="right / outer boundary")
    ax.plot(center_x_l, center_y_l, color="#E69F00", linewidth=1.4, linestyle="--", label="stadium centerline")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map-local x [m]")
    ax.set_ylabel("map-local y [m]")
    ax.set_title("Virtual stadium track: two straights and two U-turns")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.savefig(debug_png, dpi=180)
    plt.close(fig)

    center_world = local_to_world(
        np.asarray([candidate.center_x_local]),
        np.asarray([candidate.center_y_local]),
        meta,
    )
    summary = {
        "status": "feasible_virtual_stadium_track",
        "track_shape": "stadium_two_straights_two_180deg_u_turns",
        "source_map_yaml": portable_path(map_yaml, output_dir),
        "source_map_image": portable_path(image_path, output_dir),
        "resolution_m_per_px": resolution,
        "origin": [float(v) for v in meta["origin"]],
        "stadium_center_local_m": [
            candidate.center_x_local,
            candidate.center_y_local,
        ],
        "stadium_center_world_m": [
            float(center_world[0][0]),
            float(center_world[1][0]),
        ],
        "straight_length_each_m": 2.0 * candidate.half_straight_m,
        "total_straight_length_m": 4.0 * candidate.half_straight_m,
        "centerline_turn_radius_m": candidate.turn_radius_m,
        "inner_boundary_turn_radius_m": candidate.turn_radius_m - half_width,
        "outer_boundary_turn_radius_m": candidate.turn_radius_m + half_width,
        "track_width_m": args.track_width_m,
        "minimum_obstacle_clearance_m": candidate.minimum_clearance_m,
        "requested_minimum_obstacle_clearance_m": args.min_obstacle_clearance_m,
        "centerline_length_m": measured_length,
        "num_points": len(s),
        "outputs": {
            "left_boundary_csv": portable_path(output_dir / "virtual_left_boundary.csv", output_dir),
            "right_boundary_csv": portable_path(output_dir / "virtual_right_boundary.csv", output_dir),
            "corridor_csv": portable_path(corridor_csv, output_dir),
            "virtual_map_pgm": portable_path(virtual_map_pgm, output_dir),
            "virtual_map_yaml": portable_path(output_dir / "virtual_track_map.yaml", output_dir),
            "debug_png": portable_path(debug_png, output_dir),
        },
    }
    summary_path = output_dir / "virtual_track_boundaries_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
