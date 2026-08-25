#!/usr/bin/env python3
"""Generate a closed track containing straights, U-turns, and an S-chicane."""

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
class ComplexTrackCandidate:
    center_x_local: float
    center_y_local: float
    half_straight_m: float
    u_turn_radius_m: float
    s_amplitude_m: float
    s_half_span_m: float
    minimum_clearance_m: float
    centerline_length_m: float


def normalized_s_profile(u: np.ndarray) -> np.ndarray:
    """Two opposite smooth lobes with zero slope/curvature at both ends."""
    profile = np.sin(2.0 * math.pi * u) * np.sin(math.pi * u) ** 2
    scale = float(np.max(np.abs(profile)))
    return profile / max(scale, 1.0e-12)


def maximum_s_curvature(
    s_half_span: float, s_amplitude: float
) -> float:
    u = np.linspace(0.0, 1.0, 2001)
    x = s_half_span - 2.0 * s_half_span * u
    y = s_amplitude * normalized_s_profile(u)
    dx = np.gradient(x, u)
    dy = np.gradient(y, u)
    ddx = np.gradient(dx, u)
    ddy = np.gradient(dy, u)
    denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1.0e-12)
    curvature = np.abs(dx * ddy - dy * ddx) / denominator
    return float(np.max(curvature[3:-3]))


def complex_track_geometry(
    center_x: float,
    center_y: float,
    half_straight: float,
    u_turn_radius: float,
    s_amplitude: float,
    s_span_fraction: float,
    num_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return a CCW track with a bottom straight and an upper S-chicane."""
    s_half_span = s_span_fraction * half_straight
    approach_length = half_straight - s_half_span
    approximate_s_length = 2.0 * s_half_span * (
        1.0 + 0.45 * (s_amplitude / max(s_half_span, 1.0e-9)) ** 2
    )
    segment_lengths = {
        "arc": math.pi * u_turn_radius,
        "approach": approach_length,
        "s": approximate_s_length,
        "bottom": 2.0 * half_straight,
    }
    approximate_total = (
        2.0 * segment_lengths["arc"]
        + 2.0 * segment_lengths["approach"]
        + segment_lengths["s"]
        + segment_lengths["bottom"]
    )

    def count(length: float, minimum: int = 18) -> int:
        return max(minimum, int(round(num_points * length / approximate_total)))

    arc_points = count(segment_lengths["arc"], 28)
    approach_points = count(segment_lengths["approach"], 18)
    s_points = count(segment_lengths["s"], 60)
    bottom_points = count(segment_lengths["bottom"], 48)

    theta_right = np.linspace(
        -0.5 * math.pi, 0.5 * math.pi, arc_points, endpoint=False
    )
    right_u_turn = np.column_stack(
        [
            center_x + half_straight + u_turn_radius * np.cos(theta_right),
            center_y + u_turn_radius * np.sin(theta_right),
        ]
    )
    top_right_straight = np.column_stack(
        [
            np.linspace(
                center_x + half_straight,
                center_x + s_half_span,
                approach_points,
                endpoint=False,
            ),
            np.full(approach_points, center_y + u_turn_radius),
        ]
    )
    u = np.linspace(0.0, 1.0, s_points, endpoint=False)
    s_chicane = np.column_stack(
        [
            center_x + s_half_span - 2.0 * s_half_span * u,
            center_y + u_turn_radius + s_amplitude * normalized_s_profile(u),
        ]
    )
    top_left_straight = np.column_stack(
        [
            np.linspace(
                center_x - s_half_span,
                center_x - half_straight,
                approach_points,
                endpoint=False,
            ),
            np.full(approach_points, center_y + u_turn_radius),
        ]
    )
    theta_left = np.linspace(
        0.5 * math.pi, 1.5 * math.pi, arc_points, endpoint=False
    )
    left_u_turn = np.column_stack(
        [
            center_x - half_straight + u_turn_radius * np.cos(theta_left),
            center_y + u_turn_radius * np.sin(theta_left),
        ]
    )
    bottom_straight = np.column_stack(
        [
            np.linspace(
                center_x - half_straight,
                center_x + half_straight,
                bottom_points,
                endpoint=False,
            ),
            np.full(bottom_points, center_y - u_turn_radius),
        ]
    )
    points = np.vstack(
        [
            right_u_turn,
            top_right_straight,
            s_chicane,
            top_left_straight,
            left_u_turn,
            bottom_straight,
        ]
    )
    dx = np.roll(points[:, 0], -1) - np.roll(points[:, 0], 1)
    dy = np.roll(points[:, 1], -1) - np.roll(points[:, 1], 1)
    norm = np.maximum(np.hypot(dx, dy), 1.0e-12)
    return points[:, 0], points[:, 1], -dy / norm, dx / norm


def candidate_clearance(
    center_x: float,
    center_y: float,
    half_straight: float,
    u_turn_radius: float,
    s_amplitude: float,
    s_span_fraction: float,
    track_half_width: float,
    distance_m: np.ndarray,
    resolution: float,
    sample_count: int,
) -> float:
    x, y, nx, ny = complex_track_geometry(
        center_x,
        center_y,
        half_straight,
        u_turn_radius,
        s_amplitude,
        s_span_fraction,
        sample_count,
    )
    minimum = float("inf")
    for offset in np.linspace(-track_half_width, track_half_width, 7):
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
    u_turn_radii: np.ndarray,
    s_amplitude: float,
    s_span_fraction: float,
    minimum_offset_radius: float,
    envelope: tuple[float, float, float, float],
    envelope_margin: float,
    track_half_width: float,
    min_clearance: float,
    distance_m: np.ndarray,
    resolution: float,
    sample_count: int,
) -> ComplexTrackCandidate | None:
    xmin, xmax, ymin, ymax = envelope
    best: ComplexTrackCandidate | None = None
    best_score = -float("inf")
    for half_straight in half_straights:
        s_half_span = s_span_fraction * half_straight
        max_s_curvature = maximum_s_curvature(s_half_span, s_amplitude)
        if max_s_curvature > 1.0 / minimum_offset_radius:
            continue
        for u_turn_radius in u_turn_radii:
            if u_turn_radius < minimum_offset_radius:
                continue
            for center_x in centers_x:
                x_extent = half_straight + u_turn_radius + track_half_width
                if (
                    center_x - x_extent < xmin + envelope_margin
                    or center_x + x_extent > xmax - envelope_margin
                ):
                    continue
                for center_y in centers_y:
                    if (
                        center_y - u_turn_radius - track_half_width
                        < ymin + envelope_margin
                        or center_y + u_turn_radius + s_amplitude + track_half_width
                        > ymax - envelope_margin
                    ):
                        continue
                    clearance = candidate_clearance(
                        float(center_x),
                        float(center_y),
                        float(half_straight),
                        float(u_turn_radius),
                        s_amplitude,
                        s_span_fraction,
                        track_half_width,
                        distance_m,
                        resolution,
                        sample_count,
                    )
                    if clearance + 1.0e-9 < min_clearance:
                        continue
                    x, y, _, _ = complex_track_geometry(
                        float(center_x),
                        float(center_y),
                        float(half_straight),
                        float(u_turn_radius),
                        s_amplitude,
                        s_span_fraction,
                        sample_count,
                    )
                    _, length = closed_arclength(x, y)
                    bottom_straight = 2.0 * half_straight
                    score = length + 0.15 * bottom_straight + 0.30 * s_amplitude
                    if score > best_score:
                        best_score = score
                        best = ComplexTrackCandidate(
                            float(center_x),
                            float(center_y),
                            float(half_straight),
                            float(u_turn_radius),
                            s_amplitude,
                            float(s_half_span),
                            float(clearance),
                            float(length),
                        )
    return best


def auto_place_complex_track(
    occupied: np.ndarray,
    resolution: float,
    envelope: tuple[float, float, float, float],
    track_width: float,
    min_clearance: float,
    envelope_margin: float,
    minimum_u_turn_radius: float,
    s_amplitude: float,
    s_span_fraction: float,
    minimum_offset_radius_margin: float,
) -> ComplexTrackCandidate:
    xmin, xmax, ymin, ymax = envelope
    width = xmax - xmin
    height = ymax - ymin
    distance_m = distance_transform_edt(~occupied) * resolution
    half_width = 0.5 * track_width
    minimum_offset_radius = half_width + minimum_offset_radius_margin
    centers_x = np.linspace(xmin + 0.34 * width, xmin + 0.72 * width, 12)
    centers_y = np.linspace(ymin + 0.30 * height, ymin + 0.66 * height, 11)
    half_straights = np.linspace(max(2.1, 0.13 * width), 0.20 * width, 11)
    u_turn_radii = np.linspace(
        max(minimum_u_turn_radius, 0.15 * height), 0.23 * height, 10
    )
    coarse = search_grid(
        centers_x,
        centers_y,
        half_straights,
        u_turn_radii,
        s_amplitude,
        s_span_fraction,
        minimum_offset_radius,
        envelope,
        envelope_margin,
        half_width,
        min_clearance,
        distance_m,
        resolution,
        420,
    )
    if coarse is None:
        raise ValueError(
            "No collision-free S/U/straight track found. Reduce S amplitude, "
            "track width, obstacle clearance, or minimum U-turn radius."
        )

    dcx = float(centers_x[1] - centers_x[0])
    dcy = float(centers_y[1] - centers_y[0])
    dh = float(half_straights[1] - half_straights[0])
    dr = float(u_turn_radii[1] - u_turn_radii[0])
    refined = search_grid(
        np.linspace(coarse.center_x_local - dcx, coarse.center_x_local + dcx, 7),
        np.linspace(coarse.center_y_local - dcy, coarse.center_y_local + dcy, 7),
        np.linspace(
            max(1.8, coarse.half_straight_m - dh),
            coarse.half_straight_m + dh,
            7,
        ),
        np.linspace(
            max(minimum_u_turn_radius, coarse.u_turn_radius_m - dr),
            coarse.u_turn_radius_m + dr,
            7,
        ),
        s_amplitude,
        s_span_fraction,
        minimum_offset_radius,
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
    parser.add_argument("--output-dir", type=Path, default=map_dir / "virtual_track_s_u")
    parser.add_argument("--track-width-m", type=float, default=1.20)
    parser.add_argument("--min-obstacle-clearance-m", type=float, default=0.25)
    parser.add_argument("--minimum-u-turn-radius-m", type=float, default=1.40)
    parser.add_argument("--s-amplitude-m", type=float, default=0.40)
    parser.add_argument("--s-span-fraction", type=float, default=0.82)
    parser.add_argument("--minimum-offset-radius-margin-m", type=float, default=0.15)
    parser.add_argument("--wall-envelope-quantile", type=float, default=0.02)
    parser.add_argument("--wall-envelope-margin-m", type=float, default=0.10)
    parser.add_argument("--num-points", type=int, default=900)
    parser.add_argument("--line-width-px", type=int, default=2)
    args = parser.parse_args()
    if args.track_width_m < 1.20:
        parser.error("This generator enforces --track-width-m >= 1.20 m.")
    if not 0.35 <= args.s_amplitude_m <= 0.80:
        parser.error("--s-amplitude-m must be in [0.35, 0.80] m.")
    if not 0.65 <= args.s_span_fraction <= 0.88:
        parser.error("--s-span-fraction must be in [0.65, 0.88].")
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
    candidate = auto_place_complex_track(
        occupied,
        resolution,
        envelope,
        args.track_width_m,
        args.min_obstacle_clearance_m,
        args.wall_envelope_margin_m,
        args.minimum_u_turn_radius_m,
        args.s_amplitude_m,
        args.s_span_fraction,
        args.minimum_offset_radius_margin_m,
    )
    half_width = 0.5 * args.track_width_m
    center_x_l, center_y_l, nx_l, ny_l = complex_track_geometry(
        candidate.center_x_local,
        candidate.center_y_local,
        candidate.half_straight_m,
        candidate.u_turn_radius_m,
        candidate.s_amplitude_m,
        args.s_span_fraction,
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
    ax.plot(center_x_l, center_y_l, color="#E69F00", linewidth=1.4, linestyle="--", label="S/U/straight centerline")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map-local x [m]")
    ax.set_ylabel("map-local y [m]")
    ax.set_title("Complex virtual track: S-chicane, U-turns, and straights")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.savefig(debug_png, dpi=180)
    plt.close(fig)

    center_world = local_to_world(
        np.asarray([candidate.center_x_local]),
        np.asarray([candidate.center_y_local]),
        meta,
    )
    top_approach_each = candidate.half_straight_m - candidate.s_half_span_m
    max_s_kappa = maximum_s_curvature(
        candidate.s_half_span_m, candidate.s_amplitude_m
    )
    summary = {
        "status": "feasible_virtual_complex_track",
        "track_shape": "complex_s_chicane_two_u_turns_and_straights",
        "source_map_yaml": portable_path(map_yaml, output_dir),
        "source_map_image": portable_path(image_path, output_dir),
        "resolution_m_per_px": resolution,
        "origin": [float(v) for v in meta["origin"]],
        "track_center_local_m": [
            candidate.center_x_local,
            candidate.center_y_local,
        ],
        "track_center_world_m": [
            float(center_world[0][0]),
            float(center_world[1][0]),
        ],
        "bottom_straight_length_m": 2.0 * candidate.half_straight_m,
        "top_straight_length_each_m": top_approach_each,
        "total_explicit_straight_length_m": 2.0 * candidate.half_straight_m + 2.0 * top_approach_each,
        "u_turn_radius_centerline_m": candidate.u_turn_radius_m,
        "u_turn_radius_inner_boundary_m": candidate.u_turn_radius_m - half_width,
        "u_turn_radius_outer_boundary_m": candidate.u_turn_radius_m + half_width,
        "s_chicane_span_m": 2.0 * candidate.s_half_span_m,
        "s_chicane_amplitude_m": candidate.s_amplitude_m,
        "s_chicane_peak_to_peak_m": 2.0 * candidate.s_amplitude_m,
        "maximum_s_chicane_curvature_radpm": max_s_kappa,
        "minimum_s_chicane_radius_m": 1.0 / max_s_kappa,
        "minimum_offset_radius_requirement_m": half_width + args.minimum_offset_radius_margin_m,
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
