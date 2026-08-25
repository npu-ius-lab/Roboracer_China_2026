#!/usr/bin/env python3
"""Render a publication-quality map and virtual-track overview."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import FancyArrowPatch, Polygon
from PIL import Image


FIGURE_DIR = Path(__file__).resolve().parent
DEFAULT_TRACK_DIR = FIGURE_DIR.parent
DEFAULT_MAP_YAML = DEFAULT_TRACK_DIR.parent / "point_lio_map_2d.yaml"

COLORS = {
    "left": "#009E73",
    "right": "#7B61FF",
    "center": "#E69F00",
    "race": "#0072B2",
    "corridor": "#56B4E9",
    "text": "#263238",
}


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in rows[0]
        if key != "side"
    }


def world_to_local(
    x: np.ndarray, y: np.ndarray, origin: list[float]
) -> tuple[np.ndarray, np.ndarray]:
    origin_x, origin_y, yaw = (float(v) for v in origin)
    dx = np.asarray(x) - origin_x
    dy = np.asarray(y) - origin_y
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy


def add_direction_arrows(
    ax: plt.Axes, x: np.ndarray, y: np.ndarray, count: int = 8
) -> None:
    indices = np.linspace(4, len(x) - 5, count, dtype=int)
    for index in indices:
        start = np.array([x[index - 2], y[index - 2]])
        end = np.array([x[index + 2], y[index + 2]])
        arrow = FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=10,
            linewidth=1.1,
            color=COLORS["race"],
            zorder=8,
        )
        ax.add_patch(arrow)


def draw_track(
    ax: plt.Axes,
    left_x: np.ndarray,
    left_y: np.ndarray,
    right_x: np.ndarray,
    right_y: np.ndarray,
    center_x: np.ndarray,
    center_y: np.ndarray,
    race_x: np.ndarray,
    race_y: np.ndarray,
    show_labels: bool,
) -> None:
    corridor = np.column_stack(
        [np.r_[right_x, left_x[::-1]], np.r_[right_y, left_y[::-1]]]
    )
    ax.add_patch(
        Polygon(
            corridor,
            closed=True,
            facecolor=COLORS["corridor"],
            edgecolor="none",
            alpha=0.16,
            zorder=2,
        )
    )
    ax.plot(
        left_x,
        left_y,
        color=COLORS["left"],
        linewidth=2.0,
        label="Left / inner boundary" if show_labels else None,
        zorder=5,
    )
    ax.plot(
        right_x,
        right_y,
        color=COLORS["right"],
        linewidth=2.0,
        label="Right / outer boundary" if show_labels else None,
        zorder=5,
    )
    ax.plot(
        center_x,
        center_y,
        color=COLORS["center"],
        linewidth=1.7,
        linestyle=(0, (5, 3)),
        label="Centerline" if show_labels else None,
        zorder=6,
    )
    ax.plot(
        race_x,
        race_y,
        color=COLORS["race"],
        linewidth=2.3,
        label="NMPC raceline" if show_labels else None,
        zorder=7,
    )
    add_direction_arrows(ax, race_x, race_y)
    ax.scatter(
        [race_x[0]],
        [race_y[0]],
        s=55,
        marker="o",
        facecolor="white",
        edgecolor=COLORS["race"],
        linewidth=1.8,
        zorder=9,
        label="Reference start" if show_labels else None,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-dir", type=Path, default=DEFAULT_TRACK_DIR)
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--output-stem", default="track_map_overview")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    track_dir = args.track_dir.expanduser().resolve()
    map_yaml = args.map_yaml.expanduser().resolve()
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else track_dir / "figures"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    with map_yaml.open("r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = np.asarray(Image.open(image_path).convert("L"))
    resolution = float(meta["resolution"])
    extent = [0.0, image.shape[1] * resolution, 0.0, image.shape[0] * resolution]

    corridor = load_csv(track_dir / "virtual_track_corridor.csv")
    center = load_csv(track_dir / "centerline.csv")
    race = load_csv(track_dir / "raceline.csv")
    summary = json.loads((track_dir / "raceline_summary.json").read_text(encoding="utf-8"))
    boundary_summary = json.loads(
        (track_dir / "virtual_track_boundaries_summary.json").read_text(encoding="utf-8")
    )
    is_stadium = boundary_summary.get("track_shape", "").startswith("stadium")
    is_complex = boundary_summary.get("track_shape", "").startswith("complex")

    left_x, left_y = world_to_local(
        corridor["left_boundary_x_m"], corridor["left_boundary_y_m"], meta["origin"]
    )
    right_x, right_y = world_to_local(
        corridor["right_boundary_x_m"], corridor["right_boundary_y_m"], meta["origin"]
    )
    center_x, center_y = world_to_local(center["x_m"], center["y_m"], meta["origin"])
    race_x, race_y = world_to_local(race["x_m"], race["y_m"], meta["origin"])

    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["DejaVu Sans"],
            "font.size": 9.5,
            "axes.titlesize": 12,
            "axes.titleweight": "bold",
            "axes.labelsize": 10,
            "legend.fontsize": 8.5,
            "legend.frameon": True,
            "legend.framealpha": 0.92,
            "figure.dpi": 180,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.14,
        }
    )

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(13.2, 5.4),
        gridspec_kw={"width_ratios": [1.25, 1.0]},
        constrained_layout=True,
    )
    for ax in axes:
        ax.imshow(
            np.flipud(image),
            cmap="gray",
            origin="lower",
            extent=extent,
            vmin=0,
            vmax=255,
            interpolation="nearest",
            zorder=0,
        )
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("Map-local x [m]")
        ax.set_ylabel("Map-local y [m]")

    draw_track(
        axes[0],
        left_x,
        left_y,
        right_x,
        right_y,
        center_x,
        center_y,
        race_x,
        race_y,
        show_labels=False,
    )
    axes[0].set_xlim(extent[0], extent[1])
    axes[0].set_ylim(extent[2], extent[3])
    axes[0].set_title(
        "Full Point-LIO Map with Complex S/U Track"
        if is_complex
        else "Full Point-LIO Map with Stadium Track"
        if is_stadium
        else "Full Point-LIO Map with Virtual Track"
    )

    draw_track(
        axes[1],
        left_x,
        left_y,
        right_x,
        right_y,
        center_x,
        center_y,
        race_x,
        race_y,
        show_labels=True,
    )
    margin = 0.65
    axes[1].set_xlim(float(np.min(right_x)) - margin, float(np.max(right_x)) + margin)
    axes[1].set_ylim(float(np.min(right_y)) - margin, float(np.max(right_y)) + margin)
    axes[1].set_title("Track Geometry and Tracking Reference")
    axes[1].legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=3)

    info_lines = [
        f"Track width: {boundary_summary['track_width_m']:.2f} m",
        f"Centerline: {summary['centerline_length_m']:.2f} m",
        f"Raceline: {summary['raceline_length_m']:.2f} m",
    ]
    if is_stadium:
        info_lines.extend(
            [
                f"Each straight: {boundary_summary['straight_length_each_m']:.2f} m",
                f"U-turn radius: {boundary_summary['centerline_turn_radius_m']:.2f} m",
            ]
        )
    if is_complex:
        info_lines.extend(
            [
                f"Main straight: {boundary_summary['bottom_straight_length_m']:.2f} m",
                f"U-turn radius: {boundary_summary['u_turn_radius_centerline_m']:.2f} m",
                f"S span / P-P: {boundary_summary['s_chicane_span_m']:.2f} / {boundary_summary['s_chicane_peak_to_peak_m']:.2f} m",
            ]
        )
    info_lines.extend(
        [
            f"Map clearance: {boundary_summary['minimum_obstacle_clearance_m']:.3f} m",
            f"Tracker margin: {summary['minimum_margin_after_vehicle_and_buffer_m']:.3f} m",
        ]
    )
    info = "\n".join(info_lines)
    axes[1].text(
        0.03,
        0.04,
        info,
        transform=axes[1].transAxes,
        va="bottom",
        ha="left",
        color=COLORS["text"],
        fontsize=8.5,
        bbox={
            "boxstyle": "round,pad=0.45",
            "facecolor": "white",
            "edgecolor": "#CFD8DC",
            "alpha": 0.92,
        },
        zorder=10,
    )

    fig.suptitle(
        (
            "Complex Circuit with S-Chicane, U-Turns, and Straights"
            if is_complex
            else "Stadium Race Track with Straights and 180-degree U-Turns"
            if is_stadium
            else "Virtual Race Track Generated from the Point-LIO Occupancy Map"
        ),
        fontsize=14,
        fontweight="bold",
        color=COLORS["text"],
    )
    png_path = output_dir / f"{args.output_stem}.png"
    pdf_path = output_dir / f"{args.output_stem}.pdf"
    fig.savefig(png_path, dpi=300)
    fig.savefig(pdf_path)
    plt.close(fig)
    print(png_path)
    print(pdf_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
