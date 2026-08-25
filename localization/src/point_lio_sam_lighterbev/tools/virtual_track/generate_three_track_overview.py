#!/usr/bin/env python3
"""Render a reproducible overview of all three localization-map racelines."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from virtual_track_common import load_ros_map, read_numeric_csv, world_to_local


TRACKS = (
    ("virtual_track", "1. Ellipse loop"),
    ("virtual_track_u_turn", "2. U-turns and straights"),
    ("virtual_track_s_u", "3. S-chicane, U-turns and straights"),
)


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_map_dir = script_dir.parents[1] / "point_lio" / "localization_map"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, default=default_map_dir)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def close_curve(values: np.ndarray) -> np.ndarray:
    return np.r_[values, values[0]]


def main() -> int:
    args = parse_args()
    map_dir = args.map_dir.expanduser().resolve()
    output = (
        args.output.expanduser().resolve()
        if args.output is not None
        else map_dir / "three_virtual_tracks_overview.png"
    )
    image, meta, _ = load_ros_map(map_dir / "point_lio_map_2d.yaml")
    resolution = float(meta["resolution"])
    extent = [0.0, image.shape[1] * resolution, 0.0, image.shape[0] * resolution]

    fig, axes = plt.subplots(3, 1, figsize=(12, 22), constrained_layout=True)
    for ax, (track_name, title) in zip(axes, TRACKS):
        track_dir = map_dir / track_name
        left = read_numeric_csv(track_dir / "virtual_left_boundary.csv")
        right = read_numeric_csv(track_dir / "virtual_right_boundary.csv")
        center = read_numeric_csv(track_dir / "centerline.csv")
        race = read_numeric_csv(track_dir / "raceline.csv")

        ax.imshow(np.flipud(image), cmap="gray", origin="lower", extent=extent)
        for data, color, label, style, width in (
            (left, "#16a34a", "left boundary", "-", 1.5),
            (right, "#7c3aed", "right boundary", "-", 1.5),
            (center, "#f97316", "centerline", "--", 1.2),
            (race, "#2563eb", "NMPC raceline", "-", 1.8),
        ):
            local_x, local_y = world_to_local(data["x_m"], data["y_m"], meta)
            ax.plot(
                close_curve(local_x),
                close_curve(local_y),
                color=color,
                linestyle=style,
                linewidth=width,
                label=label,
            )
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("map-local x [m]")
        ax.set_ylabel("map-local y [m]")
        ax.set_title(title)
        ax.grid(True, alpha=0.2)
        ax.legend(loc="best")

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(f"[OK] Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
