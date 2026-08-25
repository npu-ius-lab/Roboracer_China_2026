#!/usr/bin/env python3
"""Validate CSV diagnostics against the one online x/y periodic spline."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack, wrap_angle


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--track", type=Path,
        default=root / "data/tracks/virtual_track/raceline.csv",
    )
    parser.add_argument(
        "--report", type=Path,
        default=root / "docs/reports/track_geometry_consistency.md",
    )
    parser.add_argument(
        "--plot", type=Path,
        default=root / "docs/plots/kappa_csv_vs_spline.png",
    )
    args = parser.parse_args()
    track = PeriodicTrack(args.track)
    with args.track.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    psi_csv = np.asarray([float(row["psi_rad"]) for row in rows])
    kappa_csv = np.asarray([float(row["kappa_radpm"]) for row in rows])
    psi_spline = np.asarray(track.tangent(track.s_nodes))
    kappa_spline = np.asarray(track.curvature(track.s_nodes))
    psi_error = np.asarray(wrap_angle(psi_spline - psi_csv))
    kappa_error = kappa_spline - kappa_csv
    high = np.abs(kappa_csv) >= np.quantile(np.abs(kappa_csv), 0.90)
    seam_probe = np.asarray([0.0, 1e-4, track.length - 1e-4, track.length])
    seam_heading = np.asarray(track.tangent(seam_probe))
    seam_kappa = np.asarray(track.curvature(seam_probe))

    args.plot.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(track.s_nodes, kappa_csv, label="CSV kappa", lw=1.0)
    axes[0].plot(track.s_nodes, kappa_spline, label="x/y spline derived", lw=1.0)
    axes[0].set_ylabel("curvature [1/m]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(track.s_nodes, kappa_error, color="tab:red")
    axes[1].set_xlabel("s [m]")
    axes[1].set_ylabel("derived - CSV [1/m]")
    axes[1].grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(args.plot, dpi=150)
    plt.close(figure)

    def rmse(values: np.ndarray) -> float:
        return float(np.sqrt(np.mean(values * values)))

    lines = [
        "# Track geometry consistency",
        "",
        "Online geometry uses only the periodic x(s)/y(s) spline. CSV psi and "
        "kappa are diagnostic cross-checks and CSV vx/ax are not race OCP references.",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Heading wrapped RMSE | {rmse(psi_error):.6f} rad |",
        f"| Heading max absolute difference | {np.max(np.abs(psi_error)):.6f} rad |",
        f"| Heading high-curvature RMSE | {rmse(psi_error[high]):.6f} rad |",
        f"| Curvature RMSE | {rmse(kappa_error):.6f} 1/m |",
        f"| Curvature max absolute difference | {np.max(np.abs(kappa_error)):.6f} 1/m |",
        f"| Curvature high-curvature RMSE | {rmse(kappa_error[high]):.6f} 1/m |",
        f"| Spline heading seam difference | {abs(float(wrap_angle(seam_heading[-1]-seam_heading[0]))):.3e} rad |",
        f"| Spline curvature seam difference | {abs(float(seam_kappa[-1]-seam_kappa[0])):.3e} 1/m |",
        "",
        f"Plot: `../../plots/{args.plot.name}`",
        "",
    ]
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text("\n".join(lines), encoding="utf-8")
    print(args.report.resolve())


if __name__ == "__main__":
    main()
