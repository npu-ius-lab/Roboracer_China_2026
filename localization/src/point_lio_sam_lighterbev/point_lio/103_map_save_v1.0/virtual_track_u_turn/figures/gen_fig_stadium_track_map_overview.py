#!/usr/bin/env python3
"""Reproduce the stadium-track map overview with the shared figure renderer."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


FIGURE_DIR = Path(__file__).resolve().parent
TRACK_DIR = FIGURE_DIR.parent
MAP_DIR = TRACK_DIR.parent
SHARED_RENDERER = MAP_DIR / "virtual_track" / "figures" / "gen_fig_track_map_overview.py"


def main() -> int:
    command = [
        sys.executable,
        str(SHARED_RENDERER),
        "--track-dir",
        str(TRACK_DIR),
        "--map-yaml",
        str(MAP_DIR / "point_lio_map_2d.yaml"),
        "--output-dir",
        str(FIGURE_DIR),
        "--output-stem",
        "stadium_track_map_overview",
    ]
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
