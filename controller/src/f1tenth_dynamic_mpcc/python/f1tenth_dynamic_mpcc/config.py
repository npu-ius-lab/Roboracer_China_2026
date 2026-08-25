"""Configuration loading with explicit schema checks."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    path = Path(path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected a YAML mapping: {path}")
    return value


def require(mapping: Mapping[str, Any], *keys: str) -> Any:
    value: Any = mapping
    walked: list[str] = []
    for key in keys:
        walked.append(key)
        if not isinstance(value, Mapping) or key not in value:
            raise KeyError("missing configuration key: " + ".".join(walked))
        value = value[key]
    return value


def apply_runtime_speed_cap(config: dict[str, Any], cap_mps: float) -> None:
    """Clamp every controller-side speed limit without changing the YAML file."""
    cap_mps = float(cap_mps)
    if not math.isfinite(cap_mps) or cap_mps <= 0.0:
        raise ValueError("runtime speed cap must be finite and positive")

    planning = config["speed_planning"]
    planning["max_speed_mps"] = min(float(planning["max_speed_mps"]), cap_mps)

    safety = config["safety"]
    for key in (
        "global_speed_max_mps",
        "baseline_speed_max_mps",
        "debug_speed_max_mps",
    ):
        safety[key] = min(float(safety[key]), cap_mps)


def package_root() -> Path:
    # config.py lives at <package>/python/f1tenth_dynamic_mpcc/config.py.
    return Path(__file__).resolve().parents[2]
