"""Classification of tiny out-of-order localization samples."""

from __future__ import annotations

import math


def benign_reorder(
    previous_stamp: float | None,
    current_stamp: float,
    position_delta_m: float,
    yaw_delta_rad: float,
    tolerance_s: float,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> bool:
    """Return true only for a small, pose-continuous non-increasing sample."""
    if previous_stamp is None:
        return False
    values = (
        previous_stamp, current_stamp, position_delta_m, yaw_delta_rad,
        tolerance_s, position_tolerance_m, yaw_tolerance_rad,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    regression = float(previous_stamp) - float(current_stamp)
    return bool(
        0.0 <= regression <= float(tolerance_s)
        and float(position_delta_m) <= float(position_tolerance_m)
        and float(yaw_delta_rad) <= float(yaw_tolerance_rad)
    )
