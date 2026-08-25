"""ROS-independent helpers for synchronizing planar vehicle states."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class TimedPlanarState:
    """A world-frame planar state at a sensor timestamp."""

    stamp: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""

    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _extrapolate(sample: TimedPlanarState, stamp: float) -> TimedPlanarState:
    dt = float(stamp) - sample.stamp
    return TimedPlanarState(
        stamp=float(stamp),
        x=sample.x + sample.vx * dt,
        y=sample.y + sample.vy * dt,
        yaw=sample.yaw,
        vx=sample.vx,
        vy=sample.vy,
    )


def interpolate_timed_state(
    samples: Sequence[TimedPlanarState],
    stamp: float,
    *,
    maximum_bracket_gap: float = 0.30,
    maximum_extrapolation: float = 0.10,
) -> Optional[TimedPlanarState]:
    """Interpolate a stamped pose, with only short bounded extrapolation.

    ``samples`` must be ordered by timestamp.  Returning ``None`` instead of a
    distant nearest neighbour is deliberate: a badly paired ego pose is more
    dangerous for track classification than dropping one target frame.
    """

    if not samples:
        return None
    requested = float(stamp)
    first = samples[0]
    last = samples[-1]
    if requested <= first.stamp:
        if first.stamp - requested <= maximum_extrapolation:
            return _extrapolate(first, requested)
        return None
    if requested >= last.stamp:
        if requested - last.stamp <= maximum_extrapolation:
            return _extrapolate(last, requested)
        return None

    for before, after in zip(samples, samples[1:]):
        if requested > after.stamp:
            continue
        gap = after.stamp - before.stamp
        if gap <= 0.0 or gap > maximum_bracket_gap:
            return None
        ratio = (requested - before.stamp) / gap
        yaw_delta = wrap_angle(after.yaw - before.yaw)
        return TimedPlanarState(
            stamp=requested,
            x=before.x + ratio * (after.x - before.x),
            y=before.y + ratio * (after.y - before.y),
            yaw=wrap_angle(before.yaw + ratio * yaw_delta),
            vx=before.vx + ratio * (after.vx - before.vx),
            vy=before.vy + ratio * (after.vy - before.vy),
        )
    return None


def body_to_world_xy(
    local_x: float, local_y: float, ego: TimedPlanarState
) -> Tuple[float, float]:
    cos_yaw = math.cos(ego.yaw)
    sin_yaw = math.sin(ego.yaw)
    return (
        ego.x + cos_yaw * float(local_x) - sin_yaw * float(local_y),
        ego.y + sin_yaw * float(local_x) + cos_yaw * float(local_y),
    )


def body_to_world_vector(
    local_x: float, local_y: float, ego_yaw: float
) -> Tuple[float, float]:
    cos_yaw = math.cos(float(ego_yaw))
    sin_yaw = math.sin(float(ego_yaw))
    return (
        cos_yaw * float(local_x) - sin_yaw * float(local_y),
        sin_yaw * float(local_x) + cos_yaw * float(local_y),
    )


def propagate_xy(x: float, y: float, vx: float, vy: float, dt: float) -> Tuple[float, float]:
    bounded_dt = max(0.0, float(dt))
    return float(x) + float(vx) * bounded_dt, float(y) + float(vy) * bounded_dt
