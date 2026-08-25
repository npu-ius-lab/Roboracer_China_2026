"""Pure policy helpers for the V5 JUBU local-avoidance candidate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional


class TemporalTargetConfirmation:
    """Counts unique, associated target samples before lateral commitment."""

    def __init__(self, minimum_frames: int, minimum_duration: float, maximum_gap: float):
        if minimum_frames < 2 or minimum_duration < 0.0 or maximum_gap <= 0.0:
            raise ValueError("invalid target-confirmation policy")
        self.minimum_frames = int(minimum_frames)
        self.minimum_duration = float(minimum_duration)
        self.maximum_gap = float(maximum_gap)
        self.reset()

    def reset(self) -> None:
        self.target_id: Optional[int] = None
        self.hits = 0
        self.first_received = 0.0
        self.last_received = 0.0
        self.last_measurement_stamp = -math.inf

    def update(
        self, target_id: int, measurement_stamp: float, received_at: float
    ) -> bool:
        if not all(math.isfinite(value) for value in (measurement_stamp, received_at)):
            self.reset()
            return False
        new_sample = measurement_stamp > self.last_measurement_stamp + 1.0e-6
        if self.target_id == int(target_id) and not new_sample:
            if measurement_stamp + 1.0e-6 < self.last_measurement_stamp:
                self.reset()
            else:
                return bool(
                    self.hits >= self.minimum_frames
                    and self.last_received - self.first_received + 1.0e-6
                    >= self.minimum_duration
                )
        associated = bool(
            self.target_id == int(target_id)
            and new_sample
            and 0.0 <= received_at - self.last_received <= self.maximum_gap
        )
        if not associated:
            self.target_id = int(target_id)
            self.hits = 1
            self.first_received = received_at
        else:
            self.hits += 1
        self.last_received = received_at
        self.last_measurement_stamp = measurement_stamp
        return bool(
            self.hits >= self.minimum_frames
            and received_at - self.first_received + 1.0e-6 >= self.minimum_duration
        )


@dataclass(frozen=True)
class SpeedPolicy:
    cruise_speed_cap: float = 3.0
    pass_speed_cap: float = 2.2
    follow_minimum_speed: float = 0.4
    follow_gap: float = 0.75
    follow_gain: float = 0.55

    def __post_init__(self) -> None:
        values = (
            self.cruise_speed_cap,
            self.pass_speed_cap,
            self.follow_minimum_speed,
            self.follow_gap,
            self.follow_gain,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("V5 JUBU speed policy must be finite")
        if (
            self.cruise_speed_cap <= 0.0
            or self.pass_speed_cap <= 0.0
            or self.pass_speed_cap > self.cruise_speed_cap
            or self.follow_minimum_speed < 0.0
            or self.follow_gap <= 0.0
            or self.follow_gain < 0.0
        ):
            raise ValueError("invalid V5 JUBU speed policy")

    def cap(self, state: str, payload: Mapping[str, object], healthy: bool) -> float:
        if not healthy or state == "ABORT":
            return 0.0
        if state in ("PASS", "RETURN"):
            return self.pass_speed_cap
        if state in ("FOLLOW", "PREPARE"):
            opponent_speed = max(0.0, float(payload.get("opponent_vs", 0.0)))
            gap = max(0.0, float(payload.get("bumper_gap", 0.0)))
            follow = opponent_speed + self.follow_gain * (gap - self.follow_gap)
            return max(
                self.follow_minimum_speed,
                min(self.cruise_speed_cap, follow),
            )
        return self.cruise_speed_cap


def inputs_healthy(
    now: float,
    odom_stamp: Optional[float],
    target_stamp: Optional[float],
    diagnostic_stamp: Optional[float],
    odom_timeout: float,
    target_timeout: float,
    diagnostic_timeout: float,
) -> bool:
    stamps = (odom_stamp, target_stamp, diagnostic_stamp)
    timeouts = (odom_timeout, target_timeout, diagnostic_timeout)
    if not math.isfinite(now) or any(timeout <= 0.0 for timeout in timeouts):
        return False
    if any(stamp is None or not math.isfinite(stamp) for stamp in stamps):
        return False
    return all(-1.0e-6 <= now - stamp <= timeout for stamp, timeout in zip(stamps, timeouts))
