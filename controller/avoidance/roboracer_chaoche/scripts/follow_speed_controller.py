#!/usr/bin/env python3
"""V3 Pro Max longitudinal following with short detector-dropout hold."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowSpeedConfig:
    standstill_gap: float = 0.70
    hard_stop_gap: float = 0.35
    time_headway: float = 0.35
    gap_gain: float = 0.90
    maximum_speed: float = 3.00
    maximum_acceleration: float = 1.50
    maximum_deceleration: float = 3.50
    target_speed_alpha: float = 0.35
    maximum_target_speed: float = 3.50
    target_dropout_hold: float = 0.65

    def validate(self) -> None:
        if self.hard_stop_gap < 0.0 or self.standstill_gap <= self.hard_stop_gap:
            raise ValueError("follow gaps must satisfy 0 <= hard_stop < standstill")
        if self.time_headway < 0.0 or self.gap_gain <= 0.0:
            raise ValueError("follow headway/gain is invalid")
        if min(
            self.maximum_speed,
            self.maximum_acceleration,
            self.maximum_deceleration,
            self.maximum_target_speed,
        ) <= 0.0:
            raise ValueError("follow speed and slew limits must be positive")
        if not 0.0 < self.target_speed_alpha <= 1.0:
            raise ValueError("target speed alpha must be in (0, 1]")
        if self.target_dropout_hold < 0.0:
            raise ValueError("target dropout hold must be non-negative")


class FollowSpeedController:
    def __init__(self, config: FollowSpeedConfig) -> None:
        config.validate()
        self.config = config
        self.last_cap = None
        self.last_time = None
        self.last_fresh_time = None
        self.filtered_target_speed = None
        self.target_id = None

    def reset(self) -> None:
        self.last_cap = None
        self.last_time = None
        self.last_fresh_time = None
        self.filtered_target_speed = None
        self.target_id = None

    def update(
        self,
        now: float,
        bumper_gap: float | None,
        ego_speed: float | None,
        target_speed: float | None,
        target_id: int | None,
        fresh: bool,
    ) -> tuple[float, dict]:
        cfg = self.config
        valid = bool(
            fresh
            and bumper_gap is not None
            and ego_speed is not None
            and target_speed is not None
            and all(
                math.isfinite(float(value))
                for value in (bumper_gap, ego_speed, target_speed)
            )
        )
        desired_gap = cfg.standstill_gap
        hard_stop = False
        dropout_age = None
        if valid:
            gap = float(bumper_gap)
            ego = max(0.0, float(ego_speed))
            measured_target = min(
                cfg.maximum_target_speed, max(0.0, float(target_speed))
            )
            # Keep the previous EMA through a dynamic-target ID change. The
            # detector frequently re-IDs the same slow car; resetting to a
            # one-frame zero velocity caused unnecessary braking. A fused
            # static target has a separate ID range and must reset to zero.
            is_static = target_id is not None and int(target_id) >= 1000000000
            if self.filtered_target_speed is None or is_static:
                self.filtered_target_speed = measured_target
            else:
                alpha = cfg.target_speed_alpha
                self.filtered_target_speed = (
                    (1.0 - alpha) * self.filtered_target_speed
                    + alpha * measured_target
                )
            self.target_id = target_id
            self.last_fresh_time = now
            desired_gap = cfg.standstill_gap + cfg.time_headway * ego
            requested = self.filtered_target_speed + cfg.gap_gain * (
                gap - desired_gap
            )
            if gap <= cfg.hard_stop_gap:
                requested = 0.0
                hard_stop = True
            elif gap < cfg.standstill_gap:
                scale = (gap - cfg.hard_stop_gap) / (
                    cfg.standstill_gap - cfg.hard_stop_gap
                )
                requested = min(requested, self.filtered_target_speed * scale)
            requested = min(cfg.maximum_speed, max(0.0, requested))
            reason = "hard_stop_gap" if hard_stop else "gap_and_target_speed"
        else:
            dropout_age = (
                max(0.0, now - self.last_fresh_time)
                if self.last_fresh_time is not None
                else math.inf
            )
            if self.last_cap is not None and dropout_age <= cfg.target_dropout_hold:
                requested = self.last_cap
                reason = "short_target_dropout_hold"
            else:
                requested = 0.0
                reason = "target_state_stale_deceleration"

        if hard_stop:
            output = 0.0
        elif self.last_cap is None or self.last_time is None:
            output = requested
        else:
            dt = min(0.25, max(0.0, now - self.last_time))
            lower = self.last_cap - cfg.maximum_deceleration * dt
            upper = self.last_cap + cfg.maximum_acceleration * dt
            output = min(upper, max(lower, requested))
            output = min(cfg.maximum_speed, max(0.0, output))
        self.last_cap = output
        self.last_time = now
        return output, {
            "follow_reason": reason,
            "follow_input_fresh": valid,
            "follow_bumper_gap_m": (
                float(bumper_gap) if bumper_gap is not None else None
            ),
            "follow_desired_gap_m": desired_gap,
            "follow_target_speed_mps": self.filtered_target_speed,
            "follow_target_dropout_age_s": (
                dropout_age if dropout_age is not None and math.isfinite(dropout_age) else None
            ),
            "follow_speed_cap_mps": output,
        }
