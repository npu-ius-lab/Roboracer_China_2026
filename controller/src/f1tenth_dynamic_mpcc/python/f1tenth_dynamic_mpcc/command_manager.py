"""Strict-rate continuous command synthesis independent of solver timing."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Mapping

import numpy as np

from .startup_state_machine import (
    BOOT, CRAWL_OBSERVE, SAFE_DECEL, STEER_SETTLE, STOPPED, StartupProfile
)


@dataclass(frozen=True)
class PublishedCommand:
    speed: float
    steering: float
    virtual_speed: float
    publish_dt: float
    slew_limiter_active: bool
    source: str


class CommandManager:
    def __init__(self, config: Mapping, max_steer: float):
        self.p = config["publisher"]
        self.startup = config["startup"]
        self.fallback = config["fallback"]
        self.max_steer = float(max_steer)
        self._lock = threading.Lock()
        self._desired = np.zeros(3)  # speed, desired delta_c, virtual speed
        self._desired_stamp = float("-inf")
        self._shifted = np.zeros(3)
        self._shifted_available = False
        self._published = np.zeros(3)
        self._last_publish_time: float | None = None
        self._solution_valid = False
        self._failure_count = 0
        self._used_shifted = False

    def set_solution(self, states: np.ndarray, controls: np.ndarray, now: float) -> None:
        # controls[:,1] is delta_c_dot; states[:,7] is command angle delta_c.
        desired = np.asarray([controls[0, 0], states[1, 7], controls[0, 2]], dtype=float)
        shifted_index = 2 if len(states) > 2 else 1
        shifted_control = 1 if len(controls) > 1 else 0
        shifted = np.asarray(
            [controls[shifted_control, 0], states[shifted_index, 7],
             controls[shifted_control, 2]], dtype=float
        )
        with self._lock:
            self._desired = desired
            self._shifted = shifted
            self._shifted_available = True
            self._desired_stamp = float(now)
            self._solution_valid = True
            self._failure_count = 0
            self._used_shifted = False

    def note_failure(self) -> None:
        with self._lock:
            self._failure_count += 1
            self._solution_valid = False

    def _slew(self, current: float, target: float, rate: float, dt: float) -> tuple[float, bool]:
        maximum = max(float(rate), 0.0) * dt
        change = float(np.clip(target - current, -maximum, maximum))
        return current + change, abs(target - current) > maximum + 1.0e-12

    def tick(self, now: float, profile: StartupProfile) -> PublishedCommand:
        now = float(now)
        with self._lock:
            dt = (
                1.0 / float(self.p["rate_hz"])
                if self._last_publish_time is None
                else now - self._last_publish_time
            )
            dt = float(np.clip(dt, 1.0e-3, float(self.p["max_dt_s"])))
            fresh = now - self._desired_stamp <= float(self.p["command_timeout_s"])
            center = float(self.startup["steering_center_command_rad"])
            source = "mpcc"
            target = self._desired.copy()
            if profile.mode in (BOOT, STEER_SETTLE):
                target = np.asarray([0.0, center, 0.0])
                source = "steer_settle"
            elif profile.mode == CRAWL_OBSERVE:
                target[0] = float(self.startup["crawl_speed_mps"])
                target[1] = float(np.clip(
                    target[1],
                    center - float(self.startup["crawl_delta_max_rad"]),
                    center + float(self.startup["crawl_delta_max_rad"]),
                ))
                source = "crawl_observe"
            elif profile.mode in (SAFE_DECEL, STOPPED) or not fresh:
                target[0] = 0.0
                target[1] = center
                target[2] = 0.0
                source = "safe_decel" if profile.mode != STOPPED else "stopped"
            elif (
                bool(self.fallback["single_failure_use_shifted_solution"])
                and not self._solution_valid
                and self._failure_count == 1
                and self._shifted_available
            ):
                target = self._shifted.copy()
                source = "shifted_solution"
                self._used_shifted = True
            target[0] = float(np.clip(target[0], 0.0, profile.speed_cap))
            target[1] = float(np.clip(target[1], -self.max_steer, self.max_steer))
            if math.isfinite(profile.steering_limit):
                target[1] = float(np.clip(target[1], -profile.steering_limit, profile.steering_limit))
            fallback_active = profile.mode in (SAFE_DECEL, STOPPED) or not fresh
            if fallback_active:
                accel_rate = float(self.fallback["decel_mps2"])
            else:
                accel_rate = (
                    float(self.p["acceleration_limit_mps2"])
                    if target[0] >= self._published[0]
                    else float(self.p["deceleration_limit_mps2"])
                )
            speed, speed_limited = self._slew(
                self._published[0], target[0], accel_rate, dt
            )
            steer_rate = min(
                float(self.p["steering_slew_rate_limit_radps"]),
                float(profile.steering_rate_cap),
            )
            if fallback_active:
                steer_rate = min(
                    steer_rate,
                    float(self.fallback["steering_recenter_rate_radps"]),
                )
            steering, steer_limited = self._slew(
                self._published[1], target[1], steer_rate, dt
            )
            self._published[:] = [speed, steering, max(target[2], 0.0)]
            self._last_publish_time = now
            return PublishedCommand(
                speed, steering, self._published[2], dt,
                bool(speed_limited or steer_limited), source
            )

    @property
    def published(self) -> np.ndarray:
        with self._lock:
            return self._published.copy()

    @property
    def shifted_previous_solution_used(self) -> bool:
        with self._lock:
            return self._used_shifted
