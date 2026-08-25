"""Progressive startup and recovery authority for the hardware controller."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Mapping

import numpy as np

BOOT = "BOOT"
STEER_SETTLE = "STEER_SETTLE"
CRAWL_OBSERVE = "CRAWL_OBSERVE"
MPCC_RAMP = "MPCC_RAMP"
RACE = "RACE"
SAFE_DECEL = "SAFE_DECEL"
STOPPED = "STOPPED"


@dataclass(frozen=True)
class StartupProfile:
    mode: str
    ramp_lambda: float
    speed_cap: float
    steering_rate_cap: float
    progress_scale: float
    steering_limit: float


class StartupStateMachine:
    def __init__(self, config: Mapping, race_speed_cap: float, race_steering_rate: float):
        self.p = config["startup"]
        self.fallback = config["fallback"]
        self.race_speed_cap = float(race_speed_cap)
        self.race_steering_rate = float(race_steering_rate)
        self.mode = BOOT
        self.entered_at: float | None = None
        self.observer_valid_samples = 0
        self.recovery_valid_solutions = 0
        self.failure_count = 0
        self._lock = threading.RLock()

    def reset(self, now: float) -> None:
        with self._lock:
            self.mode = BOOT
            self.entered_at = float(now)
            self.observer_valid_samples = 0
            self.recovery_valid_solutions = 0
            self.failure_count = 0

    def _enter(self, mode: str, now: float) -> None:
        self.mode = mode
        self.entered_at = float(now)
        if mode != CRAWL_OBSERVE:
            self.observer_valid_samples = 0
        if mode not in (SAFE_DECEL, STOPPED):
            self.recovery_valid_solutions = 0

    @staticmethod
    def _smoothstep(value: float) -> float:
        value = float(np.clip(value, 0.0, 1.0))
        return value * value * (3.0 - 2.0 * value)

    def note_solver(self, success: bool, now: float) -> None:
        with self._lock:
            if success:
                self.failure_count = 0
                if self.mode in (SAFE_DECEL, STOPPED):
                    self.recovery_valid_solutions += 1
                    if self.recovery_valid_solutions >= int(
                        self.p["mpcc_valid_solutions_required"]
                    ):
                        self._enter(MPCC_RAMP, now)
                return
            self.recovery_valid_solutions = 0
            self.failure_count += 1
            if self.mode not in (SAFE_DECEL, STOPPED) and self.failure_count >= int(
                self.fallback["enter_safe_decel_after_failures"]
            ):
                self._enter(SAFE_DECEL, now)
            if self.mode == SAFE_DECEL and self.failure_count >= int(
                self.fallback["stop_after_failures"]
            ):
                self._enter(STOPPED, now)

    def force_safe_decel(self, now: float) -> None:
        """Enter the continuous safety path for non-solver runtime faults."""
        with self._lock:
            if self.mode != STOPPED:
                self._enter(SAFE_DECEL, now)

    def update_observer(
        self, now: float, valid: bool, confidence: float, pointlio_valid: bool
    ) -> None:
        with self._lock:
            if self.entered_at is None:
                self.reset(now)
            if self.mode == BOOT:
                self._enter(STEER_SETTLE, now)
            if self.mode == STEER_SETTLE:
                if now - float(self.entered_at) >= float(self.p["steer_settle_time_s"]):
                    self._enter(CRAWL_OBSERVE, now)
            elif self.mode == CRAWL_OBSERVE:
                qualified = (
                    valid
                    and pointlio_valid
                    and confidence > float(self.p["observer_confidence_threshold"])
                )
                self.observer_valid_samples = self.observer_valid_samples + 1 if qualified else 0
                if self.observer_valid_samples >= int(self.p["observer_valid_samples_required"]):
                    self._enter(MPCC_RAMP, now)
            elif self.mode == MPCC_RAMP:
                if now - float(self.entered_at) >= float(self.p["mpcc_ramp_time_s"]):
                    self._enter(RACE, now)

    def profile(self, now: float) -> StartupProfile:
        with self._lock:
            crawl_speed = float(self.p["crawl_speed_mps"])
            crawl_rate = float(self.p["crawl_delta_rate_max_radps"])
            if self.mode in (BOOT, STEER_SETTLE):
                return StartupProfile(self.mode, 0.0, 0.0, crawl_rate, 0.0,
                                      float(self.p["crawl_delta_max_rad"]))
            if self.mode == STOPPED:
                return StartupProfile(self.mode, 0.0, 0.0,
                                      float(self.fallback["steering_recenter_rate_radps"]),
                                      0.0, float("inf"))
            if self.mode == CRAWL_OBSERVE:
                return StartupProfile(self.mode, 0.0, crawl_speed, crawl_rate, 0.0,
                                      float(self.p["crawl_delta_max_rad"]))
            if self.mode == MPCC_RAMP:
                elapsed = max(float(now) - float(self.entered_at), 0.0)
                lam = self._smoothstep(elapsed / float(self.p["mpcc_ramp_time_s"]))
                return StartupProfile(
                    self.mode,
                    lam,
                    crawl_speed + lam * (self.race_speed_cap - crawl_speed),
                    crawl_rate + lam * (self.race_steering_rate - crawl_rate),
                    lam,
                    float("inf"),
                )
            if self.mode == SAFE_DECEL:
                return StartupProfile(self.mode, 0.0, self.race_speed_cap,
                                      float(self.fallback["steering_recenter_rate_radps"]),
                                      0.0, float("inf"))
            return StartupProfile(RACE, 1.0, self.race_speed_cap,
                                  self.race_steering_rate, 1.0, float("inf"))
