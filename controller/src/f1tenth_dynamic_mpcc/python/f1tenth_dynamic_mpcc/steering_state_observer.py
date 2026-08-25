"""No-sensor steering observer using actuator prediction and vehicle response."""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass
from typing import Mapping

import numpy as np

from .steering_actuator_model import SteeringActuatorModel
from .vehicle_model import VehicleParameters

MODEL_ONLY = "MODEL_ONLY"
KINEMATIC_CORRECTION = "KINEMATIC_CORRECTION"
DYNAMIC_CORRECTION = "DYNAMIC_CORRECTION"
# Backward-compatible alias for older telemetry readers.
MODEL_PLUS_DYNAMICS_CORRECTION = DYNAMIC_CORRECTION
INVALID = "INVALID"


@dataclass(frozen=True)
class SteeringObserverParameters:
    enabled: bool
    model_only_max_speed: float
    dynamic_correction_min_speed: float
    kinematic_gain_max: float
    dynamic_gain_max: float
    vy_derivative_cutoff_hz: float
    yaw_rate_derivative_cutoff_hz: float
    pseudo_angle_cutoff_hz: float
    innovation_limit_rad: float
    dropout_timeout_s: float
    initial_variance_rad2: float

    @classmethod
    def from_config(cls, config: Mapping) -> "SteeringObserverParameters":
        source = config["steering_observer"]
        result = cls(
            enabled=bool(source.get("enabled", True)),
            model_only_max_speed=float(source["model_only_max_speed_mps"]),
            dynamic_correction_min_speed=float(source["dynamic_correction_min_speed_mps"]),
            kinematic_gain_max=float(source["kinematic_gain_max"]),
            dynamic_gain_max=float(source["dynamic_gain_max"]),
            vy_derivative_cutoff_hz=float(source["vy_derivative_cutoff_hz"]),
            yaw_rate_derivative_cutoff_hz=float(source["yaw_rate_derivative_cutoff_hz"]),
            pseudo_angle_cutoff_hz=float(source.get("pseudo_angle_cutoff_hz", 2.0)),
            innovation_limit_rad=float(source["innovation_limit_rad"]),
            dropout_timeout_s=float(source.get("dropout_timeout_s", 0.25)),
            initial_variance_rad2=float(source.get("initial_variance_rad2", 0.04)),
        )
        result.validate()
        return result

    def validate(self) -> None:
        if self.model_only_max_speed < 0.0:
            raise ValueError("minimum observer speed must be non-negative")
        if self.dynamic_correction_min_speed <= self.model_only_max_speed:
            raise ValueError("full correction speed must exceed minimum speed")
        if not 0.0 <= self.kinematic_gain_max <= 1.0:
            raise ValueError("kinematic correction gain must be in [0, 1]")
        if not 0.0 <= self.dynamic_gain_max <= 1.0:
            raise ValueError("observer correction gain must be in [0, 1]")
        if min(
            self.vy_derivative_cutoff_hz,
            self.yaw_rate_derivative_cutoff_hz,
            self.pseudo_angle_cutoff_hz,
            self.innovation_limit_rad,
            self.dropout_timeout_s,
            self.initial_variance_rad2,
        ) <= 0.0:
            raise ValueError("observer filter, limit, timeout and variance must be positive")


@dataclass(frozen=True)
class SteeringObserverEstimate:
    timestamp: float | None
    delta_hat: float
    delta_target: float
    delta_pred: float
    delta_pseudo: float | None
    delta_kinematic: float | None
    delta_dynamic: float | None
    innovation: float
    variance: float
    confidence: float
    valid: bool
    mode: str
    correction_gain: float


class SteeringStateObserver:
    def __init__(
        self,
        actuator: SteeringActuatorModel,
        vehicle: VehicleParameters,
        parameters: SteeringObserverParameters,
    ):
        self.actuator = actuator
        self.vehicle = vehicle
        self.p = parameters
        self._lock = threading.Lock()
        self._commands: list[tuple[float, float]] = []
        self._timestamp: float | None = None
        self._delta_hat = 0.0
        self._last_vy: float | None = None
        self._last_r: float | None = None
        self._vy_dot_filtered = 0.0
        self._r_dot_filtered = 0.0
        self._pseudo_filtered: float | None = None
        self._kinematic_valid_samples = 0
        self._estimate = SteeringObserverEstimate(
            None, 0.0, self.actuator.target(0.0), 0.0, None, None, None, 0.0,
            self.p.initial_variance_rad2, 0.0, False, INVALID, 0.0
        )

    @classmethod
    def from_config(
        cls, config: Mapping, actuator: SteeringActuatorModel, vehicle: VehicleParameters
    ) -> "SteeringStateObserver":
        return cls(actuator, vehicle, SteeringObserverParameters.from_config(config))

    def push_command(self, timestamp: float, delta_command: float) -> None:
        timestamp = float(timestamp)
        if not math.isfinite(timestamp) or not math.isfinite(delta_command):
            raise ValueError("observer command must be finite")
        with self._lock:
            item = (timestamp, float(delta_command))
            if self._commands and timestamp < self._commands[-1][0]:
                index = bisect.bisect_left([entry[0] for entry in self._commands], timestamp)
                self._commands.insert(index, item)
            else:
                self._commands.append(item)
            cutoff = timestamp - 3.0
            while len(self._commands) > 1 and self._commands[1][0] < cutoff:
                self._commands.pop(0)

    def _command_at(self, timestamp: float) -> float:
        query = timestamp - self.actuator.p.delay
        command = 0.0
        for stamp, value in self._commands:
            if stamp > query:
                break
            command = value
        return command

    def _propagate_command_history(
        self, delta: float, start: float, end: float
    ) -> tuple[float, float]:
        """Propagate across every delayed command edge in an observation gap."""
        if end <= start:
            command = self._command_at(end)
            return float(delta), command
        cursor = start
        command = self._command_at(cursor + 1.0e-9)
        boundaries = sorted({
            stamp + self.actuator.p.delay
            for stamp, _ in self._commands
            if start < stamp + self.actuator.p.delay < end
        })
        for boundary in boundaries:
            delta = self.actuator.propagate(delta, command, boundary - cursor)
            cursor = boundary
            command = self._command_at(cursor + 1.0e-9)
        delta = self.actuator.propagate(delta, command, end - cursor)
        return delta, self._command_at(end + 1.0e-9)

    @staticmethod
    def _low_pass(previous: float, value: float, cutoff_hz: float, dt: float) -> float:
        alpha = 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)
        return previous + alpha * (value - previous)

    def _speed_ratio(self, vx: float) -> float:
        ratio = (abs(vx) - self.p.model_only_max_speed) / (
            self.p.dynamic_correction_min_speed - self.p.model_only_max_speed
        )
        # Smoothstep avoids a gain discontinuity at either speed threshold.
        ratio = float(np.clip(ratio, 0.0, 1.0))
        ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        return ratio

    def update(self, timestamp: float, vx: float, vy: float, yaw_rate: float) -> SteeringObserverEstimate:
        values = (timestamp, vx, vy, yaw_rate)
        if not all(math.isfinite(float(value)) for value in values):
            return self.invalidate()
        timestamp, vx, vy, yaw_rate = (float(value) for value in values)
        with self._lock:
            if self._timestamp is None:
                self._timestamp = timestamp
                self._last_vy = vy
                self._last_r = yaw_rate
                target = self.actuator.target(self._command_at(timestamp))
                self._estimate = SteeringObserverEstimate(
                    timestamp, self._delta_hat, target, self._delta_hat,
                    None, None, None, 0.0, self.p.initial_variance_rad2,
                    0.2, True, MODEL_ONLY, 0.0
                )
                return self._estimate
            dt = timestamp - self._timestamp
            if dt <= 0.0:
                return self._estimate
            delta_pred, command = self._propagate_command_history(
                self._delta_hat, self._timestamp, timestamp
            )
            model_prediction = delta_pred
            target = self.actuator.target(command)
            mode = MODEL_ONLY
            innovation = 0.0
            pseudo: float | None = None
            delta_kinematic: float | None = None
            delta_dynamic: float | None = None
            gain = 0.0
            valid_interval = dt <= self.p.dropout_timeout_s
            if valid_interval and self.p.enabled:
                vy_dot_raw = (vy - float(self._last_vy)) / dt
                r_dot_raw = (yaw_rate - float(self._last_r)) / dt
                self._vy_dot_filtered = self._low_pass(
                    self._vy_dot_filtered, vy_dot_raw,
                    self.p.vy_derivative_cutoff_hz, dt
                )
                self._r_dot_filtered = self._low_pass(
                    self._r_dot_filtered, r_dot_raw,
                    self.p.yaw_rate_derivative_cutoff_hz, dt
                )
                speed_ratio = self._speed_ratio(vx)
                if abs(vx) >= self.p.dynamic_correction_min_speed:
                    gain = self.p.dynamic_gain_max
                    vx_safe = max(abs(vx), self.vehicle.vx_regularization)
                    ay = self._vy_dot_filtered + vx * yaw_rate
                    fy_front = (
                        self.vehicle.lr * self.vehicle.mass * ay
                        + self.vehicle.yaw_inertia * self._r_dot_filtered
                    ) / self.vehicle.wheelbase
                    pseudo_raw = (
                        math.atan2(vy + self.vehicle.lf * yaw_rate, vx_safe)
                        + fy_front / self.vehicle.cf
                    )
                    pseudo_raw = float(np.clip(
                        pseudo_raw,
                        self.actuator.p.delta_min,
                        self.actuator.p.delta_max,
                    ))
                    if self._pseudo_filtered is None:
                        self._pseudo_filtered = pseudo_raw
                    else:
                        self._pseudo_filtered = self._low_pass(
                            self._pseudo_filtered, pseudo_raw,
                            self.p.pseudo_angle_cutoff_hz, dt
                        )
                    pseudo = self._pseudo_filtered
                    delta_dynamic = pseudo
                    innovation = float(np.clip(
                        pseudo - delta_pred,
                        -self.p.innovation_limit_rad,
                        self.p.innovation_limit_rad,
                    ))
                    delta_pred += gain * innovation
                    delta_pred = float(np.clip(
                        delta_pred,
                        self.actuator.p.delta_min,
                        self.actuator.p.delta_max,
                    ))
                    mode = DYNAMIC_CORRECTION
                    self._kinematic_valid_samples += 1
                elif speed_ratio > 0.0:
                    gain = self.p.kinematic_gain_max * speed_ratio
                    delta_kinematic = math.atan(
                        self.vehicle.wheelbase * yaw_rate / max(abs(vx), 0.1)
                    )
                    delta_kinematic = float(np.clip(
                        delta_kinematic,
                        self.actuator.p.delta_min,
                        self.actuator.p.delta_max,
                    ))
                    innovation = float(np.clip(
                        delta_kinematic - delta_pred,
                        -self.p.innovation_limit_rad,
                        self.p.innovation_limit_rad,
                    ))
                    delta_pred = float(np.clip(
                        delta_pred + gain * innovation,
                        self.actuator.p.delta_min,
                        self.actuator.p.delta_max,
                    ))
                    mode = KINEMATIC_CORRECTION
                    self._kinematic_valid_samples += 1
                else:
                    self._kinematic_valid_samples = 0
            self._delta_hat = delta_pred
            self._timestamp = timestamp
            self._last_vy = vy
            self._last_r = yaw_rate
            previous_variance = self._estimate.variance
            variance = (
                max(previous_variance * (1.0 - 0.5 * gain), 1.0e-5)
                if mode in (KINEMATIC_CORRECTION, DYNAMIC_CORRECTION)
                else min(previous_variance + 2.0e-4 * dt, 0.25)
            )
            if mode == MODEL_ONLY:
                confidence = 0.2
            elif mode == KINEMATIC_CORRECTION:
                # Confidence represents accumulated observability, not merely
                # one instantaneous speed sample. At crawl speed it reaches
                # the 0.6 takeover threshold after sustained valid response.
                confidence = min(0.7, 0.3 + 0.02 * self._kinematic_valid_samples)
            else:
                confidence = min(
                    1.0,
                    0.7 + 0.3 * gain / max(self.p.dynamic_gain_max, 1.0e-9),
                )
            self._estimate = SteeringObserverEstimate(
                timestamp, self._delta_hat, target, model_prediction,
                pseudo, delta_kinematic, delta_dynamic, innovation, variance,
                confidence, True, mode, gain
            )
            return self._estimate

    def invalidate(self) -> SteeringObserverEstimate:
        with self._lock:
            self._estimate = SteeringObserverEstimate(
                self._estimate.timestamp,
                self._delta_hat,
                self._estimate.delta_target,
                self._estimate.delta_pred,
                self._estimate.delta_pseudo,
                self._estimate.delta_kinematic,
                self._estimate.delta_dynamic,
                0.0,
                min(self._estimate.variance + 0.01, 0.25),
                0.0,
                False,
                INVALID,
                0.0,
            )
            return self._estimate

    def estimate(self) -> SteeringObserverEstimate:
        with self._lock:
            return self._estimate
