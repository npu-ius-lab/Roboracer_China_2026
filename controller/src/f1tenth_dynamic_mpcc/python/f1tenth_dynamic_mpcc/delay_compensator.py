"""Deprecated compatibility wrapper for LowLatencyVehicleStatePredictor."""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass

import numpy as np

from .command_history_buffer import CommandHistoryBuffer, MissingCommandHistory
from .low_latency_state_predictor import LowLatencyVehicleStatePredictor
from .vehicle_model import DynamicBicycleModel


@dataclass(frozen=True)
class TimedCommand:
    stamp: float
    control: np.ndarray


class DelayCompensator:
    """Legacy API; new code must use LowLatencyVehicleStatePredictor."""
    def __init__(
        self, model: DynamicBicycleModel, integration_step_s: float = 0.01,
        command_history: CommandHistoryBuffer | None = None,
    ):
        if integration_step_s <= 0.0:
            raise ValueError("integration_step_s must be positive")
        self.model = model
        self.integration_step_s = float(integration_step_s)
        self.command_history = command_history
        self._predictor = (
            None if command_history is None else LowLatencyVehicleStatePredictor(
                model, command_history, integration_step_s
            )
        )
        self._commands: list[TimedCommand] = []
        self._lock = threading.Lock()

    def clear(self) -> None:
        with self._lock:
            self._commands.clear()

    def push(self, stamp: float, control: np.ndarray) -> None:
        stamp = float(stamp)
        control = np.asarray(control, dtype=float).copy()
        if not math.isfinite(stamp) or control.shape != (self.model.NU,):
            raise ValueError("invalid timestamp or command shape")
        if not np.all(np.isfinite(control)):
            raise ValueError("command must be finite")
        with self._lock:
            if self._commands and stamp < self._commands[-1].stamp:
                index = bisect.bisect_left([item.stamp for item in self._commands], stamp)
                self._commands.insert(index, TimedCommand(stamp, control))
            else:
                self._commands.append(TimedCommand(stamp, control))
            cutoff = stamp - 2.0
            while len(self._commands) > 1 and self._commands[1].stamp < cutoff:
                self._commands.pop(0)

    def predict(
        self,
        measured_state: np.ndarray,
        measurement_stamp: float,
        target_stamp: float,
        max_prediction_s: float,
        default_control: np.ndarray | None = None,
    ) -> tuple[np.ndarray, float]:
        measurement_stamp = float(measurement_stamp)
        target_stamp = float(target_stamp)
        age = target_stamp - measurement_stamp
        if age < -1.0e-6:
            raise ValueError("measurement timestamp is in the future")
        if age > max_prediction_s:
            raise TimeoutError(
                f"state age {age:.3f}s exceeds prediction limit {max_prediction_s:.3f}s"
            )
        state = np.asarray(measured_state, dtype=float).copy()
        if state.shape != (self.model.NX,):
            raise ValueError(f"expected a {self.model.NX}-state vector")
        if age <= 1.0e-9:
            return state, max(age, 0.0)

        zero = np.zeros(self.model.NU, dtype=float)
        if default_control is None:
            default_control = zero
        active = np.asarray(default_control, dtype=float).copy()
        with self._lock:
            commands = list(self._commands)
        for item in commands:
            if item.stamp <= measurement_stamp:
                active = item.control.copy()
            else:
                break

        cursor = measurement_stamp
        future = [item for item in commands if measurement_stamp < item.stamp < target_stamp]
        future.append(TimedCommand(target_stamp, active))
        for item in future:
            while cursor < item.stamp - 1.0e-12:
                dt = min(self.integration_step_s, item.stamp - cursor)
                model_control = active
                if self.command_history is not None:
                    try:
                        try:
                            requested = self.command_history.command_at(cursor).speed_cmd
                        except MissingCommandHistory:
                            requested = float(active[0])
                        speed_delay = (
                            self.model.p.braking_speed_dead_time
                            if self.model.p.speed_gain * requested < float(state[3]) - 0.02
                            else self.model.p.speed_dead_time
                        )
                        effective = self.command_history.effective_command_at(
                            cursor, speed_delay,
                            self.model.p.steering_dead_time,
                        )
                        model_control = np.asarray([
                            effective.speed_cmd,
                            effective.steering_rate,
                            effective.virtual_speed,
                        ])
                    except MissingCommandHistory:
                        # Startup may precede the first retained command. The
                        # caller-provided control remains the explicit fallback.
                        model_control = active
                state = self.model.step(state, model_control, dt)
                cursor += dt
            if item.stamp < target_stamp:
                active = item.control.copy()
        return state, age
