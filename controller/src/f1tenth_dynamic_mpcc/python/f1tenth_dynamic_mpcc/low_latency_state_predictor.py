"""Timestamp-driven low-latency vehicle state prediction.

Primary semantic contract: x(pointlio_header_stamp) -> x(control_now).
Actuator committed-horizon prediction is deliberately a separate operation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .command_history_buffer import CommandHistoryBuffer, MissingCommandHistory
from .vehicle_model import DynamicBicycleModel
from .vehicle_state_history import VehicleStateHistory
from .sensor_history import ImuSample, SensorHistory, WheelSample
from .track_model import wrap_angle


@dataclass(frozen=True)
class PredictedVehicleState:
    state: np.ndarray
    source_stamp: float
    output_stamp: float
    measurement_age: float
    prediction_horizon: float
    valid: bool
    confidence: float
    mode: str


class LowLatencyVehicleStatePredictor:
    MODEL_ONLY = "MODEL_ONLY"

    def __init__(
        self,
        model: DynamicBicycleModel,
        command_history: CommandHistoryBuffer,
        integration_step_s: float = 0.01,
        history_retention_s: float = 1.0,
        gyro_bias: float = 0.0,
        imu_max_age_s: float = 0.03,
        wheel_gain: float = 0.20,
        wheel_max_age_s: float = 0.06,
        wheel_innovation_limit_mps: float = 0.75,
    ):
        if integration_step_s <= 0.0:
            raise ValueError("integration_step_s must be positive")
        self.model = model
        self.command_history = command_history
        self.integration_step_s = float(integration_step_s)
        self.state_history = VehicleStateHistory(history_retention_s)
        self.sensor_history = SensorHistory(history_retention_s)
        self.gyro_bias = float(gyro_bias)
        self.imu_max_age_s = float(imu_max_age_s)
        self.wheel_gain = float(wheel_gain)
        self.wheel_max_age_s = float(wheel_max_age_s)
        self.wheel_innovation_limit_mps = float(wheel_innovation_limit_mps)
        self._last_pointlio_stamp: float | None = None
        self._current_state: np.ndarray | None = None
        self._current_stamp: float | None = None
        self.use_imu = True
        self.use_wheel = True

    def clear(self) -> None:
        self.state_history.clear()
        self.sensor_history.clear()
        self._last_pointlio_stamp = None
        self._current_state = None
        self._current_stamp = None

    def push_imu(self, stamp: float, yaw_rate: float) -> None:
        self.sensor_history.push_imu(ImuSample(float(stamp), float(yaw_rate)))

    def push_wheel(self, stamp: float, vx: float) -> None:
        self.sensor_history.push_wheel(WheelSample(float(stamp), float(vx)))

    def _speed_delay(self, stamp: float, state: np.ndarray,
                     fallback: np.ndarray) -> float:
        try:
            requested = self.command_history.command_at(stamp).speed_cmd
        except MissingCommandHistory:
            requested = float(fallback[0])
        return (
            self.model.p.braking_speed_dead_time
            if self.model.p.speed_gain * requested < float(state[3]) - 0.02
            else self.model.p.speed_dead_time
        )

    def _control_at(self, stamp: float, state: np.ndarray,
                    fallback: np.ndarray) -> np.ndarray:
        try:
            command = self.command_history.effective_command_at(
                stamp, self._speed_delay(stamp, state, fallback),
                self.model.p.steering_dead_time,
            )
            return np.asarray([
                command.speed_cmd, command.steering_rate, command.virtual_speed
            ], dtype=float)
        except MissingCommandHistory:
            return np.asarray(fallback, dtype=float).copy()

    def propagate(
        self, state: np.ndarray, start_stamp: float, end_stamp: float,
        fallback_control: np.ndarray | None = None, record_history: bool = True,
    ) -> np.ndarray:
        start_stamp, end_stamp = float(start_stamp), float(end_stamp)
        if end_stamp < start_stamp:
            raise ValueError("prediction end precedes start")
        result = np.asarray(state, dtype=float).copy()
        fallback = np.zeros(3) if fallback_control is None else np.asarray(fallback_control, dtype=float)
        cursor = start_stamp
        if record_history:
            self.state_history.push(cursor, result)
        while cursor < end_stamp - 1.0e-12:
            dt = min(self.integration_step_s, end_stamp - cursor)
            control = self._control_at(cursor, result, fallback)
            result = self.model.step(result, control, dt)
            imu = self.sensor_history.latest_imu_before(cursor + dt)
            if self.use_imu and imu is not None and -1.0e-6 <= cursor + dt - imu.stamp <= self.imu_max_age_s:
                corrected_rate = imu.yaw_rate - self.gyro_bias
                result[2] = float(wrap_angle(result[2] + (corrected_rate - result[5]) * dt))
                result[5] = corrected_rate
            wheel = self.sensor_history.latest_wheel_before(cursor + dt)
            if self.use_wheel and wheel is not None and -1.0e-6 <= cursor + dt - wheel.stamp <= self.wheel_max_age_s:
                innovation = float(np.clip(
                    wheel.vx - result[3],
                    -self.wheel_innovation_limit_mps,
                    self.wheel_innovation_limit_mps,
                ))
                # Scale the sample gain by dt/nominal-wheel-period so replay
                # does not apply a full correction at every RK4 substep.
                gain = min(1.0, self.wheel_gain * dt / 0.02)
                result[3] = max(0.0, result[3] + gain * innovation)
            cursor += dt
            if record_history:
                self.state_history.push(cursor, result)
        return result

    def predict_measurement_to_now(
        self, measured_state: np.ndarray, measurement_stamp: float,
        control_now: float, max_prediction_s: float,
        fallback_control: np.ndarray | None = None,
    ) -> PredictedVehicleState:
        measurement_stamp, control_now = float(measurement_stamp), float(control_now)
        age = control_now - measurement_stamp
        if not math.isfinite(age) or age < -1.0e-6:
            raise ValueError("PointLIO timestamp is in the future")
        if age > float(max_prediction_s):
            raise TimeoutError(
                f"state age {age:.3f}s exceeds prediction limit {max_prediction_s:.3f}s"
            )
        state = self.propagate(
            measured_state, measurement_stamp, control_now, fallback_control, True
        )
        return PredictedVehicleState(
            state=state,
            source_stamp=measurement_stamp,
            output_stamp=control_now,
            measurement_age=max(age, 0.0),
            prediction_horizon=max(age, 0.0),
            valid=True,
            confidence=max(0.0, 1.0 - max(age, 0.0) / max(float(max_prediction_s), 1e-6)),
            mode=("MODEL_IMU_WHEEL" if self.sensor_history.latest_wheel_before(control_now)
                  is not None else ("MODEL_IMU" if self.sensor_history.latest_imu_before(control_now)
                  is not None else self.MODEL_ONLY)),
        )

    def correct_delayed_measurement_and_repropagate(
        self, measured_state: np.ndarray, measurement_stamp: float,
        control_now: float, max_repropagation_s: float,
        fallback_control: np.ndarray | None = None,
    ) -> PredictedVehicleState:
        """Apply PointLIO at its historical stamp and replay to control time.

        PointLIO observes X/Y/yaw/vx/vy/r. Actuator states are retained from
        history when available. Track progress is taken from the freshly
        projected measurement, preventing a second MID360-to-CG transform.
        """
        measured = np.asarray(measured_state, dtype=float).copy()
        if measured.shape != (9,) or not np.all(np.isfinite(measured)):
            raise ValueError("invalid PointLIO vehicle-centre state")
        measurement_stamp, control_now = float(measurement_stamp), float(control_now)
        age = control_now - measurement_stamp
        if age < -1.0e-6:
            raise ValueError("PointLIO timestamp is in the future")
        if age > float(max_repropagation_s):
            raise TimeoutError(
                f"PointLIO age {age:.3f}s exceeds repropagation limit "
                f"{float(max_repropagation_s):.3f}s"
            )

        is_new_measurement = (
            self._last_pointlio_stamp is None
            or measurement_stamp > self._last_pointlio_stamp + 1.0e-9
        )
        if is_new_measurement:
            historical = self.state_history.state_at(measurement_stamp)
            corrected = measured.copy()
            if historical is not None:
                corrected[6:8] = historical[6:8]
            # measured[8] was projected at the already corrected vehicle
            # centre. DO NOT apply the MID360 0.135 m transform here.
            self.state_history.truncate_after(measurement_stamp)
            self.state_history.push(measurement_stamp, corrected)
            self._current_state = corrected
            self._current_stamp = measurement_stamp
            self._last_pointlio_stamp = measurement_stamp
        elif self._current_state is None or self._current_stamp is None:
            self._current_state = measured
            self._current_stamp = measurement_stamp

        assert self._current_state is not None and self._current_stamp is not None
        if self._current_stamp > control_now + 1.0e-9:
            raise ValueError("predictor current state lies in the future")
        if self._current_stamp < control_now - 1.0e-12:
            self._current_state = self.propagate(
                self._current_state, self._current_stamp, control_now,
                fallback_control, True,
            )
            self._current_stamp = control_now

        return PredictedVehicleState(
            state=self._current_state.copy(),
            source_stamp=measurement_stamp,
            output_stamp=control_now,
            measurement_age=max(age, 0.0),
            prediction_horizon=max(age, 0.0),
            valid=True,
            confidence=max(
                0.0, 1.0 - max(age, 0.0) / max(float(max_repropagation_s), 1e-6)
            ),
            mode="REPROPAGATION",
        )

    def predict_committed_horizon(
        self, state_now: np.ndarray, control_now: float,
        fallback_control: np.ndarray | None = None,
    ) -> PredictedVehicleState:
        """Propagate through exactly one speed dead-time committed interval.

        The dynamic model queries v_cmd(t-Lv), so every speed command used in
        [now, now+Lv] was already published at or before `now`. New solver
        commands cannot influence this interval.
        """
        start = float(control_now)
        fallback = (np.zeros(3) if fallback_control is None
                    else np.asarray(fallback_control, dtype=float))
        speed_delay = self._speed_delay(start, np.asarray(state_now), fallback)
        end = start + speed_delay
        state = self.propagate(
            state_now, start, end, fallback_control, record_history=False
        )
        return PredictedVehicleState(
            state=state,
            source_stamp=start,
            output_stamp=end,
            measurement_age=0.0,
            prediction_horizon=speed_delay,
            valid=True,
            confidence=1.0,
            mode="COMMITTED_HORIZON",
        )
