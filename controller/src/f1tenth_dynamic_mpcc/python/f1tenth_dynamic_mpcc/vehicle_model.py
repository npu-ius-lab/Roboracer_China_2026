"""Standalone dynamic bicycle and actuator model."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np

from .config import load_yaml
from .state_layout import NU, NX


@dataclass(frozen=True)
class VehicleParameters:
    wheelbase: float
    body_width: float
    mass: float
    yaw_inertia: float
    lf: float
    lr: float
    cf: float
    cr: float
    vx_regularization: float
    speed_gain: float
    speed_tau: float
    speed_dead_time: float
    braking_speed_tau: float
    braking_speed_dead_time: float
    steering_gain: float
    steering_bias: float
    steering_tau: float
    steering_dead_time: float
    max_speed: float
    max_steer: float
    max_accel: float
    max_decel: float
    max_steer_rate: float
    lateral_accel_limit: float

    @classmethod
    def from_yaml(
        cls, vehicle_path: str | Path, controller: Mapping | None = None
    ) -> "VehicleParameters":
        cfg = load_yaml(vehicle_path)
        vehicle = cfg["vehicle"]
        model = cfg["dynamic_model"]
        limits = cfg["limits"]
        actuator = {} if controller is None else controller.get("actuator", {})
        steering = cfg.get("steering_actuator", {})
        result = cls(
            wheelbase=float(vehicle["wheelbase"]),
            body_width=float(vehicle["ego_width"]),
            mass=float(model["mass"]),
            yaw_inertia=float(model["yaw_inertia"]),
            lf=float(model["lf"]),
            lr=float(model["lr"]),
            cf=float(model["tire_cornering_front"]),
            cr=float(model["tire_cornering_rear"]),
            vx_regularization=float(model["vx_epsilon"]),
            speed_gain=float(actuator.get("speed_static_gain", 1.0)),
            speed_tau=float(actuator.get("speed_time_constant_s", limits["speed_time_constant"])),
            speed_dead_time=float(actuator.get("speed_dead_time_s", 0.0)),
            braking_speed_tau=float(actuator.get(
                "braking_speed_time_constant_s",
                actuator.get("speed_time_constant_s", limits["speed_time_constant"]),
            )),
            braking_speed_dead_time=float(actuator.get(
                "braking_speed_dead_time_s", actuator.get("speed_dead_time_s", 0.0)
            )),
            steering_gain=float(steering.get(
                "gain", actuator.get("steering_static_gain", 1.0)
            )),
            steering_bias=float(steering.get("bias", 0.0)),
            steering_tau=float(steering.get(
                "tau", actuator.get("steering_time_constant_s", limits["steering_time_constant"])
            )),
            steering_dead_time=float(steering.get(
                "delay", actuator.get("steering_dead_time_s", 0.0)
            )),
            max_speed=float(limits["max_speed"]),
            max_steer=float(limits["max_steer"]),
            max_accel=float(limits["max_accel"]),
            max_decel=float(limits["max_decel"]),
            max_steer_rate=float(limits["max_steer_rate"]),
            lateral_accel_limit=float(limits["lateral_accel_limit"]),
        )
        result.validate()
        return result

    def validate(self) -> None:
        values = vars(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("vehicle parameters must be finite")
        positive = (
            self.wheelbase,
            self.body_width,
            self.mass,
            self.yaw_inertia,
            self.lf,
            self.lr,
            self.cf,
            self.cr,
            self.vx_regularization,
            self.speed_tau,
            self.braking_speed_tau,
            self.steering_tau,
            self.max_speed,
            self.max_steer,
            self.lateral_accel_limit,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("vehicle physical parameters must be positive")
        if self.speed_dead_time < 0.0:
            raise ValueError("speed dead time must be non-negative")
        if self.braking_speed_dead_time < 0.0:
            raise ValueError("braking speed dead time must be non-negative")
        if self.steering_dead_time < 0.0:
            raise ValueError("steering dead time must be non-negative")
        if abs(self.lf + self.lr - self.wheelbase) > 1.0e-6:
            raise ValueError("lf + lr must equal wheelbase")


class DynamicBicycleModel:
    """Nine-state model: x, y, yaw, vx, vy, r, delta, delta_c, theta."""

    NX = NX
    NU = NU

    def __init__(self, parameters: VehicleParameters, integration_substeps: int = 5):
        self.p = parameters
        self.integration_substeps = int(integration_substeps)
        if self.integration_substeps < 1:
            raise ValueError("integration_substeps must be positive")

    def slip_angles(self, state: np.ndarray) -> tuple[float, float]:
        _, _, _, vx, vy, yaw_rate, delta, _, _ = np.asarray(state, dtype=float)
        # Smooth positive denominator avoids singular dynamics near standstill.
        vx_safe = math.sqrt(vx * vx + self.p.vx_regularization**2)
        alpha_f = delta - math.atan2(vy + self.p.lf * yaw_rate, vx_safe)
        alpha_r = -math.atan2(vy - self.p.lr * yaw_rate, vx_safe)
        return alpha_f, alpha_r

    def derivatives(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)
        if state.shape != (self.NX,) or control.shape != (self.NU,):
            raise ValueError("expected state shape (9,) and control shape (3,)")
        if not np.all(np.isfinite(state)) or not np.all(np.isfinite(control)):
            raise ValueError("state and control must be finite")
        _, _, yaw, vx, vy, yaw_rate, delta, steering_cmd, _ = state
        speed_cmd = float(np.clip(control[0], 0.0, self.p.max_speed))
        steering_cmd_rate = float(np.clip(
            control[1], -self.p.max_steer_rate, self.p.max_steer_rate
        ))
        virtual_speed = max(float(control[2]), 0.0)
        alpha_f, alpha_r = self.slip_angles(state)
        fy_front = self.p.cf * alpha_f
        fy_rear = self.p.cr * alpha_r

        braking_blend = 0.5 * (
            math.tanh((vx - self.p.speed_gain * speed_cmd) / 0.05) + 1.0
        )
        speed_tau = (
            (1.0 - braking_blend) * self.p.speed_tau
            + braking_blend * self.p.braking_speed_tau
        )
        raw_vx_dot = (self.p.speed_gain * speed_cmd - vx) / speed_tau
        vx_dot = float(np.clip(raw_vx_dot, -self.p.max_decel, self.p.max_accel))
        vy_dot_dynamic = (fy_front + fy_rear) / self.p.mass - vx * yaw_rate
        yaw_rate_dot_dynamic = (
            self.p.lf * fy_front - self.p.lr * fy_rear
        ) / self.p.yaw_inertia

        # Match the smooth low-speed transition used by the generated acados
        # model. The linear tire dynamics are stiff near this transition.
        blend = 0.5 * (math.tanh((vx - 0.45) / 0.08) + 1.0)
        yaw_rate_kinematic = vx * math.tan(delta) / self.p.wheelbase
        vy_dot = blend * vy_dot_dynamic + (1.0 - blend) * (-vy / 0.12)
        yaw_rate_dot = blend * yaw_rate_dot_dynamic + (1.0 - blend) * (
            (yaw_rate_kinematic - yaw_rate) / 0.10
        )
        delta_target = float(np.clip(
            self.p.steering_gain * steering_cmd + self.p.steering_bias,
            -self.p.max_steer,
            self.p.max_steer,
        ))
        raw_delta_dot = (delta_target - delta) / self.p.steering_tau
        delta_dot = float(
            np.clip(raw_delta_dot, -self.p.max_steer_rate, self.p.max_steer_rate)
        )

        return np.asarray(
            [
                vx * math.cos(yaw) - vy * math.sin(yaw),
                vx * math.sin(yaw) + vy * math.cos(yaw),
                yaw_rate,
                vx_dot,
                vy_dot,
                yaw_rate_dot,
                delta_dot,
                steering_cmd_rate,
                virtual_speed,
            ],
            dtype=float,
        )

    def step(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        result = np.asarray(state, dtype=float).copy()
        sub_dt = dt / self.integration_substeps
        for _ in range(self.integration_substeps):
            k1 = self.derivatives(result, control)
            k2 = self.derivatives(result + 0.5 * sub_dt * k1, control)
            k3 = self.derivatives(result + 0.5 * sub_dt * k2, control)
            k4 = self.derivatives(result + sub_dt * k3, control)
            result = result + sub_dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        result[3] = max(result[3], 0.0)
        result[6] = float(np.clip(result[6], -self.p.max_steer, self.p.max_steer))
        result[7] = float(np.clip(result[7], -self.p.max_steer, self.p.max_steer))
        return result
