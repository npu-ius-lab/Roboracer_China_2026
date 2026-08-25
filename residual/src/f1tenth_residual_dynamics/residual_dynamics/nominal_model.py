"""Nominal body-frame dynamic bicycle model matching the MPCC equations."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import numpy as np
import yaml


def load_vehicle_config(path):
    with Path(path).open("r") as stream:
        return yaml.safe_load(stream)


@dataclass
class NominalBicycleModel:
    wheelbase: float
    mass: float
    yaw_inertia: float
    lf: float
    lr: float
    cf: float
    cr: float
    vx_epsilon: float
    speed_gain: float
    speed_tau: float
    speed_dead_time: float
    steering_gain: float
    steering_bias: float
    steering_tau: float
    steering_dead_time: float
    max_speed: float
    max_steer: float
    max_accel: float
    max_decel: float
    max_steer_rate: float
    substeps: int = 5
    blend_center: float = 0.45
    blend_width: float = 0.08
    low_speed_vy_tau: float = 0.12
    low_speed_yaw_tau: float = 0.10

    @classmethod
    def from_config(cls, config: Dict):
        v, a, limits = config["vehicle"], config["actuator"], config["limits"]
        m = config.get("model", {})
        return cls(
            wheelbase=float(v["wheelbase"]), mass=float(v["mass"]),
            yaw_inertia=float(v["yaw_inertia"]), lf=float(v["lf"]),
            lr=float(v["lr"]), cf=float(v["tire_cornering_front"]),
            cr=float(v["tire_cornering_rear"]), vx_epsilon=float(v["vx_epsilon"]),
            speed_gain=float(a["speed_gain"]), speed_tau=float(a["speed_time_constant_s"]),
            speed_dead_time=float(a.get("speed_dead_time_s", 0.0)),
            steering_gain=float(a["steering_gain"]),
            steering_bias=float(a["steering_bias_rad"]),
            steering_tau=float(a["steering_time_constant_s"]),
            steering_dead_time=float(a.get("steering_dead_time_s", 0.0)),
            max_speed=float(limits["max_speed_mps"]),
            max_steer=float(limits["max_steer_rad"]),
            max_accel=float(limits["max_accel_mps2"]),
            max_decel=float(limits["max_decel_mps2"]),
            max_steer_rate=float(limits["max_steer_rate_radps"]),
            substeps=int(m.get("integration_substeps", 5)),
            blend_center=float(m.get("low_speed_blend_center_mps", 0.45)),
            blend_width=float(m.get("low_speed_blend_width_mps", 0.08)),
            low_speed_vy_tau=float(m.get("low_speed_vy_tau_s", 0.12)),
            low_speed_yaw_tau=float(m.get("low_speed_yaw_tau_s", 0.10)),
        )

    def derivative(self, state, command):
        """Derivative of [vx, vy, yaw_rate, physical_steering]."""
        vx, vy, yaw_rate, delta = np.asarray(state, dtype=float)
        speed_cmd, steer_cmd = np.asarray(command, dtype=float)
        safe_vx = np.hypot(vx, self.vx_epsilon)
        alpha_front = delta - np.arctan2(vy + self.lf * yaw_rate, safe_vx)
        alpha_rear = -np.arctan2(vy - self.lr * yaw_rate, safe_vx)
        force_front = self.cf * alpha_front
        force_rear = self.cr * alpha_rear
        speed_cmd = np.clip(speed_cmd, 0.0, self.max_speed)
        vx_dot = np.clip(
            (self.speed_gain * speed_cmd - vx) / self.speed_tau,
            -self.max_decel, self.max_accel)
        blend = 0.5 * (np.tanh((vx - self.blend_center) / self.blend_width) + 1.0)
        vy_dynamic = (force_front + force_rear) / self.mass - vx * yaw_rate
        yaw_dynamic = (self.lf * force_front - self.lr * force_rear) / self.yaw_inertia
        yaw_kinematic = vx * np.tan(delta) / self.wheelbase
        target_delta = np.clip(
            self.steering_gain * steer_cmd + self.steering_bias,
            -self.max_steer, self.max_steer)
        delta_dot = np.clip(
            (target_delta - delta) / self.steering_tau,
            -self.max_steer_rate, self.max_steer_rate)
        return np.asarray([
            vx_dot,
            blend * vy_dynamic + (1.0 - blend) * (-vy / self.low_speed_vy_tau),
            blend * yaw_dynamic + (1.0 - blend) *
            ((yaw_kinematic - yaw_rate) / self.low_speed_yaw_tau),
            delta_dot,
        ])

    def step(self, state, command, dt):
        state = np.asarray(state, dtype=float).copy()
        h = float(dt) / self.substeps
        for _ in range(self.substeps):
            k1 = self.derivative(state, command)
            k2 = self.derivative(state + 0.5 * h * k1, command)
            k3 = self.derivative(state + 0.5 * h * k2, command)
            k4 = self.derivative(state + h * k3, command)
            state += h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        state[0] = max(0.0, state[0])
        state[3] = np.clip(state[3], -self.max_steer, self.max_steer)
        return state

    def infer_steering(self, times, steering_commands):
        """Reconstruct physical steering from commands and the nominal actuator."""
        times = np.asarray(times, dtype=float)
        commands = np.asarray(steering_commands, dtype=float)
        output = np.zeros_like(commands)
        if not len(output):
            return output
        finite = np.flatnonzero(np.isfinite(commands))
        first_command = commands[finite[0]] if len(finite) else 0.0
        output[0] = np.clip(
            self.steering_gain * first_command + self.steering_bias,
            -self.max_steer, self.max_steer)
        query = times - self.steering_dead_time
        delayed_indices = np.searchsorted(times, query, side="right") - 1
        for index in range(len(output) - 1):
            dt = np.clip(times[index + 1] - times[index], 0.0, 0.1)
            delayed_index = delayed_indices[index]
            command = (commands[delayed_index] if delayed_index >= 0 and
                       np.isfinite(commands[delayed_index]) else first_command)
            target = np.clip(self.steering_gain * command + self.steering_bias,
                             -self.max_steer, self.max_steer)
            rate = np.clip((target - output[index]) / self.steering_tau,
                           -self.max_steer_rate, self.max_steer_rate)
            output[index + 1] = np.clip(output[index] + dt * rate,
                                        -self.max_steer, self.max_steer)
        return output
