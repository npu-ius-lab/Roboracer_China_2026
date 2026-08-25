"""Steering command map and exact first-order actuator propagation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class SteeringActuatorParameters:
    identified: bool
    map_type: str
    gain: float
    bias: float
    tau: float
    delay: float
    delta_min: float
    delta_max: float

    @classmethod
    def from_config(cls, config: Mapping) -> "SteeringActuatorParameters":
        source = config["steering_actuator"]
        result = cls(
            identified=bool(source.get("identified", False)),
            map_type=str(source.get("map_type", "linear")),
            gain=float(source["gain"]),
            bias=float(source.get("bias", 0.0)),
            tau=float(source["tau"]),
            delay=float(source.get("delay", 0.0)),
            delta_min=float(source["delta_min"]),
            delta_max=float(source["delta_max"]),
        )
        result.validate()
        return result

    def validate(self) -> None:
        if self.map_type != "linear":
            raise ValueError("only the linear steering map is implemented")
        values = (self.gain, self.bias, self.tau, self.delay, self.delta_min, self.delta_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("steering actuator parameters must be finite")
        if self.tau <= 0.0 or self.delay < 0.0:
            raise ValueError("steering tau must be positive and delay non-negative")
        if self.delta_min >= self.delta_max:
            raise ValueError("invalid steering angle limits")


class SteeringActuatorModel:
    def __init__(self, parameters: SteeringActuatorParameters):
        self.p = parameters

    @classmethod
    def from_config(cls, config: Mapping) -> "SteeringActuatorModel":
        return cls(SteeringActuatorParameters.from_config(config))

    def target(self, delta_command: float) -> float:
        target = self.p.gain * float(delta_command) + self.p.bias
        return float(np.clip(target, self.p.delta_min, self.p.delta_max))

    def propagate(self, delta: float, delta_command: float, dt: float) -> float:
        """Exact discretization for arbitrary positive callback intervals."""
        dt = float(dt)
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("steering propagation dt must be finite and non-negative")
        target = self.target(delta_command)
        decay = math.exp(-dt / self.p.tau)
        estimate = target + (float(delta) - target) * decay
        return float(np.clip(estimate, self.p.delta_min, self.p.delta_max))

