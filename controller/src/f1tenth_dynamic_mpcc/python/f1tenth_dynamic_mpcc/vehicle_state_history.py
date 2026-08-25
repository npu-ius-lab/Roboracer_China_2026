"""Timestamped ring buffer for predicted nine-state vehicle states."""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass

import numpy as np

from .track_model import wrap_angle


@dataclass(frozen=True)
class TimedVehicleState:
    stamp: float
    state: np.ndarray


class VehicleStateHistory:
    def __init__(self, retention_s: float = 1.0):
        if not math.isfinite(retention_s) or retention_s <= 0.0:
            raise ValueError("retention_s must be positive")
        self.retention_s = float(retention_s)
        self._samples: list[TimedVehicleState] = []
        self._lock = threading.RLock()

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    def push(self, stamp: float, state: np.ndarray) -> None:
        stamp = float(stamp)
        value = np.asarray(state, dtype=float).copy()
        if not math.isfinite(stamp) or value.shape != (9,) or not np.all(np.isfinite(value)):
            raise ValueError("invalid timed vehicle state")
        with self._lock:
            stamps = [item.stamp for item in self._samples]
            index = bisect.bisect_left(stamps, stamp)
            item = TimedVehicleState(stamp, value)
            if index < len(self._samples) and self._samples[index].stamp == stamp:
                self._samples[index] = item
            else:
                self._samples.insert(index, item)
            cutoff = self._samples[-1].stamp - self.retention_s
            while len(self._samples) > 1 and self._samples[1].stamp < cutoff:
                self._samples.pop(0)

    def truncate_after(self, stamp: float) -> None:
        with self._lock:
            index = bisect.bisect_right([item.stamp for item in self._samples], float(stamp))
            del self._samples[index:]

    def state_at(self, stamp: float) -> np.ndarray | None:
        stamp = float(stamp)
        with self._lock:
            if not self._samples or stamp < self._samples[0].stamp or stamp > self._samples[-1].stamp:
                return None
            stamps = [item.stamp for item in self._samples]
            right = bisect.bisect_right(stamps, stamp)
            if right == 0:
                return None
            if right == len(self._samples) or self._samples[right - 1].stamp == stamp:
                return self._samples[right - 1].state.copy()
            first, second = self._samples[right - 1], self._samples[right]
            ratio = (stamp - first.stamp) / (second.stamp - first.stamp)
            result = first.state + ratio * (second.state - first.state)
            result[2] = first.state[2] + ratio * float(wrap_angle(second.state[2] - first.state[2]))
            return result

    def range(self, start: float, end: float) -> list[TimedVehicleState]:
        with self._lock:
            return [TimedVehicleState(x.stamp, x.state.copy()) for x in self._samples
                    if float(start) <= x.stamp <= float(end)]

    @property
    def oldest_stamp(self) -> float | None:
        with self._lock:
            return None if not self._samples else self._samples[0].stamp

    @property
    def newest_stamp(self) -> float | None:
        with self._lock:
            return None if not self._samples else self._samples[-1].stamp

