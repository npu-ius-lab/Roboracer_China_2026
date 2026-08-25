"""Thread-safe timestamped IMU and wheel samples for bounded replay."""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class ImuSample:
    stamp: float
    yaw_rate: float


@dataclass(frozen=True)
class WheelSample:
    stamp: float
    vx: float


class SensorHistory:
    def __init__(self, retention_s: float = 1.0):
        self.retention_s = float(retention_s)
        if not math.isfinite(self.retention_s) or self.retention_s <= 0.0:
            raise ValueError("retention_s must be positive")
        self._imu: list[ImuSample] = []
        self._wheel: list[WheelSample] = []
        self._lock = threading.RLock()

    @staticmethod
    def _insert(items, item, retention_s):
        if not all(math.isfinite(float(x)) for x in vars(item).values()):
            raise ValueError("sensor sample must be finite")
        stamps = [x.stamp for x in items]
        index = bisect.bisect_left(stamps, item.stamp)
        if index < len(items) and items[index].stamp == item.stamp:
            items[index] = item
        else:
            items.insert(index, item)
        cutoff = items[-1].stamp - retention_s
        while len(items) > 1 and items[1].stamp < cutoff:
            items.pop(0)

    def push_imu(self, sample: ImuSample) -> None:
        with self._lock:
            self._insert(self._imu, sample, self.retention_s)

    def push_wheel(self, sample: WheelSample) -> None:
        with self._lock:
            self._insert(self._wheel, sample, self.retention_s)

    def imu_range(self, start: float, end: float) -> list[ImuSample]:
        with self._lock:
            return [x for x in self._imu if float(start) <= x.stamp <= float(end)]

    def wheel_range(self, start: float, end: float) -> list[WheelSample]:
        with self._lock:
            return [x for x in self._wheel if float(start) <= x.stamp <= float(end)]

    def latest_imu_before(self, stamp: float) -> ImuSample | None:
        with self._lock:
            index = bisect.bisect_right([x.stamp for x in self._imu], float(stamp))
            return None if index == 0 else self._imu[index - 1]

    def latest_wheel_before(self, stamp: float) -> WheelSample | None:
        with self._lock:
            index = bisect.bisect_right([x.stamp for x in self._wheel], float(stamp))
            return None if index == 0 else self._wheel[index - 1]

    def clear(self) -> None:
        with self._lock:
            self._imu.clear()
            self._wheel.clear()

