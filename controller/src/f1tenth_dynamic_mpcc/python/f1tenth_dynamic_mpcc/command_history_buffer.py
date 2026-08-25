"""Timestamped history of commands that were actually published to hardware."""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class PublishedCommandSample:
    stamp: float
    speed_cmd: float
    steering_cmd: float
    virtual_speed: float = 0.0
    steering_rate: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(float(v)) for v in (
            self.stamp, self.speed_cmd, self.steering_cmd, self.virtual_speed,
            self.steering_rate,
        )):
            raise ValueError("published command values must be finite")


class MissingCommandHistory(RuntimeError):
    pass


class CommandHistoryBuffer:
    """Bounded, out-of-order tolerant, zero-order-hold command FIFO.

    Hardware holds the latest command between ROS publications, so
    :meth:`command_at` defaults to zero-order hold. Linear timestamp
    interpolation is available for diagnostics, but is not used to model a
    commanded step or actuator dead time.
    """

    def __init__(self, retention_s: float = 2.0):
        if not math.isfinite(retention_s) or retention_s <= 0.0:
            raise ValueError("retention_s must be positive")
        self.retention_s = float(retention_s)
        self._samples: list[PublishedCommandSample] = []
        self._lock = threading.RLock()

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    def push(self, command: PublishedCommandSample) -> None:
        if not isinstance(command, PublishedCommandSample):
            raise TypeError("command must be PublishedCommandSample")
        with self._lock:
            stamps = [sample.stamp for sample in self._samples]
            index = bisect.bisect_left(stamps, command.stamp)
            if index < len(self._samples) and self._samples[index].stamp == command.stamp:
                self._samples[index] = command
            else:
                self._samples.insert(index, command)
            newest = self._samples[-1].stamp
            self._prune_before_locked(newest - self.retention_s)

    def _prune_before_locked(self, stamp: float) -> None:
        # Keep one sample before the cutoff to preserve ZOH at the boundary.
        while len(self._samples) > 1 and self._samples[1].stamp < stamp:
            self._samples.pop(0)

    def prune_before(self, stamp: float) -> None:
        with self._lock:
            self._prune_before_locked(float(stamp))

    @property
    def oldest_stamp(self) -> float | None:
        with self._lock:
            return None if not self._samples else self._samples[0].stamp

    @property
    def newest_stamp(self) -> float | None:
        with self._lock:
            return None if not self._samples else self._samples[-1].stamp

    def covers(self, start: float, end: float) -> bool:
        start, end = float(start), float(end)
        if end < start:
            return False
        with self._lock:
            return bool(
                self._samples
                and self._samples[0].stamp <= start
                and self._samples[-1].stamp >= end
            )

    def command_at(
        self, stamp: float, *, interpolate: bool = False,
        allow_before_oldest: bool = False,
    ) -> PublishedCommandSample:
        stamp = float(stamp)
        with self._lock:
            if not self._samples:
                raise MissingCommandHistory("command history is empty")
            stamps = [sample.stamp for sample in self._samples]
            right = bisect.bisect_right(stamps, stamp)
            if right == 0:
                if allow_before_oldest:
                    source = self._samples[0]
                    return PublishedCommandSample(
                        stamp, source.speed_cmd, source.steering_cmd,
                        source.virtual_speed, source.steering_rate
                    )
                raise MissingCommandHistory(
                    f"query {stamp:.6f} predates oldest command {stamps[0]:.6f}"
                )
            previous = self._samples[right - 1]
            if not interpolate or right >= len(self._samples) or previous.stamp == stamp:
                return PublishedCommandSample(
                    stamp, previous.speed_cmd, previous.steering_cmd,
                    previous.virtual_speed, previous.steering_rate
                )
            following = self._samples[right]
            ratio = (stamp - previous.stamp) / (following.stamp - previous.stamp)
            return PublishedCommandSample(
                stamp,
                previous.speed_cmd + ratio * (following.speed_cmd - previous.speed_cmd),
                previous.steering_cmd + ratio * (following.steering_cmd - previous.steering_cmd),
                previous.virtual_speed + ratio * (following.virtual_speed - previous.virtual_speed),
                previous.steering_rate + ratio * (following.steering_rate - previous.steering_rate),
            )

    def effective_command_at(
        self, propagation_stamp: float, speed_dead_time_s: float,
        steering_dead_time_s: float = 0.0,
    ) -> PublishedCommandSample:
        speed_delay = float(speed_dead_time_s)
        steering_delay = float(steering_dead_time_s)
        if not math.isfinite(speed_delay) or speed_delay < 0.0:
            raise ValueError("speed dead time must be finite and non-negative")
        if not math.isfinite(steering_delay) or steering_delay < 0.0:
            raise ValueError("steering dead time must be finite and non-negative")
        # One picosecond tolerance makes an exactly represented delayed edge
        # robust to binary floating-point subtraction (for example 1.13-0.13).
        speed_source = self.command_at(
            float(propagation_stamp) - speed_delay + 1.0e-12
        )
        steering_source = self.command_at(
            float(propagation_stamp) - steering_delay + 1.0e-12
        )
        return PublishedCommandSample(
            float(propagation_stamp),
            speed_source.speed_cmd,
            steering_source.steering_cmd,
            steering_source.virtual_speed,
            steering_source.steering_rate,
        )

    def range(self, start: float, end: float) -> list[PublishedCommandSample]:
        start, end = float(start), float(end)
        with self._lock:
            return [sample for sample in self._samples if start <= sample.stamp <= end]
