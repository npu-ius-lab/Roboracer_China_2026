"""Pure safety policy for the isolated RoboRacer low-speed start candidate."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafeRaceStartPolicy:
    release_probe_speed_mps: float = 0.35
    pp_maximum_speed_mps: float = 1.60
    pp_acceleration_limit_mps2: float = 1.00
    pp_deceleration_limit_mps2: float = 3.00
    corridor_slow_margin_m: float = 0.15
    corridor_abort_margin_m: float = 0.02
    lateral_error_speed_gain: float = 1.50
    lateral_error_minimum_speed_mps: float = 0.80
    handoff_abort_heading_error_rad: float = 0.35

    def validate(self) -> None:
        values = (
            self.release_probe_speed_mps,
            self.pp_maximum_speed_mps,
            self.pp_acceleration_limit_mps2,
            self.pp_deceleration_limit_mps2,
            self.corridor_slow_margin_m,
            self.lateral_error_speed_gain,
            self.lateral_error_minimum_speed_mps,
            self.handoff_abort_heading_error_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("safe-start policy values must be finite and positive")
        if not math.isfinite(self.corridor_abort_margin_m):
            raise ValueError("corridor abort margin must be finite")
        if self.corridor_abort_margin_m >= self.corridor_slow_margin_m:
            raise ValueError("corridor abort margin must be below slow margin")
        if self.release_probe_speed_mps >= self.pp_maximum_speed_mps:
            raise ValueError("release probe speed must be below PP maximum speed")
        if self.lateral_error_minimum_speed_mps > self.pp_maximum_speed_mps:
            raise ValueError("lateral-error minimum exceeds PP maximum speed")

    def target_speed(
        self,
        state: str,
        nominal_speed_mps: float,
        corridor_margin_m: float | None,
        contour_error_m: float | None,
    ) -> float:
        """Return a bounded PP target; WAIT_RELEASE never arms race speed."""
        if state in {"WAIT_RELEASE", "ENABLING"}:
            return self.release_probe_speed_mps

        target = min(float(nominal_speed_mps), self.pp_maximum_speed_mps)
        if state not in {"HANDOFF_PP", "BLEND_TO_MPCC"}:
            return max(0.0, target)

        if corridor_margin_m is not None and math.isfinite(corridor_margin_m):
            scale = (
                (corridor_margin_m - self.corridor_abort_margin_m) /
                (self.corridor_slow_margin_m - self.corridor_abort_margin_m)
            )
            scale = max(0.0, min(1.0, scale))
            corridor_cap = self.release_probe_speed_mps + scale * (
                self.pp_maximum_speed_mps - self.release_probe_speed_mps
            )
            target = min(target, corridor_cap)

        if contour_error_m is not None and math.isfinite(contour_error_m):
            lateral_cap = max(
                self.lateral_error_minimum_speed_mps,
                self.pp_maximum_speed_mps
                - self.lateral_error_speed_gain * abs(contour_error_m),
            )
            target = min(target, lateral_cap)
        return max(0.0, target)

    def rate_limit(self, previous_mps: float, target_mps: float, dt_s: float) -> float:
        dt = max(0.0, min(0.10, float(dt_s)))
        lower = float(previous_mps) - self.pp_deceleration_limit_mps2 * dt
        upper = float(previous_mps) + self.pp_acceleration_limit_mps2 * dt
        return max(lower, min(upper, float(target_mps)))

    def abort_reason(
        self,
        state: str,
        corridor_margin_m: float | None,
        heading_error_rad: float | None,
    ) -> str | None:
        if state not in {"HANDOFF_PP", "BLEND_TO_MPCC"}:
            return None
        if (
            corridor_margin_m is not None and
            math.isfinite(corridor_margin_m) and
            corridor_margin_m <= self.corridor_abort_margin_m
        ):
            return "safe_start_corridor_margin_exhausted"
        if (
            heading_error_rad is not None and
            math.isfinite(heading_error_rad) and
            abs(heading_error_rad) >= self.handoff_abort_heading_error_rad
        ):
            return "safe_start_heading_error_exceeded"
        return None
