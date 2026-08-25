"""ROS-independent racetrack-following core for safe identification runs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class GuidedProfile:
    name: str
    speed_mps: float
    excitation_rad: float
    excitation_kind: str
    maximum_laps: float = 1.0
    timeout_s: float = 75.0
    warmup_s: float = 2.0


PROFILES = {
    # The actuator profile deliberately remains below 0.8 m/s so the
    # command-to-yaw-rate relation is predominantly kinematic.
    "track_actuator": GuidedProfile("track_actuator", 0.60, 0.070, "plateau"),
    # Tire experiments are intentionally staged.  The first field run should
    # use low before mid/high are attempted.
    "track_tire_low": GuidedProfile("track_tire_low", 0.85, 0.040, "chirp"),
    # Mid/high are straight-line caps, not fixed lap speeds. Curvature preview
    # reduces them before bends so the small venue is still respected.
    "track_tire_mid": GuidedProfile("track_tire_mid", 1.40, 0.040, "chirp"),
    "track_tire_high": GuidedProfile("track_tire_high", 1.80, 0.035, "chirp"),
    # New actuator-identification profiles. Excitation time advances only on
    # safe straight sections, so bends do not silently consume PRBS states.
    "track_static_060": GuidedProfile(
        "track_static_060", 0.60, 0.140, "static_multilevel", 1.5, 100.0
    ),
    "track_prbs_060": GuidedProfile(
        "track_prbs_060", 0.60, 0.070, "prbs", 1.5, 100.0
    ),
    "track_prbs_100": GuidedProfile(
        "track_prbs_100", 1.00, 0.050, "prbs", 2.0, 90.0
    ),
    "track_prbs_150": GuidedProfile(
        "track_prbs_150", 1.50, 0.035, "prbs", 2.0, 75.0
    ),
    "track_prbs_200": GuidedProfile(
        "track_prbs_200", 2.00, 0.030, "prbs", 2.0, 75.0
    ),
    "track_prbs_250": GuidedProfile(
        "track_prbs_250", 2.50, 0.025, "prbs", 2.0, 75.0
    ),
    "track_prbs_300": GuidedProfile(
        "track_prbs_300", 3.00, 0.020, "prbs", 2.0, 75.0
    ),
}


class SafeExcitationGenerator:
    """Deterministic, zero-mean excitation whose clock pauses when unsafe."""

    # A deterministic balanced binary sequence. Adjacent equal signs are
    # intentional and help separate pure delay from a first-order response
    # when combined with the nonuniform dwell schedule.
    _PRBS_SIGNS: Tuple[int, ...] = (
        # Every one of the four dwell-duration groups has equal positive and
        # negative time, making a complete cycle exactly zero mean.
        1, 1, -1, 1, -1, 1, 1, -1,
        1, -1, 1, 1, -1, -1, -1, 1,
        1, 1, -1, -1, -1, 1, 1, 1,
        1, -1, 1, -1, -1, -1, -1, -1,
    )
    _PRBS_DWELL: Tuple[float, ...] = (0.08, 0.12, 0.20, 0.35)
    _STATIC_LEVELS: Tuple[float, ...] = (
        # Deliberately non-monotonic to reduce correlation with track position
        # and slow localization drift while keeping a balanced complete cycle.
        0.0, 0.10, -0.04, 0.14, -0.07, 0.04, -0.14, 0.07, -0.10,
    )

    def __init__(self, profile: GuidedProfile):
        self.profile = profile
        self.active_time = 0.0
        self.enabled_time = 0.0
        self.transitions = 0
        self._last_value = 0.0

    def reset(self) -> None:
        self.active_time = 0.0
        self.enabled_time = 0.0
        self.transitions = 0
        self._last_value = 0.0

    @staticmethod
    def _scheduled_value(
        elapsed: float, values: Sequence[float], dwell: Sequence[float]
    ) -> float:
        cycle = sum(dwell[index % len(dwell)] for index in range(len(values)))
        cursor = elapsed % max(cycle, 1.0e-6)
        for index, value in enumerate(values):
            duration = dwell[index % len(dwell)]
            if cursor < duration:
                return float(value)
            cursor -= duration
        return float(values[-1])

    def step(self, dt: float, enabled: bool) -> float:
        dt = clamp(float(dt), 0.0, 0.10)
        if not enabled:
            value = 0.0
        else:
            self.enabled_time += dt
            if self.enabled_time < self.profile.warmup_s:
                value = 0.0
            elif self.profile.excitation_kind == "prbs":
                self.active_time += dt
                signs = [self.profile.excitation_rad * sign for sign in self._PRBS_SIGNS]
                value = self._scheduled_value(self.active_time, signs, self._PRBS_DWELL)
            elif self.profile.excitation_kind == "static_multilevel":
                self.active_time += dt
                scale = self.profile.excitation_rad / max(abs(v) for v in self._STATIC_LEVELS)
                levels = [scale * value for value in self._STATIC_LEVELS]
                value = self._scheduled_value(self.active_time, levels, (0.90,))
            elif self.profile.excitation_kind in ("plateau", "chirp"):
                self.active_time += dt
                if self.profile.excitation_kind == "plateau":
                    value = (
                        self.profile.excitation_rad
                        if int(self.active_time / 1.20) % 2 == 0
                        else -self.profile.excitation_rad
                    )
                else:
                    duration = max(self.profile.timeout_s, 1.0)
                    f0, f1 = 0.20, 0.90
                    slope = (f1 - f0) / duration
                    phase = 2.0 * math.pi * (
                        f0 * self.active_time + 0.5 * slope * self.active_time ** 2
                    )
                    value = self.profile.excitation_rad * math.sin(phase)
            else:
                value = 0.0
        if abs(value - self._last_value) > 1.0e-9:
            self.transitions += 1
        self._last_value = value
        return value


@dataclass(frozen=True)
class SteeringResponseModel:
    gain: float
    bias: float
    tau: float
    delay: float


@dataclass(frozen=True)
class GuidedCommand:
    speed: float
    steering: float
    base_steering: float
    excitation: float
    s: float
    contour_error: float
    heading_error: float
    physical_margin: float
    lookahead: float


@dataclass(frozen=True)
class _LocalProjection:
    """Minimum projection fields required by the identification tracker."""

    s: float
    e_contour: float
    psi_ref: float


class TrackGuidedController:
    """Pure Pursuit with bounded identification excitation and safety rollout.

    Track widths are measured from the raceline.  ``physical_margin`` is the
    shortest distance from either side of the 0.24 m wide vehicle to the
    corresponding track boundary; no extra virtual margin is hidden in it.
    """

    def __init__(
        self,
        track,
        profile: GuidedProfile,
        wheelbase_m: float = 0.320,
        body_width_m: float = 0.240,
    ):
        self.track = track
        self.profile = profile
        self.wheelbase = wheelbase_m
        self.half_width = body_width_m / 2.0
        self.max_steer = 0.35
        self.max_steer_rate = 1.50
        # Consensus from actuator_run01/run02.  The two experiments identify
        # the total response time more reliably than its tau/delay split.
        self.steering_gain = 1.147
        self.steering_bias = -0.015
        self.steering_tau = 0.073
        self.steering_delay = 0.090
        # The low-speed campaign used a 1 s / 0.12 m guard. At higher speed
        # that guard is too late: the vehicle can travel several metres while
        # the command and lower-level velocity response are settling. Keep the
        # old stable low-speed behavior, but make only the new high-speed
        # identification profiles conservative.
        high_speed_delta = max(0.0, self.profile.speed_mps - 1.50)
        self.current_abort_margin = 0.15 + 0.12 * high_speed_delta
        self.predicted_abort_margin = 0.12 + 0.30 * high_speed_delta
        self.excitation_predicted_margin = 0.15 + 0.30 * high_speed_delta
        self.start_margin = 0.25
        self.prediction_horizon = 1.00 + 0.80 * high_speed_delta
        self.prediction_dt = 0.05
        # Roll out multiple plausible actuator models and use the worst track
        # margin. This avoids declaring an experiment safe merely because one
        # unvalidated gain/delay split predicts a quick recovery.
        self.safety_models = (
            SteeringResponseModel(1.000, 0.000, 0.080, 0.000),
            SteeringResponseModel(1.158, -0.0125, 0.085, 0.075),
            SteeringResponseModel(1.136, -0.0183, 0.062, 0.105),
            SteeringResponseModel(1.080, -0.0250, 0.110, 0.135),
            SteeringResponseModel(1.200, 0.0050, 0.100, 0.135),
        )

    def physical_margin(self, projection) -> float:
        left = float(self.track.width_left(projection.s))
        right = float(self.track.width_right(projection.s))
        return min(
            left - self.half_width - projection.e_contour,
            right - self.half_width + projection.e_contour,
        )

    def _local_projection(self, x: float, y: float, s_guess: float):
        """Project near a known progress value without a new coarse search.

        Safety rollouts advance by at most 7.5 cm per integration step. Their
        previous ``s`` is therefore a much stronger initialization than the
        generic 129-point search used for live measurements. If the track
        implementation does not expose derivatives, or Newton refinement
        leaves the expected local corridor, fall back to the public projector.
        """

        if not hasattr(self.track, "derivatives"):
            return self.track.project(x, y, s_guess)
        s = float(s_guess)
        for _ in range(4):
            xr, yr = self.track.position(s)
            dx, dy, ddx, ddy = self.track.derivatives(s)
            xr, yr = float(xr), float(yr)
            dx, dy, ddx, ddy = float(dx), float(dy), float(ddx), float(ddy)
            gradient = (xr - x) * dx + (yr - y) * dy
            hessian = dx * dx + dy * dy + (xr - x) * ddx + (yr - y) * ddy
            if not math.isfinite(hessian) or abs(hessian) < 1.0e-9:
                return self.track.project(x, y, s_guess)
            step = clamp(gradient / hessian, -0.25, 0.25)
            s -= step
            if abs(step) < 1.0e-9:
                break
        xr, yr = self.track.position(s)
        dx, dy, _, _ = self.track.derivatives(s)
        xr, yr, dx, dy = float(xr), float(yr), float(dx), float(dy)
        distance = math.hypot(x - xr, y - yr)
        stationarity = abs((xr - x) * dx + (yr - y) * dy) / max(
            math.hypot(dx, dy), 1.0e-9
        )
        if (
            not math.isfinite(distance)
            or distance > 0.85
            or stationarity > 1.0e-5
        ):
            return self.track.project(x, y, s_guess)
        psi = math.atan2(dy, dx)
        e_contour = -math.sin(psi) * (x - xr) + math.cos(psi) * (y - yr)
        return _LocalProjection(s=s, e_contour=e_contour, psi_ref=psi)

    def _excitation(self, elapsed: float, curvature: float, margin: float) -> float:
        # Identification perturbations are allowed only on the two long,
        # nearly-straight parts of this particular closed track.
        if abs(curvature) >= 0.20 or margin < 0.30:
            return 0.0
        amplitude = self.profile.excitation_rad
        if self.profile.excitation_kind == "plateau":
            # 1.2 s plateaus provide enough settled samples for K/bias while
            # alternating sign prevents an accumulated drift toward one wall.
            return amplitude if int(elapsed / 1.20) % 2 == 0 else -amplitude
        if self.profile.excitation_kind == "chirp":
            duration = max(self.profile.timeout_s, 1.0)
            f0, f1 = 0.20, 0.90
            slope = (f1 - f0) / duration
            phase = 2.0 * math.pi * (f0 * elapsed + 0.5 * slope * elapsed * elapsed)
            return amplitude * math.sin(phase)
        return 0.0

    def excitation_allowed(
        self, command: GuidedCommand, holding: bool = False
    ) -> bool:
        curvature_limit = 0.22 if holding else 0.20
        margin_limit = 0.28 if holding else 0.32
        contour_limit = 0.13 if holding else 0.10
        heading_limit = 0.20 if holding else 0.15
        return (
            abs(float(self.track.curvature(command.s))) < curvature_limit
            and command.physical_margin >= margin_limit
            and abs(command.contour_error) <= contour_limit
            and abs(command.heading_error) <= heading_limit
        )

    def command(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        elapsed: float,
        s_guess: float | None,
        allow_excitation: bool = True,
        excitation_override: float | None = None,
        local_projection: bool = False,
        compute_speed_profile: bool = True,
    ) -> GuidedCommand:
        projection = (
            self._local_projection(x, y, float(s_guess))
            if local_projection and s_guess is not None
            else self.track.project(x, y, s_guess)
        )
        margin = self.physical_margin(projection)
        heading_error = wrap_angle(yaw - projection.psi_ref)
        lookahead = clamp(
            0.40 + 0.22 * max(vx, 0.0) + 0.50 * abs(projection.e_contour),
            0.45,
            0.78,
        )
        target_x, target_y = self.track.position(projection.s + lookahead)
        dx, dy = float(target_x) - x, float(target_y) - y
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)
        base = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            max(math.hypot(dx, dy), 0.05),
        )
        curvature = float(self.track.curvature(projection.s))
        if excitation_override is not None:
            excitation = float(excitation_override)
        else:
            excitation = self._excitation(elapsed, curvature, margin) if allow_excitation else 0.0
        # These are hard excitation cutoffs. Entry into an excitation section
        # uses stricter limits in excitation_allowed(); the gap provides
        # hysteresis instead of chopping a 0.9 s static platform into fragments.
        if abs(curvature) >= 0.22 or margin < 0.28:
            excitation = 0.0
        steering = clamp(base + excitation, -self.max_steer, self.max_steer)
        speed = self.profile.speed_mps
        if compute_speed_profile:
            # Preview farther ahead as speed increases. The old fixed 1.8 m
            # preview was shorter than the real stopping/response distance at
            # 2--3 m/s, so the command could remain fast until it was too late.
            preview_distance = max(1.8, 1.5 * self.profile.speed_mps)
            preview_curvature = max(
                abs(float(self.track.curvature(
                    projection.s + preview_distance * index / 12.0
                )))
                for index in range(13)
            )
            curve_cap = math.sqrt(1.35 / max(preview_curvature, 1.0e-3))
            speed = min(speed, curve_cap)
            if abs(projection.e_contour) > 0.18 or abs(heading_error) > 0.30:
                speed *= 0.65
            # Margin is measured from the vehicle side, not its centre. This
            # smoothly removes speed before the hard 0.15 m abort.
            soft_margin = max(0.30, self.current_abort_margin + 0.15)
            if margin < soft_margin:
                margin_scale = clamp(
                    (margin - self.current_abort_margin)
                    / max(soft_margin - self.current_abort_margin, 1.0e-6),
                    0.0, 1.0,
                )
                speed = min(speed, 0.45 + margin_scale * 0.75)
        return GuidedCommand(
            speed=max(speed, 0.25),
            steering=steering,
            base_steering=base,
            excitation=excitation,
            s=projection.s,
            contour_error=projection.e_contour,
            heading_error=heading_error,
            physical_margin=margin,
            lookahead=lookahead,
        )

    def predicted_minimum_margin(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        elapsed: float,
        s_guess: float,
        steering_command: float,
        future_excitation: float = 0.0,
    ) -> float:
        """Worst one-second margin over plausible steering-response models."""
        return min(
            self._predicted_minimum_margin_model(
                model, x, y, yaw, vx, elapsed, s_guess, steering_command,
                future_excitation,
            )
            for model in self.safety_models
        )

    def _predicted_minimum_margin_model(
        self,
        model: SteeringResponseModel,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        elapsed: float,
        s_guess: float,
        steering_command: float,
        future_excitation: float,
    ) -> float:
        dt = self.prediction_dt
        command_angle = steering_command
        wheel_angle = model.gain * steering_command + model.bias
        delay_steps = max(0, int(math.ceil(model.delay / dt)))
        delayed_commands = [steering_command] * delay_steps
        speed = max(vx, 0.25)
        minimum = math.inf
        steps = int(round(self.prediction_horizon / dt))
        excitation_hold = (
            0.90 if self.profile.excitation_kind == "static_multilevel" else 0.35
        )
        for index in range(steps + 1):
            rollout_excitation = future_excitation if index * dt < excitation_hold else 0.0
            target = self.command(
                x, y, yaw, speed, elapsed + index * dt, s_guess,
                allow_excitation=False, excitation_override=rollout_excitation,
                local_projection=True,
                compute_speed_profile=False,
            )
            command_angle += clamp(
                target.steering - command_angle,
                -self.max_steer_rate * dt,
                self.max_steer_rate * dt,
            )
            if delayed_commands:
                delayed_commands.append(command_angle)
                delayed_command = delayed_commands.pop(0)
            else:
                delayed_command = command_angle
            wheel_target = model.gain * delayed_command + model.bias
            wheel_angle += (1.0 - math.exp(-dt / model.tau)) * (
                wheel_target - wheel_angle
            )
            s_guess = target.s
            minimum = min(minimum, target.physical_margin)
            x += dt * speed * math.cos(yaw)
            y += dt * speed * math.sin(yaw)
            yaw = wrap_angle(yaw + dt * speed * math.tan(wheel_angle) / self.wheelbase)
        return minimum
