"""Numerical core for staged TianRacer lateral identification.

The functions in this module are ROS-independent so they can be regression
tested with synthetic data.  The sign convention matches the MPCC model:
positive steering and yaw rate turn left, and Fy = C * alpha.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter, savgol_filter


def zoh(times: np.ndarray, source_times: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Zero-order hold samples, with endpoint values outside the source span."""
    indices = np.searchsorted(source_times, times, side="right") - 1
    indices = np.clip(indices, 0, len(source_times) - 1)
    return values[indices]


def smooth(values: np.ndarray, dt: float, window_s: float = 0.18) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if len(values) < 9:
        return values.copy()
    window = max(7, int(round(window_s / max(dt, 1.0e-3))) | 1)
    window = min(window, len(values) if len(values) % 2 else len(values) - 1)
    return savgol_filter(values, window, min(3, window - 2), mode="interp")


def first_order_response(
    times: np.ndarray,
    command_times: np.ndarray,
    command: np.ndarray,
    gain: float,
    tau: float,
    delay: float,
    bias: float,
    initial: float,
) -> np.ndarray:
    """Exact discrete response of K*command(t-delay)+bias through 1/(tau*s+1)."""
    delayed = zoh(times - delay, command_times, command)
    target = gain * delayed + bias
    out = np.empty(len(times), dtype=float)
    out[0] = initial
    if len(times) < 2:
        return out
    intervals = np.diff(times)
    dt = float(np.median(intervals))
    # synchronized() deliberately creates a uniform fitting grid. lfilter is
    # exactly the same recurrence as the loop below but executes in compiled
    # code, which makes grouped bootstrap practical on full 100 Hz bags.
    if np.max(np.abs(intervals - dt)) <= max(1.0e-9, 1.0e-6 * dt):
        decay = math.exp(-max(dt, 1.0e-6) / tau)
        filtered, _ = lfilter(
            [1.0 - decay], [1.0, -decay], target,
            zi=[initial - (1.0 - decay) * target[0]],
        )
        return filtered
    for index in range(1, len(times)):
        dt = max(times[index] - times[index - 1], 1.0e-6)
        decay = math.exp(-dt / tau)
        out[index] = decay * out[index - 1] + (1.0 - decay) * target[index]
    return out


def _rmse(residual: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(residual))))


def _r2(measured: np.ndarray, prediction: np.ndarray) -> float:
    denominator = float(np.sum(np.square(measured - np.mean(measured))))
    if denominator <= 1.0e-12:
        return float("nan")
    return float(1.0 - np.sum(np.square(measured - prediction)) / denominator)


def _stable_mask(
    times: np.ndarray,
    command: np.ndarray,
    minimum_age: float = 0.70,
    maximum_rate: float = 0.12,
) -> np.ndarray:
    stable = np.zeros(len(times), dtype=bool)
    age = 0.0
    for index in range(1, len(times)):
        dt = max(times[index] - times[index - 1], 0.0)
        rate = abs(command[index] - command[index - 1]) / max(dt, 1.0e-4)
        if rate < maximum_rate:
            age += dt
        else:
            age = 0.0
        stable[index] = age >= minimum_age
    return stable


@dataclass
class ActuatorFit:
    gain: float
    bias: float
    tau: float
    delay: float
    static_rmse: float
    dynamic_rmse: float
    dynamic_r2: float
    samples: int
    static_samples: int
    source: str

    def as_dict(self) -> Dict[str, float]:
        return {
            "static_gain": self.gain,
            "offset_rad": self.bias,
            "time_constant_s": self.tau,
            "dead_time_s": self.delay,
            "static_delta_rmse_rad": self.static_rmse,
            "dynamic_delta_rmse_rad": self.dynamic_rmse,
            "dynamic_r2": self.dynamic_r2,
            "samples": self.samples,
            "static_samples": self.static_samples,
            "yaw_rate_source": self.source,
        }


@dataclass
class PreparedActuatorRun:
    """Synchronized low-speed run prepared for latent steering-angle fitting."""

    name: str
    times: np.ndarray
    command: np.ndarray
    vx: np.ndarray
    yaw_rate: np.ndarray
    delta_proxy: np.ndarray
    dynamic_mask: np.ndarray
    static_mask: np.ndarray
    static_platform_fit: bool
    source: str


def prepare_actuator_run(
    times: np.ndarray,
    command: np.ndarray,
    vx: np.ndarray,
    yaw_rate: np.ndarray,
    wheelbase: float = 0.320,
    source: str = "imu",
    name: str = "run",
    static_speed_max: float = 1.25,
    dynamic_speed_max: float = 2.10,
    platform_signal: Optional[np.ndarray] = None,
) -> PreparedActuatorRun:
    """Build the low-speed equivalent-angle proxy and informative masks.

    The proxy is intentionally restricted to the predominantly kinematic speed
    range. Higher-speed bags are useful for full bicycle-model validation, but
    must not be allowed to trade tire slip against actuator delay in this fit.
    """

    times = np.asarray(times, dtype=float)
    command = np.asarray(command, dtype=float)
    vx = np.asarray(vx, dtype=float)
    yaw_rate = np.asarray(yaw_rate, dtype=float)
    if len(times) < 200 or not (len(times) == len(command) == len(vx) == len(yaw_rate)):
        raise ValueError("actuator fit needs at least 200 synchronized samples")
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0.0):
        raise ValueError("actuator timestamps must be finite and strictly increasing")
    dt = float(np.median(np.diff(times)))
    yaw_filtered = smooth(yaw_rate, dt, 0.12)
    delta_proxy = np.arctan(wheelbase * yaw_filtered / np.maximum(np.abs(vx), 0.35))
    moving_static = (
        (vx >= 0.50) & (vx <= static_speed_max) & (np.abs(yaw_filtered) < 5.0)
    )
    moving_dynamic = (
        (vx >= 0.45) & (vx <= dynamic_speed_max) & (np.abs(yaw_filtered) < 5.0)
    )
    has_platform_signal = platform_signal is not None
    if has_platform_signal:
        platform_signal = np.asarray(platform_signal, dtype=float)
        has_platform_signal = (
            len(platform_signal) == len(times)
            and np.all(np.isfinite(platform_signal))
            and np.ptp(platform_signal) >= 0.12
        )
    platform = platform_signal if has_platform_signal else command
    platform_rate_limit = 0.01 if has_platform_signal else 0.12
    platform_stable = _stable_mask(
        times, platform, minimum_age=0.70, maximum_rate=platform_rate_limit
    )
    command_rate = np.r_[
        np.inf,
        np.abs(np.diff(command)) / np.maximum(np.diff(times), 1.0e-4),
    ]
    stable = moving_static & platform_stable & (command_rate < 0.25)
    platform_fit = (
        np.sum(stable) >= 80
        and np.ptp(command[stable]) >= 0.12
        and (not has_platform_signal or np.ptp(platform[stable]) >= 0.12)
    )
    static_mask = stable if platform_fit else np.zeros(len(times), dtype=bool)
    dynamic = moving_dynamic & (np.abs(command) >= 0.015)
    if np.sum(dynamic) < 150 or np.ptp(command[dynamic]) < 0.05:
        raise ValueError(f"{name}: not enough moving steering-transition samples")
    return PreparedActuatorRun(
        name=name,
        times=times,
        command=command,
        vx=vx,
        yaw_rate=yaw_rate,
        delta_proxy=delta_proxy,
        dynamic_mask=dynamic,
        static_mask=static_mask,
        static_platform_fit=platform_fit,
        source=source,
    )


def _equal_run_residual(parts: Sequence[np.ndarray]) -> np.ndarray:
    """Concatenate residuals while giving every complete run equal weight."""

    if not parts:
        return np.empty(0, dtype=float)
    reference = float(np.mean([len(part) for part in parts]))
    return np.concatenate([
        part * math.sqrt(reference / max(len(part), 1)) for part in parts
    ])


def evaluate_steering_actuator(
    run: PreparedActuatorRun, fit: ActuatorFit
) -> Tuple[Dict[str, float], np.ndarray]:
    prediction = first_order_response(
        run.times, run.times, run.command, fit.gain, fit.tau, fit.delay,
        fit.bias, run.delta_proxy[0],
    )
    measured = run.delta_proxy[run.dynamic_mask]
    predicted = prediction[run.dynamic_mask]
    error = predicted - measured
    same_direction = np.sign(predicted) == np.sign(measured)
    optimistic = np.where(
        same_direction,
        np.maximum(np.abs(predicted) - np.abs(measured), 0.0),
        0.0,
    )
    metrics = {
        "samples": int(np.sum(run.dynamic_mask)),
        "rmse_rad": _rmse(error),
        "r2": _r2(measured, predicted),
        "bias_rad": float(np.mean(error)),
        "p95_abs_error_rad": float(np.quantile(np.abs(error), 0.95)),
        "p95_optimistic_abs_delta_rad": float(np.quantile(optimistic, 0.95)),
    }
    return metrics, prediction


def fit_steering_actuator_joint(
    runs: Sequence[PreparedActuatorRun],
    delay_grid: Optional[np.ndarray] = None,
) -> Tuple[ActuatorFit, Dict[str, object]]:
    """Jointly fit one actuator model across complete, independently recorded runs."""

    runs = list(runs)
    if not runs:
        raise ValueError("joint actuator fit requires at least one run")

    static_commands = [run.command[run.static_mask] for run in runs if np.any(run.static_mask)]
    static_targets = [run.delta_proxy[run.static_mask] for run in runs if np.any(run.static_mask)]

    def static_residual(parameters: np.ndarray) -> np.ndarray:
        return _equal_run_residual([
            parameters[0] * command + parameters[1] - target
            for command, target in zip(static_commands, static_targets)
        ])

    if static_commands:
        static_fit = least_squares(
            static_residual, np.asarray([1.0, 0.0]),
            bounds=([0.20, -0.15], [2.50, 0.15]),
            loss="soft_l1", f_scale=0.015,
        )
        static_errors = [
            static_fit.x[0] * command + static_fit.x[1] - target
            for command, target in zip(static_commands, static_targets)
        ]
        static_rmse = _rmse(np.concatenate(static_errors))
        static_seed = static_fit.x
    else:
        # PRBS-only groups still identify K/b jointly with tau/Td. The report
        # keeps static_rmse as NaN so they cannot be mistaken for plateau data.
        static_rmse = float("nan")
        static_seed = np.asarray([1.0, 0.0])

    if delay_grid is None:
        delay_grid = np.arange(0.0, 0.2501, 0.005)
    delay_grid = np.asarray(delay_grid, dtype=float)
    if len(delay_grid) == 0 or np.any(delay_grid < 0.0):
        raise ValueError("delay grid must contain non-negative candidates")

    lower_gain = max(0.20, 0.60 * static_seed[0])
    upper_gain = min(2.50, 1.40 * static_seed[0])
    best = None
    delay_scores = []
    for delay in delay_grid:
        def residual(parameters: np.ndarray) -> np.ndarray:
            parts = []
            for run in runs:
                prediction = first_order_response(
                    run.times, run.times, run.command,
                    parameters[0], parameters[1], float(delay), parameters[2],
                    run.delta_proxy[0],
                )
                parts.append(
                    prediction[run.dynamic_mask] - run.delta_proxy[run.dynamic_mask]
                )
            return _equal_run_residual(parts)

        candidate = least_squares(
            residual,
            [static_seed[0], 0.08, static_seed[1]],
            bounds=([lower_gain, 0.015, -0.15], [upper_gain, 0.50, 0.15]),
            loss="soft_l1", f_scale=0.02,
        )
        score = _rmse(residual(candidate.x))
        delay_scores.append((float(delay), score))
        if best is None or score < best[0]:
            best = (score, float(delay), candidate.x)
    assert best is not None
    _, delay, parameters = best

    source_names = sorted({run.source for run in runs})
    provisional = ActuatorFit(
        gain=float(parameters[0]), bias=float(parameters[2]),
        tau=float(parameters[1]), delay=delay,
        static_rmse=static_rmse, dynamic_rmse=float("nan"),
        dynamic_r2=float("nan"),
        samples=int(sum(np.sum(run.dynamic_mask) for run in runs)),
        static_samples=int(sum(np.sum(run.static_mask) for run in runs)),
        source="+".join(source_names),
    )
    per_run = []
    predictions = []
    measured_parts = []
    predicted_parts = []
    for run in runs:
        metrics, prediction = evaluate_steering_actuator(run, provisional)
        metrics["name"] = run.name
        metrics["source"] = run.source
        metrics["static_platform_fit"] = run.static_platform_fit
        per_run.append(metrics)
        predictions.append(prediction)
        measured_parts.append(run.delta_proxy[run.dynamic_mask])
        predicted_parts.append(prediction[run.dynamic_mask])
    measured_all = np.concatenate(measured_parts)
    predicted_all = np.concatenate(predicted_parts)
    fit = ActuatorFit(
        gain=provisional.gain, bias=provisional.bias,
        tau=provisional.tau, delay=provisional.delay,
        static_rmse=provisional.static_rmse,
        dynamic_rmse=_rmse(predicted_all - measured_all),
        dynamic_r2=_r2(measured_all, predicted_all),
        samples=provisional.samples, static_samples=provisional.static_samples,
        source=provisional.source,
    )
    return fit, {
        "runs": runs,
        "predictions": predictions,
        "per_run": per_run,
        "delay_scores": delay_scores,
        "static_gain_seed": float(static_seed[0]),
        "static_bias_seed": float(static_seed[1]),
    }


def fit_steering_actuator(
    times: np.ndarray,
    command: np.ndarray,
    vx: np.ndarray,
    yaw_rate: np.ndarray,
    wheelbase: float = 0.320,
    source: str = "imu",
    platform_signal: Optional[np.ndarray] = None,
) -> Tuple[ActuatorFit, Dict[str, np.ndarray]]:
    """Compatibility wrapper for a single-run fit."""
    run = prepare_actuator_run(
        times, command, vx, yaw_rate, wheelbase, source, name="run",
        platform_signal=platform_signal,
    )
    fit, joint = fit_steering_actuator_joint([run])
    return fit, {
        "times": run.times, "command": run.command, "vx": run.vx,
        "yaw_rate": run.yaw_rate, "delta_proxy": run.delta_proxy,
        "delta_prediction": joint["predictions"][0],
        "dynamic_mask": run.dynamic_mask, "static_mask": run.static_mask,
        "static_platform_fit": run.static_platform_fit,
    }


@dataclass
class TireFit:
    cf: float
    cr: float
    front_rmse: float
    rear_rmse: float
    front_r2: float
    rear_r2: float
    samples_front: int
    samples_rear: int

    def as_dict(self) -> Dict[str, float]:
        return {
            "Cf_N_per_rad": self.cf,
            "Cr_N_per_rad": self.cr,
            "front_force_rmse_N": self.front_rmse,
            "rear_force_rmse_N": self.rear_rmse,
            "front_force_r2": self.front_r2,
            "rear_force_r2": self.rear_r2,
            "front_samples": self.samples_front,
            "rear_samples": self.samples_rear,
        }


def _robust_positive_slope(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    result = least_squares(
        lambda p: p[0] * x - y, [100.0], bounds=([5.0], [1000.0]),
        loss="soft_l1", f_scale=0.6,
    )
    prediction = result.x[0] * x
    return float(result.x[0]), _rmse(prediction - y), _r2(y, prediction)


def fit_tire_stiffness(
    times: np.ndarray,
    command_times: np.ndarray,
    steering_command: np.ndarray,
    vx: np.ndarray,
    vy: np.ndarray,
    yaw_rate: np.ndarray,
    actuator: ActuatorFit,
    mass: float = 3.80,
    yaw_inertia: float = 0.060,
    lf: float = 0.195,
    lr: float = 0.125,
) -> Tuple[TireFit, Dict[str, np.ndarray]]:
    """Estimate equivalent axle Cf/Cr from CG-frame odometry and actuator fit."""
    times = np.asarray(times, dtype=float)
    vx, vy, yaw_rate = map(lambda x: np.asarray(x, dtype=float), (vx, vy, yaw_rate))
    if len(times) < 200 or not (len(times) == len(vx) == len(vy) == len(yaw_rate)):
        raise ValueError("tire fit needs at least 200 synchronized odometry samples")
    dt = float(np.median(np.diff(times)))
    vx_f = smooth(vx, dt, 0.18)
    vy_f = smooth(vy, dt, 0.24)
    r_f = smooth(yaw_rate, dt, 0.18)
    delta = first_order_response(
        times, np.asarray(command_times), np.asarray(steering_command),
        actuator.gain, actuator.tau, actuator.delay, actuator.bias, 0.0,
    )
    vy_dot = np.gradient(vy_f, times)
    r_dot = np.gradient(r_f, times)
    ay = vy_dot + vx_f * r_f
    length = lf + lr
    fyf = (lr * mass * ay + yaw_inertia * r_dot) / length
    fyr = (lf * mass * ay - yaw_inertia * r_dot) / length
    safe = np.hypot(vx_f, 0.20)
    alpha_f = delta - np.arctan2(vy_f + lf * r_f, safe)
    alpha_r = -np.arctan2(vy_f - lr * r_f, safe)
    common = (
        (vx_f >= 0.80) & (vx_f <= 2.40) & (np.abs(ay) <= 5.0)
        & (np.abs(r_dot) <= 20.0) & (np.abs(delta) <= 0.35)
    )
    front = common & (np.abs(alpha_f) >= 0.008) & (np.abs(alpha_f) <= 0.25)
    rear = common & (np.abs(alpha_r) >= 0.008) & (np.abs(alpha_r) <= 0.25)
    if np.sum(front) < 100 or np.sum(rear) < 100:
        raise ValueError("not enough informative tire-slip samples")
    cf, front_rmse, front_r2 = _robust_positive_slope(alpha_f[front], fyf[front])
    cr, rear_rmse, rear_r2 = _robust_positive_slope(alpha_r[rear], fyr[rear])
    fit = TireFit(
        cf=cf, cr=cr, front_rmse=front_rmse, rear_rmse=rear_rmse,
        front_r2=front_r2, rear_r2=rear_r2,
        samples_front=int(np.sum(front)), samples_rear=int(np.sum(rear)),
    )
    return fit, {
        "times": times, "delta": delta, "vx": vx_f, "vy": vy_f,
        "yaw_rate": r_f, "alpha_f": alpha_f, "alpha_r": alpha_r,
        "fyf": fyf, "fyr": fyr, "front_mask": front, "rear_mask": rear,
    }
