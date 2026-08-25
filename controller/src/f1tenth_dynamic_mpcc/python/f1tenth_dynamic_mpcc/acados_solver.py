"""acados SQP_RTI OCP for low-speed baseline and Dynamic MPCC.

V1 follows the execution specification's approximate-MPCC strategy: periodic
track splines are evaluated outside acados at the shifted warm-start theta for
each stage, then passed as stage parameters. Geometry is frozen only within one
SQP_RTI solve; theta and virtual progress speed remain optimization variables.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .track_model import PeriodicTrack
from .state_layout import NU, NX
from .vehicle_model import DynamicBicycleModel, VehicleParameters
from .warm_start import WarmStart


NP = 14
# xr, yr, psi_r, kappa, w_left, w_right, speed_prior,
# previous v_cmd, previous delta_c_dot, previous v_theta,
# track margin, speed-prior weight, progress reward, heading weight


def v2_residual_correction_symbolic(ca, features, residual):
    """Build the schema-V2 residual exactly as the Python/C++ runtime does."""
    mean = ca.DM(residual["mean"])
    scale = ca.DM(residual["scale"])
    weights = ca.DM(residual["weights"])
    limits = ca.DM(residual["output_limits"])
    envelope = ca.DM(residual["envelope"])
    normalized_tail = (features[1:] - mean) / scale
    normalized = ca.vertcat(features[0], normalized_tail)
    raw = (
        ca.mtimes(normalized.T, weights).T
        * float(residual.get("deployment_scale", 1.0))
    )
    bounded = limits * ca.tanh(raw / limits)
    ratio = ca.fabs(normalized_tail) / envelope
    temperature = max(
        float(residual.get("gate", {}).get("temperature", 20.0)), 1.0e-3)
    # CasADi's logsumexp is stable and smooth; unlike an explicit mmax
    # stabilization it does not add a non-smooth operation to the OCP graph.
    q = (
        ca.logsumexp(temperature * ratio)
        - math.log(len(residual["envelope"]))
    ) / temperature
    gate_excess = temperature * (q - 1.0)
    softplus = ca.logsumexp(ca.vertcat(0.0, gate_excess)) / temperature
    confidence = ca.exp(
        -float(residual.get("gate", {}).get("confidence_softness", 8.0))
        * softplus * softplus)
    active = ca.DM(residual.get("active_outputs", [1, 1, 1]))
    return confidence * (bounded * active)


@dataclass
class SolverResult:
    success: bool
    status: int
    solve_time_s: float
    objective: float
    states: np.ndarray
    controls: np.ndarray
    track_slack_max: float
    tire_slack_max: float
    qp_iterations: int
    rti_iterations: int
    first_steer_rate_radps: float
    reason: str = ""


def stage_parameters(
    track: PeriodicTrack,
    theta_sequence: np.ndarray,
    previous_control: np.ndarray,
    track_margin: float,
    speed_prior_weight: float,
    progress_reward: float,
    heading_weight: float,
) -> np.ndarray:
    theta_sequence = np.asarray(theta_sequence, dtype=float)
    previous_control = np.asarray(previous_control, dtype=float)
    if theta_sequence.ndim != 1 or previous_control.ndim not in (1, 2):
        raise ValueError("invalid theta sequence or previous control")
    if previous_control.ndim == 1:
        if previous_control.shape != (3,):
            raise ValueError("invalid previous control")
        previous_control = np.tile(previous_control, (len(theta_sequence), 1))
    elif previous_control.shape != (len(theta_sequence), 3):
        raise ValueError("previous-control schedule must match theta sequence")
    values = np.zeros((len(theta_sequence), NP), dtype=float)
    for index, theta in enumerate(theta_sequence):
        values[index, :7] = track.geometry(float(theta))
        values[index, 7:10] = previous_control[index]
        values[index, 10:] = [
            track_margin, speed_prior_weight, progress_reward, heading_weight
        ]
    return values


class AcadosDynamicMPCC:
    NX = NX
    NU = NU

    def __init__(
        self,
        track: PeriodicTrack,
        vehicle: VehicleParameters,
        controller: dict[str, Any],
        generated_dir: str | Path,
        formulation: str = "mpcc",
        build: bool = True,
    ):
        if formulation not in {"baseline", "mpcc"}:
            raise ValueError("formulation must be baseline or mpcc")
        self.track = track
        self.vehicle = vehicle
        self.controller = controller
        self.formulation = formulation
        timing = controller["timing"]
        self.N = int(timing["horizon_steps"])
        self.dt = float(timing["horizon_dt_s"])
        self.generated_dir = Path(generated_dir).expanduser().resolve()
        self.generated_dir.mkdir(parents=True, exist_ok=True)
        self.warm = WarmStart(self.N, self.NX, self.NU)
        self.rollout_model = DynamicBicycleModel(vehicle)
        self.previous_control = np.zeros(self.NU)
        self._last_runtime_bounds: tuple[float, float] | None = None
        self.solver = self._create_solver(build=build)

    def _speed_cap(self) -> float:
        """Return the one speed ceiling used by bounds, references and warm starts."""
        safety = self.controller["safety"]
        speed_cap = min(float(safety["global_speed_max_mps"]), self.vehicle.max_speed)
        if self.formulation == "baseline":
            speed_cap = min(speed_cap, float(safety["baseline_speed_max_mps"]))
        elif str(self.controller.get("mode", "debug")).lower() == "debug":
            speed_cap = min(speed_cap, float(safety["debug_speed_max_mps"]))
        return speed_cap

    def _create_solver(self, build: bool):
        try:
            import casadi as ca
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except ImportError as exc:
            raise RuntimeError(
                "casadi/acados_template unavailable; source the acados environment"
            ) from exc

        p_v = self.vehicle
        cfg = self.controller
        constraints = cfg["constraints"]
        cost = cfg["cost"]
        safety = cfg["safety"]

        x = ca.SX.sym("x", self.NX)
        xdot = ca.SX.sym("xdot", self.NX)
        u = ca.SX.sym("u", self.NU)
        p = ca.SX.sym("p", NP)
        px, py, yaw, vx, vy, yaw_rate, delta, steering_cmd, theta = ca.vertsplit(x)
        speed_cmd, steering_cmd_rate, virtual_speed = ca.vertsplit(u)
        xr, yr, psi_r, kappa, w_left, w_right, speed_prior = ca.vertsplit(p[:7])
        previous_u = p[7:10]
        track_margin, speed_prior_weight, progress_reward, heading_weight = ca.vertsplit(p[10:])

        vx_safe = ca.sqrt(vx * vx + p_v.vx_regularization**2)
        alpha_f = delta - ca.atan2(vy + p_v.lf * yaw_rate, vx_safe)
        alpha_r = -ca.atan2(vy - p_v.lr * yaw_rate, vx_safe)
        beta = ca.atan2(vy, vx_safe)
        fy_front = p_v.cf * alpha_f
        fy_rear = p_v.cr * alpha_r
        braking_blend = 0.5 * (
            ca.tanh((vx - p_v.speed_gain * speed_cmd) / 0.05) + 1.0
        )
        speed_tau = (
            (1.0 - braking_blend) * p_v.speed_tau
            + braking_blend * p_v.braking_speed_tau
        )
        speed_accel = (p_v.speed_gain * speed_cmd - vx) / speed_tau
        delta_target = ca.fmin(
            ca.fmax(
                p_v.steering_gain * steering_cmd + p_v.steering_bias,
                -p_v.max_steer,
            ),
            p_v.max_steer,
        )
        steer_rate = (delta_target - delta) / p_v.steering_tau

        # Smooth low-speed dynamic blending, equivalent to the standalone model.
        blend = 0.5 * (ca.tanh((vx - 0.45) / 0.08) + 1.0)
        yaw_rate_kinematic = vx * ca.tan(delta) / p_v.wheelbase
        vy_dot_dynamic = (fy_front + fy_rear) / p_v.mass - vx * yaw_rate
        yaw_rate_dot_dynamic = (
            p_v.lf * fy_front - p_v.lr * fy_rear
        ) / p_v.yaw_inertia
        vx_dot = speed_accel
        vy_dot = blend * vy_dot_dynamic + (1.0 - blend) * (-vy / 0.12)
        yaw_rate_dot = (
            blend * yaw_rate_dot_dynamic
            + (1.0 - blend) * ((yaw_rate_kinematic - yaw_rate) / 0.10)
        )
        nominal_vx_dot = vx_dot
        nominal_vy_dot = vy_dot
        nominal_yaw_rate_dot = yaw_rate_dot
        residual_cfg = cfg.get("residual_dynamics", {})
        if bool(residual_cfg.get("enabled", False)):
            with Path(residual_cfg["model_path"]).open("r", encoding="utf-8") as stream:
                residual = yaml.safe_load(stream)
            schema_version = int(residual.get("schema_version", 1))
            feature_set = residual.get("feature_set") or residual.get("metadata", {}).get("feature_set")
            required = residual_cfg.get("required_feature_set", feature_set)
            if feature_set != required:
                raise RuntimeError(
                    f"residual feature set {feature_set!r} does not match {required!r}"
                )
            base_features = ca.vertcat(
                1.0, vx, vy, yaw_rate, delta, speed_cmd, steering_cmd,
                speed_cmd - vx, steering_cmd - delta,
                vx * yaw_rate, vy * yaw_rate, ca.fabs(vx) * vy,
                ca.fabs(vx) * yaw_rate,
                vx * ca.tan(delta if schema_version == 2 else ca.fmin(ca.fmax(delta, -0.7), 0.7)),
                vx * steering_cmd, vx * vx * steering_cmd,
            )
            expected_base_names = [
                "bias", "vx", "vy", "yaw_rate", "delta", "speed_cmd",
                "steer_cmd", "speed_error", "steer_error", "vx_yaw_rate",
                "vy_yaw_rate", "abs_vx_vy", "abs_vx_yaw_rate",
                "vx_tan_delta", "vx_steer_cmd", "vx2_steer_cmd",
            ]
            if schema_version == 1:
                if residual["feature_names"] != expected_base_names:
                    raise RuntimeError("residual feature ordering does not match acados model")
                mean = ca.DM(residual["feature_mean"])
                scale = ca.DM(residual["feature_scale"])
                coefficients = ca.DM(residual["coefficients_feature_by_output"])
                limits = ca.DM(residual["output_limits"])
                envelope = ca.DM(residual["feature_abs_z_limit"])
                fade = float(residual.get("ood_fade_ratio", 1.5))
                normalized = (base_features - mean) / scale
                raw = ca.mtimes(normalized.T, coefficients).T
                bounded = ca.fmin(ca.fmax(raw, -limits), limits)
                ratio = ca.mmax(ca.fabs(normalized) / envelope)
                confidence = ca.fmin(ca.fmax((fade - ratio) / (fade - 1.0), 0.0), 1.0)
                correction = confidence * bounded
            elif schema_version == 2:
                physics_features = ca.vertcat(
                    beta,
                    alpha_f,
                    alpha_r,
                    vx * alpha_f,
                    vx * alpha_r,
                    delta_target - delta,
                    nominal_vx_dot,
                    nominal_vy_dot,
                    nominal_yaw_rate_dot,
                )
                physics_names = expected_base_names + [
                    "beta", "alpha_f", "alpha_r", "vx_alpha_f", "vx_alpha_r",
                    "delta_target_delta", "nominal_vx_dot", "nominal_vy_dot",
                    "nominal_yaw_rate_dot",
                ]
                history_names = physics_names + [
                    "steer_cmd_t050ms",
                    "steer_cmd_t100ms", "steer_cmd_t150ms",
                    "steer_cmd_t200ms", "speed_cmd_t100ms",
                    "speed_cmd_t200ms",
                ]
                if residual["feature_names"] == history_names:
                    raise RuntimeError(
                        "markov_history_v2 cannot be deployed causally in the current "
                        "acados state; use a validated physics_v2 model"
                    )
                if residual["feature_names"] != physics_names:
                    raise RuntimeError(
                        "residual v2 feature ordering does not match physics_v2 acados model"
                    )
                features = ca.vertcat(base_features, physics_features)
                correction = v2_residual_correction_symbolic(
                    ca, features, residual)
            else:
                raise RuntimeError(f"unsupported residual schema {schema_version}")
            vx_dot += correction[0]
            vy_dot += correction[1]
            yaw_rate_dot += correction[2]
        f_expl = ca.vertcat(
            vx * ca.cos(yaw) - vy * ca.sin(yaw),
            vx * ca.sin(yaw) + vy * ca.cos(yaw),
            yaw_rate,
            vx_dot,
            vy_dot,
            yaw_rate_dot,
            steer_rate,
            steering_cmd_rate,
            virtual_speed,
        )

        dx, dy = px - xr, py - yr
        e_contour = -ca.sin(psi_r) * dx + ca.cos(psi_r) * dy
        e_lag = ca.cos(psi_r) * dx + ca.sin(psi_r) * dy
        e_heading = ca.atan2(ca.sin(yaw - psi_r), ca.cos(yaw - psi_r))
        v_parallel = vx * ca.cos(yaw - psi_r) - vy * ca.sin(yaw - psi_r)
        delta_u = u - previous_u

        def weighted_residual(weight_name, expression):
            return math.sqrt(float(cost[weight_name])) * expression

        if self.formulation == "baseline":
            geometry_residuals = [
                weighted_residual("contour", dx),
                weighted_residual("contour", dy),
            ]
        else:
            geometry_residuals = [
                weighted_residual("contour", e_contour),
                weighted_residual("lag", e_lag),
            ]
        state_residuals = geometry_residuals + [
            ca.sqrt(ca.fmax(heading_weight, 0.0)) * e_heading,
            weighted_residual("sideslip", beta),
            weighted_residual("yaw_rate_tracking", yaw_rate - kappa * v_parallel),
            ca.sqrt(ca.fmax(speed_prior_weight, 0.0)) * (vx - speed_prior),
        ]
        virtual_weight = float(cost["virtual_speed"])
        # This square is exactly q*v_theta^2-r*v_theta up to a constant, while
        # keeping a positive-semidefinite Gauss-Newton Hessian.
        virtual_target = progress_reward / (2.0 * virtual_weight)
        stage_residuals = state_residuals + [
            weighted_residual("speed_command", speed_cmd),
            weighted_residual("steering_command", steering_cmd),
            math.sqrt(virtual_weight) * (virtual_speed - virtual_target),
            weighted_residual("speed_command_rate", delta_u[0]),
            # delta_c_dot is now the optimized steering input itself. Penalize
            # its magnitude; the hard input bound enforces the actual slew.
            weighted_residual("steering_command_rate", steering_cmd_rate),
            weighted_residual("virtual_speed_rate", delta_u[2]),
        ]
        if self.formulation == "mpcc":
            stage_residuals.append(
                weighted_residual(
                    "progress_consistency", virtual_speed - v_parallel
                )
            )

        model = AcadosModel()
        model.name = f"f1tenth_dynamic_{self.formulation}"
        model.x = x
        model.xdot = xdot
        model.u = u
        model.p = p
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl
        model.cost_y_expr = ca.vertcat(*stage_residuals)
        model.cost_y_expr_e = ca.vertcat(*state_residuals)
        # First two constraints are the physical track corridor with numerical
        # slack. Tire slip follows as a first-phase soft envelope. Acceleration
        # and actual steering rate are hard.
        model.con_h_expr = ca.vertcat(
            e_contour - (w_left - track_margin),
            -e_contour - (w_right - track_margin),
            alpha_f,
            alpha_r,
            speed_accel,
            steer_rate,
        )

        ocp = AcadosOcp()
        ocp.model = model
        ocp.code_gen_options.code_export_directory = str(
            self.generated_dir / f"c_generated_code_{self.formulation}"
        )
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.N * self.dt
        ocp.parameter_values = np.zeros(NP)
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ny = len(stage_residuals)
        ny_e = len(state_residuals)
        ocp.cost.W = np.eye(ny)
        ocp.cost.W_e = np.eye(ny_e)
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        speed_cap = self._speed_cap()
        ocp.constraints.idxbu = np.asarray([0, 1, 2], dtype=int)
        startup_rate = float(cfg["startup"]["steering_rate_max_radps"])
        steering_rate_bound = min(p_v.max_steer_rate, startup_rate)
        ocp.constraints.lbu = np.asarray([0.0, -steering_rate_bound, 0.0])
        ocp.constraints.ubu = np.asarray(
            [speed_cap, steering_rate_bound, float(constraints["virtual_speed_max_mps"])]
        )
        ocp.constraints.idxbx = np.asarray([3, 6, 7], dtype=int)
        ocp.constraints.lbx = np.asarray([0.0, -p_v.max_steer, -p_v.max_steer])
        ocp.constraints.ubx = np.asarray([speed_cap, p_v.max_steer, p_v.max_steer])
        slip = float(constraints["tire_slip_soft_limit_rad"])
        ocp.constraints.lh = np.asarray(
            [-1.0e6, -1.0e6, -slip, -slip, -p_v.max_decel, -p_v.max_steer_rate]
        )
        ocp.constraints.uh = np.asarray(
            [0.0, 0.0, slip, slip, p_v.max_accel, p_v.max_steer_rate]
        )
        ocp.constraints.idxsh = np.asarray([0, 1, 2, 3], dtype=int)
        ocp.cost.zl = np.asarray(
            [cost["track_slack_linear"], cost["track_slack_linear"], 0.0, 0.0],
            dtype=float,
        )
        ocp.cost.zu = ocp.cost.zl.copy()
        ocp.cost.Zl = np.asarray(
            [
                cost["track_slack_quadratic"],
                cost["track_slack_quadratic"],
                cost["tire_slack_quadratic"],
                cost["tire_slack_quadratic"],
            ],
            dtype=float,
        )
        ocp.cost.Zu = ocp.cost.Zl.copy()
        ocp.constraints.x0 = np.zeros(self.NX)

        solver_cfg = cfg["solver"]
        ocp.solver_options.nlp_solver_type = str(solver_cfg["nlp_solver_type"])
        ocp.solver_options.qp_solver = str(solver_cfg["qp_solver"])
        ocp.solver_options.integrator_type = str(solver_cfg["integrator_type"])
        ocp.solver_options.hessian_approx = str(solver_cfg["hessian_approx"])
        ocp.solver_options.levenberg_marquardt = float(
            solver_cfg["levenberg_marquardt"]
        )
        ocp.solver_options.sim_method_num_steps = int(solver_cfg["sim_method_num_steps"])
        ocp.solver_options.hpipm_mode = str(solver_cfg["hpipm_mode"])
        ocp.solver_options.print_level = int(solver_cfg["print_level"])

        json_file = self.generated_dir / f"acados_ocp_{self.formulation}.json"
        return AcadosOcpSolver(
            ocp,
            json_file=str(json_file),
            generate=bool(build),
            build=bool(build),
        )

    def _mode_weights(self) -> tuple[float, float, float]:
        cost = self.controller["cost"]
        mode = str(self.controller.get("mode", "debug")).lower()
        if self.formulation == "baseline" or mode == "debug":
            return (float(cost["speed_prior_debug"]),
                    float(cost["progress_reward_debug"]),
                    float(cost["heading_debug"]))
        return (float(cost["speed_prior_race"]),
                float(cost["progress_reward_race"]),
                float(cost["heading_race"]))

    def _initialize_reachable_warm_start(
        self,
        initial_state: np.ndarray,
        target_speed_cap: float,
        input_speed_cap: float,
        steering_rate_cap: float,
    ) -> None:
        state = initial_state.copy()
        self.warm.states[0] = state
        previous_speed = max(float(state[3]), 0.0)
        previous_steer = float(state[7])
        for stage in range(self.N):
            geometry = self.track.geometry(float(state[8]))
            speed_prior = min(float(geometry[6]), target_speed_cap)
            next_speed = float(np.clip(
                speed_prior,
                previous_speed - self.vehicle.max_decel * self.dt,
                previous_speed + self.vehicle.max_accel * self.dt,
            ))
            accel = (next_speed - previous_speed) / self.dt
            speed_tau = (
                self.vehicle.braking_speed_tau if accel < 0.0
                else self.vehicle.speed_tau
            )
            speed_cmd = np.clip(
                (previous_speed + speed_tau * accel)
                / self.vehicle.speed_gain,
                0.0,
                input_speed_cap,
            )
            steer_ff = float(
                np.clip(
                    math.atan(self.vehicle.wheelbase * float(geometry[3])),
                    -self.vehicle.max_steer,
                    self.vehicle.max_steer,
                )
            )
            steer_rate = float(np.clip(
                (steer_ff - previous_steer) / self.dt,
                -steering_rate_cap,
                steering_rate_cap,
            ))
            control = np.asarray([speed_cmd, steer_rate, max(next_speed, 0.2)])
            self.warm.controls[stage] = control
            state = self.rollout_model.step(state, control, self.dt)
            self.warm.states[stage + 1] = state
            previous_speed = max(float(state[3]), 0.0)
            previous_steer = float(state[7])
        self.warm.valid = True

    def solve(
        self,
        initial_state: np.ndarray,
        speed_cap: float | None = None,
        steering_rate_cap: float | None = None,
        progress_scale: float = 1.0,
    ) -> SolverResult:
        initial_state = np.asarray(initial_state, dtype=float)
        if initial_state.shape != (self.NX,) or not np.all(np.isfinite(initial_state)):
            raise ValueError("initial state must be a finite nine-vector")
        runtime_speed_cap = self._speed_cap() if speed_cap is None else min(
            max(float(speed_cap), 0.0), self._speed_cap()
        )
        runtime_input_speed_cap = runtime_speed_cap
        if bool(self.controller["safety"].get(
            "dynamic_speed_cap_feasibility_guard", True
        )):
            runtime_input_speed_cap = min(
                self._speed_cap(),
                max(
                    runtime_speed_cap,
                    (max(float(initial_state[3]), 0.0)
                     - self.vehicle.max_decel * self.vehicle.braking_speed_tau)
                    / self.vehicle.speed_gain + 0.02,
                ),
            )
        runtime_steering_rate = min(
            self.vehicle.max_steer_rate,
            self.vehicle.max_steer_rate
            if steering_rate_cap is None else max(float(steering_rate_cap), 0.0),
        )
        if not self.warm.valid:
            self._initialize_reachable_warm_start(
                initial_state, runtime_speed_cap, runtime_input_speed_cap,
                runtime_steering_rate
            )

        if self.formulation == "baseline":
            theta = float(initial_state[8])
            speed_cap = runtime_speed_cap
            speed = min(max(float(initial_state[3]), 0.2), speed_cap)
            for stage in range(self.N + 1):
                self.warm.states[stage, 8] = theta
                speed = min(
                    float(self.track.speed_prior(theta)),
                    speed + self.vehicle.max_accel * self.dt,
                    speed_cap,
                )
                theta += self.dt * max(speed, 0.2)

        speed_weight, progress_reward, heading_weight = self._mode_weights()
        progress_reward *= float(np.clip(progress_scale, 0.0, 1.0))
        margin = (
            self.vehicle.body_width / 2.0
            + float(self.controller["safety"]["track_control_margin_m"])
            + 0.05
        )
        previous_controls = np.vstack(
            [self.previous_control, self.warm.controls]
        )
        parameters = stage_parameters(
            self.track,
            self.warm.theta_sequence(),
            previous_controls,
            margin,
            speed_weight,
            progress_reward,
            heading_weight,
        )
        parameters[:, 6] = np.minimum(parameters[:, 6], runtime_speed_cap)
        lower_u = np.asarray([0.0, -runtime_steering_rate, 0.0])
        upper_u = np.asarray([
            runtime_input_speed_cap,
            runtime_steering_rate,
            float(self.controller["constraints"]["virtual_speed_max_mps"]),
        ])
        runtime_bounds = (runtime_input_speed_cap, runtime_steering_rate)
        if (
            self._last_runtime_bounds is None
            or abs(self._last_runtime_bounds[0] - runtime_bounds[0]) >= 0.02
            or abs(self._last_runtime_bounds[1] - runtime_bounds[1]) >= 0.02
        ):
            for stage in range(self.N):
                self.solver.set(stage, "lbu", lower_u)
                self.solver.set(stage, "ubu", upper_u)
            self._last_runtime_bounds = runtime_bounds
        self.solver.set(0, "lbx", initial_state)
        self.solver.set(0, "ubx", initial_state)
        # Bulk transfers avoid dozens of Python/ctypes calls every cycle.
        self.solver.set_flat("x", self.warm.states.reshape(-1))
        self.solver.set_flat("u", self.warm.controls.reshape(-1))
        self.solver.set_flat("p", parameters.reshape(-1))

        started = time.perf_counter()
        status = 0
        rti_iterations = max(
            1, int(self.controller["solver"].get("rti_iterations", 1))
        )
        minimum_rti_iterations = min(
            rti_iterations,
            max(1, int(self.controller["solver"].get("rti_min_iterations", 1))),
        )
        early_stop_slack = float(
            self.controller["solver"].get(
                "rti_early_stop_track_slack_m",
                self.controller["safety"]["emergency_track_slack_m"],
            )
        )
        completed_rti_iterations = 0
        qp_iterations = 0
        flat_lower = np.zeros(0)
        flat_upper = np.zeros(0)
        first_steer_rate = float("inf")
        for _ in range(rti_iterations):
            status = int(self.solver.solve())
            completed_rti_iterations += 1
            try:
                qp_iterations += int(np.sum(self.solver.get_stats("qp_iter")))
            except Exception:
                qp_iterations = -1
            if status != 0:
                break
            if completed_rti_iterations >= minimum_rti_iterations:
                # Two bulk reads replace 2*(N-1) stage-wise Python calls.
                flat_lower = np.asarray(self.solver.get_flat("sl"), dtype=float)
                flat_upper = np.asarray(self.solver.get_flat("su"), dtype=float)
                current_slack = np.maximum(flat_lower, flat_upper)
                current_track_slack = (
                    float(np.max(current_slack.reshape(-1, 4)[:, :2]))
                    if current_slack.size
                    else 0.0
                )
                first_control = np.asarray(self.solver.get(0, "u"), dtype=float)
                first_steer_rate = abs(float(first_control[1]))
                if current_track_slack <= early_stop_slack:
                    break
        elapsed = time.perf_counter() - started
        states = np.asarray(self.solver.get_flat("x"), dtype=float).reshape(
            self.N + 1, self.NX
        )
        controls = np.asarray(self.solver.get_flat("u"), dtype=float).reshape(
            self.N, self.NU
        )
        if not flat_lower.size:
            flat_lower = np.asarray(self.solver.get_flat("sl"), dtype=float)
            flat_upper = np.asarray(self.solver.get_flat("su"), dtype=float)
        flat_slack = np.maximum(flat_lower, flat_upper)
        slack = (
            np.max(flat_slack.reshape(-1, 4), axis=0)
            if flat_slack.size
            else np.zeros(4)
        )
        first_steer_rate = abs(float(controls[0, 1]))
        objective = float(self.solver.get_cost())
        finite = np.all(np.isfinite(states)) and np.all(np.isfinite(controls))
        success = bool(status == 0 and finite)
        deadline = float(self.controller["timing"]["solver_deadline_s"])
        reason = ""
        if status != 0:
            reason = f"acados_status_{status}"
        elif not finite:
            reason = "nonfinite_solution"
        elif elapsed > deadline:
            success = False
            reason = f"deadline_{elapsed:.6f}s"
        elif len(slack) and float(np.max(slack[:2])) > float(
            self.controller["safety"]["emergency_track_slack_m"]
        ):
            success = False
            reason = "emergency_track_slack"
        if success:
            self.previous_control = controls[0].copy()
            self.warm.shift(states, controls)
        return SolverResult(
            success=success,
            status=status,
            solve_time_s=elapsed,
            objective=objective,
            states=states,
            controls=controls,
            track_slack_max=float(np.max(slack[:2])) if slack.size >= 2 else 0.0,
            tire_slack_max=float(np.max(slack[2:])) if slack.size >= 4 else 0.0,
            qp_iterations=qp_iterations,
            rti_iterations=completed_rti_iterations,
            first_steer_rate_radps=first_steer_rate,
            reason=reason,
        )
