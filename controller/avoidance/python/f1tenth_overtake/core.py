"""Stateless overtaking assessment and smooth candidate trajectory generation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping

import numpy as np

from .track import FrenetProjection, PeriodicTrack


def _smootherstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value**3 * (value * (value * 6.0 - 15.0) + 10.0)


@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    target_id: int = -1


@dataclass(frozen=True)
class PlannerConfig:
    ego_width: float = 0.24
    ego_length: float = 0.40
    opponent_width: float = 0.35
    opponent_length: float = 0.40
    boundary_margin: float = 0.08
    minimum_boundary_clearance_for_pass: float = 0.0
    lateral_clearance: float = 0.06
    longitudinal_clearance: float = 0.20
    detection_distance: float = 6.0
    trigger_distance: float = 4.0
    trigger_ttc: float = 4.0
    minimum_closing_speed: float = 0.10
    minimum_lane_change_length: float = 0.80
    nominal_lane_change_length: float = 1.60
    pass_hold_after_opponent: float = 0.70
    return_length: float = 1.60
    horizon: float = 7.0
    maximum_candidate_length: float = 12.0
    sample_ds: float = 0.10
    minimum_plan_speed: float = 0.80
    maximum_plan_speed: float = 3.00
    pass_speed_advantage: float = 0.60
    minimum_opponent_prediction_speed: float = 0.0
    entry_plan_speed: float = 0.0
    entry_plan_duration: float = 0.0
    longitudinal_progress_speed_scale: float = 1.0
    maximum_path_curvature: float = 2.50
    maximum_track_curvature_for_pass: float = 2.50
    maximum_projection_distance: float = 1.00
    preferred_side: str = "left"

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "PlannerConfig":
        known = asdict(cls())
        supplied = {key: values[key] for key in known if key in values}
        return cls(**supplied)

    def validate(self) -> None:
        positive = (
            self.ego_width,
            self.ego_length,
            self.opponent_width,
            self.opponent_length,
            self.detection_distance,
            self.trigger_distance,
            self.trigger_ttc,
            self.minimum_lane_change_length,
            self.nominal_lane_change_length,
            self.return_length,
            self.horizon,
            self.maximum_candidate_length,
            self.sample_ds,
            self.minimum_plan_speed,
            self.maximum_plan_speed,
            self.maximum_path_curvature,
            self.maximum_track_curvature_for_pass,
            self.maximum_projection_distance,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("planner lengths, speeds, time, and curvature limits must be positive")
        if self.minimum_boundary_clearance_for_pass < 0.0:
            raise ValueError("minimum pass boundary clearance must be non-negative")
        if self.entry_plan_speed < 0.0 or self.entry_plan_duration < 0.0:
            raise ValueError("entry plan speed and duration must be non-negative")
        if self.minimum_opponent_prediction_speed < 0.0:
            raise ValueError("minimum opponent prediction speed must be non-negative")
        if self.entry_plan_duration > 0.0 and self.entry_plan_speed <= 0.0:
            raise ValueError("entry_plan_speed must be positive when entry_plan_duration is used")
        if not 0.0 < self.longitudinal_progress_speed_scale <= 1.0:
            raise ValueError("longitudinal_progress_speed_scale must be in (0, 1]")
        if self.maximum_plan_speed < self.minimum_plan_speed:
            raise ValueError("maximum_plan_speed must not be below minimum_plan_speed")
        if self.preferred_side not in ("left", "right", "none"):
            raise ValueError("preferred_side must be left, right, or none")


@dataclass(frozen=True)
class CandidateTrajectory:
    side: str
    feasible: bool
    reason: str
    s: np.ndarray
    ey: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    speed: np.ndarray
    target_offset: float
    minimum_boundary_clearance: float
    minimum_collision_metric: float
    maximum_curvature: float
    score: float


@dataclass(frozen=True)
class PlanningResult:
    relevant: bool
    risk: bool
    reason: str
    ego: FrenetProjection
    opponent: FrenetProjection
    ego_vs: float
    opponent_vs: float
    delta_s: float
    bumper_gap: float
    closing_speed: float
    ttc: float
    left: CandidateTrajectory
    right: CandidateTrajectory
    selected_side: str

    @property
    def selected(self) -> CandidateTrajectory | None:
        if self.selected_side == "left":
            return self.left
        if self.selected_side == "right":
            return self.right
        return None


class OvertakePlanner:
    """Generate left/right pass-and-return paths without retaining behavior state."""

    def __init__(self, track: PeriodicTrack, config: PlannerConfig | None = None) -> None:
        self.track = track
        self.config = config or PlannerConfig()
        self.config.validate()

    @staticmethod
    def _longitudinal_speed(state: VehicleState, projection: FrenetProjection) -> float:
        return float(state.vx * math.cos(projection.yaw) + state.vy * math.sin(projection.yaw))

    def _empty_candidate(self, side: str, reason: str) -> CandidateTrajectory:
        empty = np.empty(0, dtype=float)
        return CandidateTrajectory(
            side=side,
            feasible=False,
            reason=reason,
            s=empty,
            ey=empty,
            x=empty,
            y=empty,
            yaw=empty,
            speed=empty,
            target_offset=math.nan,
            minimum_boundary_clearance=-math.inf,
            minimum_collision_metric=-math.inf,
            maximum_curvature=math.inf,
            score=-math.inf,
        )

    def _lateral_profile(
        self,
        q: float,
        start_ey: float,
        target_ey: float,
        entry_end: float,
        hold_end: float,
        return_end: float,
    ) -> float:
        if q <= entry_end:
            blend = _smootherstep(q / entry_end)
            return start_ey + blend * (target_ey - start_ey)
        if q <= hold_end:
            return target_ey
        if q <= return_end:
            blend = _smootherstep((q - hold_end) / (return_end - hold_end))
            return target_ey * (1.0 - blend)
        return 0.0

    @staticmethod
    def _path_curvature(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
        if len(x) < 3:
            return np.zeros_like(x), 0.0
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
        curvature = (dx * ddy - dy * ddx) / denominator
        return curvature, float(np.max(np.abs(curvature)))

    def _candidate(
        self,
        side: str,
        ego: FrenetProjection,
        opponent: FrenetProjection,
        delta_s: float,
        ego_vs: float,
        opponent_vs: float,
    ) -> CandidateTrajectory:
        cfg = self.config
        lateral_separation = 0.5 * (cfg.ego_width + cfg.opponent_width) + cfg.lateral_clearance
        direction = 1.0 if side == "left" else -1.0
        target_offset = opponent.ey + direction * lateral_separation

        longitudinal_separation = 0.5 * (cfg.ego_length + cfg.opponent_length) + cfg.longitudinal_clearance
        entry_available = delta_s - longitudinal_separation
        entry_end = min(cfg.nominal_lane_change_length, entry_available)
        if entry_end < cfg.minimum_lane_change_length:
            return self._empty_candidate(side, "insufficient_lane_change_distance")

        # Gazebo reports a near-zero twist for a few frames while the front
        # controller is starting.  A configured floor prevents that transient
        # from producing a pass candidate that returns in front of a vehicle
        # which is about to accelerate to its known cruise speed.
        opponent_prediction_speed = max(
            opponent_vs, cfg.minimum_opponent_prediction_speed, 0.0
        )
        plan_speed = float(
            np.clip(
                max(
                    ego_vs,
                    opponent_prediction_speed + cfg.pass_speed_advantage,
                    cfg.minimum_plan_speed,
                ),
                cfg.minimum_plan_speed,
                cfg.maximum_plan_speed,
            )
        )
        entry_duration = max(cfg.entry_plan_duration, 0.0)
        entry_speed = plan_speed
        if entry_duration > 0.0:
            entry_speed = float(
                np.clip(cfg.entry_plan_speed, cfg.minimum_plan_speed, plan_speed)
            )
        entry_progress = entry_speed * entry_duration
        prediction_speed = plan_speed * cfg.longitudinal_progress_speed_scale
        relative_progress_ratio = 1.0 - opponent_prediction_speed / prediction_speed
        if (
            prediction_speed - opponent_prediction_speed < cfg.minimum_closing_speed
            or relative_progress_ratio <= 1e-3
        ):
            return self._empty_candidate(side, "insufficient_pass_speed_advantage")

        # q is ego path progress. Keep the lateral pass offset until the ego
        # rear bumper is longitudinally clear of the moving opponent front.
        # Using the opponent's current s alone would start the return while the
        # cars are still side-by-side whenever the opponent is moving.
        required_relative_progress = delta_s + longitudinal_separation
        opponent_forward_speed = opponent_prediction_speed
        entry_relative_ratio = 1.0 - opponent_forward_speed / entry_speed
        entry_relative_progress = entry_progress - opponent_forward_speed * entry_duration
        if (
            entry_duration > 0.0
            and entry_relative_ratio > 1.0e-3
            and required_relative_progress <= entry_relative_progress
        ):
            clear_opponent_q = required_relative_progress / entry_relative_ratio
        else:
            # The tracker intentionally limits speed while establishing the
            # lateral offset.  Account for the opponent's motion during that
            # phase before solving the constant pass-speed intercept.  Without
            # this term the candidate starts returning before the real vehicle
            # has cleared a moving opponent.
            clear_opponent_q = (
                required_relative_progress
                + opponent_forward_speed * entry_duration
                - opponent_forward_speed * entry_progress / prediction_speed
            ) / relative_progress_ratio
        hold_end = clear_opponent_q + cfg.pass_hold_after_opponent
        return_end = hold_end + cfg.return_length
        if return_end > cfg.maximum_candidate_length:
            return self._empty_candidate(side, "pass_exceeds_candidate_horizon")
        horizon = max(cfg.horizon, return_end + cfg.sample_ds)
        q = np.arange(0.0, horizon + 0.5 * cfg.sample_ds, cfg.sample_ds, dtype=float)
        s = np.asarray([self.track.wrap_s(ego.s + value) for value in q], dtype=float)
        ey = np.asarray(
            [
                self._lateral_profile(value, ego.ey, target_offset, entry_end, hold_end, return_end)
                for value in q
            ],
            dtype=float,
        )

        x = np.empty_like(q)
        y = np.empty_like(q)
        minimum_boundary_clearance = math.inf
        maximum_track_curvature = 0.0
        corridor_ok = True
        for index, (path_progress, progress, offset) in enumerate(zip(q, s, ey)):
            sample = self.track.sample(float(progress))
            if path_progress <= return_end + 1.0e-9:
                maximum_track_curvature = max(
                    maximum_track_curvature, abs(float(sample.curvature))
                )
            normal_x = -math.sin(sample.yaw)
            normal_y = math.cos(sample.yaw)
            x[index] = sample.x + offset * normal_x
            y[index] = sample.y + offset * normal_y
            left_limit = sample.width_left - 0.5 * cfg.ego_width - cfg.boundary_margin
            right_limit = -sample.width_right + 0.5 * cfg.ego_width + cfg.boundary_margin
            clearance = min(left_limit - offset, offset - right_limit)
            minimum_boundary_clearance = min(minimum_boundary_clearance, clearance)
            corridor_ok = bool(corridor_ok and bool(clearance >= -1e-9))

        if maximum_track_curvature > cfg.maximum_track_curvature_for_pass:
            return self._empty_candidate(side, "track_curvature_outside_pass_zone")

        yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
        _, maximum_curvature = self._path_curvature(x, y)

        if entry_duration > 0.0:
            speed = np.where(q <= entry_progress, entry_speed, plan_speed)
            time = np.where(
                q <= entry_progress,
                q / entry_speed,
                entry_duration + (q - entry_progress) / prediction_speed,
            )
        else:
            speed = np.full_like(q, plan_speed)
            time = q / max(prediction_speed, cfg.minimum_plan_speed)
        collision_metric = math.inf
        collision_free = True
        for progress, offset, prediction_time in zip(s, ey, time):
            opponent_s = self.track.wrap_s(
                opponent.s + opponent_prediction_speed * float(prediction_time)
            )
            longitudinal = abs(self.track.signed_delta(float(progress), opponent_s))
            lateral = abs(float(offset) - opponent.ey)
            metric = max(
                longitudinal / max(longitudinal_separation, 1e-6),
                lateral / max(lateral_separation, 1e-6),
            )
            collision_metric = min(collision_metric, metric)
            if (
                longitudinal + 1e-9 < longitudinal_separation
                and lateral + 1e-9 < lateral_separation
            ):
                collision_free = False

        curvature_ok = maximum_curvature <= cfg.maximum_path_curvature
        tracking_clearance_ok = (
            minimum_boundary_clearance + 1.0e-9
            >= cfg.minimum_boundary_clearance_for_pass
        )
        feasible = bool(
            corridor_ok and collision_free and curvature_ok and tracking_clearance_ok
        )
        if not corridor_ok:
            reason = "track_corridor_violation"
        elif not collision_free:
            reason = "predicted_opponent_collision"
        elif not curvature_ok:
            reason = "path_curvature_limit"
        elif not tracking_clearance_ok:
            reason = "insufficient_tracking_clearance"
        else:
            reason = "feasible"

        preference_bonus = 0.02 if cfg.preferred_side == side else 0.0
        score = (
            minimum_boundary_clearance
            + preference_bonus
            - 0.02 * maximum_curvature
            - 0.01 * abs(target_offset - ego.ey)
        ) if feasible else -math.inf
        return CandidateTrajectory(
            side=side,
            feasible=feasible,
            reason=reason,
            s=s,
            ey=ey,
            x=x,
            y=y,
            yaw=yaw,
            speed=speed,
            target_offset=target_offset,
            minimum_boundary_clearance=minimum_boundary_clearance,
            minimum_collision_metric=collision_metric,
            maximum_curvature=maximum_curvature,
            score=score,
        )

    def evaluate(self, ego_state: VehicleState, opponent_state: VehicleState) -> PlanningResult:
        cfg = self.config
        ego = self.track.project(ego_state.x, ego_state.y)
        opponent = self.track.project_near(
            opponent_state.x,
            opponent_state.y,
            ego.s,
            cfg.detection_distance + 2.0,
        )
        ego_vs = self._longitudinal_speed(ego_state, ego)
        opponent_vs = self._longitudinal_speed(opponent_state, opponent)
        delta_s = self.track.forward_delta(ego.s, opponent.s)
        bumper_gap = delta_s - 0.5 * (cfg.ego_length + cfg.opponent_length)
        closing_speed = ego_vs - opponent_vs
        ttc = bumper_gap / closing_speed if closing_speed > cfg.minimum_closing_speed and bumper_gap > 0.0 else math.inf
        relevant = (
            0.0 < delta_s <= cfg.detection_distance
            and ego.distance <= cfg.maximum_projection_distance
            and opponent.distance <= cfg.maximum_projection_distance
            and -opponent.width_right <= opponent.ey <= opponent.width_left
        )
        risk = relevant and (bumper_gap <= cfg.trigger_distance or ttc <= cfg.trigger_ttc)

        if not relevant:
            left = self._empty_candidate("left", "opponent_not_relevant")
            right = self._empty_candidate("right", "opponent_not_relevant")
            return PlanningResult(
                relevant=False,
                risk=False,
                reason="opponent_not_ahead_on_track",
                ego=ego,
                opponent=opponent,
                ego_vs=ego_vs,
                opponent_vs=opponent_vs,
                delta_s=delta_s,
                bumper_gap=bumper_gap,
                closing_speed=closing_speed,
                ttc=ttc,
                left=left,
                right=right,
                selected_side="none",
            )

        left = self._candidate("left", ego, opponent, delta_s, ego_vs, opponent_vs)
        right = self._candidate("right", ego, opponent, delta_s, ego_vs, opponent_vs)
        feasible = [candidate for candidate in (left, right) if candidate.feasible]
        selected = max(feasible, key=lambda candidate: candidate.score).side if risk and feasible else "none"
        if not risk:
            reason = "no_overtake_trigger"
        elif selected == "none":
            reason = "no_feasible_corridor"
        else:
            reason = f"selected_{selected}"
        return PlanningResult(
            relevant=True,
            risk=risk,
            reason=reason,
            ego=ego,
            opponent=opponent,
            ego_vs=ego_vs,
            opponent_vs=opponent_vs,
            delta_s=delta_s,
            bumper_gap=bumper_gap,
            closing_speed=closing_speed,
            ttc=ttc,
            left=left,
            right=right,
            selected_side=selected,
        )
