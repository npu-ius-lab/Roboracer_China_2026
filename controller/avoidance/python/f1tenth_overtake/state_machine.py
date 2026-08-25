"""Small deterministic behaviour state machine for the simulation tracker."""

from __future__ import annotations

from dataclasses import dataclass


FREE = "FREE"
FOLLOW = "FOLLOW"
PREPARE = "PREPARE"
PASS = "PASS"
RETURN = "RETURN"
ABORT = "ABORT"
STATES = (FREE, FOLLOW, PREPARE, PASS, RETURN, ABORT)


@dataclass(frozen=True)
class StateMachineConfig:
    prepare_time: float = 0.40
    follow_target_loss_timeout: float = 1.00
    return_confirm_gap: float = 0.45
    return_ey_tolerance: float = 0.10
    abort_center_distance: float = 0.30
    abort_recovery_distance: float = 0.55

    def validate(self) -> None:
        if self.prepare_time < 0.0:
            raise ValueError("prepare_time must be non-negative")
        positive = (
            self.follow_target_loss_timeout,
            self.return_confirm_gap,
            self.return_ey_tolerance,
            self.abort_center_distance,
            self.abort_recovery_distance,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("state-machine distances and tolerances must be positive")
        if self.abort_recovery_distance <= self.abort_center_distance:
            raise ValueError("abort recovery distance must exceed abort distance")


@dataclass(frozen=True)
class StateObservation:
    target_visible: bool
    relevant: bool
    valid_area: bool
    authorized: bool
    candidate_ready: bool
    reference_available: bool
    boundary_safe: bool
    opponent_signed_delta: float
    ego_ey: float
    center_distance: float


class OvertakeStateMachine:
    """Own behaviour persistence while leaving path generation stateless."""

    def __init__(self, config: StateMachineConfig | None = None) -> None:
        self.config = config or StateMachineConfig()
        self.config.validate()
        self.state = FREE
        self.entered_at: float | None = None
        self.history = [FREE]
        self.last_transition = "INITIAL->FREE"
        self.opponent_clear_confirmed = False
        self.follow_target_lost_at: float | None = None

    def _transition(self, state: str, now: float) -> None:
        if state == self.state:
            return
        previous = self.state
        self.state = state
        if state != FOLLOW:
            self.follow_target_lost_at = None
        self.entered_at = now
        self.history.append(state)
        self.last_transition = f"{previous}->{state}"

    def update(self, now: float, observation: StateObservation) -> str:
        if self.entered_at is None:
            self.entered_at = now
        age = max(now - self.entered_at, 0.0)
        cfg = self.config

        if self.state == FOLLOW and not observation.target_visible:
            # FOLLOW has no committed lateral manoeuvre. Hold a fail-safe
            # speed through a short radar dropout, then resume the raceline if
            # no target reappears. Committed states still ABORT immediately.
            if self.follow_target_lost_at is None:
                self.follow_target_lost_at = now
            elif now - self.follow_target_lost_at >= cfg.follow_target_loss_timeout:
                self._transition(FREE, now)
            return self.state
        if self.state == FOLLOW:
            self.follow_target_lost_at = None

        if self.state in (PREPARE, PASS) and not observation.target_visible:
            self._transition(ABORT, now)
            return self.state

        if (
            self.state == RETURN
            and not observation.target_visible
            and not self.opponent_clear_confirmed
        ):
            self._transition(ABORT, now)
            return self.state

        if self.state in (PREPARE, PASS, RETURN) and not observation.boundary_safe:
            self._transition(ABORT, now)
            return self.state

        if (
            self.state in (PREPARE, PASS, RETURN)
            and observation.center_distance <= cfg.abort_center_distance
        ):
            self._transition(ABORT, now)
            return self.state

        if self.state == FREE:
            if observation.relevant:
                self._transition(FOLLOW, now)
        elif self.state == FOLLOW:
            if not observation.relevant:
                self._transition(FREE, now)
            elif (
                observation.valid_area
                and observation.authorized
                and observation.candidate_ready
            ):
                self._transition(PREPARE, now)
        elif self.state == PREPARE:
            if not observation.relevant:
                self._transition(FREE, now)
            elif not (
                observation.valid_area
                and observation.authorized
                and observation.candidate_ready
            ):
                self._transition(FOLLOW, now)
            elif age >= cfg.prepare_time:
                self._transition(PASS, now)
        elif self.state == PASS:
            if not observation.reference_available:
                self._transition(ABORT, now)
            elif observation.opponent_signed_delta <= -cfg.return_confirm_gap:
                self.opponent_clear_confirmed = True
                self._transition(RETURN, now)
        elif self.state == RETURN:
            returned = (
                abs(observation.ego_ey) <= cfg.return_ey_tolerance
                and (
                    self.opponent_clear_confirmed
                    or observation.opponent_signed_delta <= -cfg.return_confirm_gap
                )
            )
            if returned:
                self.opponent_clear_confirmed = False
                self._transition(FREE, now)
            elif not observation.reference_available:
                self._transition(ABORT, now)
        elif self.state == ABORT:
            # Unknown target state is not clearance. Hold the offset reference
            # and reduced speed until the radar sees the opponent again.
            if (
                observation.target_visible
                and observation.boundary_safe
                and observation.center_distance >= cfg.abort_recovery_distance
            ):
                if (
                    observation.reference_available
                    and abs(observation.ego_ey) > cfg.return_ey_tolerance
                ):
                    # Keep the accepted offset path while braking.  Once the
                    # opponent has opened a gap, finish the lateral recovery
                    # through RETURN instead of snapping to the raceline.
                    self._transition(RETURN, now)
                elif abs(observation.ego_ey) <= cfg.return_ey_tolerance:
                    self.opponent_clear_confirmed = False
                    self._transition(FREE, now)
        return self.state
