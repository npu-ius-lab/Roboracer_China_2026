from f1tenth_v5_jubu.state_machine import (
    ABORT,
    FOLLOW,
    FREE,
    PASS,
    PREPARE,
    RETURN,
    OvertakeStateMachine,
    StateObservation,
)


def observation(**overrides):
    values = {
        "target_visible": True,
        "relevant": True,
        "valid_area": True,
        "authorized": True,
        "candidate_ready": True,
        "reference_available": True,
        "boundary_safe": True,
        "opponent_signed_delta": 2.0,
        "ego_ey": 0.0,
        "center_distance": 2.0,
    }
    values.update(overrides)
    return StateObservation(**values)


def test_nominal_pass_sequence():
    machine = OvertakeStateMachine()
    assert machine.update(0.0, observation()) == FOLLOW
    assert machine.update(0.1, observation()) == PREPARE
    assert machine.update(0.3, observation()) == PREPARE
    assert machine.update(0.6, observation()) == PASS
    assert machine.update(2.0, observation(opponent_signed_delta=-0.6, ego_ey=0.45)) == RETURN
    assert machine.update(3.0, observation(opponent_signed_delta=-1.0, ego_ey=0.05)) == FREE
    assert machine.history == [FREE, FOLLOW, PREPARE, PASS, RETURN, FREE]


def test_abort_and_recovery():
    machine = OvertakeStateMachine()
    machine.update(0.0, observation())
    machine.update(0.1, observation())
    machine.update(0.6, observation())
    assert machine.state == PASS
    assert machine.update(0.7, observation(center_distance=0.25)) == ABORT
    assert machine.update(1.0, observation(center_distance=0.50, ego_ey=0.30)) == ABORT
    assert machine.update(1.2, observation(center_distance=0.60, ego_ey=0.30)) == RETURN
    assert machine.update(
        1.4,
        observation(center_distance=0.80, opponent_signed_delta=-0.7, ego_ey=0.05),
    ) == FREE


def test_abort_without_reference_recovers_only_on_centerline():
    machine = OvertakeStateMachine()
    machine.update(0.0, observation())
    machine.update(0.1, observation())
    machine.update(0.6, observation())
    assert machine.update(0.7, observation(center_distance=0.25)) == ABORT
    assert machine.update(
        1.0,
        observation(center_distance=0.70, ego_ey=0.30, reference_available=False),
    ) == ABORT
    assert machine.update(
        1.2,
        observation(center_distance=0.70, ego_ey=0.05, reference_available=False),
    ) == FREE


def test_follow_waits_for_authorization():
    machine = OvertakeStateMachine()
    assert machine.update(0.0, observation(authorized=False)) == FOLLOW
    assert machine.update(1.0, observation(authorized=False)) == FOLLOW
    assert machine.update(1.1, observation()) == PREPARE


def test_uncommitted_follow_holds_then_releases_after_radar_timeout():
    machine = OvertakeStateMachine()
    assert machine.update(0.0, observation()) == FOLLOW
    assert machine.update(0.1, observation(target_visible=False)) == FOLLOW
    assert machine.update(1.0, observation(target_visible=False)) == FOLLOW
    assert machine.update(1.2, observation(target_visible=False)) == FREE


def test_radar_loss_aborts_and_cannot_be_treated_as_clearance():
    machine = OvertakeStateMachine()
    machine.update(0.0, observation())
    machine.update(0.1, observation())
    machine.update(0.6, observation())
    assert machine.state == PASS
    assert machine.update(0.7, observation(target_visible=False)) == ABORT
    assert machine.update(
        1.0,
        observation(target_visible=False, center_distance=10.0, ego_ey=0.30),
    ) == ABORT


def test_runtime_boundary_loss_aborts_committed_pass():
    machine = OvertakeStateMachine()
    machine.update(0.0, observation())
    machine.update(0.1, observation())
    machine.update(0.6, observation())
    assert machine.state == PASS
    assert machine.update(0.7, observation(boundary_safe=False)) == ABORT


def test_return_can_finish_after_radar_confirmed_opponent_clearance():
    machine = OvertakeStateMachine()
    machine.update(0.0, observation())
    machine.update(0.1, observation())
    machine.update(0.6, observation())
    assert machine.update(
        1.0, observation(opponent_signed_delta=-0.8, ego_ey=0.35)
    ) == RETURN
    assert machine.update(
        1.4,
        observation(
            target_visible=False,
            opponent_signed_delta=float("inf"),
            center_distance=float("inf"),
            ego_ey=0.05,
        ),
    ) == FREE
