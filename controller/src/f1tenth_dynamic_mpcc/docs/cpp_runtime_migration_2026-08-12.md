# C++ realtime runtime migration

The production control path is now C++17. Python remains only for offline
acados code generation, bag replay, identification, plotting, and an optional
asynchronous JSONL telemetry logger.

## C++ realtime components

- ROS subscriptions and fixed-rate command publication
- PointLIO timestamp validation and delayed-measurement repropagation
- Livox gyro and TianRacer wheel-velocity correction
- published-command FIFO with the 0.13 s longitudinal dead time
- dynamic bicycle RK4 propagation and 0.20 s longitudinal time constant
- no-sensor steering actuator observer
- startup/ramp/safe-deceleration state machine
- periodic cubic track geometry and projection
- generated acados C solver API, loaded directly without Python/Cython calls
- fail-closed Ackermann hardware command gate

The generated acados solver is expected under
`~/.cache/f1tenth_dynamic_mpcc/acados/c_generated_code_{baseline,mpcc}` and
`ACADOS_SOURCE_DIR` points to the matching acados installation.

## Python retained by design

- `replay_delay_compensation.py`: five-mode timestamp replay and report
- identification and model-validation scripts
- `telemetry_logger.py`: non-control-thread JSONL logging

Both `debug_mpcc.launch` and `hardware_mpcc.launch` select `mpcc_node_cpp`.
Hardware launch selects `ackermann_command_gate_cpp`.

## Validation evidence

- Local Python regression: 53/53 tests passed.
- Local C++ core smoke test passed.
- C++ and Python track spline/model outputs matched to displayed machine
  precision at four track locations and one nine-state RK4 propagation case.
- Remote TianRacer Release build completed for all C++ targets.
- Remote generated-acados smoke test on the raceline returned `status=0`,
  solve time `4.34 ms`, track slack `9.58e-14 m`, and tire slack `2.03e-05`.
- Isolated ROS bag replay kept the C++ node alive and measured command output
  at `19.97-20.00 Hz`; telemetry ran at approximately `19.85 Hz`.
- The replay bag was intentionally outside the configured control corridor for
  89.9% of PointLIO samples (median contour error `0.819 m` versus median
  available centre corridor `0.478 m`), so `emergency_track_slack` rejection
  in that replay is the correct safety behavior, not a solver/runtime fault.

All runtime validation used either `start_enabled=false` or an isolated debug
command topic with no hardware subscriber. No real-vehicle command was sent.
