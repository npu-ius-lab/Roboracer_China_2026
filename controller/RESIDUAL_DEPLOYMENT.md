# Residual MPCC deployment

This is an independent copy of the controller. It does not modify or build in
`/home/tianbot/f1tenth_mpcc`.

The deployed `markov_v1` residual was trained on the first 75% of both
`mpcc_20260814_164049.bag` and
`residual_mpcc_3mps_20260814_180454.bag`. The final 25% of each run is held out
chronologically. A conservative coefficient scale of 0.75 is compiled into
both the C++ propagation model and the generated acados OCP.

Offline held-out results:

- one-second final-position RMSE: 0.365 m nominal, 0.138 m corrected;
- one-step RMSE improvement: vx 19.3%, vy 43.2%, yaw rate 69.6% (scaled model);
- new-run held-out inner-track yaw-rate improvement: 54.9% old, 70.1% combined;
- out-of-distribution fade and residual output limits are enabled.

Closed-loop delayed simulation at scale 0.75:

- 600 solves, no solver failures and no status 4;
- 3.55 laps completed;
- maximum contour error 0.212 m and minimum track margin 0.259 m;
- solver p95 2.30 ms.

## 40 Hz controller trial (2026-08-14)

The C++ MPCC solve timer now runs at `40 Hz`; the Ackermann command publisher
remains at the TianRacer interface rate of `50 Hz`. The generated OCP horizon
is unchanged (`25 x 0.05 s = 1.25 s`). Because the solve interval is now half an
OCP stage, the warm start is advanced by measured wall time instead of being
shifted by one whole stage on every solve. A non-blocking solve guard drops an
overlapping timer event rather than running two acados solves concurrently.

Runtime steering pure delay is set to `steering_dead_time_s: 0.0` so it matches
the configuration used to train `residual_markov_v1.yaml`. The separate
first-order steering time constant remains `0.08 s` and continues to represent
the measured actuator response. The command-history implementation still
supports a nonzero pure delay for future retraining and comparison.

Local verification after the change:

- ROS telemetry/solve rate: `40.00 Hz`;
- isolated Ackermann command output: approximately `50.2 Hz`;
- warm-start advance: approximately `0.025 s` / `0.50` OCP stage per solve;
- 1200-solve delayed closed loop: zero failures/status-4 events, solver p95
  `2.06 ms`, maximum `7.95 ms`.

Pre-change backup:

```text
/home/tianbot/backups/f1tenth_residual_controller_ws_before_40hz_20260814_191924.tar.gz
SHA256 374e8b4427036a40eab06bb9bc1fc4f637a0864016c92fa7c5f13e58eb709381
```

## Safe shadow mode

Shadow mode solves and logs but does not create the hardware command gate:

```bash
cd /home/tianbot/f1tenth_residual_controller_ws
./scripts/start_residual_shadow.sh
```

## First hardware test

The default first-test cap is 2.0 m/s:

```bash
cd /home/tianbot/f1tenth_residual_controller_ws
./scripts/start_mpcc_hardware.sh mpcc
```

For a separate 3 m/s hardware profile, keep the original 2 m/s command
available as a conservative fallback and run:

```bash
./scripts/start_mpcc_hardware_3mps.sh
```

This changes only the runtime command speed cap to `3.0 m/s`; the deployed
bounded Markov residual model remains the same (`deployment_scale=0.75`).

The Pure Pursuit safety/recovery candidate keeps at least `1.2 m/s` of forward
speed when upstream safety limits permit it. This prevents the Ackermann
recovery path from losing steering authority at the previous `0.6 m/s` floor.

To run the same 3 m/s controller while recording a residual-dynamics training
bag, use:

```bash
./scripts/start_mpcc_hardware_3mps_record_residual.sh
```

The optional first argument is a run identifier.  With no argument, a unique
timestamped name is used.  Press `Ctrl-C` once to disable the controller and
hardware gate, stop rosbag cleanly, and print the saved bag path under
`/home/tianbot/f1tenth_residual_ws/data/bags`.

At very low speed the controller conditions the Point-LIO body twist before it
is passed to the dynamic bicycle model.  With both measured and commanded
longitudinal speed near zero, controller-side `vx`, `vy`, and yaw rate are set
to zero and the acados warm start is reset.  Hysteresis releases the stationary
mode above `0.20 m/s`, followed by a smooth kinematic-to-dynamic blend through
`0.45 m/s`.  Point-LIO messages are not modified.  Telemetry records both the
raw twist (`raw_pointlio_vx`, `raw_pointlio_vy`,
`raw_pointlio_yaw_rate`) and the conditioned mode/blend so that suppression can
be audited after a run.

Do not source this workspace together with the original controller workspace.
The launch scripts source only this overlay and its residual-model dependency.

## V2.1 candidate and deterministic V1 rollback (2026-08-14)

V1 remains the default in `config/controller.yaml`. Its deployed model is
byte-identical to the authoritative combined V1 artifact (SHA256
`16a18c94442f90415cf8bd0cea81b7c7887e0624adfa1f3e80a26e44246fe3fc`).

The repaired V2.1 candidate uses the causally deployable `physics_v2` feature
set (25 features). The 31-feature command-history model is rejected by both
C++ and acados until command history is represented as real solver state; the
runtime no longer substitutes the current command for all history fields.
V2.1 also fixes the smooth OOD gate and constrains Stage-B with continuous-RK4
one-step loss. The selected candidate applies 11% of the Stage-B update to the
Stage-A ridge initializer.

On the bag-split validation run, V2.1 versus V1 produced:

- one-step RMSE ratios: vx `0.9752`, vy `0.9992`, yaw rate `0.9645`;
- 0.50 s position RMSE ratio: `0.9362`;
- 1.00 s position RMSE ratio: `0.8102`.

This passes the validation thresholds, but it is not the default because a
new untouched final-test bag is still required. The two profiles have separate
acados generated-code caches, preventing a model/generated-solver mismatch.

Explicit V2.1 candidate test (3 m/s default cap):

```bash
cd /home/tianbot/f1tenth_residual_controller_ws
./scripts/start_mpcc_hardware_v2.sh
```

Immediate V1 rollback on the next launch:

```bash
cd /home/tianbot/f1tenth_residual_controller_ws
./scripts/start_mpcc_hardware_v1.sh
```

Both versioned test wrappers default to a 3.0 m/s runtime cap. For a temporary
lower-speed run, override it without editing configuration, for example:

```bash
RESIDUAL_MPCC_SPEED_CAP=2.0 ./scripts/start_mpcc_hardware_v2.sh
```

Stop the currently running controller before changing profiles. Verify both
model schemas without publishing any command using:

```bash
./scripts/check_residual_profiles.sh
```
