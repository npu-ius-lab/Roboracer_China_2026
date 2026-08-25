# Status=4, raceline simplification and delay validation

## Conclusion

The observed `acados_status_4` events were not caused by the number of MPCC
constraints. The dominant trigger was an infeasible runtime speed-command upper
bound. With the identified longitudinal model,

```text
tau_v = 0.20 s
max_decel = 4.0 m/s^2
```

the hard acceleration constraint requires, at the first stage,

```text
v_cmd >= (vx - max_decel * tau_v) / speed_gain.
```

For an observed `vx=2.36 m/s`, an immediate `v_cmd<=0.50 m/s` bound conflicts
with the required `v_cmd>=1.56 m/s`. This makes the QP infeasible before the
vehicle has had time to decelerate.

The implementation now separates:

- `target_speed_cap`: desired safety/risk target used by the speed prior and
  command publisher;
- `speed_input_cap`: dynamically reachable hard solver input bound;
- publisher deceleration slew: applies the lower target continuously over
  subsequent control cycles.

Predictive track-risk speed reduction is held for 2.0 s and released at
0.5 m/s², rather than lasting one solver cycle.

## Constraint audit

The generated OCP still keeps all physical and safety constraints:

- three input bounds (`v_cmd`, steering command rate, virtual speed);
- three state bounds (`vx`, actual steering, steering-command state);
- left and right track constraints with soft numerical slack;
- front and rear tire-slip soft envelopes;
- hard longitudinal acceleration and steering-actuator-rate constraints.

No track boundary, body margin, acceleration bound, steering bound or tire
constraint was deleted. Reducing this constraint set is neither necessary nor
the correct fix for the observed cap conflict.

## Raceline online interface

The online track interface directly requires only:

```text
s, x, y, w_left, w_right
```

Heading and curvature are derived from one C2 periodic x/y spline. CSV heading
and curvature remain offline diagnostics. CSV `vx` and `ax` are not race-mode
OCP tracking references; the runtime curvature, steering-rate and
acceleration-limited speed envelope is the sole speed plan.

Race MPCC uses:

```text
heading weight = 0.5 (weak regularization)
CSV speed-prior weight = 0.0
progress reward = 1.0
```

Debug/baseline keeps the original stronger heading/speed references for
comparison and cold-start diagnostics.

Track consistency is good enough to use the unified spline: heading RMSE is
0.000332 rad and curvature RMSE is 0.005029 1/m. Full metrics and plot are in
`track_geometry_consistency.md` and `plots/kappa_csv_vs_spline.png`.

## RTI policy

Each 50 ms control cycle now permits one initial RTI and at most one geometry
correction. A predictive speed retry uses one fresh RTI. Repeated corrections
inside one cycle were removed because they increased solve-time jitter without
fixing infeasible speed bounds.

## Delayed C++ closed-loop simulation

The validation plant and controller use:

```text
control/publish period       0.050 s
plant integration step       0.005 s
MPCC ERK internal step       0.010 s (5 steps per 0.05 s interval)
Point-LIO age                0.09..0.13 s, p95 ~= 0.1298 s
IMU                           200 Hz
wheel odometry                50 Hz
speed actuator dead time      0.13 s
speed time constant           0.20 s
```

The controller only observes historical vehicle state, IMU/wheel samples and
commands already published. It first repropagates the delayed Point-LIO state
to `now`, then separately propagates the committed actuator horizon to
`now+0.13 s`. The delays are never added into one fixed delay.

An abrupt risk target of 0.50 m/s is injected from simulation time 8 to 10 s to
reproduce the real failure mechanism.

| Variant | status=4 | total failures | laps / 30 s | max |ec| | min physical margin | solve p95 |
|---|---:|---:|---:|---:|---:|---:|
| Legacy references, no cap guard | 0* | 0* | 2.266 | 0.132 m | 0.250 m | 1.425 ms |
| Simplified references only, no cap guard | 26 | 440 | 0.956 | 1.324 m | -0.932 m | 7.246 ms |
| Cap guard only, legacy references | 0 | 0 | 2.246 | 0.125 m | 0.257 m | 1.424 ms |
| Full: cap guard + simplified race references | 0 | 0 | 3.442 | 0.388 m | 0.190 m | 1.656 ms |

`*` The legacy-reference controller happened to command a lower speed before
the injected drop, so this particular run did not enter the infeasible region.
The simplified-only variant reached higher speed and exposed the cap conflict;
the controlled comparison therefore shows the cap guard is the mechanism that
eliminates status=4. Reference simplification improves progress and removes
conflicting targets, but is not sufficient by itself.

Run the repeatable matrix with:

```bash
./scripts/run_cpp_delay_matrix.sh
```

## Deployment notes

The acados parameter vector changed from 13 to 14 fields to make heading weight
mode-dependent. `start_mpcc_hardware.sh` now checks an interface version marker
and regenerates both C solvers when needed. Generated binaries must never be
reused across this interface change.

The real vehicle remains subject to incremental speed validation. The
simulation validates solver/runtime logic and delay semantics; it does not
claim that tire, steering or high-speed vehicle dynamics are fully identified.
