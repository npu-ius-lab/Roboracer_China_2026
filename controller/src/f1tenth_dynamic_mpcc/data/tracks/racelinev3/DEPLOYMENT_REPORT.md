# racelinev3 validation and deployment report

Generated and deployed as a stopped candidate on 2026-08-20. The existing
`racelineV2` track and launcher remain available and unchanged.

## A3-A6 geometry and speed

- Dynamic arc zone: `s = 70.482021971 .. 82.548982931 m`
- Local reference-speed limit: `4.0 m/s`
- Runtime speed reference maximum in the zone: `4.000000000 m/s`
- Minimum interpolated centerline-to-physical-boundary width: `0.475897498 m`
- Required design clearance: `0.450 m`
- Maximum implied steering angle: `0.476647 rad` (limit `0.55 rad`)
- Maximum planned lateral acceleration: `3.2 m/s^2`
- Maximum absolute curvature gradient: old `2.1861`, racelinev3 `0.4291 1/m^2`

The online Python and C++ track implementations clamp cubic speed interpolation
to the adjacent feasible speed-node envelope. This prevents interpolation-only
overshoot above the 4.0 m/s local limit.

## Offline closed-loop validation

- Python speed/periodic-track tests: 13 passed.
- C++ core smoke test: passed on `racelinev3`.
- Pure-pursuit recovery closed loop: 1.1848 laps, maximum contour error
  `0.1860 m`, minimum physical margin `0.2864 m`, one expected takeover, zero
  unsafe candidate rollouts.
- Delayed MPCC closed loop at 3.0 m/s: 1200 solves, zero failures, zero status-4
  solves, minimum track margin `0.1382 m`, solve-time p95 `3.20 ms`.
- Delayed MPCC closed loop with a 4.0 m/s requested cap: 1200 solves, zero
  failures, zero status-4 solves, A3-A6 minimum track margin `0.1113 m`, global
  minimum track margin `0.0871 m`, solve-time p95 `3.19 ms`.
- In the 4.0 m/s simulation, the dynamic vehicle state peaked at `4.1545 m/s`
  inside A3-A6 (`4.1979 m/s` globally) because of drivetrain inertia and the
  residual plant model. The reference and command cap remained 4.0 m/s.

## Vehicle deployment state

- Vehicle track: `/home/tianbot/f1tenth_residual_controller_ws/src/f1tenth_dynamic_mpcc/data/tracks/racelinev3`
- Dedicated launcher: `/home/tianbot/f1tenth_residual_controller_ws/scripts/start_mpcc_hardware_stable_v2_racelinev3.sh`
- Backup made before source replacement:
  `/home/tianbot/f1tenth_residual_controller_ws/backups/racelinev3_deploy_20260820_232145`
- Vehicle C++ workspace: built successfully.
- Dedicated baseline and MPCC acados solvers: generated successfully.
- Baseline and MPCC solver smoke tests: status 0, success.
- Controller state after deployment: not running; no command output was started.

For the first physical run, use a global runtime cap of 3.0 m/s, inspect the
A3-A6 bag and boundary margin, and only then raise the runtime cap to 4.0 m/s.
