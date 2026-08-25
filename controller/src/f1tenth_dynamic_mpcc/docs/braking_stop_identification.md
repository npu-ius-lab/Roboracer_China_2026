# Compact 5 m/s braking identification

This is a single straight-line pass for a short open test area. It does not
start MPCC. It commands 5 m/s with zero steering and changes immediately to a
zero-speed command after localized longitudinal speed stays above 4.8 m/s for
0.1 s. There is no high-speed cruise plateau.

## Space and safety

- Absolute minimum clear length at 5 m/s: 18 m. Recommended: 20--25 m.
- Use a dry, level surface at least 4 m wide.
- The last 5 m and all space beyond the endpoint must be free of obstacles.
- Mechanically center the front wheels and point the car down the test line.
- A human must hold the physical emergency stop throughout the run.
- Do not run MPCC or any other Ackermann command publisher at the same time.

The script stops early if localization becomes stale or jumps, the vehicle
deviates laterally by 0.75 m, heading changes by 0.35 rad, or the remaining
course reaches a conservative dynamic stopping-distance reserve. These checks
reduce risk but cannot replace a clear run-off area or the physical E-stop.

## Commands

Inspect the plan without moving:

```bash
./scripts/start_braking_stop_identification.sh --plan --course-length 20
```

First validate at 3 m/s:

```bash
./scripts/start_braking_stop_identification.sh brake_3mps_run01 \
  --target-speed 3.0 --trigger-speed 2.8 --course-length 20 --execute
```

After the 3 m/s bag has been checked, run one 5 m/s pass:

```bash
./scripts/start_braking_stop_identification.sh brake_5mps_run01 \
  --target-speed 5.0 --trigger-speed 4.8 --course-length 20 \
  --execute --high-speed-confirm
```

Each invocation records and finalizes a bag plus a JSON sidecar in
`braking_identification/data/`. Reposition the stopped car before starting the
next run. Collect at least three usable runs for fitting and two separate runs
for validation.
