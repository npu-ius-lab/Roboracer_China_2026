# Stable V5 speed-smooth a2.0/d1.8 candidate

This candidate keeps Stable V5 geometry, controller, vehicle, residual model,
60 Hz pipeline and 4.0 m/s^2 publisher/fallback braking unchanged.

Only `speed_limit_mps`, `vx_mps` and `ax_mps2` differ from
`racelinev3_stable_v5_fast5_std32`.  The node-wise envelope is generated with a maximum
planned acceleration of 2.0 m/s^2 and maximum planned
deceleration of 1.8 m/s^2.  Runtime planning reproduces
that envelope while retaining the stronger Stable V5 feasibility and safety
braking limits.

Run from the controller workspace:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \
  ./scripts/start_mpcc_hardware_stable_v5_speed_smooth_a20_d18_candidate.sh mpcc
```

Rollback is immediate: stop this launcher and run the unchanged
`./scripts/start_mpcc_hardware_stable_v5.sh` (or frozen StableV3 launcher).
