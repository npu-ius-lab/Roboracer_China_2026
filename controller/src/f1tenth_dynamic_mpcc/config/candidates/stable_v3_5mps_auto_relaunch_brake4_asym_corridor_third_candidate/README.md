# stable_v3_5mps_auto_relaunch_brake4_asym_corridor_third_candidate

Temporary test candidate based exactly on `stable_v3_5mps_auto_relaunch_brake4_asym_candidate`.

Only one parameter changes:

- `safety.track_control_margin_m`: `0.05 -> 0.016667 m` (one third)

The vehicle half-width and the implementation's fixed 0.05 m numerical buffer
are unchanged. Therefore the MPCC track constraint footprint changes from
`body_width/2 + 0.100 m` to `body_width/2 + 0.066667 m`.

PP `boundary_margin_m: 0.10`, raceline geometry, 5/3 m/s zones, 4 m/s^2
profile/publisher deceleration, asymmetric longitudinal model, residual model,
MPCC weights, steering settings, and automatic relaunch behavior are unchanged.
