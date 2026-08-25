# stable_v3_5mps_auto_relaunch_brake4_asym_candidate

This is an isolated candidate based exactly on
`stable_v3_5mps_auto_relaunch_candidate` (controller configuration:
`stable_v3_5mps_arc_straight_std3_auto_relaunch_candidate`).

Only these longitudinal/braking changes are intentional:

- `publisher.deceleration_limit_mps2`: `2.0 -> 4.0`
- `speed_planning.profile_max_decel_mps2`: `3.0 -> 4.0`
- acceleration actuator model remains `delay/tau = 0.13/0.20 s`
- braking actuator model uses identified `delay/tau = 0.060/0.166 s`
- model hard deceleration bound remains `4.0 m/s^2`

The raceline geometry, 5/3 m/s speed zones, residual model, MPCC weights,
steering rate limit, PP recovery, and automatic relaunch logic are unchanged
from the base candidate.

The asymmetric fields are optional. Configurations without them fall back to
the original symmetric speed delay/time constant, preserving existing stable
config behavior.
