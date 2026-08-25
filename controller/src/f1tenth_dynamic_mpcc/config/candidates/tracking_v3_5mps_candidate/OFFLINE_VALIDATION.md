# Tracking V3 5.0/3.5 candidate offline validation

This candidate is isolated from the current 4 m/s tracking profile.

## Policy

- global MPCC ceiling: 5.0 m/s;
- straight local limit: 6.0 m/s, clipped to 5.0 m/s globally;
- ordinary and former 0.8x tight-bend local limits: 3.5 m/s;
- A3--A6 dynamic arc local limit: unchanged at 4.0 m/s;
- pure-pursuit hard maximum: unchanged at 4.0 m/s;
- pure-pursuit recovery: unchanged at 1.20--1.50 m/s;
- online profile deceleration envelope: 1.5 m/s^2.

## Replanned envelope

The runtime-equivalent `PeriodicTrack` planner was evaluated with the
candidate controller, vehicle and raceline files.

| Continuous region | Planned minimum | Planned maximum | Planned P95 |
|---|---:|---:|---:|
| straight after s=0 | 1.987 | 5.000 | 5.000 |
| ordinary region | 1.484 | 3.500 | 3.500 |
| tight bend 1 | 1.423 | 3.274 | 3.195 |
| short ordinary bridge | 2.874 | 3.145 | 3.130 |
| tight bend 2 | 1.417 | 2.853 | 2.705 |
| A3--A6 dynamic arc | 2.823 | 4.000 | 4.000 |
| straight before s=0 | 4.752 | 5.000 | 5.000 |

Only about 5.09 m of the lap is planned at or above 4.9 m/s because the
1.5 m/s^2 backward braking envelope starts slowing before the next turn.

The candidate raceline has the same 320 geometry/width rows as racelineV3.
The only changed CSV cells are `speed_limit_mps` in 225 `standard`,
`tight_bend_1_standard`, and `tight_bend_2_standard` rows.

## Checks

- candidate hash check: passed;
- launch XML validation: passed;
- Python syntax and shell syntax: passed;
- speed-planning, periodic-track, command-manager and startup-state-machine
  unit tests: 19 passed;
- local acados generation: not completed because this workstation does not
  contain the CasADi/acados_template source environment. The candidate uses a
  separate solver cache and can be generated on the car without touching the
  currently running solver cache.

This validation is not closed-loop approval for 5 m/s. The residual model's
identified range does not establish accuracy at 5 m/s.
