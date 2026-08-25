# Stable RoboRacer arc/straight speed candidates

Both candidates are isolated derivatives of the frozen `stable_roboracer`
release. Geometry, track widths, controller weights, risk tiering, carry
detection, arbitrary-position recovery, race start, PP-to-MPCC handoff,
publisher rate and recording behavior are unchanged.

Only the normal MPCC speed ceilings and the named fast-zone local limits differ:

- `4p5`: dynamic arc, round-arc transition and straight acceleration = 4.5 m/s.
- `5p0`: dynamic arc, round-arc transition and straight acceleration = 5.0 m/s.
- Both: standard and the two tight-bend zones remain strictly capped at 3.0 m/s.
- Both: PP fallback/recovery maximum remains 4.0 m/s.
- Both: normal published-command deceleration is limited to 2.2 m/s².
- Both: predictive risk/hold, PP safety candidate, out-of-bounds recovery,
  checkpoint recovery, and solver/input-failure slowdown use 4.0 m/s².

## Offline closed-loop screen

The real acados solvers were generated on the car and screened for one lap from
nominal, +0.10 m/+0.08 rad and -0.10 m/-0.08 rad initial conditions.

| Candidate | Scenario | Lap (s) | Max speed (m/s) | RMS contour (m) | Max contour (m) | Min physical margin (m) | Solver failures |
|---|---|---:|---:|---:|---:|---:|---:|
| 4.5 | nominal | 31.150 | 4.481 | 0.0761 | 0.1803 | 0.2078 | 0 |
| 4.5 | left offset | 31.125 | 4.486 | 0.0784 | 0.1773 | 0.2077 | 0 |
| 4.5 | right offset | 31.150 | 4.495 | 0.0782 | 0.1835 | 0.2081 | 0 |
| 5.0 | nominal | 30.925 | 5.000 | 0.0764 | 0.1804 | 0.2078 | 0 |
| 5.0 | left offset | 30.900 | 5.000 | 0.0787 | 0.1772 | 0.2077 | 0 |
| 5.0 | right offset | 30.925 | 4.992 | 0.0785 | 0.1836 | 0.2082 | 0 |

All six scenarios completed a lap with 100% solver success. P95 solver time was
3.48--3.60 ms against the 25 ms solve deadline. Separate C++ tests verified the
2.2/4.0 m/s² publisher split, including first-failure shifted-solution braking,
and exercised PP/checkpoint takeover with fault injection. The screen does not model the
approximately 0.41 s speed response delay observed in the previous 5 m/s
hardware bag, so it is not hardware approval. The 4.5 m/s candidate should be
tested first with recording and the physical E-stop available.
