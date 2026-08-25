# V3PRO control-curvature-speed raceline candidate

Status: offline validated candidate; not yet hardware approved.

The frozen V3PRO controller, vehicle model, residual model, braking, safety,
automatic relaunch and race-start behavior are reused unchanged. Only the track
raceline and its regional raw speed limits differ.

## Method

- Training evidence: clean RUNNING interval 19.5--137.5 s from
  `stable_v3pro_20260822_015637.bag` (5903 moving samples, about 2.82 laps).
- Geometry objective: reduce curvature RMS/peak, curvature gradient and the
  feed-forward steering-rate proxy while respecting the physical corridor.
- Maximum lateral movement from the geometry seed: 0.16 m.
- Minimum center-to-boundary clearance: 0.34 m.
- Raw speed limits are never lower than the source V3PRO limits.
- Standard-road boosts are kept only in continuous control-clear windows:
  3.3 m/s for the window and 3.5 m/s for well-observed cores. Short isolated
  recommendations are discarded to avoid target-speed chatter.

## Dense offline audit

| Metric | V3PRO | Candidate | Change |
|---|---:|---:|---:|
| maximum absolute curvature | 1.5929 1/m | 1.3952 1/m | -12.4% |
| curvature RMS | 0.4927 1/m | 0.4623 1/m | -6.2% |
| maximum curvature gradient | 2.1877 1/m2 | 1.7681 1/m2 | -19.2% |
| steering-rate proxy p95 | 0.9005 rad/s | 0.7139 rad/s | -20.7% |
| steering-rate proxy maximum | 1.6691 rad/s | 1.4366 rad/s | -13.9% |
| planned-speed minimum | 1.4173 m/s | 1.5144 m/s | +0.0971 m/s |
| planned-speed mean | 2.9134 m/s | 2.9792 m/s | +0.0659 m/s |
| ideal profile lap | 32.2705 s | 32.2403 s | -0.0301 s |

All C0--C6 corner regions reduce their peak curvature. Tight-bend raw limits
remain 3.0 m/s; the 4.0 m/s dynamic arc and runtime cap remain unchanged.

## Delayed closed-loop audit

The C++ V3PRO closed-loop test used a 4.0 m/s runtime cap, 110--130 ms odometry
age, dynamic speed-drop/recovery, one deliberately unsafe MPCC horizon and PP
takeover/release. Each test ran 1200 MPCC solves over 30 seconds.

| Start offset | Track | distance in 30 s | max |e_contour| | min track margin | status-4 | PP takeovers |
|---:|---|---:|---:|---:|---:|---:|
| 0.00 m | V3PRO | 85.265 m | 0.2391 m | 0.1366 m | 0 | 1 |
| 0.00 m | candidate | 91.909 m | 0.2380 m | 0.1107 m | 0 | 1 |
| +0.15 m | V3PRO | 84.327 m | 0.2403 m | 0.1365 m | 0 | 1 |
| +0.15 m | candidate | 94.037 m | 0.2002 m | 0.1091 m | 0 | 1 |
| -0.15 m | V3PRO | 84.883 m | 0.3610 m | 0.0897 m | 0 | 1 |
| -0.15 m | candidate | 84.896 m | 0.3612 m | 0.1157 m | 0 | 1 |

The candidate is not slower in any of the three tested initial placements.
Its nominal-start minimum margin is 0.1107 m, so the first hardware run should
still be treated as a candidate validation run with bag recording and E-stop
coverage.

