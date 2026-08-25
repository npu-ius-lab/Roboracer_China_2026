# MPCC 40 Hz, steering Td=0 bag analysis

Bag: `mpcc_40hz_td0_20260814_193817.bag` (98.94 s, 10 complete laps plus a partial lap)

## Conclusion

The 40 Hz conversion is working. acados returned status 0 for every one of the
3958 solves. Three otherwise valid solves took longer than the strict 25 ms
deadline and were rejected by the controller; the failure streak never
exceeded one and no solve tick was skipped. This was not a status-4 run.

Tracking is now limited mainly by prediction accuracy, not by solver frequency,
wheel-speed feedback, or a localization jump. The vehicle left the controller's
5 cm inset corridor three times but did not cross the physical boundary. The
minimum estimated physical margin was only 2 mm, so the third event was close.
The Pure Pursuit candidate activated 12 times and prevented these events from
growing into a larger departure.

Keeping runtime steering pure delay at `Td=0` matches the residual training
configuration. It does not prove that the real steering path has zero delay.
Command-to-yaw data has a best equivalent alignment near 0.195 s. That value
includes servo dynamics, tire/yaw dynamics, estimator filtering, and
closed-loop correlation, so it must not be copied directly into the pure
transport-delay parameter.

## Frequencies and solver

- MPCC telemetry/solve rate: 40.003 Hz.
- MPCC command and hardware command gate: 50.000 Hz.
- Point-LIO vehicle odometry: 32.86 Hz.
- Wheel odometry: 50.00 Hz.
- Solve time: median 3.58 ms, p95 8.48 ms, p99 12.98 ms, maximum 37.03 ms.
- Deadline rejects: 3/3958 (0.076%); acados status for all samples: 0.
- Warm-start advance: median 25.00 ms, or 0.500 OCP stage.
- Skipped overlapping solve ticks: 0.

## Tracking and boundary events

- Absolute contour error: median 0.132 m, p95 0.319 m, maximum 0.560 m.
- Actual control-corridor margin: minimum -0.048 m.
- Actual physical-boundary margin: minimum +0.002 m; no physical-boundary
  crossing was detected.
- Control-corridor excursions:
  - t=19.59..20.01 s, near track s=2.34 m;
  - t=30.02..30.47 s, near track s=1.51 m;
  - t=67.09..67.33 s, near track s=17.32 m.
- The first two events repeat in the same early-track corner on successive
  laps. The third event is on another part of the track and is the closest to
  the physical line.
- Laps 8 and 9 completed without candidate activation; their minimum control
  margins were 0.224 m and 0.241 m. Performance therefore varies by lap rather
  than failing everywhere in a fixed direction.

One prediction anomaly occurred at t=51.83 s: MPCC reported a minimum future
margin of -1.69 m while the reconstructed current physical margin was 0.342 m.
The independent candidate remained safe and took over for about 0.5 s. This is
a false-positive long-horizon prediction/track-projection event, not an actual
1.69 m departure.

## Model prediction versus later measured position

The prediction starts after the 0.13 s committed speed-command horizon. The
comparison below uses future Point-LIO poses by their source timestamps:

| Future time | Position p50 | Position p95 | Lateral p95 |
| --- | ---: | ---: | ---: |
| 0.13 s | 0.025 m | 0.049 m | 0.027 m |
| 0.38 s | 0.070 m | 0.162 m | 0.094 m |
| 0.63 s | 0.124 m | 0.321 m | 0.192 m |
| 0.88 s | 0.177 m | 0.487 m | 0.286 m |
| 1.13 s | 0.236 m | 0.721 m | 0.315 m |

Short compensation is accurate, but model uncertainty grows materially after
about 0.4--0.6 s. This explains why long-horizon boundary predictions can
trigger candidate recovery even while the current pose is still safe.

## Speed and steering response

- Command speed: mean 2.226 m/s, maximum 3.0 m/s.
- Point-LIO vx: mean 2.200 m/s, maximum 3.318 m/s.
- Wheel vx and Point-LIO vx align at 0 s lag with gain 1.000, correlation
  0.995, and RMSE 0.081 m/s. The lateral events are not caused by disagreement
  between these two speed measurements.
- Command speed to measured vx has an equivalent alignment near 0.28 s. This
  includes motor/vehicle response and must not be interpreted as pure network
  delay.
- Steering command to measured yaw response has an equivalent alignment near
  0.195 s and an effective kinematic gain near 0.91. There is no steering-angle
  sensor in the bag, so actual front-wheel angle and pure transport delay cannot
  be separated from these data alone.

## Localization health

- All 910 localization status samples were state 2, localized, frontend
  healthy, and `continuous_tracking`.
- No relocalization confirmation or frontend failure occurred.
- Motion-consistency residual: translation p95 7.6 mm, maximum 16.2 mm; yaw
  p95 0.254 deg, maximum 0.928 deg.
- No motion-corrected pose jump exceeded 8 cm or 5 deg.
- Three sub-0.1 ms, sub-millimetre Point-LIO reorder samples were logged and
  benignly discarded. They did not invalidate localization.
- Point-LIO receive age: mean 114 ms, p95 144 ms, maximum 170 ms.

## Recommended next experiment

Do not infer a new pure-delay value from this lap bag alone. For the next model
iteration, keep this deployed `Td=0` baseline and train/evaluate a residual
feature set with command history at approximately 0, 0.05, 0.10, 0.15, and
0.20 s. Compare held-out 0.4--1.1 s rollout error and the three recurrent
boundary locations. This can represent steering phase lag without forcing the
entire observed 0.195 s equivalent response into a single transport-delay
constant.
