# V3pro raceline_smooth run analysis

Only the second complete lap is compared. The launch/recovery lap and later stop/carry data are excluded.
No rosbag was produced for the smooth run; this analysis uses the complete controller JSONL telemetry and lap log.

| Metric | Original V3pro | raceline_smooth | Change |
|---|---:|---:|---:|
| Lap time [s] | 35.252 | 32.989 | -6.4% |
| Lap mean speed [m/s] | 2.515 | 2.649 | +5.3% |
| Contour MAE [cm] | 14.521 | 8.712 | -40.0% |
| Contour P95 [cm] | 34.598 | 18.613 | -46.2% |
| Heading P95 [rad] | 0.352 | 0.238 | -32.5% |
| PP active [s] | 1.964 | 0.687 | -65.0% |
| Prediction-risk fraction [%] | 4.260 | 0.552 | -87.1% |
| Track slack >1 mm [%] | 4.720 | 0.250 | -94.7% |
| Steering-rate P95 [rad/s] | 0.967 | 1.065 | +10.1% |
| Solve P95 [ms] | 8.880 | 7.677 | -13.5% |

The remaining smooth-run PP event is at estimated s=57.4–59.1 m.
