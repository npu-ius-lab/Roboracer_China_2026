# Stable RoboRacer

Frozen on 2026-08-22 from the validated
`stable_v3pro_raceline_smooth_race_start_v2_candidate` hardware setup.

Release contract:

- `raceline_smooth`; only the left/right boundary widths differ from Raceline V3.
- The default RViz config listens only to the dedicated
  `/f1tenth_mpcc/stable_roboracer/*` topics, preventing latched markers from a
  simultaneously running V3/V5 process from masquerading as ProMax boundaries.
- MPCC solve rate 40 Hz and hardware command/supervisor rate 50 Hz.
- Runtime speed cap defaults to and cannot exceed 4.0 m/s.
- Cost overrides: contour 45.0, race heading 1.0, steering-command rate 0.60.
- Steering rate limit 3.4 rad/s.
- Arbitrary-position carry recovery keeps the conservative 2.0 m/s2 lateral limit.
- Recognized grid starts use a 3.2 m/s2 PP lateral limit and speed-relative MPCC handoff.
- Recognized grid starts ignore extracted-map vehicle/path margin gates so either
  physically checked grid slot can launch. The start-zone, progress, heading,
  localization-health and stable-ground gates remain active.
- Recovery projection is refreshed at 10 Hz while waiting, preventing the
  50/60 Hz odometry callbacks from starving the 0.30 s stable-ground window.
- Handoff still requires at most 0.08 rad steering disagreement for three consecutive
  samples and uses a 0.20 s blend.

Validation evidence:

- Hardware bag: `stable_v3pro_raceline_smooth_20260822_033149.bag`
- Bag SHA-256: `4625dfb0904da57d3ffea40170bfdd69a578c5787de2e1ced858b1d954749646`
- The promoted race-start fix was subsequently accepted by the operator after an
  on-vehicle test.

Start with recording:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=4.0 \
  ./scripts/start_mpcc_hardware_stable_roboracer.sh mpcc
```

The launcher verifies frozen hashes before preparing solvers or touching hardware.

To open only the matching ProMax visualization locally:

```bash
rviz -d $(rospack find f1tenth_dynamic_mpcc)/rviz/stable_roboracer.rviz
```
