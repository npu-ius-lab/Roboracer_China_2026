# Cost-term and geometry-refresh diagnostics

Production C++ telemetry now includes these per-horizon diagnostic terms:

```text
cost_contour
cost_lag
cost_heading
cost_speed_prior
cost_progress
cost_control
cost_delta_control
cost_track_slack
cost_tire_slack
```

It also records:

```text
geometry_theta_shift
geometry_heading_shift
geometry_curvature_shift
```

The cost values are diagnostic reconstructions using the configured stage
weights and horizon integration interval. The acados `objective` field remains
the authoritative total NLP objective. Slack penalties use the solver's lower
and upper slack vectors. These signals are generated in the C++ control process
without file I/O; the existing asynchronous logger writes them to JSONL.

Race-mode reference scaling is intentionally:

| Term | Setting | Meaning |
|---|---:|---|
| contour | 18.0 | nominal-line lateral error |
| lag | 40.0 | projection/progress consistency |
| heading | 0.5 | weak regularization, not hard alignment |
| CSV speed prior | 0.0 | disabled in race mode |
| progress reward | 1.0 | primary race advancement incentive |
| track slack | 10000 linear + 100000 quadratic | dominant safety penalty |

Interpretation for the next real-car log:

- large `cost_heading` relative to `cost_contour` would indicate an unexpected
  reference conflict;
- `cost_speed_prior` must remain zero in race mode;
- large geometry shifts on the corrective RTI indicate theta/geometry
  linearization mismatch;
- a low `speed_cap_used` with a higher `speed_input_cap` confirms that the
  dynamic feasibility guard is actively allowing physically reachable braking;
- any `status=4` while the input cap is reachable should shift investigation to
  warm-start quality, Jacobian conditioning or HPIPM scaling rather than to
  raceline field count.

The delayed closed-loop A/B matrix is documented in
`status4_raceline_delay_validation_2026-08-12.md`.
