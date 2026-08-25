# racelinev4 P3/P4 offline candidate

Status: local offline candidate, not deployed to the vehicle.

Candidate raceline SHA-256:
`5f81d1ef3ef0a9f44ffa02ae3912c7ac678a3646057502dce6773315e2ea3196`.

## Geometry changes

- P3 uses a smooth signed normal offset from `-0.08 m` to `+0.03 m`. The entry
  and exit move toward the repeatable right-side driven path, while the tight
  U section is opened enough to remove the old curvature peak.
- P4 uses a C2-continuous outside/right shift up to `0.08 m`, including a long
  exit buffer because the confirmed P1 takeover begins during the P4 exit.
- Original physical left/right boundaries are preserved by adjusting the
  raceline-relative widths with the centerline offset.
- Minimum CSV widths remain `0.4675 m` left and `0.4475 m` right.

## Old versus candidate

| Metric | racelinev3 | candidate |
|---|---:|---:|
| P3 maximum absolute curvature | 1.342 1/m | 1.290 1/m |
| P3 maximum curvature gradient | 2.048 1/m² | 1.895 1/m² |
| P3 minimum planned speed | 1.544 m/s | 1.573 m/s |
| P3 maximum steering rate at prior | 1.669 rad/s | 1.708 rad/s |
| P4 maximum absolute curvature | 0.401 1/m | 0.395 1/m |
| P4 maximum curvature gradient | 0.265 1/m² | 0.298 1/m² |
| P4 minimum planned speed | 2.790 m/s | 2.845 m/s |
| P4 maximum steering rate at prior | 0.254 rad/s | 0.330 rad/s |

Both steering-rate estimates are below the effective hardware-launch limit of
`3.4 rad/s`. The physical actuator limit remains `5.5 rad/s`.

Re-applying the candidate normal offset to the latest bag's online compensated
contour diagnostic gives the following screening result. This is not a
closed-loop guarantee:

| Zone | v3 absolute contour P95 | candidate counterfactual P95 |
|---|---:|---:|
| P3 | 0.385 m | 0.330 m |
| P4 | 0.317 m | 0.239 m |

## Removal of the 0.8 speed multiplier

The two former `tight_bend_*_80pct` zones no longer multiply the normal
`3.0 m/s` ceiling by `0.8`:

- `s=59.786–64.723 m`: local ceiling `2.4 -> 3.0 m/s`.
- `s=65.820–69.934 m`: local ceiling `2.4 -> 3.0 m/s`.

They are now labelled `tight_bend_1_standard` and
`tight_bend_2_standard`. Curvature, lateral-acceleration, steering-rate and
longitudinal acceleration constraints remain active, so this is not a forced
3 m/s command through the hairpin center. With runtime planning, the global
minimum planned speed changes from about `1.124` to `1.417 m/s`.

## Validation

- 320 points; periodic seam position and tangent errors are zero.
- Runtime speed envelope: `1.417–4.000 m/s`.
- Longitudinal acceleration envelope: `-4.0–2.4 m/s²`.
- Maximum lateral acceleration: `3.2 m/s²`.
- Python periodic-track and speed-planning tests: `13 passed`.
- Standalone C++ core smoke with this candidate: passed, exit `0`.
- The legacy `cpp_candidate_closed_loop` test exits `21` for both unchanged
  racelinev3 and this candidate under the current controller/config pair; it
  assumes an obsolete recovery-speed behavior and is not candidate-specific.

Closed-loop vehicle testing is still required before deployment approval.

