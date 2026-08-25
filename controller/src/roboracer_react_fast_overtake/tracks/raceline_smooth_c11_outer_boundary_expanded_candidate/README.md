# React Fast C11 outer-boundary candidate

This is an independent copy of the 2026-08-23 C11 outer-boundary candidate.
It leaves the reference position, heading, curvature, speed profile, zones,
and right boundary unchanged. Only `w_tr_left_m` is expanded through the C11
right hairpin:

- transition in: approximately `s=66.20..67.10 m`
- full expansion: `+0.35 m` over approximately `s=67.10..69.75 m`
- transition out: approximately `s=69.75..70.65 m`

Expected SHA-256:
`cae8c6833492457bf2a884eb4c4760960b5b49bc7eac5fa16e1cd2a98194d354`

The original `f1tenth_dynamic_mpcc` and `raceline_smooth` files are not
modified by this package.
