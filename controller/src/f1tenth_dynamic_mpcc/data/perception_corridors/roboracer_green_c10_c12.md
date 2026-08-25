# RoboRacer C10-C12 green perception corridor

- MPCC track: `raceline_smooth_c11_outer_boundary_expanded_candidate`
- Perception boundary inset: `0.22 m`
- C10 entry: `s=59.276202114 m`
- C12 exit: `s=81.297932048 m`
- Within that interval, `w_tr_left_m` and `w_tr_right_m` are copied from
  `raceline_smooth` (the unexpanded track).
- Outside that interval, every row comes from the expanded MPCC track.

The centerline, heading, curvature, speed profile, and all MPCC inputs remain
unchanged. This CSV is used only by raw-cloud obstacle filtering.
