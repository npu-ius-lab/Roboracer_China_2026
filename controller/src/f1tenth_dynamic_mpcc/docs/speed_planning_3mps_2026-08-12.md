# Runtime speed planning with a 3 m/s cap

The local controller no longer fixes baseline/debug operation to 1 m/s.

The runtime profile is computed from the unchanged `virtual_track/raceline.csv`
geometry and the configured vehicle limits:

- maximum speed: 3.0 m/s;
- maximum lateral acceleration: 3.2 m/s²;
- maximum longitudinal acceleration: 2.4 m/s²;
- maximum deceleration: 4.0 m/s²;
- steering angle/rate limits: 0.55 rad and 5.5 rad/s.

The initial pointwise cap combines curvature, steering-angle and steering-rate
limits. Circular forward/backward passes then enforce longitudinal acceleration
and braking continuity across the track seam.

For the active `virtual_track`, static validation gives:

- planned speed: 1.852779–3.000000 m/s;
- longitudinal acceleration: -4.000000–2.400000 m/s²;
- maximum lateral acceleration: 3.200000 m/s²;
- original CSV SHA-256 remains
  `f90c6de4e38e00764293ba58f3eadfa34d19f9d5c267180de42e6e18a25a2478`.

The 3 m/s value is therefore a ceiling, not a fixed command. Startup crawl,
observer/latency safety caps and smooth recovery remain active.
