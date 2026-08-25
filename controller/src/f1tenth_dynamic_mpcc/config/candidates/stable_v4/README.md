# Stable V4

Stable V4 is frozen from
`start_mpcc_hardware_stable_v3_5mps_auto_relaunch_brake4_asym_candidate.sh`
on 2026-08-21.

It includes:

- guarded initial launch;
- E-stop carry detection and restart from an arbitrary track position;
- global nearest-track reprojection after placement;
- forward Pure Pursuit launch and smooth PP-to-MPCC handoff;
- raceline V3 with 5 m/s fast zones and 3 m/s standard zones;
- 4.0 m/s^2 raceline profile and publisher deceleration;
- asymmetric longitudinal dynamics (accel delay/tau 0.13/0.20 s,
  brake delay/tau 0.060/0.166 s);
- stable V3 H3 residual dynamics, contour 45, heading 1.0, steering command
  rate 0.60.

Stable V4 keeps the original MPCC track-control margin of 0.05 m. The temporary
one-third corridor candidate is deliberately separate and is not part of this
release.

Run:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \
  ./scripts/start_mpcc_hardware_stable_v4.sh mpcc
```
