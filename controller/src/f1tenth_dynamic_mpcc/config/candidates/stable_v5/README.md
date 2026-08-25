# Stable V5

Stable V5 was frozen on 2026-08-21 from the validated Stable V3 60 Hz
brake4/asymmetric automatic-relaunch race candidate.

Release behavior:

- Stable V3 H3/scale-0.30 residual dynamics;
- contour 45, race heading 1.0 and steering-rate cost 0.60;
- 60 Hz MPCC solve, command publisher and hardware supervisor;
- E-stop/carry detection, ground stability gate and global-S reprojection;
- 2.0 m/s arbitrary-position recovery PP with adaptive minimum 1.2 m/s;
- arbitrary-position PP-to-MPCC handoff permitted from 1.0 m/s;
- alternating left/right race-grid launch within 2.0 m at 4.0 m/s;
- race-start PP-to-MPCC handoff permitted from 1.2 m/s;
- smooth PP-to-MPCC handoff after steering/speed agreement;
- 5.0 m/s local ceilings in designated arc/straight fast zones;
- 3.2 m/s local ceilings in all ordinary zones;
- 4.0 m/s^2 speed-profile, publisher and fallback deceleration;
- asymmetric acceleration (delay/tau 0.13/0.20 s) and braking
  (delay/tau 0.060/0.166 s) dynamics.

Track: `racelinev3_stable_v5_fast5_std32`, 320 rows,
87.759715 m.

Run:

```bash
MPCC_RECORD_BAG=true RESIDUAL_MPCC_SPEED_CAP=5.0 \
  ./scripts/start_mpcc_hardware_stable_v5.sh mpcc
```
