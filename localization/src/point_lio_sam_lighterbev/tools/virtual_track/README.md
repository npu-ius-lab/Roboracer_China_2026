# Point-LIO map to virtual NMPC track

This tool turns the open indoor occupancy map into a **virtual**, paired-boundary
race track. It deliberately does not run medial-axis extraction: the source map
contains room walls, not two physical track walls, so a medial axis would describe
the room rather than a usable closed race corridor.

The workflow has two explicit stages:

1. Select one boundary generator:
   - `generate_track_boundaries.py`: collision-free ellipse loop.
   - `generate_stadium_track.py`: two straights and two 180-degree U-turns.
   - `generate_complex_s_u_track.py`: straights, two U-turns, and an S-chicane.
   Each generator exports paired left/right boundaries, a corridor table, and a
   PGM/YAML map with the virtual boundary lines drawn into it.
2. `generate_centerline_raceline.py` reconstructs/resamples the boundary midpoint,
   performs a width-constrained low-curvature optimization, and exports the NMPC
   raceline contract used by the downstream NMPC workspace.

The stadium and complex generators enforce a minimum track width of `1.20 m`.
Use separate output directories so all three alternatives are preserved.

## Current `localization_map` dataset

The currently generated tracks use:

```text
point_lio/localization_map/point_lio_map_2d.yaml
point_lio/localization_map/point_lio_map_2d.pgm
```

From the workspace root, regenerate all three NMPC references from the existing
paired boundaries:

```bash
src/point_lio_sam_lighterbev/tools/virtual_track/generate_localization_map_racelines.sh
```

To use the same vehicle contract as the NMPC controller:

```bash
MAP_DIR="$(rospack find point_lio_sam_lighterbev)/point_lio/localization_map"
VEHICLE_CONFIG="$(rospack find f1tenth_nmpc_tracker)/config/tianracer_vehicle.yaml"
src/point_lio_sam_lighterbev/tools/virtual_track/generate_localization_map_racelines.sh \
  "$MAP_DIR" "$VEHICLE_CONFIG"
```

`generate_centerline_raceline.py --vehicle-config FILE` reads `vehicle`,
`limits`, and `planning` from the schema-version-1 YAML. Explicit numeric CLI
arguments still override values read from the file.

The command does not regenerate or overwrite `virtual_left_boundary.csv`,
`virtual_right_boundary.csv`, or `virtual_track_corridor.csv`. It updates only
`centerline.csv`, `raceline.csv`, `raceline_summary.json`, the three per-track
debug figures, and `three_virtual_tracks_overview.png`. An alternative map
directory can be supplied as the first argument.

## Raceline algorithm

For paired left/right samples, the centerline and widths are

```text
c_i       = 0.5 * (p_left_i + p_right_i)
w_left_i  = ||p_left_i  - c_i||
w_right_i = ||p_right_i - c_i||
```

The closed centerline is periodically resampled to 260 equally spaced points.
The optimizer represents lateral displacement with a periodic cubic spline:

```text
p_race(s) = c(s) + alpha(s) * n_left(s)
J = w_kappa * mean(kappa^2)
  + w_smooth * mean((alpha_i - alpha_{i-1})^2)
  + w_offset * mean(alpha^2)
```

The admissible alpha range is the conservative intersection of every local
corridor after subtracting half vehicle width, `0.08 m` planning buffer,
`0.25 m` tracker margin, and a `0.002 m` numerical guard. The ellipse and
stadium use 24 offset control points; the S/U track uses 16 to suppress local
oscillation through the chicane.

The speed profile starts from the most restrictive geometric limit:

```text
v_kappa = sqrt(a_y_max / max(|kappa|, epsilon))
delta   = atan(wheelbase * kappa)
v_ref   = min(v_max, v_kappa, v_steer, v_steer_rate)
```

Circular forward/backward passes then enforce acceleration and deceleration.
The resulting reference is not constant-speed: tight curvature or rapid steering
variation can reduce `vx_mps` below `1.85 m/s`.

`validate_localization_map_racelines.py` rejects bad schemas, non-finite values,
non-monotonic arc length, broken loop spacing, map-bound violations, insufficient
body clearance, and static steering/acceleration/lateral-acceleration limit
violations. This is a static precheck; low-speed closed-loop validation remains
required before increasing real-car speed.

## Outputs

Each track output directory contains:

```text
virtual_track*/
  virtual_left_boundary.csv
  virtual_right_boundary.csv
  virtual_track_corridor.csv
  virtual_track_map.pgm
  virtual_track_map.yaml
  centerline.csv
  raceline.csv
  virtual_track_boundaries_summary.json
  raceline_summary.json
  virtual_track_boundaries_debug.png
  centerline_raceline_debug.png
```

`raceline.csv` is directly compatible with the NMPC workspace trajectory reader:

```csv
s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,w_tr_right_m,w_tr_left_m
```

The defaults mirror the current real-car configuration used by the downstream
controller: wheelbase `0.265 m`, vehicle width `0.22 m`, speed range
`0.20..1.85 m/s`, steering limit `0.55 rad`, and track safety margin `0.08 m`.
The optimized line also keeps at least `0.25 m` after subtracting half the
vehicle width and that safety margin, matching the stricter current tracker
precheck.

The boundary generators remain available when a virtual track itself must be
redrawn. Their no-argument defaults now target `point_lio/localization_map`:

```bash
python3 generate_track_boundaries.py \
  --track-width-m 1.20 \
  --min-obstacle-clearance-m 0.25
python3 generate_stadium_track.py
python3 generate_complex_s_u_track.py
```

Do not run those three boundary commands when the hand-drawn/approved boundaries
must be preserved. Use `generate_localization_map_racelines.sh` instead.

The boundary search works in map-local coordinates and applies the ROS YAML origin
and yaw when writing CSV world coordinates. The debug figures use map-local axes so
they remain visually aligned with the PGM even if the YAML origin is non-zero.

For the NMPC ground-truth tracker, override the trajectory with the generated file
(the simulator/localization map frame must be this map frame):

```bash
roslaunch f1tenth_nmpc_tracker nmpc_tracker_gt_sim.launch \
  trajectory_csv:="$(rospack find point_lio_sam_lighterbev)/point_lio/localization_map/virtual_track/raceline.csv"
```
