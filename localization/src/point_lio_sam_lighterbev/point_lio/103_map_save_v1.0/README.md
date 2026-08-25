# Point-LIO map products

- `point_lio_map.pcd`: source 3D Point-LIO map (59,016 points).
- `point_lio_map_2d.pgm`: ROS occupancy image generated from the source PCD.
- `point_lio_map_2d.yaml`: `map_server` metadata for the PGM.

The checked-in conversion uses points between `z=0.1 m` and `z=1.5 m`, a
`0.5 m` radius outlier filter with at least 10 neighbours, and a map resolution
of `0.05 m/pixel`.

Regenerate both 2D files from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch point_lio_sam_lighterbev convert_point_lio_map_to_2d.launch
```

Generate a collision-checked virtual two-sided track, its centerline, and an
NMPC-compatible raceline from this map:

```bash
cd "$(rospack find point_lio_sam_lighterbev)/tools/virtual_track"
python3 generate_track_boundaries.py
python3 generate_centerline_raceline.py
```

The generated files are placed in `virtual_track/`. See
`../../tools/virtual_track/README.md` for schemas, parameters, and output details.

All settings and the input/output paths can be overridden as roslaunch args,
for example `resolution:=0.1 z_max:=2.0`.
