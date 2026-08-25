# Straight-line speed identification bag protocol

This protocol identifies Ackermann command to physical longitudinal speed. It
does not start MPCC and does not publish vehicle commands automatically.

## Preconditions

1. Use a straight, level, grippy area with enough braking distance and a human
   holding the remote emergency stop.
2. Keep tyres, payload and battery type unchanged throughout a dataset.
3. Start MID360, Point-LIO and localization. Confirm:

   ```bash
   rostopic hz /localization/vehicle_odom
   rostopic echo -n 1 /localization/status
   ```

   Require about 30--50 Hz, `localized: True`, and frames
   `map -> localization_base_link`.
4. Centre steering mechanically. Do not collect static-map data in curves.
5. Confirm there is only one intended publisher on
   `/tianracer/ackermann_cmd`.

## Record each run

On the car:

```bash
cd /home/tianbot/f1tenth_mpcc
./scripts/record_speed_identification.sh fitting run01_up
```

The recorder only records. In a second terminal, after clearing the straight
test area and holding the physical remote ready, publish the guarded sequence:

```bash
cd /home/tianbot/f1tenth_mpcc
source scripts/ros_env.sh
python3 scripts/run_speed_identification_steps.py up --execute
```

The publisher fixes steering at zero, publishes at 20 Hz, and publishes zero
repeatedly on completion or Ctrl-C. Stop the recorder with Ctrl-C afterwards.

For one fixed speed with automatic recording and stopping, use the simpler
single-terminal command:

```bash
source scripts/ros_env.sh
python3 scripts/capture_fixed_speed.py fitting run01_050_up 0.50 --duration 5 --execute
```

It records 2 s at zero, 5 s at the requested fixed command, then 2 s at zero.
Use `--duration 0` to hold continuously until Ctrl-C. Ctrl-C always sends a
zero-command tail before closing the bag.

First low-speed sweep:

```text
up:   0 -> 0.50 -> 0.75 -> 1.00 -> 1.25 -> 1.50 -> 2.00 -> 0
down: 0 -> 2.00 -> 1.50 -> 1.25 -> 1.00 -> 0.75 -> 0.50 -> 0
```

For every plateau: allow at least 0.8 s transient, then hold for 3--5 s. Make
three up/down repetitions for fitting (`run01`--`run03`). Repeat two more runs
under the same conditions in the `validation` split (`run04`--`run05`). Never
reuse a fitting bag as validation.

Only after low-speed plots and frame checks pass may the measured command range
be expanded gradually. The current vehicle software limit is 3 m/s; do not
exceed it merely to satisfy a generic protocol.

## Mandatory topics

```text
/tianracer/ackermann_cmd
/localization/vehicle_odom
/tianracer/odom
/tianracer/imu
/livox/imu
/localization/status
/diagnostics
/tf
/tf_static
```

If `/diagnostics` has no messages it remains optional; extraction fills battery
voltage with NaN. Do not substitute wheel speed for localization speed without
the scale/slip report.

## Dataset acceptance before fitting

- Both increasing and decreasing sequences are present.
- At least three fitting and two independent validation runs exist.
- Every intended plateau has at least 1.5 s automatically detected steady data.
- `abs(vy_loc)` and yaw rate are small during accepted straight segments.
- Command, localization, wheel odom and IMU timestamps overlap.
- No localization reset/jump occurs inside an accepted segment.
