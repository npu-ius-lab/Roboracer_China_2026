# ROBORACER ETH H2H candidate

This is an additive candidate. It does not modify or replace the frozen
`start_mpcc_hardware_stable_roboracer.sh`, localization, or
`raw_cloud_obstacle_perception` packages.

The architecture follows the ForzaETH race stack separation between global
tracking, trailing and overtaking. The deployed implementation remains C++17:

> ROS Noetic starts launch files through its Python `roslaunch` infrastructure.
> That wrapper is not part of the overtaking algorithm; the planner, state
> machine, MPCC runtime and command gate below are compiled C++ executables.

- `/raw_cloud_perception/obstacle_cloud` is consumed directly;
- timestamp-aligned `/localization/odom` converts detections to track progress;
- Euclidean components and persistent obstacle velocity are maintained in C++;
- `FREE/FOLLOW/PREPARE/PASS/RETURN/ABORT` is the C++ behavior machine;
- left/right Frenet candidates are checked against width, curvature and every
  detected component;
- the selected local reference is consumed by the isolated C++ MPCC runtime;
- only the C++ hardware gate can publish `/tianracer/ackermann_cmd`.

Modes:

```bash
./scripts/start_mpcc_hardware_stable_roboracer_eth_h2h_candidate.sh --shadow
./scripts/start_mpcc_hardware_stable_roboracer_eth_h2h_candidate.sh --static-test
./scripts/start_mpcc_hardware_stable_roboracer_eth_h2h_candidate.sh --hardware
```

Direct hardware deployment entry (still starts with the C++ command gate
disarmed):

```bash
MPCC_RECORD_BAG=true H2H_SPEED_CAP_MPS=2.5 \
  ./scripts/start_mpcc_hardware_stable_roboracer_eth_h2h.sh
```

`--shadow` is the default and starts no controller or chassis publisher.
`--static-test` starts the isolated MPCC output but no hardware gate.
`--hardware` starts the hardware gate disarmed; arm it only after checking
`/roboracer_eth_h2h/health`:

```bash
rosservice call /roboracer_eth_h2h_command_gate_cpp/set_armed "data: true"
```

Rollback is simply stopping this candidate. The frozen ROBORACER launcher and
its files are untouched.
