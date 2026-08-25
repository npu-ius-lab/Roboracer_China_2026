# TianRacer overtaking/avoidance shadow module

This folder is intentionally independent from the residual-MPCC controller.
It subscribes to localization and the existing MID360 opponent-perception
output, then publishes:

- `/avoidance/status` and `/avoidance/decision`
- `/avoidance/markers` with ON-TRACK / OFF-TRACK labels
- `/avoidance/local_plan_red` and a red local-plan marker
- `/overtake/*` candidate paths and diagnostics

It never advertises or publishes an Ackermann command topic. The existing
MPCC trajectory selection, controller configuration and command gate stay
unchanged.

Run `scripts/start_avoidance.sh --check-only` for a non-running preflight.
Run `scripts/start_avoidance.sh --foreground` beside the real perception
stack. The residual controller wrapper lives at
`../scripts/start_mpcc_hardware_stablev2_dev_tracking_with_avoidance.sh`.

For raw MID360 bags, use `scripts/run_mid360_bag_shadow_test.sh`. It replays
the real Point-LIO localization and 3D opponent-perception chain on an
isolated ROS master and never starts a controller. `run_bag_shadow_test.sh`
is only a lightweight 2D `/scan` regression adapter; its synthetic targets
must not be used to validate the rear square-marker car.
