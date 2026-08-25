# Startup oscillation fix deployment — 2026-08-12

Implemented from `F1TENTH_MPCC_Startup_Oscillation_Fix_Codex.md`.

- Startup authority: `BOOT -> STEER_SETTLE -> CRAWL_OBSERVE -> MPCC_RAMP -> RACE`.
- Recovery authority: shifted previous horizon, then continuous `SAFE_DECEL`, then `STOPPED`; valid recovery returns through `MPCC_RAMP`.
- MPCC state is `[x,y,psi,vx,vy,r,delta,delta_c,theta]`; input is `[v_cmd,delta_c_dot,v_theta]`.
- `delta_c_dot` is bounded inside acados. The 20 Hz publisher has a final continuous slew limiter only as a safety backstop.
- Steering observation uses model-only, kinematic and dynamic correction modes with confidence gating.
- Point-LIO timestamp regression or pose jump triggers safe deceleration and requires a stable new sample before solving resumes.
- Solver and actuator publisher use separate ROS timers. Solver failure does not stop the 20 Hz publisher.

Validation:

- Local unit tests: 30 passed.
- Remote unit tests: 30 passed after deployment.
- Remote 9-state acados baseline generation: passed.
- Zero-speed offline solve: acados status 0, state horizon `(26, 9)`, input horizon `(25, 3)`.

The real controller was intentionally left stopped after deployment. Steering actuator and tire parameters remain assumed/nominal and require identification before increasing authority.
