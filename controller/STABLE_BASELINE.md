# Stable MPCC baseline

This branch freezes the best verified TianRacer MPCC configuration before
steering-actuator identification work begins.

## Runtime profile

- V1 Markov residual dynamics model
- 40 Hz MPCC solve and 50 Hz command publication
- 3.0 m/s runtime speed cap
- 1.5 rad/s MPCC steering-command-rate limit
- 1.5 rad/s Pure Pursuit candidate steering-rate limit
- 1.5 rad/s final command slew limit
- contour cost 45.0, race heading cost 1.0, steering-rate cost 0.90
- tiered prediction-risk handling and the mature MPCC/Pure Pursuit arbitration

Run it with:

```bash
./scripts/start_mpcc_hardware_stable.sh
```

The launcher verifies the controller, residual-model and vehicle-parameter
hashes before allowing the stable name to be used.

## Verification evidence

Reference run: `hardware_mpcc_20260815_225632.jsonl` and
`laps_mpcc_20260815_225632.csv`.

Representative pure-MPCC laps completed in approximately 7.37--7.46 s, with
about 2.91 m/s mean speed and 0.10 m mean absolute contour error across the
selected stable laps. The 3.4 rad/s experiment is retained separately on the
`experiment/steer-rate-3p4` branch.

## Frozen file hashes

```text
a78197b6c064c9f5f02bf6ed5d08dae0bbf231958d917a69a0ec6593d46e4f54  controller.yaml
16a18c94442f90415cf8bd0cea81b7c7887e0624adfa1f3e80a26e44246fe3fc  residual_markov_v1.yaml
2afe83c6a651a1a95a62d3a8f1d79145b85acfc46e2b63e4df715c0f9b62061e  vehicle.yaml
```

New actuator-identification work must be developed on a branch created from
this baseline. Do not tune unverified actuator parameters directly on
`stable`.
