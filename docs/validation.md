# Validation

## What "validated" means here

An experiment is validated when **all** of these hold:

1. The task is defined (start, goal, world, policy, seeds).
2. A full run completed on at least the kinematic reference.
3. Metrics were collected from the state history, not the engine's claims.
4. The JSON report is committed under `benchmarks/`.
5. Any cross-simulator claim cites the `sim2sim` deviation number.

## Sim-to-sim

`sim2sim(...)` runs the identical policy on multiple backends and reports:

- per-backend aggregate metrics (`success_rate`, `collision_rate`,
  `mean_path_length`, `min_clearance_p50`),
- the **max mean-path-length deviation** across backends.

This number is the honest measure of "do the simulators agree?" Small deviation
means the policy is robust to integrator differences; large deviation is a
finding worth investigating.

## Test matrix

| Suite | Coverage | Runs without physics engines? |
| --- | --- | --- |
| `test_pure.py` | core, world, policy, controller, metrics | ✅ yes |
| `test_experiment.py` | full kinematic stack, determinism, sim2sim shape | ✅ yes |
| `test_physics.py` | MuJoCo + PyBullet episodes | ❌ (auto-skips) |
