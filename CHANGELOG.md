# Changelog

All notable changes to `robot-sim-policy-lab`.

## [Unreleased]

### Added
- Architecture diagram (`docs/diagrams/robotics-policy-lab.mmd`, rendered
  inline in `docs/architecture.md`): the control loop, the physics-free layer
  boundary, the three interchangeable backends, and the sim-to-sim validation
  flow, each mapped to its module and test in the evidence map.

## [v0.1.0] — 2026-08-14

Initial release.

### Added
- Policy layer: `GoToGoalPolicy`, `WaypointPolicy`, `ObstacleAvoidancePolicy`
  (potential-field, deterministic, parameterised).
- Planner layer: `DirectPlanner`, `ClearancePlanner`.
- Controller: PD velocity controller with bounded outputs.
- Simulators behind one interface: `kinematic` (unicycle reference),
  `mujoco`, `pybullet` (real physics).
- Metrics: success, collisions, contacts, path length, min clearance, duration.
- Validation: seeded experiment runner + `sim2sim` cross-simulator comparison.
- Example: `examples/run_obstacle_navigation.py`.
- Tests: pure + kinematic suite (16 tests, any machine) + optional physics
  tests (auto-skip when engines absent).
- Benchmark artifact: `benchmarks/result.json` (3 simulators, 3 seeds).

### Known limitations
- Robots are modelled as free bodies / unicycles, not articulated legged
  robots — the lab targets policy measurement, not mechanical fidelity.
- MuJoCo/PyBullet contact models differ from each other and from the kinematic
  reference by design; results are backend-specific.
