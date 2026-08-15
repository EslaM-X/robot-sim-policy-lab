# Architecture

## Layering

```
Task
  │
  ▼
Policy ──────────▶ target pose
  │
  ▼
Controller ─────▶ Action (linear, angular velocity)
  │
  ▼
Simulator ──────▶ State (pose, velocities, contact)
  │
  ▼
Metrics ────────▶ EpisodeReport
  │
  ▼
Validation ─────▶ ExperimentSummary / sim2sim report
```

## Invariants

- **Policy/planner/controller/metrics are pure** — no physics imports, fully
  unit-testable without any engine.
- **Simulators are interchangeable** — `reset()`, `step(action) -> State`,
  `state` and `name` are the entire contract.
- **Metrics are backend-agnostic** — derived from the state history alone, so
  comparing backends is meaningful.
- **Determinism** — the only randomness is the per-seed goal jitter; the same
  seed yields the same report on the same backend.

## Why three backends

| Backend | Motion model | Contact model | Value |
| --- | --- | --- | --- |
| `kinematic` | exact unicycle | analytic circles | reference, fast, portable |
| `mujoco` | MuJoCo free-joint integration | MuJoCo solver | real physics, reproducible |
| `pybullet` | Bullet rigid-body | Bullet contact | real physics, independent |

The kinematic backend is the **reference**. Physics backends are real — their
numbers can and do differ, and sim-to-sim exists to *measure* that difference.

## Diagram

Source: [diagrams/robotics-policy-lab.mmd](diagrams/robotics-policy-lab.mmd)
(rendered inline for GitHub; edit the `.mmd`, regenerate with `mmdc`).

```mermaid
flowchart TB
    subgraph PURE["Physics-free layers - no engine import, unit-testable without MuJoCo/PyBullet"]
        TASK["Task - rspl/core<br/>goal + waypoints + tolerance"]
        WORLD["World model - rspl/world<br/>circle obstacles, clearance(), collides()"]
        PLANNER["Planner - rspl/planner<br/>Direct / Clearance<br/>geometry only, no physics"]
        POLICY["Policy - rspl/policy<br/>GoToGoal / Waypoint / PotentialField<br/>pure: state + task + world -> target Pose or None"]
        CONTROLLER["Controller - rspl/controller<br/>VelocityController<br/>heading error -> angular velocity"]
        COLLECT["EpisodeCollector - rspl/metrics<br/>measures state history only,<br/>never the engine's internal claims"]
    end

    subgraph ENGINES["Engine-dependent layers - optional backends"]
        SIM["Simulator - rspl/sim<br/>make_simulator(name) factory"]
        KIN["KinematicSimulator<br/>rspl/sim/kinematic.py<br/>exact unicycle, no RNG<br/>deterministic reference"]
        MJ["MuJoCoSimulator<br/>rspl/sim/__init__.py<br/>free-joint box body"]
        PB["PyBulletSimulator<br/>rspl/sim/__init__.py<br/>DIRECT headless mode"]
    end

    REPORT["EpisodeReport - rspl/core<br/>machine-readable"]
    EXP["run_experiment - rspl/validation<br/>ExperimentSummary over seeds"]
    S2S["sim2sim - rspl/validation<br/>same policy across simulators,<br/>max path-length deviation"]
    JSON["benchmarks/result.json<br/>measured result, not estimation"]

    TASK --> PLANNER
    WORLD --> PLANNER
    WORLD --> POLICY
    PLANNER --> POLICY
    SIM -- "state" --> POLICY
    POLICY -- "target Pose or None" --> CONTROLLER
    CONTROLLER -- "Action (clamped)" --> SIM
    SIM --> KIN
    SIM --> MJ
    SIM --> PB
    SIM -- "states" --> COLLECT
    COLLECT --> REPORT
    REPORT --> EXP
    EXP --> S2S
    S2S --> JSON

    subgraph VALIDATION["Validation layer - every number from a real run"]
        REPORT
        EXP
        S2S
        JSON
    end
```

## Evidence map (diagram -> code)

| Component | Module | What it proves | Evidence |
| --- | --- | --- | --- |
| Planner | `rspl/planner/__init__.py` | geometry-only waypoints, no physics imports | `tests/test_pure.py` |
| Policy | `rspl/policy/__init__.py` | pure, deterministic target selection | `tests/test_pure.py` |
| Controller | `rspl/controller/__init__.py` | bounded velocity commands | `tests/test_pure.py` |
| Kinematic backend | `rspl/sim/kinematic.py` | exact unicycle reference, no engine, no RNG | `tests/test_pure.py` |
| MuJoCo / PyBullet | `rspl/sim/__init__.py` | real integrators behind the same contract | `tests/test_physics.py` |
| EpisodeCollector | `rspl/metrics/__init__.py` | metrics from state history only | `tests/test_experiment.py` |
| sim2sim | `rspl/validation/__init__.py` | measures cross-engine deviation, treats it as data | `tests/test_experiment.py` + `benchmarks/result.json` |

## See also

- [Methodology](methodology.md)
- [Validation](validation.md)
