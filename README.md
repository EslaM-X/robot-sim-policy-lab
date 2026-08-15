# robot-sim-policy-lab

> Policy-driven robotics simulation experiments with measurable task execution,
> physics validation, and simulator-to-simulator testing.

A small but real policy lab: the same task runs through **policy → planner →
controller → simulator → metrics → validation**, on a kinematic reference and
two full physics engines (MuJoCo and PyBullet). Every number in this README
comes from an actual run of this code (see `benchmarks/result.json`).

**Part of [EslaM-X's engineering portfolio](https://github.com/EslaM-X/portfolio).**

---

## Demo (real run, ~1 second)

```sh
pip install -e ".[test]"
python examples/run_obstacle_navigation.py --sim kinematic --seeds 0
```

Actual output — a robot navigating an obstacle field from `(0,0)` to `(5.0, 0.5)`:

```json
{
  "simulator": "kinematic",
  "policy": "potential-field",
  "success_rate": 1.0,
  "collision_rate": 0.0,
  "mean_path_length": 5.1545,
  "min_clearance_p50": 0.2356
}
```

The same task also runs on real physics engines (MuJoCo, PyBullet) and compares
them head-to-head in the sim-to-sim report below.

## What it does

```
Task → Policy (potential-field) → Controller (PD on heading/distance)
     → Simulator (kinematic | mujoco | pybullet) → Metrics → Validation report
```

- **Policy layer** — `GoToGoalPolicy`, `WaypointPolicy`, `ObstacleAvoidancePolicy`
  (attractive + repulsive potential field). Pure, deterministic, parameterised.
- **Planner layer** — `DirectPlanner`, `ClearancePlanner` (waypoint insertion with
  clearance checks).
- **Controller** — proportional on heading + distance error, with bounded
  linear/angular velocity and slow-down-on-turn logic.
- **Simulators** — one shared interface, three backends:
  - `kinematic` — exact unicycle integrator (reference, runs anywhere)
  - `mujoco` — real physics via a free-joint body in an MJCF world
  - `pybullet` — real physics via DIRECT headless client
- **Metrics** — success, collisions, contacts, path length, min clearance,
  duration, collected identically across all backends.
- **Validation** — reproducible experiments (seeded) and `sim2sim` comparison
  that reports cross-simulator path deviation.

---

## Measured results

Reference experiment — obstacle navigation through `sparse_obstacle_field()`
from `(0, 0)` to `(5.0, 0.5)`, `ObstacleAvoidancePolicy`, 3 seeds
(`benchmarks/result.json`):

| Simulator | Success rate | Collision rate | Mean path length (m) | Min clearance p50 (m) |
| --- | --- | --- | --- | --- |
| kinematic | 100% | 0% | 5.14 | 0.235 |
| mujoco | 100% | 0% | 5.15 | 0.236 |
| pybullet | 100% | 0% | 5.40 | 0.086 |

**Sim-to-sim:** max mean-path-length deviation across simulators = **0.26 m**.

> MuJoCo tracks the kinematic reference within ~1 cm of mean path length while
> PyBullet's contact model produces a slightly longer path — exactly the kind
> of measurable difference this lab exists to surface.

---

## Install

```bash
pip install -e .           # core (numpy only)
pip install -e ".[mujoco,pybullet]"   # physics backends
pip install -e ".[test]"   # test tooling
```

Requires Python ≥ 3.10.

## Run

```bash
# kinematic reference
python examples/run_obstacle_navigation.py --sim kinematic

# one physics engine
python examples/run_obstacle_navigation.py --sim mujoco
python examples/run_obstacle_navigation.py --sim pybullet

# all three, with sim-to-sim comparison
python examples/run_obstacle_navigation.py --sim all --seeds 0,1,2
```

## Test

```bash
python -m pytest tests/            # pure + kinematic (any machine)
python -m pytest tests/            # physics tests auto-run if engines installed
```

## Repo layout

```
rspl/
  core/       # data structures (no physics imports)
  policy/     # task-conditioned policies
  planner/    # waypoint planners
  controller/ # low-level velocity controller
  sim/        # kinematic + MuJoCo + PyBullet backends
  metrics/    # episode collector and experiment summary
  validation/ # experiment runner + sim2sim comparison
examples/     # runnable experiments
tests/        # pure + kinematic + physics tests
benchmarks/   # measured result artifacts
docs/         # architecture, methodology, validation notes
```

## Honesty boundary

- The kinematic backend is the **reference**; physics backends are real engines
  with real contact models, so their numbers may differ — and **do**, which is
  the point of sim-to-sim.
- Experiments are seeded and deterministic for a given backend.
- Collisions are detected by each backend's own contact model, not shared
  geometry — so comparing collision counts across backends is meaningful, not
  circular.

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Security issues: [SECURITY.md](SECURITY.md).

If this work is useful to you, consider starring the repository — it helps the
project reach more engineers.

## License

Apache-2.0. Copyright © 2026 EslaM-X. See [LICENSE](LICENSE).
