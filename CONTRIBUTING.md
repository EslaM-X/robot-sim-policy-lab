# Contributing

Thanks for your interest in `robot-sim-policy-lab`.

## Ground rules

- **Evidence over claims.** If a PR changes results, it must come with a run
  artifact under `benchmarks/` (or a test).
- **Keep the core physics-free.** `rspl/core`, `policy`, `planner`,
  `controller`, and `metrics` must import nothing from MuJoCo/PyBullet.
- **One shared interface.** New simulator backends implement the same API
  (`reset`, `step`, `state`, `name`) as the existing three.

## Getting started

1. Fork and clone.
2. `pip install -e ".[test]"` then `python -m pytest tests/`.
3. For physics work: `pip install -e ".[mujoco,pybullet]"`.

## Pull requests

- Small, focused changes.
- Every change adds or updates a test.
- Run the full suite locally before opening the PR.

## Code of conduct

Be respectful and constructive. See `CODE_OF_CONDUCT.md`.
