# Methodology

## How a claim is made in this repo

1. Define the task (start, goal, world, policy).
2. Run `examples/run_obstacle_navigation.py` with pinned seeds.
3. Save the JSON report to `benchmarks/`.
4. Cite the numbers in README/docs with a pointer to the artifact.

No number in this repo is estimated — every figure in `README.md` exists in
`benchmarks/result.json`.

## Reproducibility contract

- Python version and dependency groups pinned via `pyproject.toml`.
- Experiments seeded explicitly (`--seeds 0,1,2,...`).
- Physics backends use fixed timesteps and headless modes.

## Honesty rules

- Physics backends are **real engines with real differences**; we do not
  post-process results to make them look identical.
- Known limitations are listed in the CHANGELOG, not hidden.
- A failing experiment is a result, not an embarrassment — log it, fix the
  cause, and let the report show the change.
