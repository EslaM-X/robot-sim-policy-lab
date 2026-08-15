# Roadmap

## North star

> A stranger can reproduce a policy benchmark (baseline vs. a new policy) in
> under five minutes and trust the numbers — because the harness, not the
> policy author, owns correctness.

Signal that matters, in order:

1. `--sim all --seeds` reproduces a full benchmark from a clean clone.
2. The `compare_to_baseline` results are understandable to a newcomer.
3. External contributors add a policy and run the benchmark themselves.

## Sequenced next

1. **Simpler entry point** — one documented command for "benchmark + compare"
   that hides harness details (the rspl CLI is deliberately NOT a thing;
   the run script is the CLI).
2. **More policies** — grow the policy library with clearly commented
   examples that map to documented policy-lab concepts.
3. **Regression report** — CI or a script that flags when a new policy
   degrades a key safety metric versus baseline.
4. **Adoption funnel** — CONTRIBUTING guide and good-first-issue labels.

## Explicitly out of scope (for now)

- A separate CLI package. `run_obstacle_navigation.py` already covers
  benchmark + sim2sim; `validate` is just pytest.
- New simulators without a demonstrated need.
- Cosmetic visuals that don't represent real runs.
