"""Validation: reproducible experiment execution and sim-to-sim comparison.

The runner executes the exact same policy/planner/controller/simulator stack
for a batch of seeds and emits machine-readable JSON reports. This is the
honesty boundary of the whole lab: every number in the README comes from a run
of this code, not from estimation.
"""

from __future__ import annotations

import json
import statistics
from typing import List

from rspl.controller import VelocityController
from rspl.core import Action, EpisodeReport, Pose, Task
from rspl.metrics import EpisodeCollector, ExperimentSummary
from rspl.policy import ObstacleAvoidancePolicy, Policy
from rspl.sim import make_simulator
from rspl.world import World


def build_policy(name: str) -> Policy:
    if name == "potential-field":
        return ObstacleAvoidancePolicy()
    raise ValueError(f"unknown policy: {name}")


def run_episode(
    simulator: str,
    world: World,
    policy: Policy,
    start: Pose,
    goal: Pose,
    max_steps: int = 400,
    goal_tolerance: float = 0.15,
    seed: int = 0,
) -> EpisodeReport:
    """Run one episode to completion and return its report.

    The policy selects targets; the controller converts them to velocity
    commands; the simulator integrates; the collector measures. This loop is
    deliberately shared by every backend.
    """
    import random

    rng = random.Random(seed)
    goal_jitter = Pose(
        goal.x + rng.uniform(-0.05, 0.05),
        goal.y + rng.uniform(-0.05, 0.05),
    )
    sim = make_simulator(simulator, world, start=start)
    sim.reset(start)
    controller = VelocityController()
    collector = EpisodeCollector(
        task_name="obstacle-navigation",
        simulator=sim.name,
        world=world,
        start=start,
        seed=seed,
        goal_tolerance=goal_tolerance,
    )
    success = False
    for _ in range(max_steps):
        state = sim.state
        collector.record(state)
        target = policy.select_target(state, Task(goal=goal_jitter, max_steps=max_steps), world)
        if target is None:
            break
        action = controller.compute(state, target)
        state = sim.step(action)
        collector.record(state)
        if state.pose.distance_to(goal_jitter) <= goal_tolerance:
            success = True
            break
    report = collector.report(success, goal_jitter, max_steps)
    return report


def run_experiment(
    simulator: str,
    world: World,
    policy: Policy,
    start: Pose,
    goal: Pose,
    seeds=None,
    max_steps: int = 400,
    goal_tolerance: float = 0.15,
) -> ExperimentSummary:
    """Run the same experiment over multiple seeds and aggregate."""
    seeds = seeds or [0, 1, 2, 3, 4]
    reports = [
        run_episode(
            simulator,
            world,
            policy,
            start,
            goal,
            max_steps=max_steps,
            goal_tolerance=goal_tolerance,
            seed=s,
        )
        for s in seeds
    ]
    successes = [r for r in reports if r.success]
    clearances = [r.min_clearance for r in reports]
    return ExperimentSummary(
        simulator=simulator,
        policy=policy.name,
        episodes=len(reports),
        success_count=len(successes),
        collision_count=sum(1 for r in reports if r.collisions > 0),
        mean_path_length=statistics.mean(r.path_length for r in reports) if reports else 0.0,
        mean_duration=statistics.mean(r.duration for r in reports) if reports else 0.0,
        min_clearance_p50=statistics.median(clearances) if clearances else 0.0,
    )


def sim2sim(
    simulators,
    world: World,
    policy: Policy,
    start: Pose,
    goal: Pose,
    seeds=None,
) -> dict:
    """Compare aggregate metrics across simulators (sim-to-sim validation).

    Returns a JSON-ready dict with per-simulator summaries plus the maximum
    deviation of the mean path length across simulators — the key number that
    says "both simulators agree within X".
    """
    summaries = []
    for sim in simulators:
        summary = run_experiment(sim, world, policy, start, goal, seeds=seeds)
        summaries.append(summary)
    if len(summaries) >= 2:
        lengths = [s.mean_path_length for s in summaries]
        max_dev = max(lengths) - min(lengths)
    else:
        max_dev = 0.0
    return {
        "simulators": [s.to_dict() for s in summaries],
        "max_path_length_deviation_m": round(max_dev, 4),
        "policies": [s.policy for s in summaries],
        "goal": {"x": round(goal.x, 4), "y": round(goal.y, 4)},
    }


def write_report(data, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
