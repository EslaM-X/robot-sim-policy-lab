"""Metrics: deterministic measurements collected during an episode.

Every metric is derived from the state history, not from the simulator's
internal claims, so the same collector works for every backend.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

from rspl.core import EpisodeReport, Pose, State
from rspl.world import World


@dataclass
class EpisodeCollector:
    """Collects states and computes summary metrics on completion."""

    task_name: str
    simulator: str
    world: World
    start: Pose
    seed: int = 0
    goal_tolerance: float = 0.15
    states: List[State] = field(default_factory=list)
    collisions: int = 0
    contacts: int = 0

    def record(self, state: State) -> None:
        self.states.append(state)
        if state.in_contact:
            self.contacts += 1
            # count a collision only on transitions into contact (avoid dupes)
            if len(self.states) < 2 or not self.states[-2].in_contact:
                self.collisions += 1

    def path_length(self) -> float:
        total = 0.0
        for a, b in zip(self.states[:-1], self.states[1:]):
            total += a.pose.distance_to(b.pose)
        return total

    def min_clearance(self) -> float:
        if not self.states:
            return 0.0
        return min(self.world.clearance(s.pose.x, s.pose.y) for s in self.states)

    def report(self, success: bool, goal: Pose, max_steps: int) -> EpisodeReport:
        steps = len(self.states)
        duration = steps * 0.05 if steps else 0.0
        end = self.states[-1].pose if self.states else self.start
        return EpisodeReport(
            task_name=self.task_name,
            simulator=self.simulator,
            success=success,
            steps=steps,
            duration=duration,
            path_length=self.path_length(),
            min_clearance=self.min_clearance(),
            collisions=self.collisions,
            contacts=self.contacts,
            start=self.start,
            end=end,
            seed=self.seed,
            metadata={"goal": {"x": round(goal.x, 4), "y": round(goal.y, 4)}, "max_steps": max_steps},
        )


@dataclass
class ExperimentSummary:
    """Aggregated metrics over many episodes (deterministic given seeds)."""

    simulator: str
    policy: str
    episodes: int
    success_count: int
    collision_count: int
    mean_path_length: float
    mean_duration: float
    min_clearance_p50: float

    @property
    def success_rate(self) -> float:
        return self.success_count / max(self.episodes, 1)

    @property
    def collision_rate(self) -> float:
        return self.collision_count / max(self.episodes, 1)

    def to_dict(self) -> dict:
        return {
            "simulator": self.simulator,
            "policy": self.policy,
            "episodes": self.episodes,
            "success_rate": round(self.success_rate, 4),
            "collision_rate": round(self.collision_rate, 4),
            "mean_path_length": round(self.mean_path_length, 4),
            "mean_duration": round(self.mean_duration, 4),
            "min_clearance_p50": round(self.min_clearance_p50, 4),
        }
