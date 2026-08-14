"""Core data structures shared across the policy lab.

Kept free of any physics-engine import so the whole pipeline can be unit
tested without MuJoCo or PyBullet installed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Pose:
    """A 2D pose: position (x, y) in metres plus heading in radians."""

    x: float
    y: float
    theta: float = 0.0

    def distance_to(self, other: Pose) -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def heading_to(self, other: Pose) -> float:
        return math.atan2(other.y - self.y, other.x - self.x)

    def __add__(self, other: Pose) -> Pose:
        return Pose(self.x + other.x, self.y + other.y, self.theta + other.theta)


@dataclass(frozen=True)
class State:
    """Full observation of the robot inside a simulation step."""

    pose: Pose
    velocity: float = 0.0  # linear speed m/s
    angular_velocity: float = 0.0  # rad/s
    in_contact: bool = False
    t: float = 0.0


@dataclass(frozen=True)
class Action:
    """A low-level control command emitted by a controller."""

    linear_velocity: float  # m/s
    angular_velocity: float  # rad/s

    def clamped(self, max_linear: float, max_angular: float) -> Action:
        return Action(
            linear_velocity=max(-max_linear, min(max_linear, self.linear_velocity)),
            angular_velocity=max(-max_angular, min(max_angular, self.angular_velocity)),
        )


@dataclass(frozen=True)
class Task:
    """A task the policy must satisfy."""

    goal: Pose
    waypoints: list[Pose] = field(default_factory=list)
    max_steps: int = 1000
    goal_tolerance: float = 0.15  # m
    name: str = "go-to-goal"


@dataclass
class EpisodeReport:
    """Machine-readable outcome of a single episode."""

    task_name: str
    simulator: str
    success: bool
    steps: int
    duration: float
    path_length: float
    min_clearance: float
    collisions: int
    contacts: int
    start: Pose
    end: Pose
    seed: int
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "task_name": self.task_name,
            "simulator": self.simulator,
            "success": self.success,
            "steps": self.steps,
            "duration": round(self.duration, 4),
            "path_length": round(self.path_length, 4),
            "min_clearance": round(self.min_clearance, 4),
            "collisions": self.collisions,
            "contacts": self.contacts,
            "start": {"x": round(self.start.x, 4), "y": round(self.start.y, 4)},
            "end": {"x": round(self.end.x, 4), "y": round(self.end.y, 4)},
            "seed": self.seed,
            "metadata": self.metadata,
        }
