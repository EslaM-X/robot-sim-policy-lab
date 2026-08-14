"""World model: obstacles expressed as circles, plus spatial queries.

The world is defined in 2D. Obstacles are approximated as circles so distance
queries are trivial, deterministic, and identical across simulators.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float
    name: str = "obstacle"


class World:
    """A planar world with circular obstacles and a safety margin for the robot."""

    def __init__(self, obstacles: Optional[List[CircleObstacle]] = None, robot_radius: float = 0.2):
        self.obstacles: List[CircleObstacle] = list(obstacles or [])
        self.robot_radius = robot_radius

    def add(self, obstacle: CircleObstacle) -> None:
        self.obstacles.append(obstacle)

    def clearance(self, x: float, y: float) -> float:
        """Minimum distance from point to any obstacle surface (negative = inside)."""
        if not self.obstacles:
            return math.inf
        return min(
            math.hypot(x - o.x, y - o.y) - o.radius for o in self.obstacles
        ) - self.robot_radius

    def collides(self, x: float, y: float) -> bool:
        for o in self.obstacles:
            if math.hypot(x - o.x, y - o.y) <= o.radius + self.robot_radius:
                return True
        return False

    def nearest_obstacle(self, x: float, y: float) -> Optional[Tuple[CircleObstacle, float]]:
        best = None
        best_dist = math.inf
        for o in self.obstacles:
            d = math.hypot(x - o.x, y - o.y)
            if d < best_dist:
                best_dist = d
                best = o
        return (best, best_dist) if best else None

    def to_dict(self) -> dict:
        return {
            "robot_radius": self.robot_radius,
            "obstacles": [
                {"name": o.name, "x": o.x, "y": o.y, "radius": o.radius}
                for o in self.obstacles
            ],
        }


def sparse_obstacle_field() -> World:
    """A sparse world with three well-separated obstacles (used in examples/benchmarks)."""
    world = World(robot_radius=0.2)
    world.add(CircleObstacle(3.0, 1.0, 0.5, "box-a"))
    world.add(CircleObstacle(2.0, 2.2, 0.6, "box-b"))
    world.add(CircleObstacle(4.2, 2.6, 0.4, "box-c"))
    return world
