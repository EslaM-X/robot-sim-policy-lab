"""Planners: turn a task into an ordered set of waypoints the policy chases.

The planner operates purely on the world model (circle obstacles) using
visibility and clearance checks. It is geometry — no physics, no randomness.
"""

from __future__ import annotations

from rspl.core import Pose, Task
from rspl.world import World


class Planner:
    """Base class for task planners."""

    name: str = "planner"

    def plan(self, task: Task, world: World) -> Task:
        raise NotImplementedError


class DirectPlanner(Planner):
    """No planning: task goal is the only waypoint.

    Serves as the trivial baseline.
    """

    name = "direct"

    def plan(self, task: Task, world: World) -> Task:
        return task


class ClearancePlanner(Planner):
    """Insert intermediate waypoints that keep the path clear.

    Given start and goal, this planner samples candidate waypoints on the line
    between them and keeps only those with sufficient clearance from all
    obstacles, falling back to a dog-leg around the nearest blocking obstacle
    when the direct segment is unsafe.
    """

    name = "clearance"

    def __init__(self, clearance_threshold: float = 0.6, step: float = 0.4, detour: float = 1.2):
        self.clearance_threshold = clearance_threshold
        self.step = step
        self.detour = detour

    def plan(self, task: Task, world: World) -> Task:
        start = task.waypoints[0] if task.waypoints else Pose(0.0, 0.0)
        goal = task.goal
        if world.clearance(goal.x, goal.y) < self.clearance_threshold:
            # Goal is too close to an obstacle; route around the blocker.
            return Task(
                goal=goal,
                waypoints=[start, self._detour_point(start, goal, world), goal],
                max_steps=task.max_steps,
                goal_tolerance=task.goal_tolerance,
                name=task.name,
            )

        waypoints = [start]
        dist = start.distance_to(goal)
        steps = max(1, int(dist / self.step))
        for i in range(1, steps):
            t = i / steps
            x = start.x + (goal.x - start.x) * t
            y = start.y + (goal.y - start.y) * t
            if world.clearance(x, y) >= self.clearance_threshold:
                waypoints.append(Pose(x, y))
        waypoints.append(goal)
        return Task(
            goal=goal,
            waypoints=waypoints,
            max_steps=task.max_steps,
            goal_tolerance=task.goal_tolerance,
            name=task.name,
        )

    def _detour_point(self, start: Pose, goal: Pose, world: World) -> Pose:
        nearest, _ = world.nearest_obstacle(goal.x, goal.y)
        if nearest is None:
            return Pose((start.x + goal.x) / 2, (start.y + goal.y) / 2)
        dx = goal.y - start.y
        dy = start.x - goal.x
        norm = max((dx**2 + dy**2) ** 0.5, 1e-6)
        dx, dy = dx / norm, dy / norm
        return Pose(goal.x + dx * self.detour, goal.y + dy * self.detour)
