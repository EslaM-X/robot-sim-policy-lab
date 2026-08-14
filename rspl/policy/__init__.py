"""Policies: task-conditioned behaviour that turns a goal into waypoints.

A policy is a callable that, given the current state, the task, and the world,
produces a desired target pose (or None to stop). The controller then chases
that target. Policies are pure and deterministic — no physics imports.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from rspl.core import Pose, State, Task
from rspl.world import World


class Policy(ABC):
    """Base class for task-conditioned policies."""

    name: str = "policy"

    @abstractmethod
    def select_target(self, state: State, task: Task, world: World) -> Pose | None:
        """Return the next target the robot should steer toward, or None to stop."""


class GoToGoalPolicy(Policy):
    """The simplest policy: head straight to the goal.

    Pure line-of-sight navigation. Used as the baseline every other policy is
    compared against.
    """

    name = "go-to-goal"

    def select_target(self, state: State, task: Task, world: World) -> Pose | None:
        if state.pose.distance_to(task.goal) <= task.goal_tolerance:
            return None
        return task.goal


class WaypointPolicy(Policy):
    """Follow a fixed list of waypoints in order, then stop at the goal.

    The planner is responsible for ordering waypoints so that each segment is
    clear; this policy only chases the current one.
    """

    name = "waypoint"

    def __init__(self, waypoints):
        self._waypoints = list(waypoints)

    def select_target(self, state: State, task: Task, world: World) -> Pose | None:
        if not self._waypoints:
            return None
        current = self._waypoints[0]
        if state.pose.distance_to(current) <= task.goal_tolerance:
            self._waypoints.pop(0)
            return None if not self._waypoints else self._waypoints[0]
        return current


class ObstacleAvoidancePolicy(Policy):
    """Blend goal-seeking with an artificial potential field.

    Attractive force pulls toward the goal; repulsive forces push away from
    nearby obstacles. The resulting target is a blend — deterministic and
    parameterised, which makes it reproducible and tunable.
    """

    name = "potential-field"

    def __init__(
        self, obstacle_gain: float = 0.9, safe_radius: float = 1.2, max_force: float = 1.5
    ):
        self.obstacle_gain = obstacle_gain
        self.safe_radius = safe_radius
        self.max_force = max_force

    def select_target(self, state: State, task: Task, world: World) -> Pose | None:
        pose = state.pose
        if pose.distance_to(task.goal) <= task.goal_tolerance:
            return None

        dx, dy = task.goal.x - pose.x, task.goal.y - pose.y
        goal_dist = max((dx**2 + dy**2) ** 0.5, 1e-6)
        # Attraction grows with distance up to a cap so it dominates far from
        # obstacles but does not overshoot when already close.
        pull = min(goal_dist, 1.5)
        attractive = (dx / goal_dist * pull, dy / goal_dist * pull)

        repulsive_x, repulsive_y = 0.0, 0.0
        for o in world.obstacles:
            ox, oy = pose.x - o.x, pose.y - o.y
            odist = max((ox**2 + oy**2) ** 0.5, 1e-6)
            if odist <= self.safe_radius:
                magnitude = self.obstacle_gain * (1.0 / max(odist, 0.1))
                magnitude = min(magnitude, self.max_force)
                repulsive_x += magnitude * (ox / odist)
                repulsive_y += magnitude * (oy / odist)

        tx = pose.x + attractive[0] + repulsive_x
        ty = pose.y + attractive[1] + repulsive_y
        return Pose(tx, ty)
