"""Simulators: the physics-free kinematic backend.

The kinematic backend integrates a unicycle model exactly. It is fully
deterministic (pure arithmetic, no engine, no RNG) and is used for unit tests,
fast sweeps, and as the reference for sim-to-sim comparison.
"""

from __future__ import annotations

import math

from rspl.core import Action, Pose, State
from rspl.world import World


class KinematicSimulator:
    """Exact unicycle integrator with contact detection against the world."""

    def __init__(self, world: World, dt: float = 0.05, start: Pose = Pose(0.0, 0.0)):
        self.world = world
        self.dt = dt
        self.pose = start
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.in_contact = False
        self.t = 0.0

    def reset(self, start: Pose = Pose(0.0, 0.0)) -> None:
        self.pose = start
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.in_contact = False
        self.t = 0.0

    def step(self, action: Action) -> State:
        a = action.clamped(1.0, 2.0)
        self.pose = Pose(
            self.pose.x + a.linear_velocity * math.cos(self.pose.theta) * self.dt,
            self.pose.y + a.linear_velocity * math.sin(self.pose.theta) * self.dt,
            self.pose.theta + a.angular_velocity * self.dt,
        )
        self.velocity = a.linear_velocity
        self.angular_velocity = a.angular_velocity
        self.in_contact = self.world.collides(self.pose.x, self.pose.y)
        self.t += self.dt
        return State(
            pose=self.pose,
            velocity=self.velocity,
            angular_velocity=self.angular_velocity,
            in_contact=self.in_contact,
            t=self.t,
        )

    @property
    def name(self) -> str:
        return "kinematic"

    @property
    def state(self) -> State:
        return State(
            pose=self.pose,
            velocity=self.velocity,
            angular_velocity=self.angular_velocity,
            in_contact=self.in_contact,
            t=self.t,
        )
