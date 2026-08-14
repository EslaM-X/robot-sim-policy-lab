"""Controllers: turn a policy target into low-level velocity commands.

A classic two-stage approach: a heading error drives angular velocity, and a
distance term drives linear velocity. Parameters are bounded so commands stay
within the robot's kinematic limits.
"""

from __future__ import annotations

from typing import Optional

from rspl.core import Action, Pose, State


class VelocityController:
    """Proportional controller on heading and distance errors."""

    def __init__(
        self,
        max_linear: float = 1.0,
        max_angular: float = 2.0,
        k_linear: float = 1.5,
        k_angular: float = 2.5,
        min_linear: float = 0.05,
        arrival_threshold: float = 0.12,
    ):
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.k_linear = k_linear
        self.k_angular = k_angular
        self.min_linear = min_linear
        self.arrival_threshold = arrival_threshold

    def compute(self, state: State, target: Optional[Pose]) -> Action:
        if target is None:
            return Action(0.0, 0.0)

        dx = target.x - state.pose.x
        dy = target.y - state.pose.y
        dist = (dx**2 + dy**2) ** 0.5

        if dist <= self.arrival_threshold:
            return Action(0.0, 0.0)

        heading_err = state.pose.heading_to(target) - state.pose.theta
        # wrap to [-pi, pi]
        heading_err = (heading_err + 3.141592653589793) % (2 * 3.141592653589793) - 3.141592653589793

        linear = self.k_linear * dist
        angular = self.k_angular * heading_err
        # slow down when heading is way off — no forward drift while turning
        if abs(heading_err) > 0.9:
            linear = self.min_linear
        return Action(linear, angular).clamped(self.max_linear, self.max_angular)
