"""Unit tests for the pure policy / planner / controller / metrics layers.

No physics engine required — these run on the kinematic backend and plain
arithmetic, so they execute anywhere Python runs.
"""

import math

import pytest

from rspl.controller import VelocityController
from rspl.core import Action, Pose, State
from rspl.metrics import EpisodeCollector
from rspl.policy import GoToGoalPolicy, ObstacleAvoidancePolicy
from rspl.world import CircleObstacle, World, sparse_obstacle_field


class TestCore:
    def test_pose_distance(self):
        assert Pose(0, 0).distance_to(Pose(3, 4)) == 5.0

    def test_action_clamp(self):
        a = Action(5.0, 5.0).clamped(1.0, 2.0)
        assert a.linear_velocity == 1.0
        assert a.angular_velocity == 2.0


class TestWorld:
    def test_clearance_inf_without_obstacles(self):
        w = World()
        assert w.clearance(0, 0) == math.inf

    def test_collision_detection(self):
        w = World(robot_radius=0.2)
        w.add(CircleObstacle(1.0, 1.0, 0.5))
        assert w.collides(1.0, 1.0)
        assert w.collides(1.5, 1.0)  # 0.5 away > 0.5+0.2? no -> collides
        assert not w.collides(3.0, 3.0)

    def test_sparse_field_has_three_obstacles(self):
        w = sparse_obstacle_field()
        assert len(w.obstacles) == 3


class TestPolicy:
    def test_go_to_goal_stops_at_goal(self):
        p = GoToGoalPolicy()
        task = type("T", (), {"goal": Pose(0, 0), "goal_tolerance": 0.15})
        state = State(pose=Pose(0.05, 0))
        assert p.select_target(state, task, World()) is None

    def test_go_to_goal_targets_goal(self):
        p = GoToGoalPolicy()
        task = type("T", (), {"goal": Pose(5, 0), "goal_tolerance": 0.15})
        state = State(pose=Pose(0, 0))
        t = p.select_target(state, task, World())
        assert t.x == 5 and t.y == 0

    def test_obstacle_avoidance_pulls_away_from_obstacle(self):
        p = ObstacleAvoidancePolicy(obstacle_gain=2.0, safe_radius=1.0)
        world = World()
        # obstacle slightly off the +x axis, robot between it and the goal
        world.add(CircleObstacle(0.6, 0.3, 0.2))
        task = type("T", (), {"goal": Pose(5, 0), "goal_tolerance": 0.15})
        state = State(pose=Pose(0.0, 0.0))
        target = p.select_target(state, task, world)
        # repulsion pushes the target below the +x axis, away from the obstacle
        assert target.y < 0.0


class TestController:
    def test_stops_when_no_target(self):
        c = VelocityController()
        action = c.compute(State(pose=Pose(0, 0)), None)
        assert action.linear_velocity == 0.0
        assert action.angular_velocity == 0.0

    def test_turns_toward_target(self):
        c = VelocityController()
        # robot facing +x, target straight ahead
        action = c.compute(State(pose=Pose(0, 0, 0.0)), Pose(1.0, 0.0))
        assert action.linear_velocity > 0.0
        assert abs(action.angular_velocity) < 0.5

    def test_slow_when_big_heading_error(self):
        c = VelocityController()
        # target is directly behind -> heading error ~pi -> slow crawl
        action = c.compute(State(pose=Pose(0, 0, 0.0)), Pose(-1.0, 0.0))
        assert action.linear_velocity <= c.min_linear + 1e-9


class TestCollector:
    def test_reports_success_and_counts(self):
        w = World()
        c = EpisodeCollector(task_name="t", simulator="kinematic", world=w, start=Pose(0, 0))
        for i in range(10):
            c.record(State(pose=Pose(i * 0.1, 0), in_contact=(i == 3)))
        r = c.report(True, Pose(1, 0), 100)
        assert r.success
        assert r.collisions == 1
        assert r.contacts == 1
        assert r.path_length == pytest.approx(0.9, abs=1e-6)
