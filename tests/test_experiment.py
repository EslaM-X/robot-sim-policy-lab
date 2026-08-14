"""End-to-end tests on the kinematic backend.

These run the full stack — policy -> controller -> simulator -> metrics ->
validation — and assert the numbers are plausible and deterministic. They are
the reproducibility contract of the lab.
"""

import pytest

from rspl.core import Pose
from rspl.policy import ObstacleAvoidancePolicy
from rspl.validation import run_episode, run_experiment, sim2sim
from rspl.world import sparse_obstacle_field


@pytest.fixture
def world():
    return sparse_obstacle_field()


def test_episode_runs_to_goal(world):
    report = run_episode(
        "kinematic", world, ObstacleAvoidancePolicy(), Pose(0, 0), Pose(5.0, 0.5), seed=0
    )
    assert report.steps > 0
    assert report.success is True
    assert report.collisions >= 0


def test_experiment_aggregates(world):
    summary = run_experiment(
        "kinematic",
        world,
        ObstacleAvoidancePolicy(),
        Pose(0, 0),
        Pose(5.0, 0.5),
        seeds=[0, 1, 2],
    )
    assert summary.episodes == 3
    assert 0.0 <= summary.success_rate <= 1.0
    assert summary.mean_path_length > 0.0


def test_experiment_is_deterministic(world):
    a = run_experiment(
        "kinematic", world, ObstacleAvoidancePolicy(), Pose(0, 0), Pose(5.0, 0.5), seeds=[0]
    )
    b = run_experiment(
        "kinematic", world, ObstacleAvoidancePolicy(), Pose(0, 0), Pose(5.0, 0.5), seeds=[0]
    )
    assert a.mean_path_length == b.mean_path_length
    assert a.success_rate == b.success_rate


def test_sim2sim_single_backend_reports_zero_deviation(world):
    result = sim2sim(["kinematic"], world, ObstacleAvoidancePolicy(), Pose(0, 0), Pose(5.0, 0.5))
    assert result["max_path_length_deviation_m"] == 0.0
    assert len(result["simulators"]) == 1
