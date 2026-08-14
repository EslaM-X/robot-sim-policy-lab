"""Optional physics-engine integration tests.

These tests run the real MuJoCo and PyBullet backends. They are skipped
automatically when the corresponding packages are not installed, so CI on a
bare Python runner still passes while a full run exercises real physics.
"""

import pytest

from rspl.core import Pose
from rspl.policy import ObstacleAvoidancePolicy
from rspl.validation import run_episode
from rspl.world import sparse_obstacle_field

mujoco = pytest.importorskip("mujoco")
pybullet = pytest.importorskip("pybullet")


@pytest.fixture
def world():
    return sparse_obstacle_field()


def test_mujoco_episode_reaches_goal(world):
    report = run_episode(
        "mujoco", world, ObstacleAvoidancePolicy(), Pose(0, 0), Pose(5.0, 0.5), seed=0
    )
    assert report.success is True
    assert report.collisions == 0


def test_pybullet_episode_reaches_goal(world):
    report = run_episode(
        "pybullet", world, ObstacleAvoidancePolicy(), Pose(0, 0), Pose(5.0, 0.5), seed=0
    )
    assert report.success is True
    assert report.collisions == 0
