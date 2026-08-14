"""Example: obstacle-navigation experiment on all three simulators.

Run:
    python examples/run_obstacle_navigation.py [--sim kinematic|mujoco|pybullet|all]
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rspl.core import Pose  # noqa: E402
from rspl.policy import ObstacleAvoidancePolicy  # noqa: E402
from rspl.validation import run_experiment, sim2sim  # noqa: E402
from rspl.world import sparse_obstacle_field  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", default="all", help="kinematic | mujoco | pybullet | all")
    parser.add_argument("--seeds", default="0,1,2,3,4")
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]
    world = sparse_obstacle_field()
    policy = ObstacleAvoidancePolicy()

    sims = {
        "kinematic": ["kinematic"],
        "mujoco": ["mujoco"],
        "pybullet": ["pybullet"],
        "all": ["kinematic", "mujoco", "pybullet"],
    }.get(args.sim, ["kinematic"])

    result = sim2sim(sims, world, policy, Pose(0, 0), Pose(5.0, 0.5), seeds=seeds)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
