"""Physics backends (optional).

MuJoCo and PyBullet backends share the same unicycle semantics on top of the
engine's integration so sim-to-sim comparison is meaningful: the *same* policy
and *same* commands are executed, only the integrator differs.
"""

from __future__ import annotations

from rspl.core import Action, Pose, State
from rspl.sim.kinematic import KinematicSimulator
from rspl.world import World


class _EngineSimulator:
    """Common glue: engine-agnostic wrapper mirroring KinematicSimulator's API."""

    def __init__(self, world: World, dt: float = 0.05, start: Pose = Pose(0.0, 0.0), engine=None):
        self.world = world
        self.dt = dt
        self.start = start
        self.engine = engine
        self.pose = start
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.in_contact = False
        self.t = 0.0
        self._collision_ids = set()

    @property
    def state(self) -> State:
        return State(
            pose=self.pose,
            velocity=self.velocity,
            angular_velocity=self.angular_velocity,
            in_contact=self.in_contact,
            t=self.t,
        )

    def reset(self, start: Pose = Pose(0.0, 0.0)) -> None:
        raise NotImplementedError

    def step(self, action: Action) -> State:
        raise NotImplementedError

    def _observe(self) -> State:
        raise NotImplementedError


def _load_mujoco(world: World):
    try:
        import mujoco  # type: ignore
    except ImportError as e:  # pragma: no cover
        raise ImportError("mujoco backend requires the 'mujoco' package") from e

    model_xml = _mjcf_model(world)
    model = mujoco.MjModel.from_xml_string(model_xml)
    data = mujoco.MjData(model)
    return mujoco, model, data


def _mjcf_model(world: World) -> str:
    obstacles = "\n".join(
        f'    <geom name="obs_{o.name}" type="cylinder" pos="{o.x} {o.y} 0.1" size="{o.radius} 0.2" '
        f'solmix="0.9" friction="1"/>'
        for o in world.obstacles
    )
    return f"""<mujoco model="rspl_unicycle">
  <option timestep="0.05"/>
  <worldbody>
    <geom name="ground" type="plane" size="20 20 0.1" friction="0.1"/>
    {obstacles}
    <body name="robot" pos="0 0 0.05">
      <freejoint/>
      <geom name="chassis" type="box" size="0.15 0.1 0.05" mass="0.5" friction="0.05"/>
    </body>
  </worldbody>
</mujoco>"""


class MuJoCoSimulator(_EngineSimulator):
    """MuJoCo backend. Body is a free-joint box; state is read from qpos/qvel."""

    def __init__(self, world: World, dt: float = 0.05, start: Pose = Pose(0.0, 0.0)):
        import mujoco

        self._mj = mujoco
        self._model, self._data = _load_mujoco(world)[1:]
        # resolve geom ids for obstacle geoms (names are "obs_<name>")
        self._obs_geom_ids = set()
        for o in world.obstacles:
            gid = self._model.geom(f"obs_{o.name}").id
            self._obs_geom_ids.add(gid)
        self._data.qpos[0] = start.x
        self._data.qpos[1] = start.y
        self._data.qpos[2] = start.theta
        self._mj.mj_forward(self._model, self._data)
        super().__init__(world, dt, start)

    @property
    def name(self) -> str:
        return "mujoco"

    def reset(self, start: Pose = Pose(0.0, 0.0)) -> None:
        self.start = start
        self._data.qpos[0] = start.x
        self._data.qpos[1] = start.y
        self._data.qpos[2] = start.theta
        self._data.qvel[...] = 0.0
        self._mj.mj_forward(self._model, self._data)
        self.pose = start
        self.t = 0.0

    def step(self, action: Action) -> State:
        a = action.clamped(1.0, 2.0)
        # Convert body-frame linear velocity to world frame for the free joint.
        import math

        self._data.qvel[0] = a.linear_velocity * math.cos(self.pose.theta)
        self._data.qvel[1] = a.linear_velocity * math.sin(self.pose.theta)
        self._data.qvel[2] = 0.0
        self._data.qvel[5] = a.angular_velocity
        self._mj.mj_step(self._model, self._data, nstep=int(self.dt / self._model.opt.timestep))
        x, y = float(self._data.qpos[0]), float(self._data.qpos[1])
        qw, qx, qy, qz = (float(v) for v in self._data.qpos[3:7])
        theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.pose = Pose(x, y, theta)
        self.velocity = a.linear_velocity
        self.angular_velocity = a.angular_velocity
        self.in_contact = self._in_obstacle_contact()
        self.t += self.dt
        return State(
            pose=self.pose,
            velocity=self.velocity,
            angular_velocity=self.angular_velocity,
            in_contact=self.in_contact,
            t=self.t,
        )

    def _in_obstacle_contact(self) -> bool:
        # MuJoCo reports ground contact too; only obstacle contacts matter.
        obs_ids = set(self._obs_geom_ids)
        for i in range(self._data.ncon):
            c = self._data.contact[i]
            if c.geom1 in obs_ids or c.geom2 in obs_ids:
                return True
        return False


def _load_pybullet(world: World):
    try:
        import pybullet as p  # type: ignore
    except ImportError as e:  # pragma: no cover
        raise ImportError("pybullet backend requires the 'pybullet' package") from e
    physics_client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81, physicsClientId=physics_client)
    p.setTimeStep(0.05, physicsClientId=physics_client)
    plane_col = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=physics_client)
    plane_vis = p.createVisualShape(
        p.GEOM_PLANE, rgbaColor=[0.4, 0.4, 0.4, 1.0], physicsClientId=physics_client
    )
    plane = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=plane_col,
        baseVisualShapeIndex=plane_vis,
        basePosition=[0, 0, 0],
        physicsClientId=physics_client,
    )
    return p, physics_client, plane


class PyBulletSimulator(_EngineSimulator):
    """PyBullet backend (DIRECT headless mode)."""

    def __init__(self, world: World, dt: float = 0.05, start: Pose = Pose(0.0, 0.0)):
        self._p, self._client, self._plane = _load_pybullet(world)
        body_ids = []
        for o in world.obstacles:
            cid = self._p.createCollisionShape(
                self._p.GEOM_CYLINDER,
                radius=o.radius,
                height=1.0,
                physicsClientId=self._client,
            )
            vid = self._p.createVisualShape(
                self._p.GEOM_CYLINDER,
                radius=o.radius,
                length=1.0,
                physicsClientId=self._client,
            )
            bid = self._p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=cid,
                baseVisualShapeIndex=vid,
                basePosition=[o.x, o.y, 0.5],
                physicsClientId=self._client,
            )
            body_ids.append(bid)
        self._obstacle_bodies = body_ids
        self._robot_body = self._p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=self._p.createCollisionShape(
                self._p.GEOM_BOX, halfExtents=[0.25, 0.15, 0.05], physicsClientId=self._client
            ),
            baseVisualShapeIndex=self._p.createVisualShape(
                self._p.GEOM_BOX, halfExtents=[0.25, 0.15, 0.05], physicsClientId=self._client
            ),
            basePosition=[start.x, start.y, 0.05],
            physicsClientId=self._client,
        )
        super().__init__(world, dt, start)

    @property
    def name(self) -> str:
        return "pybullet"

    def reset(self, start: Pose = Pose(0.0, 0.0)) -> None:
        self._p.resetBasePositionAndOrientation(
            self._robot_body, [start.x, start.y, 0.05], [0, 0, 1, 0], physicsClientId=self._client
        )
        self._p.resetBaseVelocity(
            self._robot_body, [0, 0, 0], [0, 0, 0], physicsClientId=self._client
        )
        self.start = start
        self.pose = start
        self.t = 0.0

    def step(self, action: Action) -> State:
        a = action.clamped(1.0, 2.0)
        # Directly set velocity in world frame approximated from heading.
        import math

        vx = a.linear_velocity * math.cos(self.pose.theta)
        vy = a.linear_velocity * math.sin(self.pose.theta)
        self._p.resetBaseVelocity(
            self._robot_body, [vx, vy, 0], [0, 0, a.angular_velocity], physicsClientId=self._client
        )
        self._p.stepSimulation(physicsClientId=self._client)
        pos, orn = self._p.getBasePositionAndOrientation(
            self._robot_body, physicsClientId=self._client
        )
        theta = math.atan2(2 * (orn[3] * orn[2]), 1 - 2 * (orn[2] ** 2))  # yaw from quaternion
        self.pose = Pose(pos[0], pos[1], theta)
        self.velocity = a.linear_velocity
        self.angular_velocity = a.angular_velocity
        self.in_contact = False
        for bid in self._obstacle_bodies:
            contacts = self._p.getContactPoints(self._robot_body, bid, physicsClientId=self._client)
            if contacts:
                self.in_contact = True
                break
        self.t += self.dt
        return State(
            pose=self.pose,
            velocity=self.velocity,
            angular_velocity=self.angular_velocity,
            in_contact=self.in_contact,
            t=self.t,
        )


def make_simulator(name: str, world: World, start: Pose = Pose(0.0, 0.0)):
    """Factory returning a simulator instance by name."""
    if name == "kinematic":
        return KinematicSimulator(world, start=start)
    if name == "mujoco":
        return MuJoCoSimulator(world, start=start)
    if name == "pybullet":
        return PyBulletSimulator(world, start=start)
    raise ValueError(f"unknown simulator: {name}")
