"""Two-wheel Nova adapter using the installed Isaac DifferentialController."""
import numpy as np
from research.control import planar_pose, rotation_wxyz
from research.nova_source import slew


class NovaCarter:
    def __init__(self, robot, cfg, constants, variant="direct", controller=None):
        if cfg.get("type") != "nova_carter" or variant not in ("direct", "official_slew"):
            raise ValueError("Nova adapter requires explicit embodiment and command variant")
        self.robot, self.cfg, self.constants, self.variant = robot, cfg, constants, variant
        if controller is None:
            from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
            controller = DifferentialController(name="research_nova_diff", wheel_radius=cfg["wheel_radius"], wheel_base=cfg["wheel_base"])
        self.controller = controller
        self.target = np.zeros(2); self.applied = np.zeros(2); self.wheel_targets = [0., 0.]

    def _push(self):
        action = self.controller.forward(command=self.applied.copy())
        self.robot.apply_wheel_actions(action)
        self.wheel_targets = list(map(float, action.joint_velocities))
        return self.wheel_targets

    def apply_target(self, v, w):
        if not np.isfinite([v, w]).all():
            raise ValueError("Nonfinite Nova target")
        self.target = np.array([v, w], dtype=float)
        if self.variant == "direct":
            self.applied = self.target.copy()
            self._push()
        return list(self.wheel_targets)

    def before_step(self, dt):
        if self.variant == "official_slew":
            c = self.constants
            self.applied = np.array([slew(self.applied[i], self.target[i], c[f"_max_accel_{axis}"],
                c[f"_max_decel_{axis}"], dt, c[f"_{axis}_deadband"]) for i, axis in enumerate(["lin", "ang"])])
        self._push()

    def wheel_state(self):
        return self.robot.get_joint_velocities()[[self.robot.get_dof_index(n) for n in self.cfg["wheel_joints"]]].tolist()

    def state(self):
        p, q = self.robot.get_world_pose()
        v, w = self.robot.get_linear_velocity(), self.robot.get_angular_velocity()
        r = rotation_wxyz(q)
        return {"position": p.tolist(), "quaternion_wxyz": q.tolist(), "pose": planar_pose(p, q),
                "linear_velocity_world": v.tolist(), "angular_velocity_world": w.tolist(),
                "linear_velocity_body_flu": (r.T@v).tolist(), "angular_velocity_body_flu": (r.T@w).tolist(),
                "wheel_velocities": self.wheel_state()}

    def stop(self):
        self.target = np.zeros(2); self.applied = np.zeros(2)
        return self._push()
