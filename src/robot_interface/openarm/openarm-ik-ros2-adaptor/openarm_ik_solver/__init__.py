"""
OpenArm IK Solver — Isolated Tier 1 inverse kinematics for the OpenArm bimanual robot.

Provides SE3 target poses as input, joint angles as output.
No VR awareness, no controller logic — just the solver.

Example:
    ```python
    import jaxlie
    import jax.numpy as jnp
    from openarm_ik_solver import OpenArmRobot, PyrokiSolver

    robot = OpenArmRobot()
    solver = PyrokiSolver(robot)
    q = robot.get_default_config()  # shape: (18,)

    target_right = jaxlie.SE3.from_rotation_and_translation(
        jaxlie.SO3.identity(),
        jnp.array([0.3, -0.2, 0.5])
    )
    new_q = solver.solve(None, target_right, None, q)
    ```
"""

from openarm_ik_solver.robot import BaseRobot, Cost
from openarm_ik_solver.solver import PyrokiSolver
from openarm_ik_solver.openarm_robot import OpenArmRobot
from openarm_ik_solver.config import RobotVisConfig

__all__ = [
    "BaseRobot",
    "Cost",
    "PyrokiSolver",
    "OpenArmRobot",
    "RobotVisConfig",
]
