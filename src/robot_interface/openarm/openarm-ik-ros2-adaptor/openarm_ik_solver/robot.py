import jax.numpy as jnp
import jaxlie
from abc import ABC, abstractmethod
from typing import Any

from openarm_ik_solver.config import RobotVisConfig

# Cost is a jaxls.CostBase object (or similar depending on implementation)
Cost = Any


class BaseRobot(ABC):
    """
    Abstract base class for robot models used in IK optimization.

    This class defines the interface required by the IK solver
    to compute kinematics and optimization costs.
    """

    def __init__(self):
        self.urdf_path: str = ""
        self.mesh_path: str | None = None

    @property
    def orientation(self) -> jaxlie.SO3:
        """
        Rotation from the robot's base frame to the canonical ROS2 frame (X-forward, Z-up).
        """
        return jaxlie.SO3.identity()

    @property
    def base_to_ros(self) -> jaxlie.SO3:
        """Rotation from the robot base frame to the canonical ROS2 frame."""
        return self.orientation

    @property
    def ros_to_base(self) -> jaxlie.SO3:
        """Rotation from the canonical ROS2 frame to the robot base frame."""
        return self.orientation.inverse()

    @property
    def model_scale(self) -> float:
        """Get the model scale for visualization."""
        return 1.0

    def get_vis_config(self) -> RobotVisConfig | None:
        """
        Get the visualization configuration for this robot.
        """
        if not self.urdf_path:
            return None
        return RobotVisConfig(
            urdf_path=self.urdf_path,
            mesh_path=self.mesh_path,
            model_scale=self.model_scale,
            initial_rotation_euler=[
                float(x) for x in self.orientation.as_rpy_radians()
            ],
        )

    @abstractmethod
    def _load_default_urdf(self) -> Any:
        """
        Load the default URDF for this robot.

        Returns:
            yourdfpy.URDF: The loaded URDF object.
        """
        pass

    def _load_urdf(self, urdf_string: str | None = None) -> Any:
        """
        Load a URDF from a string or from the default location.

        Args:
            urdf_string: Optional URDF XML string. If provided, it overrides the default URDF.

        Returns:
            yourdfpy.URDF: The loaded URDF object.
        """
        import yourdfpy
        from openarm_ik_solver import ram

        if urdf_string is not None:
            path, mesh = ram.from_string(urdf_string)
            self.urdf_path = str(path)
            self.mesh_path = mesh
            return yourdfpy.URDF.load(self.urdf_path)

        return self._load_default_urdf()

    @property
    @abstractmethod
    def actuated_joint_names(self) -> list[str]:
        """Get the names of the actuated joints."""
        pass

    @property
    @abstractmethod
    def joint_var_cls(self) -> Any:
        """The jaxls.Var class used for joint configurations."""
        pass

    @property
    def supported_frames(self) -> set[str]:
        """Get the set of supported frames for this robot."""
        return {"left", "right", "head"}

    @property
    def default_speed_ratio(self) -> float:
        """Get the default teleop speed ratio for this robot."""
        return 1.0

    @abstractmethod
    def forward_kinematics(self, config: jnp.ndarray) -> dict[str, jaxlie.SE3]:
        """
        Compute the forward kinematics for the given configuration.

        Args:
            config: The robot configuration (joint angles) as a JAX array.

        Returns:
            dict[str, jaxlie.SE3]: A dictionary mapping link names to SE3 poses.
        """
        pass

    @abstractmethod
    def get_default_config(self) -> jnp.ndarray:
        """Get the default configuration for the robot."""
        pass

    @abstractmethod
    def build_costs(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
        target_Head: jaxlie.SE3 | None,
        q_current: jnp.ndarray | None = None,
    ) -> list[Cost]:
        """
        Build a list of costs for the robot-specific formulation.

        Args:
            target_L: Target pose for the left end-effector.
            target_R: Target pose for the right end-effector.
            target_Head: Target pose for the head.
            q_current: Current joint configuration (initial guess).

        Returns:
            list[Cost]: A list of jaxls Cost objects.
        """
        pass
