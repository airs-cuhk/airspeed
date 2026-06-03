"""
ROS2 publisher for OpenArm IK output.

Publishes per-arm JointState and target PoseStamped following the
AIRSPEED robot_interface convention.

Topics:
  /arm/left/joint_commands   — sensor_msgs/JointState  (7 arm + 1 gripper)
  /arm/right/joint_commands  — sensor_msgs/JointState  (7 arm + 1 gripper)
  /arm/left/target_pose      — geometry_msgs/PoseStamped
  /arm/right/target_pose     — geometry_msgs/PoseStamped
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class ArmCommandPublisher:
    """Publishes IK output as standard JointState + PoseStamped per arm.

    Gracefully degrades when ROS2 is not installed — publish() becomes a no-op.
    """

    def __init__(
        self,
        left_joint_names: list[str],
        right_joint_names: list[str],
        node_name: str = "ik_command_publisher",
    ) -> None:
        self._left_names = left_joint_names
        self._right_names = right_joint_names
        self._node_name = node_name
        self._ros2_available = False
        self._thread: Optional[threading.Thread] = None

        # Set by _run_ros2
        self._left_joint_pub: object = None
        self._right_joint_pub: object = None
        self._left_pose_pub: object = None
        self._right_pose_pub: object = None
        self._get_clock: object = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        try:
            import rclpy  # noqa: F401
            self._ros2_available = True
        except ImportError:
            logger.warning("ROS2 (rclpy) not available — arm publisher disabled")
            return False

        self._thread = threading.Thread(
            target=self._run_ros2, daemon=True, name="arm-command-publisher"
        )
        self._thread.start()
        logger.info("Arm command publisher started (per-arm JointState + PoseStamped)")
        return True

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stop(self) -> None:
        if self._thread is not None:
            t = self._thread
            self._thread = None
            t.join(timeout=2.0)

    # ------------------------------------------------------------------
    # Publish (called from asyncio loop or solver thread)
    # ------------------------------------------------------------------

    def publish(
        self,
        left_joints: list[float],
        right_joints: list[float],
        left_target_xyz: Optional[list[float]] = None,
        left_target_quat_xyzw: Optional[list[float]] = None,
        right_target_xyz: Optional[list[float]] = None,
        right_target_quat_xyzw: Optional[list[float]] = None,
    ) -> None:
        """Publish joint commands and target poses for both arms.

        Args:
            left_joints:  [j1..j7, gripper] in radians.
            right_joints: [j1..j7, gripper] in radians.
            left_target_xyz: Optional VR target position [x, y, z] in meters.
            left_target_quat_xyzw: Optional VR target orientation quaternion.
            right_target_xyz, right_target_quat_xyzw: Same for right arm.
        """
        if not self._ros2_available or self._thread is None:
            return

        # Shared-state handoff: the solver's async event loop calls publish()
        # and writes the latest data to a tuple. The ROS2 daemon thread reads
        # this tuple on its next spin cycle. No queue — only the latest solve
        # matters. This is safe because we only care about freshest data
        # (BEST_EFFORT QoS means dropped intermediate values are acceptable).
        self._latest = (
            left_joints, right_joints,
            left_target_xyz, left_target_quat_xyzw,
            right_target_xyz, right_target_quat_xyzw,
        )
        self._has_data = True

    # ------------------------------------------------------------------
    # ROS2 daemon thread — runs rclpy.spin_once() in a loop
    # ------------------------------------------------------------------

    def _run_ros2(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from sensor_msgs.msg import JointState
            from geometry_msgs.msg import PoseStamped
        except ImportError as exc:
            logger.error(f"ROS2 import failed: {exc}")
            self._ros2_available = False
            return

        if not rclpy.ok():
            rclpy.init()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        node = Node(self._node_name)
        self._left_joint_pub = node.create_publisher(JointState, '/arm/left/joint_commands', qos)
        self._right_joint_pub = node.create_publisher(JointState, '/arm/right/joint_commands', qos)
        self._left_pose_pub = node.create_publisher(PoseStamped, '/arm/left/target_pose', qos)
        self._right_pose_pub = node.create_publisher(PoseStamped, '/arm/right/target_pose', qos)
        clock = node.get_clock()

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        self._has_data = False
        self._latest = None

        try:
            # Spin loop: check for new data from solver, publish if available.
            # spin_once() at 20 ms = up to 50 Hz publish rate, matching solver cadence.
            while self._thread is not None and rclpy.ok():
                if self._has_data:
                    self._has_data = False
                    (lj, rj, lxyz, lquat, rxyz, rquat) = self._latest
                    now = clock.now().to_msg()

                    # Left joint commands
                    self._publish_joint_state(self._left_joint_pub, now, 'base_link',
                                              self._left_names, lj)

                    # Right joint commands
                    self._publish_joint_state(self._right_joint_pub, now, 'base_link',
                                              self._right_names, rj)

                    # Left target pose
                    if lxyz is not None:
                        self._publish_pose(self._left_pose_pub, now, 'base_link',
                                           lxyz, lquat)

                    # Right target pose
                    if rxyz is not None:
                        self._publish_pose(self._right_pose_pub, now, 'base_link',
                                           rxyz, rquat)

                executor.spin_once(timeout_sec=0.02)
        except Exception as exc:
            logger.error(f"ROS2 publisher error: {exc}")
        finally:
            node.destroy_node()

    @staticmethod
    def _publish_joint_state(pub, stamp, frame_id: str, names: list[str], values: list[float]) -> None:
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.name = names
        msg.position = [float(v) for v in values]
        pub.publish(msg)

    @staticmethod
    def _publish_pose(pub, stamp, frame_id: str, xyz: list[float], quat_xyzw: Optional[list[float]]) -> None:
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        if quat_xyzw:
            msg.pose.orientation.x = float(quat_xyzw[0])
            msg.pose.orientation.y = float(quat_xyzw[1])
            msg.pose.orientation.z = float(quat_xyzw[2])
            msg.pose.orientation.w = float(quat_xyzw[3])
        else:
            msg.pose.orientation.w = 1.0
        pub.publish(msg)
