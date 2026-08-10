"""In-process /arm/{left,right}/joint_state publisher for arm_controller.

The controller reads fresh motor states every control cycle for gravity
compensation, so joint state is published from the same process with stamps
taken at observation read time. Single SocketCAN owner on can0/can1 — a
second reader's unfiltered socket would saturate its kernel RX buffer and
publish stale positions with fresh timestamps (the cmd→state lag artifact
this design eliminated).

velocity[] and effort[] are filled from the same observation (the MIT status
frame carries both) for live debugging; the collector's session YAML still
records only position — no recording-schema change.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState

# Topic contract: BEST_EFFORT / VOLATILE / KEEP_LAST 1 (unchanged from the
# retired standalone publisher — subscribers see no difference).
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ArmJointStatePublisher:
    """Publishes JointState from the arm_controller's own observation reads."""

    def __init__(self, cfg: dict) -> None:
        self._left_names = cfg.get("left_joints", [f"left_joint_{i}" for i in range(1, 9)])
        self._right_names = cfg.get("right_joints", [f"right_joint_{i}" for i in range(1, 9)])
        # Dexterous-hand sessions: publish 7 joints only — never fabricate a
        # gripper value for a motor that does not exist.
        self._left_gripper_enabled = cfg.get("left_gripper_enabled", True)
        self._right_gripper_enabled = cfg.get("right_gripper_enabled", True)
        if not self._left_gripper_enabled:
            self._left_names = [n for n in self._left_names if "gripper" not in n]
        if not self._right_gripper_enabled:
            self._right_names = [n for n in self._right_names if "gripper" not in n]
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._left_pub = None
        self._right_pub = None

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        # Same node name as the legacy subprocess — the ROS graph looks
        # identical to subscribers either way.
        self._node = Node("arm_state_publisher")
        self._left_pub = self._node.create_publisher(JointState, "/arm/left/joint_state", _QOS)
        self._right_pub = self._node.create_publisher(JointState, "/arm/right/joint_state", _QOS)
        # Dedicated executor: rclpy.spin() would share the global executor,
        # which cannot be spun from two threads (ValueError: generator
        # already executing) if this process ever spins anything else.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True, name="arm-state-publisher-spin"
        )
        self._spin_thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def stamp_now(self):
        """Stamp for an observation read that has just completed."""
        return self._node.get_clock().now().to_msg()

    @staticmethod
    def _fill(msg: JointState, names: list[str], side: str, obs: dict, stamp,
              gripper_enabled: bool = True) -> None:
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.name = names
        joints = range(1, 8)
        msg.position = [float(obs.get(f"{side}_joint_{i}.pos", 0.0)) for i in joints]
        msg.velocity = [float(obs.get(f"{side}_joint_{i}.vel", 0.0)) for i in joints]
        msg.effort = [float(obs.get(f"{side}_joint_{i}.torque", 0.0)) for i in joints]
        # Velocity/effort are filled for live consumers only — the session YAML
        # records position only.
        if gripper_enabled:
            msg.position.append(float(obs.get(f"{side}_gripper.pos", 0.0)))
            msg.velocity.append(float(obs.get(f"{side}_gripper.vel", 0.0)))
            msg.effort.append(float(obs.get(f"{side}_gripper.torque", 0.0)))

    def publish(self, obs: dict, stamp) -> None:
        """Publish both arms' JointState from a follower.get_observation() dict."""
        lm = JointState()
        self._fill(lm, self._left_names, "left", obs, stamp, self._left_gripper_enabled)
        self._left_pub.publish(lm)
        rm = JointState()
        self._fill(rm, self._right_names, "right", obs, stamp, self._right_gripper_enabled)
        self._right_pub.publish(rm)

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()
