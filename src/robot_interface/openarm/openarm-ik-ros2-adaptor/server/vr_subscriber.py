"""ROS2 VR topic subscriber — subscribes to dry VR bridge topics in a daemon thread."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from server.config_loader import VRConfig

logger = logging.getLogger(__name__)


@dataclass
class VRPose:
    """A single VR controller pose with timestamp."""
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation_wxyz: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    timestamp_s: float = 0.0


@dataclass
class VRButtonState:
    """Button state from a VR controller with timestamp."""
    buttons: list[float] = field(default_factory=list)
    timestamp_s: float = 0.0


@dataclass
class VRDataStore:
    """Thread-safe store for the latest VR poses and button states.

    Written from ROS2 callback thread, read from solver loop (asyncio).
    Last write wins — older data is silently discarded.
    """

    stale_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self.left_pose: VRPose = VRPose()
        self.right_pose: VRPose = VRPose()
        self.head_pose: VRPose = VRPose()
        self.left_buttons: VRButtonState = VRButtonState()
        self.right_buttons: VRButtonState = VRButtonState()
        self.raw_data: dict[str, Any] = field(default_factory=dict)
        self._raw_timestamp_s: float = 0.0
        self._connected: bool = False
        self._last_message_s: float = 0.0

    def update_pose(self, side: str, position: list[float], orientation_wxyz: list[float]) -> None:
        now = time.monotonic()
        with self._lock:
            pose = VRPose(position=position, orientation_wxyz=orientation_wxyz, timestamp_s=now)
            if side == "left":
                self.left_pose = pose
            elif side == "right":
                self.right_pose = pose
            elif side == "head":
                self.head_pose = pose
            self._connected = True
            self._last_message_s = now

    def update_buttons(self, side: str, buttons: list[float]) -> None:
        now = time.monotonic()
        with self._lock:
            state = VRButtonState(buttons=buttons, timestamp_s=now)
            if side == "left":
                self.left_buttons = state
            elif side == "right":
                self.right_buttons = state
            self._connected = True
            self._last_message_s = now

    def update_raw(self, data: dict[str, Any]) -> None:
        now = time.monotonic()
        with self._lock:
            self.raw_data = data
            self._raw_timestamp_s = now
            self._connected = True
            self._last_message_s = now

    def get_latest_poses(self) -> dict[str, VRPose]:
        """Return left and right poses if not stale, else empty dict."""
        now = time.monotonic()
        with self._lock:
            result: dict[str, VRPose] = {}
            for side, pose in [("left", self.left_pose), ("right", self.right_pose)]:
                if (now - pose.timestamp_s) < self.stale_timeout_s:
                    result[side] = pose
            return result

    def get_head_pose(self) -> VRPose | None:
        now = time.monotonic()
        with self._lock:
            if (now - self.head_pose.timestamp_s) < self.stale_timeout_s:
                return self.head_pose
            return None

    def get_button_states(self) -> dict[str, VRButtonState]:
        now = time.monotonic()
        with self._lock:
            result: dict[str, VRButtonState] = {}
            for side, state in [("left", self.left_buttons), ("right", self.right_buttons)]:
                if (now - state.timestamp_s) < self.stale_timeout_s:
                    result[side] = state
            return result

    def is_connected(self) -> bool:
        now = time.monotonic()
        with self._lock:
            return self._connected and (now - self._last_message_s) < self.stale_timeout_s

    def get_status(self) -> dict[str, Any]:
        """Return a status dict for the snapshot."""
        now = time.monotonic()
        with self._lock:
            poses = {}
            for side in ("left", "right", "head"):
                pose = getattr(self, f"{side}_pose")
                stale = (now - pose.timestamp_s) >= self.stale_timeout_s
                poses[side] = {
                    "stale": stale,
                    "age_ms": round((now - pose.timestamp_s) * 1000, 1),
                    "position": pose.position if not stale else None,
                    "orientation_wxyz": pose.orientation_wxyz if not stale else None,
                }

            buttons = {}
            for side in ("left", "right"):
                btn = getattr(self, f"{side}_buttons")
                btn_stale = (now - btn.timestamp_s) >= self.stale_timeout_s
                buttons[side] = {
                    "stale": btn_stale,
                    "values": btn.buttons if not btn_stale else [],
                    "age_ms": round((now - btn.timestamp_s) * 1000, 1),
                }

            return {
                "connected": self._connected and (now - self._last_message_s) < self.stale_timeout_s,
                "last_message_age_ms": round((now - self._last_message_s) * 1000, 1),
                "poses": poses,
                "buttons": buttons,
            }


class VRSubscriber:
    """Manages a ROS2 subscriber node in a daemon thread.

    Gracefully handles ROS2 not being installed — is_connected() returns False
    and the system falls back to keyboard/idle control.
    """

    def __init__(self, config: VRConfig) -> None:
        self.config = config
        self.data_store = VRDataStore(stale_timeout_s=config.calibration.stale_timeout_s)
        self._thread: threading.Thread | None = None
        self._ros2_available = False

    def start(self) -> bool:
        """Start the subscriber in a daemon thread. Returns True if ROS2 is available."""
        try:
            import rclpy  # noqa: F401 — test importability
            self._ros2_available = True
        except ImportError:
            logger.warning("ROS2 (rclpy) not available — VR data stream disabled, keyboard fallback active")
            self._ros2_available = False
            return False

        self._thread = threading.Thread(target=self._run_ros2, daemon=True, name="vr-ros2-spin")
        self._thread.start()
        logger.info("VR subscriber started (ROS2 daemon thread)")
        return True

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def ros2_installed(self) -> bool:
        """Whether rclpy is importable on this system."""
        return self._ros2_available

    def stop(self) -> None:
        """Stop the ROS2 subscriber thread and clear connection state."""
        if self._ros2_available and self._thread is not None:
            thread = self._thread
            self._thread = None
            # Let the daemon thread's finally block handle rclpy.shutdown().
            # Since the thread is a daemon, it will die when the process exits.
            # For graceful stop, the executor.spin() will raise when the node
            # is destroyed from the thread's finally block.
            thread.join(timeout=2.0)
        with self.data_store._lock:
            self.data_store._connected = False

    def restart(self) -> bool:
        """Stop and re-start the subscriber. Returns True if ROS2 is available.

        Idempotent — safe to call even if not currently running.
        """
        self.stop()
        return self.start()

    def _run_ros2(self) -> None:
        """Run rclpy spin in a daemon thread. Must not be called from asyncio loop."""
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Float32MultiArray, String
        except ImportError as exc:
            logger.error(f"ROS2 import failed in subscriber thread: {exc}")
            self._ros2_available = False
            return

        if not rclpy.ok():
            rclpy.init()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config.ros2.qos.history_depth,
        )

        topics = self.config.ros2.topics
        node_name = self.config.ros2.node_name
        node = Node(node_name)

        def _parse_pose(msg: PoseStamped, side: str) -> None:
            p = msg.pose.position
            o = msg.pose.orientation
            self.data_store.update_pose(
                side=side,
                position=[p.x, p.y, p.z],
                orientation_wxyz=[o.w, o.x, o.y, o.z],
            )

        def _parse_buttons(msg: Float32MultiArray, side: str) -> None:
            self.data_store.update_buttons(side=side, buttons=list(msg.data))

        def _parse_raw(msg: String) -> None:
            import json
            try:
                self.data_store.update_raw(json.loads(msg.data))
            except json.JSONDecodeError:
                pass

        node.create_subscription(PoseStamped, topics.left_pose, lambda m: _parse_pose(m, "left"), qos_profile)
        node.create_subscription(PoseStamped, topics.right_pose, lambda m: _parse_pose(m, "right"), qos_profile)
        node.create_subscription(PoseStamped, topics.head_pose, lambda m: _parse_pose(m, "head"), qos_profile)
        node.create_subscription(Float32MultiArray, topics.left_buttons, lambda m: _parse_buttons(m, "left"), qos_profile)
        node.create_subscription(Float32MultiArray, topics.right_buttons, lambda m: _parse_buttons(m, "right"), qos_profile)
        node.create_subscription(String, topics.raw_data, _parse_raw, qos_profile)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except Exception as exc:
            logger.error(f"ROS2 executor error: {exc}")
        finally:
            node.destroy_node()
            # Do NOT call rclpy.shutdown() here — the main process owns
            # the context and may have other ROS2 threads running.
