#!/usr/bin/env python3
"""VR → ROH hand controller node (canonical).

Consumes sensor_msgs/Joy from the VR bridge (/vr/<side>_buttons, axes) and
publishes stamped sensor_msgs/JointState radian commands on
/roh/<side>/joint_command for every enabled hand with a `vr:` block in
config/roh_hand.yaml. The gesture state machine lives in roh_gesture_fsm.py
(pure logic); this file is only the rclpy glue.

Button staleness: a Joy message older than VR_STALE_TIMEOUT_S is treated as
absent — with no fresh buttons the trigger reads 0.0, which commands the
hand open through the normal interpolation path (same as the 115 behavior
with its 1 s stale gate).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from roh_config import load_hand_configs
from roh_gesture_fsm import RohGestureFSM

# Gesture pose table comes from the vendored wrapper (single source of truth).
_TP = Path(__file__).resolve().parent / "third_party"
sys.path.insert(0, str(_TP / "ohand_serial_sdk_python-main" / "src"))
sys.path.insert(0, str(_TP / "wrappers"))
from roh_gesture_controller import GESTURE_POSITIONS  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState, Joy  # noqa: E402

REALTIME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

FINGERS = ["thumb", "index", "middle", "ring", "pinky", "thumb_root"]
VR_STALE_TIMEOUT_S = 1.0


class VrHandControllerNode(Node):
    def __init__(self, hands) -> None:
        super().__init__("roh_vr_hand_controller")
        self._latest_axes: dict[str, tuple[list[float], float]] = {}
        self._fsms: dict[str, RohGestureFSM] = {}
        self._vr_cfgs: dict[str, object] = {}
        self._pubs: dict[str, object] = {}
        self._joint_names: dict[str, list[str]] = {}
        self._periods: list[float] = []

        topics: set[str] = set()
        for hand in hands:
            if hand.vr is None or not hand.vr.enabled:
                continue
            side = hand.driver.side
            self._fsms[side] = RohGestureFSM(hand.vr.fsm, dict(GESTURE_POSITIONS))
            self._vr_cfgs[side] = hand.vr
            self._pubs[side] = self.create_publisher(
                JointState, hand.ros2.joint_command_topic, REALTIME_QOS
            )
            self._joint_names[side] = [
                f"{hand.ros2.joint_name_prefix}_{f}" for f in FINGERS
            ]
            topics.update(hand.vr.buttons_topics.values())
            self._periods.append(1.0 / hand.vr.control_rate_hz)

        if not self._fsms:
            raise RuntimeError("no enabled hands with a vr: block in config")

        for topic in sorted(topics):
            self.create_subscription(
                Joy, topic, self._make_joy_cb(topic), REALTIME_QOS
            )

        # All configured hands share the VR topics; tick at the fastest rate.
        self._timer = self.create_timer(min(self._periods), self._tick)
        self.get_logger().info(
            f"VR->hand controller: sides={sorted(self._fsms)} topics={sorted(topics)}"
        )

    def _make_joy_cb(self, topic: str):
        def _cb(msg: Joy) -> None:
            self._latest_axes[topic] = (list(msg.axes), time.monotonic())

        return _cb

    def _axes(self, topic: str) -> list[float]:
        entry = self._latest_axes.get(topic)
        if entry is None:
            return []
        axes, stamp = entry
        if time.monotonic() - stamp > VR_STALE_TIMEOUT_S:
            return []
        return axes

    def _tick(self) -> None:
        for side, fsm in self._fsms.items():
            cfg = fsm.config
            # Resolve own/other button topics from the FSM side config.
            own_topic_side = side
            other_side = "right" if side == "left" else "left"
            vr_cfg = self._vr_cfgs[side]
            own = self._axes(vr_cfg.buttons_topics[own_topic_side])
            other = self._axes(vr_cfg.buttons_topics[other_side])
            command, event = fsm.tick(own, other)
            if event:
                self.get_logger().info(event)
            if command is None:
                continue
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._joint_names[side]
            msg.position = command
            self._pubs[side].publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Canonical VR→ROH hand controller")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "config" / "roh_hand.yaml"),
        help="path to roh_hand.yaml",
    )
    args = parser.parse_args()

    hands = load_hand_configs(args.config)
    vr_hands = [h for h in hands if h.vr is not None and h.vr.enabled]
    if not vr_hands:
        print("No enabled hands with a vr: block — nothing to do.", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = None
    try:
        node = VrHandControllerNode(vr_hands)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
