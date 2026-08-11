#!/usr/bin/env python3
"""ROS2 node for the canonical ROH hand driver.

One process manages every ENABLED hand in config/roh_hand.yaml (D9: hands are
individually enable-able). Per hand it:

- publishes <joint_state_topic> as stamped sensor_msgs/JointState at
  publish_rate_hz — position = radians (0 = open), effort = currents (mA,
  NaN for per-finger read failures); a whole read failure skips the publish
  rather than fabricating data
- subscribes <joint_command_topic> (sensor_msgs/JointState, position in
  radians) and executes it through RohHandDriver's safety gates

If a hand's telemetry reads keep failing (CAN dead / hand unplugged), the node
exits non-zero so the session fails fast instead of recording fabricated
gaps (D10).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from roh_config import NodeHandConfig, load_hand_configs
from roh_hand_driver import RohHandDriver

# ROS2 imports are required here (this file is the rclpy glue layer).
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

FINGERS = ["thumb", "index", "middle", "ring", "pinky", "thumb_root"]

# Match the bridge/other adaptors: sensor-data QoS.
REALTIME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

# Fail-fast: give up after this many consecutive seconds of read failures.
READ_FAILURE_TIMEOUT_S = 2.0


class RohHandNode(Node):
    def __init__(self, hand: NodeHandConfig, driver: RohHandDriver) -> None:
        super().__init__(f"roh_hand_{hand.driver.side}")
        self.hand = hand
        self.driver = driver
        self._joint_names = [f"{hand.ros2.joint_name_prefix}_{f}" for f in FINGERS]
        self._first_failure_at: float | None = None

        self._pub = self.create_publisher(JointState, hand.ros2.joint_state_topic, REALTIME_QOS)
        self._sub = self.create_subscription(
            JointState, hand.ros2.joint_command_topic, self._on_command, REALTIME_QOS
        )
        self._timer = self.create_timer(1.0 / hand.ros2.publish_rate_hz, self._publish_state)
        self.get_logger().info(
            f"hand={hand.driver.side} pub={hand.ros2.joint_state_topic} "
            f"@{hand.ros2.publish_rate_hz:.0f}Hz sub={hand.ros2.joint_command_topic} "
            f"dry_run={hand.driver.dry_run}"
        )

    def _publish_state(self) -> None:
        positions_rad, currents_ma = self.driver.read_joint_state()
        if positions_rad is None:
            if self._first_failure_at is None:
                self._first_failure_at = time.monotonic()
            elif time.monotonic() - self._first_failure_at > READ_FAILURE_TIMEOUT_S:
                self.get_logger().error(
                    f"hand={self.hand.driver.side}: telemetry reads failing for "
                    f">{READ_FAILURE_TIMEOUT_S}s — failing fast"
                )
                raise SystemExit(2)
            return
        self._first_failure_at = None

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = positions_rad
        msg.effort = currents_ma if currents_ma is not None else [math.nan] * len(FINGERS)
        self._pub.publish(msg)

    def _on_command(self, msg: JointState) -> None:
        if len(msg.position) < len(FINGERS):
            self.get_logger().warning(
                f"ignoring command with {len(msg.position)} positions (need {len(FINGERS)})"
            )
            return
        try:
            sent = self.driver.command_radians(list(msg.position[: len(FINGERS)]))
        except ValueError as exc:
            self.get_logger().warning(f"invalid command rejected: {exc}")
            return
        if not sent and self.driver.contact_latched:
            self.get_logger().warning(
                "command rejected: contact latch active (retreat or open to clear)",
                throttle_duration_sec=1.0,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Canonical ROH hand driver node")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "config" / "roh_hand.yaml"),
        help="path to roh_hand.yaml",
    )
    args = parser.parse_args()

    hands = load_hand_configs(args.config)
    if not hands:
        print("No enabled hands in config — nothing to do.", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    drivers: list[RohHandDriver] = []
    nodes: list[RohHandNode] = []
    try:
        for hand in hands:
            driver = RohHandDriver(hand.driver)
            driver.connect()
            drivers.append(driver)
            nodes.append(RohHandNode(hand, driver))

        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except SystemExit:
        raise
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        for driver in drivers:
            driver.close()
        if rclpy.ok():  # executor may already have shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
