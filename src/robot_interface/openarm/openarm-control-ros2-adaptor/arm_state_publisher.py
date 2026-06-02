#!/usr/bin/env python3
"""
Arm State ROS2 Publisher — CAN bus arm joints + grippers → JointState topics.

Publishes per-arm JointState matching the AIRSPEED robot_interface convention.
Read-only observer — no motor commands, no calibration, no torque control.

Does NOT call follower.connect() (which would cycle torque via configure()).
Instead connects the CAN buses directly — pure passive read, zero interference
with the arm controller.

Usage:
  python3 arm_state_publisher.py
  python3 arm_state_publisher.py --config-dir config
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, Any

import numpy as np
import yaml

_lerobot_src = os.environ.get("LEROBOT_SRC", "")
if _lerobot_src and _lerobot_src not in sys.path:
    sys.path.insert(0, _lerobot_src)

from lerobot.robots.openarms.config_openarms_follower import OpenArmsFollowerConfig
from lerobot.robots.openarms.openarms_follower import OpenArmsFollower

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _load_config(path: Path) -> Dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


def _read_arm_state(follower):
    obs = follower.get_observation()
    left = [float(obs.get(f"left_joint_{i}.pos", 0.0)) for i in range(1, 8)]
    right = [float(obs.get(f"right_joint_{i}.pos", 0.0)) for i in range(1, 8)]
    left_grip = float(obs.get("left_gripper.pos", 0.0))
    right_grip = float(obs.get("right_gripper.pos", 0.0))
    return left, right, left_grip, right_grip


class ArmStatePublisherNode(Node):
    """Publishes arm JointState at the configured rate."""

    def __init__(self, follower, cfg: Dict) -> None:
        super().__init__("arm_state_publisher")
        self._follower = follower
        self._hz = cfg.get("arm_state_hz", 20)
        self._left_names = cfg.get("left_joints", [f"left_joint_{i}" for i in range(1, 9)])
        self._right_names = cfg.get("right_joints", [f"right_joint_{i}" for i in range(1, 9)])

        self._left_pub = self.create_publisher(JointState, "/arm/left/joint_state", _QOS)
        self._right_pub = self.create_publisher(JointState, "/arm/right/joint_state", _QOS)
        self._timer = self.create_timer(1.0 / self._hz, self._callback)

        self._count = 0
        self._last_print = time.monotonic()
        self.get_logger().info(f"Arm State Publisher: {self._hz} Hz, {len(self._left_names)} joints/arm")

    def _callback(self) -> None:
        try:
            left, right, left_grip, right_grip = _read_arm_state(self._follower)
        except Exception as exc:
            self.get_logger().error(f"Arm read error: {exc}", throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        lm = JointState()
        lm.header.stamp = now
        lm.header.frame_id = "base_link"
        lm.name = self._left_names
        lm.position = [float(v) for v in left[:7]] + [float(left_grip)]
        self._left_pub.publish(lm)

        rm = JointState()
        rm.header.stamp = now
        rm.header.frame_id = "base_link"
        rm.name = self._right_names
        rm.position = [float(v) for v in right[:7]] + [float(right_grip)]
        self._right_pub.publish(rm)

        self._count += 1
        self._maybe_log()

    def stop_timers(self) -> None:
        """Destroy all timers to stop callbacks before shutdown."""
        if hasattr(self, "_timer") and self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

    def _maybe_log(self) -> None:
        now = time.monotonic()
        if now - self._last_print >= 10.0:
            rate = self._count / (now - self._last_print)
            self.get_logger().info(f"Arm: {rate:.1f} Hz ({self._count} frames)")
            self._count = 0
            self._last_print = now


def main() -> None:
    parser = argparse.ArgumentParser(description="Arm State ROS2 Publisher")
    parser.add_argument("--config-dir", default="config", help="Config directory")
    args = parser.parse_args()

    config_dir = Path(args.config_dir)
    if not config_dir.is_absolute():
        config_dir = Path(__file__).resolve().parent / config_dir
    cfg = _load_config(config_dir / "robot.yaml")

    can_left = cfg.get("can_left", "can0")
    can_right = cfg.get("can_right", "can1")
    can_iface = cfg.get("can_interface", "socketcan")

    print("=" * 50)
    print("  Arm State ROS2 Publisher")
    print("=" * 50)
    print(f"  CAN: left={can_left}, right={can_right}")
    print(f"  Rate: {cfg.get('arm_state_hz', 20)} Hz")
    print(f"  Topics: /arm/left/joint_state, /arm/right/joint_state")
    print()

    # Build follower config with motor info, but disable torque-on-disconnect.
    # We never call follower.connect() — connect buses directly to avoid the
    # torque cycle in configure().
    follower_config = OpenArmsFollowerConfig(
        port_left=can_left, port_right=can_right,
        can_interface=can_iface, id="arm_state_publisher",
        disable_torque_on_disconnect=False, max_relative_target=5.0,
    )
    print("[1/2] Connecting CAN buses (read-only, no configure)...")
    follower = OpenArmsFollower(follower_config)
    # Bypass follower.connect() — it calls configure() which cycles torque.
    # Connect the CAN buses directly instead.
    follower.bus_right.connect()
    follower.bus_left.connect()
    print("      Connected (pure read-only, no torque cycle)")

    print("[2/2] Starting ROS2 publisher...")
    rclpy.init(args=sys.argv)
    node = ArmStatePublisherNode(follower, cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_timers()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Disconnect CAN buses (no torque commands)
        try:
            follower.bus_right.disconnect()
        except Exception:
            pass
        try:
            follower.bus_left.disconnect()
        except Exception:
            pass
        print("Done.")


if __name__ == "__main__":
    main()
