#!/usr/bin/env python3
"""
OpenArm 3D Monitor — standalone ROS2 subscriber + WebSocket server.

Subscribes to ROS2 topics (arm joints, targets, VR poses), assembles
snapshots, and broadcasts to browser clients via WebSocket. Completely
decoupled from the IK solver — deploy independently.

Usage:
  python3 monitor_server.py
  python3 monitor_server.py --config config/monitor.yaml
"""

from __future__ import annotations

import argparse
import asyncio
import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    ROS2_OK = True
except ImportError:
    ROS2_OK = False

from aiohttp import web


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

def _load_config(path: Path) -> Dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
) if ROS2_OK else None


# ---------------------------------------------------------------------------
# ROS2 subscriber
# ---------------------------------------------------------------------------

class _TopicStore:
    """Thread-safe store for latest values from ROS2 topics."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data: Dict[str, Any] = {}

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self._data[key] = value

    def get(self, key: str) -> Any:
        with self._lock:
            return self._data.get(key)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._data)


class _ROS2Monitor:
    """Subscribes to ROS2 topics and populates a TopicStore."""

    def __init__(self, store: _TopicStore, cfg: Dict) -> None:
        self._store = store
        self._cfg = cfg
        self._node: Optional[Node] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> bool:
        if not ROS2_OK:
            print("[monitor] ROS2 not available — running in UI-only mode")
            return False
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _spin(self) -> None:
        rclpy.init()
        self._node = Node("openarm_monitor")

        topics = self._cfg.get("ros2_topics", {})

        # ARMS — JointState per arm
        self._node.create_subscription(
            JointState,
            topics.get("left_joint_commands", "/arm/left/joint_commands"),
            lambda msg: self._store.set("left_joints", list(msg.position)),
            _QOS,
        )
        self._node.create_subscription(
            JointState,
            topics.get("right_joint_commands", "/arm/right/joint_commands"),
            lambda msg: self._store.set("right_joints", list(msg.position)),
            _QOS,
        )

        # TARGETS — PoseStamped per arm
        self._node.create_subscription(
            PoseStamped,
            topics.get("left_target_pose", "/arm/left/target_pose"),
            lambda msg: self._store.set("left_target", _pose_to_dict(msg)),
            _QOS,
        )
        self._node.create_subscription(
            PoseStamped,
            topics.get("right_target_pose", "/arm/right/target_pose"),
            lambda msg: self._store.set("right_target", _pose_to_dict(msg)),
            _QOS,
        )

        # VR POSES — PoseStamped per body part
        for key, default_topic in [
            ("head_pose", "/vr/head_pose"),
            ("left_pose", "/vr/left_pose"),
            ("right_pose", "/vr/right_pose"),
        ]:
            self._node.create_subscription(
                PoseStamped,
                topics.get(key, default_topic),
                lambda msg, k=key: self._store.set(k, _pose_to_dict(msg)),
                _QOS,
            )

        print(f"[monitor] Subscribed to ROS2 topics on domain {os.environ.get('ROS_DOMAIN_ID', 0)}")

        import os
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _pose_to_dict(msg) -> Dict[str, Any]:
    return {
        "position": {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
        },
        "orientation_wxyz": [
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ],
        "stale": False,
    }


# ---------------------------------------------------------------------------
# HTTP / WebSocket server
# ---------------------------------------------------------------------------

async def _handle_index(request: web.Request) -> web.Response:
    web_dir = request.app["web_dir"]
    path = web_dir / "index.html"
    if not path.exists():
        return web.Response(status=404, text="index.html not found")
    return web.Response(text=path.read_text(), content_type="text/html")


async def _handle_static(request: web.Request) -> web.Response:
    web_dir: Path = request.app["web_dir"]
    rel = request.match_info.get("path", "")
    full = web_dir / rel
    if not full.exists() or full.is_dir():
        return web.Response(status=404, text="Not found")
    ct = {
        ".js": "application/javascript", ".css": "text/css",
        ".html": "text/html", ".json": "application/json",
        ".png": "image/png", ".glb": "model/gltf-binary",
    }.get(full.suffix, "application/octet-stream")
    return web.Response(body=full.read_bytes(), content_type=ct)


async def _handle_3d_assets(request: web.Request) -> web.Response:
    assets_dir: Path = request.app["assets_dir"]
    rel = request.match_info.get("path", "")
    full = assets_dir / rel
    if not full.exists() or full.is_dir():
        return web.Response(status=404, text="Not found")
    ct = {
        ".js": "application/javascript", ".urdf": "application/xml",
        ".stl": "application/octet-stream", ".dae": "application/xml",
        ".glb": "model/gltf-binary",
    }.get(full.suffix, "application/octet-stream")
    return web.Response(body=full.read_bytes(), content_type=ct)


async def _ws_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    store: _TopicStore = request.app["store"]

    async def _sender():
        hz = request.app.get("broadcast_hz", 20)
        period = 1.0 / hz
        while not ws.closed:
            data = store.snapshot()
            payload = _build_snapshot(data)
            try:
                await ws.send_json(payload)
            except Exception:
                break
            await asyncio.sleep(period)

    sender = asyncio.create_task(_sender())
    try:
        async for _ in ws:
            pass  # client messages ignored
    finally:
        sender.cancel()
    return ws


def _build_snapshot(data: Dict) -> Dict:
    return {
        "solver": {"warmup_complete": True, "warmup_error": None},
        "arms": {
            "left": _arm_snapshot(data, "left"),
            "right": _arm_snapshot(data, "right"),
        },
        "vr_status": {
            "poses": {
                "head": data.get("head_pose", {}),
                "left": data.get("left_pose", {}),
                "right": data.get("right_pose", {}),
            },
            "calibration": {"state": "ready"},
        },
        "control_source": "vr",
        "meta": {"solve_index": 0},
        "audit": None,
    }


def _arm_snapshot(data: Dict, side: str) -> Dict:
    joints = data.get(f"{side}_joints", [])
    target = data.get(f"{side}_target", {})
    t_pos = target.get("position", {})
    t_quat = target.get("orientation_wxyz", [1, 0, 0, 0])
    return {
        "joint_positions": joints,
        "target_active": bool(target),
        "target_position": [t_pos.get("x", 0), t_pos.get("y", 0), t_pos.get("z", 0)],
        "target_quaternion_wxyz": t_quat,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="OpenArm 3D Monitor")
    parser.add_argument("--config", default="config/monitor.yaml", help="Config file")
    parser.add_argument("--port", type=int, default=None, help="HTTP server port")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = Path(__file__).resolve().parent / config_path
    cfg = _load_config(config_path)

    root = Path(__file__).resolve().parent
    web_dir = root / "web"
    assets_dir = root / "3D_assets"

    # Start ROS2 subscriber
    store = _TopicStore()
    ros2 = _ROS2Monitor(store, cfg)
    ros2.start()

    port = args.port or cfg.get("server", {}).get("port", 5201)
    hz = cfg.get("server", {}).get("broadcast_hz", 20)

    # Build app
    app = web.Application()
    app["store"] = store
    app["web_dir"] = web_dir
    app["assets_dir"] = assets_dir
    app["broadcast_hz"] = hz

    app.router.add_get("/", _handle_index)
    app.router.add_get("/ws", _ws_handler)
    app.router.add_get("/web/{path:.*}", _handle_static)
    app.router.add_get("/3D_assets/{path:.*}", _handle_3d_assets)

    print(f"OpenArm 3D Monitor: http://0.0.0.0:{port}")
    web.run_app(app, host="0.0.0.0", port=port)


if __name__ == "__main__":
    main()
