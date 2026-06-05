#!/usr/bin/env python3
"""Publish synthetic mock data on real ROS2 topics so the platform collector can
subscribe without real hardware.

from __future__ import annotations

Run this in one terminal, then launch the collector in another:

    Terminal 1:
        python3 tools/dev_mock_ros2_publishers.py \\
            --config config/session_vr_ik_robot_button_control.yaml

    Terminal 2:
        ros2 launch launch/platform_collection.launch.py \\
            session_config:=config/session_vr_ik_robot_button_control.yaml

Requires a sourced ROS2 environment with rclpy, geometry_msgs, sensor_msgs,
and std_msgs.
"""


import argparse
import atexit
import os
from pathlib import Path
import sys
import threading
import time as time_module

ROOT = Path(os.environ.get("DATA_COLLECTION_SERVICE_ROOT", Path.cwd()))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import (
    QoSDurability,
    QoSHistory,
    QoSReliability,
    StreamConfig,
    load_session_config,
)

try:
    import rclpy
    from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import Float32MultiArray, Header
except ImportError:
    rclpy = None
    Node = object
    PoseStamped = object
    RosImage = object
    Float32MultiArray = object
    Header = object
    Point = object
    Pose = object
    Quaternion = object
    QoSProfile = object
    ReliabilityPolicy = object
    DurabilityPolicy = object
    HistoryPolicy = object


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish synthetic mock data on ROS2 topics for offline collector testing.",
    )
    parser.add_argument(
        "--config",
        required=True,
        help="Path to the session YAML config (e.g. config/session_vr_ik_robot_button_control.yaml)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=None,
        help="Override publish rate in Hz (default: highest configured expected_rate_hz)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish one message per stream and exit (for quick smoke test)",
    )
    parser.add_argument(
        "--large-images",
        action="store_true",
        help="Generate ~200KB image frames for bandwidth stress testing",
    )
    parser.add_argument(
        "--jpeg-size-kb",
        type=int,
        default=None,
        help="Generate JPEG frames of exactly this size in KB (e.g., 40 for ~40KB JPEG)",
    )
    return parser


class MockPublisherNode(Node):

    def __init__(self, config, *, rate_hz: float | None = None, large_images: bool = False, jpeg_size_kb: int | None = None):
        super().__init__("platform_mock_publishers")

        self._config = config
        self._large_images = large_images
        self._jpeg_size_kb = jpeg_size_kb
        self._pub_map: dict[str, object] = {}
        self._streams: dict[str, StreamConfig] = {}
        self._step_idx = 0

        for stream_name, stream in config.streams:
            msg_cls = _MESSAGE_CLASSES.get(stream.message_type)
            if msg_cls is None:
                self.get_logger().warn(
                    f"skipping {stream_name}: unsupported message type {stream.message_type}"
                )
                continue

            qos = _qos_profile_from_stream(stream)
            pub = self.create_publisher(msg_cls, stream.topic, qos)
            self._pub_map[stream_name] = pub
            self._streams[stream_name] = stream

            req = "recorded"
            fid = f" frame_id={stream.frame_id}" if stream.frame_id else ""
            self.get_logger().info(
                f"  {stream_name}: {stream.message_type} → {stream.topic} "
                f"[{req}] QoS=({stream.qos.reliability.value},{stream.qos.durability.value}){fid}"
            )

        if not self._pub_map:
            raise RuntimeError("no supported streams found in config")

        # Per-stream threads with Rate.sleep() — precise timing per C7 MCP docs.
        # create_timer is executor-scheduled; Rate.sleep() in a thread is exact.
        self._running = True
        self._threads: list[threading.Thread] = []
        global_rate = rate_hz or 10.0

        for stream_name, stream in self._streams.items():
            stream_rate = stream.expected_rate_hz or global_rate
            self.get_logger().info(
                f"  {stream_name}: {stream_rate:.1f} Hz (period={1000.0/stream_rate:.1f} ms)"
            )
            t = threading.Thread(
                target=self._publish_loop,
                args=(stream_name, stream_rate),
                daemon=True,
            )
            t.start()
            self._threads.append(t)

        self.get_logger().info(
            f"publishing {len(self._pub_map)} streams with per-stream Rate threads"
        )

    @property
    def stream_count(self) -> int:
        return len(self._pub_map)

    def _publish_loop(self, stream_name: str, rate_hz: float) -> None:
        """Dedicated thread. Uses time.perf_counter for exact timing."""
        interval_s = 1.0 / rate_hz
        next_time = time_module.perf_counter() + interval_s
        stream = self._streams[stream_name]
        pub = self._pub_map.get(stream_name)
        if pub is None:
            return
        # Publish loop — runs in a background thread, pushes synthetic data at configured rate
        while self._running and rclpy.ok():
            ros_time = self.get_clock().now().to_msg()
            msg = _build_message(stream_name, stream, ros_time,
                                 large_images=self._large_images,
                                 jpeg_size_kb=self._jpeg_size_kb)
            if msg is not None:
                pub.publish(msg)
            sleep_s = next_time - time_module.perf_counter()
            if sleep_s > 0:
                time_module.sleep(sleep_s)
            next_time += interval_s
            # If we fell behind, reset to avoid burst catching up
            if time_module.perf_counter() > next_time + interval_s:
                next_time = time_module.perf_counter() + interval_s


_MESSAGE_CLASSES: dict[str, type] = {
    "geometry_msgs/PoseStamped": PoseStamped,
    "sensor_msgs/Image": RosImage,
    "std_msgs/Float32MultiArray": Float32MultiArray,
}


def _qos_profile_from_stream(stream: StreamConfig) -> QoSProfile:
    reliability = {
        QoSReliability.BEST_EFFORT: ReliabilityPolicy.BEST_EFFORT,
        QoSReliability.RELIABLE: ReliabilityPolicy.RELIABLE,
    }[stream.qos.reliability]

    durability = {
        QoSDurability.VOLATILE: DurabilityPolicy.VOLATILE,
        QoSDurability.TRANSIENT_LOCAL: DurabilityPolicy.TRANSIENT_LOCAL,
    }[stream.qos.durability]

    history = {
        QoSHistory.KEEP_LAST: HistoryPolicy.KEEP_LAST,
        QoSHistory.KEEP_ALL: HistoryPolicy.KEEP_ALL,
    }[stream.qos.history]

    return QoSProfile(
        depth=stream.qos.depth,
        reliability=reliability,
        durability=durability,
        history=history,
    )


def _build_message(
    stream_name: str,
    stream: StreamConfig,
    ros_time: object,
    *,
    large_images: bool = False,
    jpeg_size_kb: int | None = None,
) -> object | None:
    msg_type = stream.message_type
    frame_id = stream.frame_id or ""
    header = Header(stamp=ros_time, frame_id=frame_id)

    if msg_type == "geometry_msgs/PoseStamped":
        return PoseStamped(
            header=header,
            pose=Pose(
                position=Point(x=0.1, y=0.2, z=0.3),
                orientation=Quaternion(x=0.0, y=0.1, z=0.2, w=0.9),
            ),
        )

    if msg_type == "std_msgs/Float32MultiArray":
        if "button" in stream_name:
            return Float32MultiArray(data=[1.0, 0.0, 0.0, 0.0, 0.0, 0.5])
        return Float32MultiArray(data=[0.0, 0.1, 0.2])

    if msg_type == "sensor_msgs/Image":
        if jpeg_size_kb is not None:
            jpeg_data = _cached_jpeg(jpeg_size_kb)
            return RosImage(
                header=header,
                height=1080, width=1920,
                encoding="jpeg",
                is_bigendian=0,
                step=1920 * 3,
                data=jpeg_data,
            )
        if large_images:
            return RosImage(
                header=header,
                height=1080, width=1920,
                encoding="jpeg",
                is_bigendian=0,
                step=1920 * 3,
                data=_PRECOMPUTED_LARGE_JPEG,
            )
        return RosImage(
            header=header,
            height=1, width=1,
            encoding="rgb8",
            is_bigendian=0,
            step=3,
            data=_MINIMAL_JPEG,
        )

    # Generic fallback: construct mock data from YAML fields contract.
    # For new message types that need actual ROS2 message classes, add an
    # import and a branch above this fallback.
    if stream.fields:
        return _build_generic_message(stream_name, stream, header)

    return None


def main(argv: list[str] | None = None) -> int:
    if rclpy is None:
        print(
            "error: rclpy is not installed. Source a ROS2 environment first.",
            file=sys.stderr,
        )
        return 1

    parser = build_parser()
    args = parser.parse_args(argv)

    config_path = Path(args.config).expanduser().resolve()
    if not config_path.exists():
        print(f"error: config file not found: {config_path}", file=sys.stderr)
        return 1

    try:
        config = load_session_config(config_path)
    except Exception as exc:
        print(f"error: failed to load session config: {exc}", file=sys.stderr)
        return 1

    rclpy.init(args=None)

    # Prevent multiple instances on the same topics
    _pid_file = Path("/tmp/dev_mock_ros2_publishers.pid")
    if _pid_file.exists():
        try:
            old_pid = int(_pid_file.read_text().strip())
            os.kill(old_pid, 0)  # signal 0 just checks if process exists
            print(f"error: another mock publisher is already running (PID {old_pid})",
                  file=sys.stderr)
            print("  stop it first: kill " + str(old_pid), file=sys.stderr)
            rclpy.try_shutdown()
            return 1
        except (OSError, ValueError):
            _pid_file.unlink(missing_ok=True)
    _pid_file.write_text(str(os.getpid()))
    atexit.register(lambda: _pid_file.unlink(missing_ok=True))

    try:
        node = MockPublisherNode(config, rate_hz=args.rate_hz, large_images=args.large_images, jpeg_size_kb=args.jpeg_size_kb)
    except Exception as exc:
        print(f"error: failed to create mock publisher node: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 1

    print(f"Publishing {node.stream_count} mock streams. Ctrl+C to stop.")
    print()

    if args.once:
        ros_time = node.get_clock().now().to_msg()
        for stream_name, stream in node._streams.items():
            msg = _build_message(stream_name, stream, ros_time,
                                 large_images=args.large_images,
                                 jpeg_size_kb=args.jpeg_size_kb)
            if msg is not None and stream_name in node._pub_map:
                node._pub_map[stream_name].publish(msg)
        print("Published one message per stream. Done.")
        node.destroy_node()
        rclpy.try_shutdown()
        return 0

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nshutting down mock publishers...")
        node._running = False
        node.destroy_node()
        rclpy.try_shutdown()
        print("mock publishers stopped.")

    return 0


_MINIMAL_JPEG = bytes([
    0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xff, 0xdb, 0x00, 0x43,
    0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08, 0x07, 0x07, 0x07, 0x09,
    0x09, 0x08, 0x0a, 0x0c, 0x14, 0x0d, 0x0c, 0x0b, 0x0b, 0x0c, 0x19, 0x12,
    0x13, 0x0f, 0x14, 0x1d, 0x1a, 0x1f, 0x1e, 0x1d, 0x1a, 0x1c, 0x1c, 0x20,
    0x24, 0x2e, 0x27, 0x20, 0x22, 0x2c, 0x23, 0x1c, 0x1c, 0x28, 0x37, 0x29,
    0x2c, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1f, 0x27, 0x39, 0x3d, 0x38, 0x32,
    0x3c, 0x2e, 0x33, 0x34, 0x32, 0xff, 0xdb, 0x00, 0x43, 0x01, 0x09, 0x09,
    0x09, 0x0c, 0x0b, 0x0c, 0x18, 0x0d, 0x0d, 0x18, 0x32, 0x21, 0x1c, 0x21,
    0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32,
    0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32,
    0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32,
    0xff, 0xc0, 0x00, 0x11, 0x08, 0x00, 0x01, 0x00, 0x01, 0x03, 0x01, 0x22,
    0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xff, 0xc4, 0x00, 0x1f, 0x00,
    0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0xff, 0xc4, 0x00, 0xb5, 0x10, 0x00, 0x02, 0x01,
    0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01,
    0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41,
    0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1,
    0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62,
    0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
    0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2,
    0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5,
    0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8,
    0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xff, 0xda, 0x00, 0x0c, 0x03,
    0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3f, 0x00, 0x62, 0xa8, 0x55,
    0x00, 0x0c, 0x00, 0x30, 0x05, 0x7f, 0xff, 0xd9,
])

# Precomputed ~200 KB pseudo-JPEG for bandwidth testing. Generated once at
# import time using a fixed byte pattern repeated, not random.randint per frame.
_PRNG_BLOCK = bytes(i % 256 for i in range(256))
_PRNG_DATA = (_PRNG_BLOCK * 800)[:200 * 1024 - 4]
_PRECOMPUTED_LARGE_JPEG = bytes([0xff, 0xd8]) + _PRNG_DATA + bytes([0xff, 0xd9])

_JPEG_CACHE: dict[int, bytes] = {}

def _cached_jpeg(size_kb: int) -> bytes:
    if size_kb not in _JPEG_CACHE:
        size = size_kb * 1024
        _JPEG_CACHE[size_kb] = bytes([0xff, 0xd8]) + bytes(size - 4) + bytes([0xff, 0xd9])
    return _JPEG_CACHE[size_kb]

def _build_generic_message(stream_name, stream, header):
    """Construct a mock message from the YAML fields contract.

    Walks the field paths and builds a SimpleNamespace tree with default
    values: 0.0 for numeric, "" for string, minimal JPEG for bytes,
    [0.0, 0.1, 0.2] for sequence.
    """
    from types import SimpleNamespace

    def _default_for(field):
        if field.type in ("float64", "float32", "int32", "uint32", "uint64"):
            return 0.0
        if field.type == "string":
            return stream.frame_id or ""
        if field.type == "bytes":
            return _MINIMAL_JPEG
        if field.type == "sequence":
            return [0.0, 0.1, 0.2]
        if field.type == "bool":
            return False
        return 0.0

    root = SimpleNamespace()
    if stream.frame_id:
        root.header = header

    for field in stream.fields:
        segments = field.path.split(".")
        obj = root
        for seg in segments[:-1]:
            if not hasattr(obj, seg) or getattr(obj, seg) is None:
                setattr(obj, seg, SimpleNamespace())
            obj = getattr(obj, seg)
        setattr(obj, segments[-1], _default_for(field))

    return root


if __name__ == "__main__":
    raise SystemExit(main())
