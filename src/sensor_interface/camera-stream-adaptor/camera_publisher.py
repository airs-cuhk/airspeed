#!/usr/bin/env python3
"""
Camera Stream ROS2 Publisher — RealSense cameras → Image + CameraInfo topics.

from __future__ import annotations

Publishes JPEG-encoded Image + CameraInfo per camera matching the
AIRSPEED sensor_interface convention. Streams at native camera rate
via a background thread — no artificial rate cap.

Stream types (color, depth, infra) are enabled per-camera in config/camera.yaml.
Unavailable stream types configured for a camera are warned in the console.

Usage:
  python3 camera_publisher.py
  python3 camera_publisher.py --config-dir config
"""


import os
import sys
_lerobot_src = os.environ.get("LEROBOT_SRC", "")
if _lerobot_src and _lerobot_src not in sys.path:
    sys.path.insert(0, _lerobot_src)

import argparse
import threading
import time
from pathlib import Path
from typing import List, Tuple, Dict, Any, Optional

import yaml

_HAS_REALSENSE = False
_HAS_CV2 = False

try:
    from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
    _HAS_REALSENSE = True
except Exception:
    RealSenseCamera = None
    RealSenseCameraConfig = None

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    pass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_CAMERA_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Stream types that require pyrealsense2 direct access (lerobot wrapper only gives color)
_DIRECT_API_STREAMS = {"depth", "infra_left", "infra_right"}


def _load_config(path: Path) -> Dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Camera discovery + connection
# ---------------------------------------------------------------------------

def _discover_and_connect(cfg: Dict) -> List[Tuple[str, str, Dict, object]]:
    """Discover RealSense cameras and connect those enabled in config.

    Returns list of (camera_key, topic, stream_configs, camera_object).
    Warns when a configured camera is not found or a stream type is unavailable.
    """
    if not _HAS_REALSENSE:
        print("      RealSenseCamera not available, skipping cameras")
        return []

    cameras_cfg = cfg.get("cameras", {})
    if not cameras_cfg:
        print("      No cameras configured")
        return []

    # Discover
    try:
        found = RealSenseCamera.find_cameras()
    except Exception as e:
        print(f"      Camera discovery failed: {e}")
        return []

    max_cameras = cfg.get("max_cameras", len(found))
    n_found = len(found)
    print(f"      Found {n_found} RealSense camera(s) (max {max_cameras})")
    for i, info in enumerate(found[:max_cameras]):
        print(f"        [{i}] {info.get('name', 'Unknown')}  SN={info.get('id', 'unknown')}")

    # Map config keys to discovery indices
    config_keys = list(cameras_cfg.keys())
    connected: List[Tuple[str, str, Dict, object]] = []

    for idx, info in enumerate(found[:max_cameras]):
        if idx >= len(config_keys):
            print(f"      [{idx}] no config entry — skipping")
            continue

        cam_key = config_keys[idx]
        entry = cameras_cfg[cam_key]
        sn = info.get("id", "")
        cfg_sn = entry.get("serial", "auto")

        if cfg_sn != "auto" and cfg_sn != sn:
            print(f"      [{idx}] serial mismatch (config={cfg_sn}, found={sn}) — skipping")
            continue

        topic = entry.get("topic", f"/camera/{cam_key}")
        frame_id = entry.get("frame_id", f"camera_{cam_key}_optical_frame")

        try:
            cam_cfg = RealSenseCameraConfig(serial_number_or_name=sn)
            cam = RealSenseCamera(cam_cfg)
            cam.connect()
            connected.append((cam_key, topic, entry, cam))
            print(f"        {cam_key} → {topic}  connected")
        except Exception as e:
            print(f"        {cam_key} → {topic}  FAILED: {e}")
            continue

        # Warn about unavailable stream types
        streams_cfg = entry.get("streams", {})
        for stype, scfg in streams_cfg.items():
            if scfg.get("enabled", False) and stype in _DIRECT_API_STREAMS:
                print(f"          [WARN] {stype}: requires pyrealsense2 direct access, not yet supported")

    # Warn about config entries that couldn't be matched
    connected_keys = {c[0] for c in connected}
    for cam_key in config_keys:
        if cam_key not in connected_keys:
            print(f"      [WARN] {cam_key}: configured but not connected")

    return connected


def _build_camera_info(frame_id: str, intrinsics: Dict) -> CameraInfo:
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.height = intrinsics.get("height", 480)
    msg.width = intrinsics.get("width", 640)
    msg.distortion_model = intrinsics.get("distortion_model", "plumb_bob")
    msg.d = intrinsics.get("distortion_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
    fx, fy = intrinsics.get("fx", 615.0), intrinsics.get("fy", 615.0)
    cx, cy = intrinsics.get("cx", 320.0), intrinsics.get("cy", 240.0)
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    return msg


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class CameraPublisherNode(Node):
    """Publishes Image + CameraInfo per camera at the hardware's native rate."""

    def __init__(self, cameras: List[Tuple[str, str, Dict, object]], cfg: Dict) -> None:
        super().__init__("camera_publisher")
        self._intrinsics = cfg.get("intrinsics", {})

        # Build stream entries: one per (camera, enabled stream type)
        self._streams: List[Dict] = []
        for cam_key, topic, entry, cam_obj in cameras:
            frame_id = entry.get("frame_id", f"camera_{cam_key}_optical_frame")
            streams_cfg = entry.get("streams", {})

            for stype, scfg in streams_cfg.items():
                if not scfg.get("enabled", False):
                    continue
                if stype in _DIRECT_API_STREAMS:
                    continue  # warned during discovery

                topic_suffix = {"color": "image_raw", "depth": "depth/image_raw",
                                "infra_left": "infra_left/image_raw",
                                "infra_right": "infra_right/image_raw"}.get(stype, f"{stype}/image_raw")

                img_pub = self.create_publisher(Image, f"{topic}/{topic_suffix}", _QOS)
                info_pub = self.create_publisher(CameraInfo, f"{topic}/camera_info", _CAMERA_INFO_QOS)
                self._streams.append({
                    "cam_key": cam_key,
                    "topic": topic,
                    "frame_id": frame_id,
                    "cam": cam_obj,
                    "img_pub": img_pub,
                    "info_pub": info_pub,
                    "stype": stype,
                    "encoding": scfg.get("encoding", "jpeg"),
                    "jpeg_q": scfg.get("jpeg_quality", 70),
                })
                self.get_logger().info(f"Camera: {topic}/{topic_suffix} ({stype})")

        # Publish CameraInfo once at startup (latched)
        for s in self._streams:
            cam_key = s["cam_key"]
            intrinsics = self._intrinsics.get(cam_key, {})
            if intrinsics:
                ci = _build_camera_info(s["frame_id"], intrinsics)
                s["info_pub"].publish(ci)

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._counts: Dict[str, int] = {}
        self._last_print = time.monotonic()
        self.get_logger().info(f"Camera Publisher: native rate, {len(self._streams)} stream(s)")

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name="camera-stream")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _stream_loop(self) -> None:
        """Run in background thread at camera native rate.

        No create_timer() cap — each camera delivers frames at its hardware rate.
        Single color stream: ~30 Hz. Multi-stream (depth+IR+color): ~15 Hz (USB 3.2 limit).
        Each iteration reads ALL cameras before looping, so cameras are sampled in lockstep
        within a single frameset (~5 ms timeout between cameras).
        """
        while self._running:
            if not _HAS_CV2:
                time.sleep(0.1)
                continue

            for s in self._streams:
                try:
                    frame = s["cam"].read()
                    if frame is None or frame.size == 0:
                        continue

                    if len(frame.shape) == 3 and frame.shape[2] == 3:
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    else:
                        frame_bgr = frame

                    ret, jpeg_buf = cv2.imencode(
                        ".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, s["jpeg_q"]]
                    )
                    if not ret:
                        continue
                    jpeg_bytes = jpeg_buf.tobytes()

                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = s["frame_id"]
                    msg.height = frame.shape[0]
                    msg.width = frame.shape[1]
                    msg.encoding = s["encoding"]
                    msg.is_bigendian = 0
                    msg.step = len(jpeg_bytes)
                    msg.data = jpeg_bytes
                    s["img_pub"].publish(msg)

                    key = s["cam_key"]
                    self._counts[key] = self._counts.get(key, 0) + 1

                except Exception as exc:
                    self.get_logger().error(
                        f"Camera {s['cam_key']}: {exc}", throttle_duration_sec=5.0
                    )

            self._maybe_log()

    def _maybe_log(self) -> None:
        now = time.monotonic()
        if now - self._last_print >= 10.0:
            total = sum(self._counts.values())
            rate = total / (now - self._last_print)
            parts = ", ".join(f"{k}={self._counts.get(k,0)}" for k in sorted(self._counts))
            self.get_logger().info(f"Frames: {rate:.1f} Hz ({total} total) | {parts}")
            self._counts.clear()
            self._last_print = now


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Camera Stream ROS2 Publisher")
    parser.add_argument("--config-dir", default="config", help="Config directory")
    args = parser.parse_args()

    config_dir = Path(args.config_dir)
    if not config_dir.is_absolute():
        config_dir = Path(__file__).resolve().parent / config_dir
    cfg = _load_config(config_dir / "camera.yaml")

    if not _HAS_REALSENSE:
        print("ERROR: RealSenseCamera not available. Add lerobot to PYTHONPATH.")
        sys.exit(1)

    print("=" * 50)
    print("  Camera Stream ROS2 Publisher (native rate)")
    print("=" * 50)
    print()

    print("[1/2] Discovering cameras...")
    cameras = _discover_and_connect(cfg)
    if not cameras:
        print("      No cameras could be connected. Exiting.")
        sys.exit(1)

    print("\n[2/2] Starting ROS2 publisher...")
    rclpy.init(args=sys.argv)
    node = CameraPublisherNode(cameras, cfg)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
