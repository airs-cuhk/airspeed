#!/usr/bin/env python3
"""

VR-Standard ROS2 Bridge Adaptor
from __future__ import annotations


HTTPS server that receives VR device data and publishes to standard ROS2 topics
matching the AIRSPEED teleoperation interface convention.

Published topics:
  /vr_raw_data         — std_msgs/String        (raw JSON, for debugging)
  /vr/head_pose        — geometry_msgs/PoseStamped
  /vr/left_pose        — geometry_msgs/PoseStamped
  /vr/right_pose       — geometry_msgs/PoseStamped
  /vr/left_buttons     — sensor_msgs/Joy
  /vr/right_buttons    — sensor_msgs/Joy

Dependencies: Python 3.10+, aiohttp, rclpy, std_msgs, geometry_msgs
"""


import json
import os
import shutil
import ssl
import sys
import subprocess
import time
import threading
from pathlib import Path
from typing import Dict, Any, Optional, List
from collections import deque
import logging
from logging.handlers import RotatingFileHandler
import argparse

from aiohttp import web

# ---------------------------------------------------------------------------
# ROS2 imports — degrade gracefully if ROS2 is not installed
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Running in standalone mode (no publishing).")


# ---------------------------------------------------------------------------
# Path resolution — everything relative to this file's directory
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent


def resolve_path(relative_path: str) -> str:
    return str(ROOT / relative_path)


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
LOG_DIR = ROOT / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

logger = logging.getLogger("vr_bridge")
logger.setLevel(logging.INFO)
logger.handlers.clear()
logger.propagate = False

_fh = RotatingFileHandler(LOG_DIR / "vr_bridge.log", maxBytes=5 * 1024 * 1024, backupCount=3)
_fh.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
logger.addHandler(_fh)

_ch = logging.StreamHandler()
_ch.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
logger.addHandler(_ch)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
_config: Optional[Dict[str, Any]] = None


def load_config() -> Dict[str, Any]:
    global _config
    if _config is not None:
        return _config
    config_path = resolve_path("config/config.json")
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            _config = json.load(f)
        logger.info("Loaded config from %s", config_path)
        return _config
    _config = {"port": 5000, "vr_data_rate": 60, "ip": "127.0.0.1"}
    logger.warning("No config file found, using defaults")
    return _config


# ---------------------------------------------------------------------------
# SSL
# ---------------------------------------------------------------------------
def _create_self_signed_cert(ip_address: str) -> None:
    pem_dir = resolve_path("pem")
    os.makedirs(pem_dir, exist_ok=True)

    cert_file = os.path.join(pem_dir, "cert.pem")
    key_file = os.path.join(pem_dir, "key.pem")
    ip_file = os.path.join(pem_dir, "IP_address.txt")

    try:
        from OpenSSL import crypto
    except ImportError:
        logger.error("pyOpenSSL required for certificate generation. Install: pip install pyOpenSSL")
        raise

    k = crypto.PKey()
    k.generate_key(crypto.TYPE_RSA, 2048)

    cert = crypto.X509()
    cert.get_subject().CN = ip_address
    cert.set_serial_number(1000)
    cert.gmtime_adj_notBefore(0)
    cert.gmtime_adj_notAfter(365 * 24 * 60 * 60)
    cert.set_issuer(cert.get_subject())
    cert.set_pubkey(k)
    cert.sign(k, 'sha256')

    with open(cert_file, "wt") as f:
        f.write(crypto.dump_certificate(crypto.FILETYPE_PEM, cert).decode('utf-8'))
    with open(key_file, "wt") as f:
        f.write(crypto.dump_privatekey(crypto.FILETYPE_PEM, k).decode('utf-8'))
    with open(ip_file, "w") as f:
        f.write(ip_address)

    logger.info("Created self-signed SSL certificate for %s", ip_address)


def create_ssl_context() -> ssl.SSLContext:
    cert_file = resolve_path("pem/cert.pem")
    key_file = resolve_path("pem/key.pem")
    if os.path.exists(cert_file) and os.path.exists(key_file):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert_file, key_file)
        return ctx
    logger.warning("No SSL certificates found — using insecure fallback")
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


# ---------------------------------------------------------------------------
# ADB port forwarding
# ---------------------------------------------------------------------------
def _adb_env_with_null_log() -> tuple[dict[str, str], str]:
    """Return an env dict and a temp dir that redirects adb daemon logs to /dev/null.

    The ADB daemon writes all stdout/stderr to $TMPDIR/adb.<UID>.log, which can
    grow without bound. By symlinking that file to /dev/null inside a private
    TMPDIR we keep adb fully functional while discarding the daemon log.
    """
    import tempfile

    uid = os.getuid()
    tmpdir = tempfile.mkdtemp(prefix="vr_bridge_adb_")
    log_link = os.path.join(tmpdir, f"adb.{uid}.log")
    try:
        os.symlink(os.devnull, log_link)
    except FileExistsError:
        pass
    env = os.environ.copy()
    env["TMPDIR"] = tmpdir
    return env, tmpdir


def setup_adb_reverse(port: int) -> bool:
    """Refresh ADB server, then set up tcp:{port} reverse tunnel on all devices."""
    env, adb_tmpdir = _adb_env_with_null_log()
    try:
        subprocess.run(["adb", "kill-server"], capture_output=True, timeout=5, env=env)
        subprocess.run(["adb", "start-server"], capture_output=True, timeout=5, env=env)
    except Exception as e:
        logger.warning("ADB: could not refresh server: %s", e)
        return False
    finally:
        # Daemon has opened /dev/null via the symlink; the temp dir is no longer needed.
        try:
            shutil.rmtree(adb_tmpdir)
        except Exception:
            pass

    try:
        r = subprocess.run(["adb", "devices"], capture_output=True, text=True, timeout=5, env=env)
        devices = [l.split('\t')[0] for l in r.stdout.strip().split('\n')[1:]
                   if '\t' in l and l.split('\t')[1] == 'device']
        if not devices:
            logger.warning("ADB: no devices connected")
            return False
        ok = True
        for dev in devices:
            rr = subprocess.run(
                ["adb", "-s", dev, "reverse", f"tcp:{port}", f"tcp:{port}"],
                capture_output=True, text=True, timeout=10, env=env)
            if rr.returncode == 0:
                logger.info("ADB reverse tcp:%s → %s", port, dev)
            else:
                logger.warning("ADB reverse failed for %s: %s", dev, rr.stderr.strip())
                ok = False
        return ok
    except Exception as e:
        logger.warning("ADB not available: %s", e)
        return False


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
REALTIME_QOS = None
if ROS2_AVAILABLE:
    REALTIME_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

_ros_node: Optional['VrBridgeNode'] = None


class VrBridgeNode(Node):
    """Publishes VR data to standard ROS2 teleoperation topics."""

    def __init__(self):
        super().__init__('vr_bridge')

        # Raw JSON (debug / backward compat)
        self.raw_pub = self.create_publisher(String, '/vr_raw_data', REALTIME_QOS)

        # Canonical pose topics
        self.head_pub = self.create_publisher(PoseStamped, '/vr/head_pose', REALTIME_QOS)
        self.left_pub = self.create_publisher(PoseStamped, '/vr/left_pose', REALTIME_QOS)
        self.right_pub = self.create_publisher(PoseStamped, '/vr/right_pose', REALTIME_QOS)

        # Button topics
        self.left_btn_pub = self.create_publisher(Joy, '/vr/left_buttons', REALTIME_QOS)
        self.right_btn_pub = self.create_publisher(Joy, '/vr/right_buttons', REALTIME_QOS)

        self._count = 0
        self._last = time.time()

        self.get_logger().info('VR Bridge node ready — 5 canonical topics + raw')

    def publish(self, data: Dict[str, Any], raw_text: str) -> None:
        now = self.get_clock().now().to_msg()

        # Raw
        rm = String()
        rm.data = raw_text
        self.raw_pub.publish(rm)

        # Head
        if 'head' in data:
            self.head_pub.publish(_make_pose(data['head'], now, 'vr_head'))

        # Left controller
        if 'left' in data:
            self.left_pub.publish(_make_pose(data['left'], now, 'vr_left'))
            if 'button' in data['left']:
                self.left_btn_pub.publish(_make_buttons(data['left']['button'], now))

        # Right controller
        if 'right' in data:
            self.right_pub.publish(_make_pose(data['right'], now, 'vr_right'))
            if 'button' in data['right']:
                self.right_btn_pub.publish(_make_buttons(data['right']['button'], now))

        # Stats (every 5 s)
        self._count += 1
        now_t = time.time()
        if now_t - self._last >= 5.0:
            self.get_logger().info(f'Publish rate: {self._count / (now_t - self._last):.0f} Hz')
            self._count = 0
            self._last = now_t


def _make_pose(data: Dict, stamp, frame_id: str) -> PoseStamped:
    m = PoseStamped()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    pos = data.get('position', {})
    m.pose.position.x = float(pos.get('x', 0.0))
    m.pose.position.y = float(pos.get('y', 0.0))
    m.pose.position.z = float(pos.get('z', 0.0))
    rot = data.get('rotation', {})
    m.pose.orientation.x = float(rot.get('x', 0.0))
    m.pose.orientation.y = float(rot.get('y', 0.0))
    m.pose.orientation.z = float(rot.get('z', 0.0))
    m.pose.orientation.w = float(rot.get('w', 1.0))
    return m


def _make_buttons(buttons: List, stamp) -> Joy:
    m = Joy()
    m.header.stamp = stamp
    vals = [float(b.get('value', 0)) for b in buttons[:6]]
    # Pad to exactly 6 channels — missing buttons default to 0.0 (released)
    while len(vals) < 6:
        vals.append(0.0)
    # All channels are analog-capable (triggers/grips) → axes; buttons stays empty
    m.axes = vals
    return m


def _ros_spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------
async def handle_pose(request: web.Request) -> web.Response:
    raw_text = await request.text()

    if _ros_node is not None:
        try:
            data = json.loads(raw_text)
        except json.JSONDecodeError:
            return web.Response(status=400, text="Invalid JSON")
        _ros_node.publish(data, raw_text)

    return web.Response(text="OK")


async def handle_index(request: web.Request) -> web.Response:
    html_path = resolve_path("static/VRDriverPlaneCamera.html")
    if not os.path.exists(html_path):
        return web.Response(status=404, text="VR page not found")
    with open(html_path, 'r') as f:
        return web.Response(text=f.read(), content_type="text/html")


async def handle_static(request: web.Request) -> web.Response:
    rel = request.match_info.get('path', '')
    full = resolve_path(f"static/{rel}")
    if not os.path.exists(full) or os.path.isdir(full):
        return web.Response(status=404, text="Not found")
    ctypes = {'.js': 'application/javascript', '.css': 'text/css', '.html': 'text/html',
              '.json': 'application/json', '.png': 'image/png', '.jpg': 'image/jpeg'}
    ext = os.path.splitext(rel)[1].lower()
    with open(full, 'rb') as f:
        body = f.read()
    # Rewrite hardcoded port and protocol in JS to match actual server
    if ext == '.js':
        port = request.app['port']
        proto = 'http' if request.app.get('no_ssl') else 'https'
        body = body.replace(b'https://${this._str_pose_ip}:5000/poseData',
                            f'{proto}://${{this._str_pose_ip}}:{port}/poseData'.encode())
        body = body.replace(b':5000/', f':{port}/'.encode())
    return web.Response(body=body, content_type=ctypes.get(ext, 'application/octet-stream'))


async def handle_config(request: web.Request) -> web.Response:
    return web.Response(text=json.dumps(load_config()), content_type="application/json")


async def handle_log(request: web.Request) -> web.Response:
    try:
        data = await request.json()
        logger.info("[CLIENT] %s", data.get('log', ''))
    except Exception:
        pass
    return web.Response()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    global _ros_node

    p = argparse.ArgumentParser(description="VR-Standard ROS2 Bridge Adaptor")
    p.add_argument("--port", type=int, default=None, help="Server port (default: from config or 5000)")
    p.add_argument("--host", default=None, help="Bind address (default: from config or 0.0.0.0)")
    p.add_argument("--no-adb", action="store_true", help="Skip ADB port forwarding")
    p.add_argument("--no-ssl", action="store_true", help="Serve plain HTTP instead of HTTPS (use when behind ADB)")
    args = p.parse_args()

    # Priority: CLI flag > config file > hardcoded fallback
    cfg = load_config()
    port = args.port if args.port is not None else cfg.get("port", 5000)
    host = args.host if args.host is not None else cfg.get("ip", "0.0.0.0")

    # ROS2
    if ROS2_AVAILABLE:
        logger.info("Initializing ROS2...")
        rclpy.init()
        _ros_node = VrBridgeNode()
        threading.Thread(target=_ros_spin, args=(_ros_node,), daemon=True).start()
        logger.info("ROS2 node started")

    # SSL
    if host not in ("127.0.0.1", "localhost", "0.0.0.0"):
        cert_file = resolve_path("pem/cert.pem")
        if not os.path.exists(cert_file):
            logger.info("Generating self-signed certificate for %s...", host)
            _create_self_signed_cert(host)

    # ADB
    if not args.no_adb:
        setup_adb_reverse(port)

    # Routes
    app = web.Application()
    app['port'] = port
    app['no_ssl'] = args.no_ssl
    app.router.add_get("/", handle_index)
    app.router.add_get("/{path:.*}", handle_static)
    app.router.add_post("/poseData", handle_pose)
    app.router.add_post("/log", handle_log)
    app.router.add_post("/config", handle_config)

    ssl_ctx = None if args.no_ssl else create_ssl_context()
    proto = "http" if args.no_ssl else "https"

    topics = (
        "  /vr_raw_data        (String)\n"
        "  /vr/head_pose       (PoseStamped)\n"
        "  /vr/left_pose       (PoseStamped)\n"
        "  /vr/right_pose      (PoseStamped)\n"
        "  /vr/left_buttons    (Joy)\n"
        "  /vr/right_buttons   (Joy)"
    )
    logger.info("=" * 55)
    logger.info("VR-Standard ROS2 Bridge Adaptor")
    logger.info("%s://%s:%s", proto, host, port)
    logger.info("=" * 55)
    logger.info("ROS2 topics:\n%s", topics)
    logger.info("=" * 55)

    try:
        web.run_app(app, host=host, port=port, ssl_context=ssl_ctx)
    finally:
        if _ros_node is not None:
            _ros_node.destroy_node()
            rclpy.shutdown()
            logger.info("ROS2 shut down")


if __name__ == "__main__":
    main()
