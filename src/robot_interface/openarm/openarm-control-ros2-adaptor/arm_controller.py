"""WebSocket Follower Arm Controller — receives IK commands, drives motors."""

from __future__ import annotations

import os
import sys
_lerobot_src = os.environ.get("LEROBOT_SRC", "")
if not _lerobot_src:
    _lerobot_src = os.path.dirname(os.path.abspath(__file__))  # project root, contains lerobot/
if _lerobot_src not in sys.path:
    sys.path.insert(0, _lerobot_src)

import argparse
import asyncio
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, Any

import numpy as np
import yaml
import websockets
from websockets.exceptions import ConnectionClosed

from lerobot.robots.openarms.config_openarms_follower import OpenArmsFollowerConfig
from lerobot.robots.openarms.openarms_follower import OpenArmsFollower


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def _load_config(config_dir: Path) -> Dict[str, Any]:
    with open(config_dir / "robot.yaml") as f:
        cfg = yaml.safe_load(f)
    shared_path = (config_dir / ".." / ".." / "robot_shared.yaml").resolve()
    if shared_path.exists():
        shared = yaml.safe_load(shared_path.read_text())
        cfg.setdefault("home_position_deg", shared.get("home_position_deg", {}))
    return cfg


# ---------------------------------------------------------------------------
# Gravity
# ---------------------------------------------------------------------------

def _compute_gravity(follower: OpenArmsFollower, follower_obs: dict) -> dict:
    if follower.pin_robot is None:
        return {}
    follower_q = {
        f"{side}_{m}": follower_obs.get(f"{side}_{m}.pos", 0.0)
        for side in ["right", "left"]
        for m in follower.bus_right.motors
    }
    return follower._gravity_from_q(follower_q)


# ---------------------------------------------------------------------------
# Motion primitives — unchanged from original
# ---------------------------------------------------------------------------

def _move_to_home(follower: OpenArmsFollower, cfg: dict, hold_seconds: float = 5.0) -> None:
    home = cfg.get("home_position_deg", {})
    gripper_open = cfg.get("gripper_open_deg", -65.0)
    fps = cfg.get("arm_state_hz", 30)

    has_gravity = follower.pin_robot is not None
    print(f"      Gravity compensation: {'ON' if has_gravity else 'OFF'}")

    start_obs = follower.get_observation()
    start_pos = {
        f"{side}_{motor}": np.rad2deg(start_obs.get(f"{side}_{motor}.pos", 0.0))
        for side, bus in [("right", follower.bus_right), ("left", follower.bus_left)]
        for motor in bus.motors
    }

    t0 = time.perf_counter()
    # Interpolation loop: linearly ramp from current pose to home over hold_seconds.
    # frac goes 0→1, giving smooth motion with bounded velocity.
    while time.perf_counter() - t0 < hold_seconds:
        frac = min(1.0, (time.perf_counter() - t0) / hold_seconds)
        follower_obs = follower.get_observation()
        gravity = _compute_gravity(follower, follower_obs)

        for side, bus in [("right", follower.bus_right), ("left", follower.bus_left)]:
            for motor in bus.motors:
                joint_name = f"{side}_{motor}"
                target = home.get(joint_name, 0.0)
                pos_deg = start_pos[joint_name] + (target - start_pos[joint_name]) * frac
                kp = _get_kp(cfg, motor)
                kd = _get_kd(cfg, motor)
                torque = gravity.get(joint_name, 0.0)
                bus._mit_control(
                    motor=motor, kp=kp, kd=kd,
                    position_degrees=pos_deg,
                    velocity_deg_per_sec=0.0, torque=torque,
                )
        time.sleep(1.0 / fps)


def _test_grippers(follower: OpenArmsFollower, cfg: dict,
                   open_seconds: float = 2.5, hold_seconds: float = 2.0) -> None:
    gripper_open = cfg.get("gripper_open_deg", -65.0)
    fps = cfg.get("arm_state_hz", 30)
    print("      Testing grippers (open → hold)...")

    start_obs = follower.get_observation()
    start_pos = {
        "right": np.rad2deg(start_obs.get("right_gripper.pos", 0.0)),
        "left": np.rad2deg(start_obs.get("left_gripper.pos", 0.0)),
    }

    # Interpolate grippers from current position to open over open_seconds
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < open_seconds:
        frac = min(1.0, (time.perf_counter() - t0) / open_seconds)
        for side, bus in [("right", follower.bus_right), ("left", follower.bus_left)]:
            motor = "gripper"
            pos_deg = start_pos[side] + (gripper_open - start_pos[side]) * frac
            bus._mit_control(
                motor=motor, kp=_get_kp(cfg, motor), kd=_get_kd(cfg, motor),
                position_degrees=pos_deg, velocity_deg_per_sec=0.0, torque=0.0,
            )
        time.sleep(1.0 / fps)

    # Hold grippers open at the target position
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < hold_seconds:
        for bus in [follower.bus_right, follower.bus_left]:
            bus._mit_control(
                motor="gripper", kp=_get_kp(cfg, "gripper"), kd=_get_kd(cfg, "gripper"),
                position_degrees=gripper_open, velocity_deg_per_sec=0.0, torque=0.0,
            )
        time.sleep(1.0 / fps)


# ---------------------------------------------------------------------------
# Safety clamping — unchanged
# ---------------------------------------------------------------------------

def _clamp_joint_deltas(
    left_rad, right_rad, left_grip_deg, right_grip_deg, prev: dict, cfg: dict,
) -> tuple:
    max_joint = cfg.get("max_joint_delta_deg", 3.0)
    max_gripper = cfg.get("max_gripper_delta_deg", 10.0)

    left_deg = np.rad2deg(left_rad[:7])
    right_deg = np.rad2deg(right_rad[:7])

    def _clamp(val, prev_val, limit):
        delta = val - prev_val
        if abs(delta) > limit:
            return prev_val + limit if delta > 0 else prev_val - limit
        return val

    for _side, new_deg, key in [("left", left_deg, "left_joints"), ("right", right_deg, "right_joints")]:
        prev_arr = prev.get(key)
        if prev_arr is not None and len(prev_arr) == len(new_deg):
            for i in range(len(new_deg)):
                new_deg[i] = _clamp(new_deg[i], prev_arr[i], max_joint)
        prev[key] = list(new_deg)

    for _side, new_grip, key in [("left", left_grip_deg, "left_gripper"), ("right", right_grip_deg, "right_gripper")]:
        prev_val = prev.get(key)
        if new_grip is not None:
            prev[key] = _clamp(new_grip, prev_val, max_gripper) if prev_val is not None else new_grip
        else:
            prev[key] = prev_val

    return left_deg, right_deg, prev.get("left_gripper"), prev.get("right_gripper")


def _apply_joints(follower, left_deg, right_deg, left_grip_deg, right_grip_deg, cfg: dict) -> None:
    follower_obs = follower.get_observation()
    gravity = _compute_gravity(follower, follower_obs)

    for i, motor in enumerate(follower.bus_right.motors):
        if i >= 7:
            break
        if i < len(right_deg):
            torque = gravity.get(f"right_{motor}", 0.0)
            follower.bus_right._mit_control(
                motor=motor, kp=_get_kp(cfg, motor), kd=_get_kd(cfg, motor),
                position_degrees=float(right_deg[i]), velocity_deg_per_sec=0.0, torque=torque,
            )

    for i, motor in enumerate(follower.bus_left.motors):
        if i >= 7:
            break
        if i < len(left_deg):
            torque = gravity.get(f"left_{motor}", 0.0)
            follower.bus_left._mit_control(
                motor=motor, kp=_get_kp(cfg, motor), kd=_get_kd(cfg, motor),
                position_degrees=float(left_deg[i]), velocity_deg_per_sec=0.0, torque=torque,
            )

    if left_grip_deg is not None:
        follower.bus_left._mit_control(
            motor="gripper", kp=_get_kp(cfg, "gripper"), kd=_get_kd(cfg, "gripper"),
            position_degrees=left_grip_deg, velocity_deg_per_sec=0.0, torque=0.0,
        )
    if right_grip_deg is not None:
        follower.bus_right._mit_control(
            motor="gripper", kp=_get_kp(cfg, "gripper"), kd=_get_kd(cfg, "gripper"),
            position_degrees=right_grip_deg, velocity_deg_per_sec=0.0, torque=0.0,
        )


def _get_kp(cfg: dict, motor: str) -> float:
    kp = cfg.get("kp", {})
    return kp.get(motor, 25.0)


def _get_kd(cfg: dict, motor: str) -> float:
    kd = cfg.get("kd", {})
    return kd.get(motor, 0.3)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _start_publisher(python_bin: str, config_dir: str) -> subprocess.Popen | None:
    """Spawn arm_state_publisher.py as a subprocess. Returns None on failure."""
    publisher_script = Path(__file__).resolve().parent / "arm_state_publisher.py"
    if not publisher_script.exists():
        print("  [publisher] arm_state_publisher.py not found — skipping")
        return None
    try:
        proc = subprocess.Popen(
            [python_bin, str(publisher_script), "--config-dir", config_dir],
            stdout=sys.stdout, stderr=sys.stderr,
            stdin=subprocess.DEVNULL,
            preexec_fn=os.setsid,  # isolate in its own process group
        )
        print(f"  [publisher] Started (PID={proc.pid})")
        return proc
    except Exception as e:
        print(f"  [publisher] Failed to start: {e}")
        return None


def _stop_publisher(proc: subprocess.Popen | None) -> None:
    """Gracefully stop the publisher subprocess.

    Handles KeyboardInterrupt internally — the user may press Ctrl+C again
    while we're waiting for the publisher to shutdown. In that case, kill
    immediately and move on.
    """
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=3)  # shorter timeout for cleaner shutdown
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=1)
        except KeyboardInterrupt:
            proc.kill()  # user pressed Ctrl+C again — force kill
            proc.wait(timeout=1)
        print("  [publisher] Stopped")
    except (ProcessLookupError, OSError, KeyboardInterrupt):
        pass


async def run(cfg: dict, ws_uri: str, *, start_publisher: bool = True) -> None:
    can_left = cfg.get("can_left", "can0")
    can_right = cfg.get("can_right", "can1")
    can_iface = cfg.get("can_interface", "socketcan")
    fps = cfg.get("arm_state_hz", 30)
    stream_timeout = cfg.get("stream_timeout_s", 0.5)
    return_home_s = cfg.get("return_home_seconds", 3.0)
    gripper_open = cfg.get("gripper_open_deg", -65.0)
    home = cfg.get("home_position_deg", {})

    print("=" * 60)
    print("  WebSocket Follower Arm Controller")
    print("=" * 60)
    print(f"  WS:  {ws_uri}")
    print(f"  CAN: left={can_left}, right={can_right}")
    print(f"  FPS: {fps}")

    follower = None
    publisher_proc = None
    try:
        # 1. Connect + calibrate
        follower_config = OpenArmsFollowerConfig(
            port_left=can_left, port_right=can_right, can_interface=can_iface,
            id="openarms_follower", disable_torque_on_disconnect=True, max_relative_target=5.0,
        )
        print("\n[1/3] Connecting follower arm...")
        follower = OpenArmsFollower(follower_config)

        # Auto-copy meshes from the IK adaptor if missing here.
        # Both adaptors use the same mesh files — no need to download twice.
        _mesh_src = (
            Path(__file__).resolve().parent.parent / "openarm-ik-ros2-adaptor"
            / "frontend" / "3d_assets" / "urdf" / "meshes"
        )
        _mesh_dst = (
            Path(__file__).resolve().parent / "lerobot" / "robots"
            / "openarms" / "urdf" / "meshes"
        )
        if _mesh_src.is_dir() and not any(_mesh_dst.iterdir()) if _mesh_dst.is_dir() else True:
            import shutil
            _mesh_dst.mkdir(parents=True, exist_ok=True)
            for _item in _mesh_src.iterdir():
                _dst = _mesh_dst / _item.name
                if _item.is_dir():
                    if not _dst.exists():
                        shutil.copytree(_item, _dst)
                else:
                    shutil.copy2(_item, _dst)
            print(f"      Meshes copied from IK adaptor: {_mesh_dst}")

        # Gravity compensation — load URDF from the bundled lerobot package
        urdf_path = cfg.get("urdf_path", "")
        if follower.pin_robot is None and urdf_path:
            try:
                import pinocchio as pin
                import lerobot.robots.openarms as _openarms_pkg
                if not os.path.isabs(urdf_path):
                    urdf_path = os.path.join(
                        os.path.dirname(_openarms_pkg.__file__), urdf_path)
                urdf_dir = os.path.dirname(urdf_path)
                if os.path.exists(urdf_path):
                    follower.pin_robot = pin.RobotWrapper.BuildFromURDF(urdf_path, urdf_dir)
                    follower.pin_robot.data = follower.pin_robot.model.createData()
                    print("      URDF loaded for gravity compensation")
            except Exception as e:
                print(f"      Could not load URDF: {e}")
                print("      ─────────────────────────────────────────────")
                print("      WARNING: Gravity compensation DISABLED.")
                print("      The arm will work but may sag under its own weight.")
                print("")
                print("      To enable gravity compensation, install lerobot and copy meshes:")
                print("        pip install lerobot")
                print("        cp -r \\")
                print("          $(python3 -c 'import lerobot.robots.openarms as p,os; print(os.path.dirname(p.__file__))')/urdf/meshes/* \\")
                print("          lerobot/robots/openarms/urdf/meshes/")
                print("      ─────────────────────────────────────────────")

        follower.connect(calibrate=True)
        print("      Calibrated")

        follower.bus_right.enable_torque()
        follower.bus_left.enable_torque()
        time.sleep(0.2)
        print("      Torque enabled")

        # 2. Home
        print("\n[2/3] Moving to home position...")
        _move_to_home(follower, cfg, hold_seconds=5.0)
        _test_grippers(follower, cfg)
        print("      Home position reached")

        # 3. Wait for ENTER
        print("\n[3/3] Press ENTER to connect to WebSocket and start control...")
        input()

        # 4. Stream
        print(f"\nConnecting to {ws_uri} ...")
        async with websockets.connect(ws_uri) as ws:
            print("Connected! Ctrl+C to stop.\n")

            # Start ROS2 publisher in background
            if start_publisher:
                publisher_proc = _start_publisher(sys.executable, str(Path(__file__).resolve().parent / "config"))

            count = 0
            t0 = time.perf_counter()
            last_print = t0
            target_period = 1.0 / fps
            latest = {"left": [], "right": [], "left_gripper_deg": None, "right_gripper_deg": None}
            last_msg_time = time.perf_counter()
            prev_cmd: dict = {}  # seeded on first frame (same as ws_follower_arm_control.py)

            # Background task that continuously reads WebSocket messages
            # and updates the shared 'latest' dict with the newest joint commands.
            # This runs concurrently with the motor control loop — the reader
            # never blocks the motor loop, and the motor loop always reads the
            # freshest data without waiting for the next WS message.
            async def ws_reader():
                nonlocal last_msg_time
                async for msg in ws:
                    data = json.loads(msg)
                    latest["left"] = data.get("left", [])
                    latest["right"] = data.get("right", [])
                    latest["left_gripper_deg"] = data.get("left_gripper_deg")
                    latest["right_gripper_deg"] = data.get("right_gripper_deg")
                    last_msg_time = time.perf_counter()  # heartbeat for timeout detection

            reader = asyncio.create_task(ws_reader())

            try:
                timeout_start = None  # non-None when we are in timeout → return-to-home
                timeout_start_pos = {}  # snapshot of joint positions at timeout start
                while True:
                    loop_start = time.perf_counter()

                    # ---- SAFETY: stream timeout — no messages → return to home ----
                    # If the IK server stops sending data (crash, network drop),
                    # we interpolate all joints back to home over return_home_s seconds.
                    # This prevents the arm from freezing in its last commanded position.
                    if time.perf_counter() - last_msg_time > stream_timeout:
                        if timeout_start is None:
                            timeout_start = now
                            obs = follower.get_observation()
                            timeout_start_pos = {
                                f"{side}_{motor}": np.rad2deg(obs.get(f"{side}_{motor}.pos", 0.0))
                                for side, bus in [("right", follower.bus_right), ("left", follower.bus_left)]
                                for motor in bus.motors
                            }
                            print("  Stream timeout — returning to home...")

                        frac = min(1.0, (now - timeout_start) / return_home_s)
                        follower_obs = follower.get_observation()
                        gravity = _compute_gravity(follower, follower_obs)

                        for side, bus in [("right", follower.bus_right), ("left", follower.bus_left)]:
                            for motor in bus.motors:
                                joint_name = f"{side}_{motor}"
                                target = gripper_open if motor == "gripper" else home.get(joint_name, 0.0)
                                pos_deg = timeout_start_pos[joint_name] + (target - timeout_start_pos[joint_name]) * frac
                                kp = _get_kp(cfg, motor)
                                kd = _get_kd(cfg, motor)
                                torque = gravity.get(joint_name, 0.0)
                                bus._mit_control(
                                    motor=motor, kp=kp, kd=kd,
                                    position_degrees=pos_deg, velocity_deg_per_sec=0.0, torque=torque,
                                )
                    else:
                        # ---- NORMAL: data is flowing — drive motors ----
                        if timeout_start is not None:
                            print("  Stream resumed")
                            timeout_start = None
                            prev_cmd.clear()  # reset clamp tracking after a gap

                        # Clamp per-joint deltas for safety — no joint can change
                        # more than MAX_JOINT_DELTA_DEG per cycle (~90 deg/s at 30 Hz)
                        left_deg, right_deg, left_grip, right_grip = _clamp_joint_deltas(
                            latest["left"], latest["right"],
                            latest.get("left_gripper_deg"), latest.get("right_gripper_deg"),
                            prev_cmd, cfg,
                        )
                        # Send MIT impedance commands to CAN bus motors
                        # Uses position + velocity + torque feed-forward (gravity comp)
                        _apply_joints(follower, left_deg, right_deg, left_grip, right_grip, cfg)

                    count += 1
                    now = time.perf_counter()
                    if now - last_print >= 1.0:
                        print(f"  {count} cmds | {count / (now - t0):.1f} Hz")
                        last_print = now

                    elapsed = time.perf_counter() - loop_start
                    if (sleep_time := target_period - elapsed) > 0:
                        await asyncio.sleep(sleep_time)
            finally:
                if not reader.done():
                    reader.cancel()
                try:
                    await reader
                except (asyncio.CancelledError, ConnectionClosed):
                    pass

    except ConnectionClosed:
        print("\nWebSocket connection closed by server.")
    finally:
        # Stop publisher first (before arm moves — it needs the CAN bus)
        _stop_publisher(publisher_proc)

        if follower is not None:
            print("\nReturning to home (3s)...")
            try:
                _move_to_home(follower, cfg, hold_seconds=return_home_s)
            except (Exception, KeyboardInterrupt):
                pass
            print("Disconnecting...")
            try:
                follower.bus_right.disable_torque()
                follower.bus_left.disable_torque()
                follower.disconnect()
            except (Exception, KeyboardInterrupt):
                pass
            print("Done.")


def main() -> None:
    parser = argparse.ArgumentParser(description="WS Follower Arm Controller")
    parser.add_argument("--config-dir", default="config", help="Config directory path")
    parser.add_argument("--ws-uri", default=None, help="WebSocket URI (default: from robot.yaml)")
    parser.add_argument("--no-publisher", action="store_true", help="Skip starting arm_state_publisher")
    args = parser.parse_args()

    config_dir = Path(args.config_dir)
    if not config_dir.is_absolute():
        config_dir = Path(__file__).resolve().parent / config_dir
    cfg = _load_config(config_dir)

    ws_uri = args.ws_uri or cfg.get("ws_uri", "ws://localhost:5200/ws/arm")

    # Suppress the noisy asyncio traceback on Ctrl+C — the cleanup in run()'s
    # finally block handles everything (return home, disable torque, disconnect).
    try:
        asyncio.run(run(cfg, ws_uri, start_publisher=not args.no_publisher))
    except KeyboardInterrupt:
        pass
    print("")


if __name__ == "__main__":
    main()
