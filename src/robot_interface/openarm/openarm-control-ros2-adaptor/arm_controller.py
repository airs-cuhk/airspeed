"""WebSocket Follower Arm Controller — receives IK commands, drives motors."""

from __future__ import annotations

import os
import sys
_lerobot_src = os.environ.get("LEROBOT_SRC", "")
# LEROBOT_SRC may point at a stock upstream lerobot checkout (e.g. for the
# camera adaptor) that has no openarms robot — fall back to the bundled fork.
if not _lerobot_src or not os.path.isdir(
    os.path.join(_lerobot_src, "lerobot", "robots", "openarms")
):
    _lerobot_src = os.path.dirname(os.path.abspath(__file__))  # project root, contains lerobot/
if _lerobot_src not in sys.path:
    sys.path.insert(0, _lerobot_src)

import argparse
import asyncio
import json
import os
import threading
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

    # Only test sides that actually have an OpenArm gripper motor — in
    # dexterous-hand sessions the gripper motor is absent from the bus.
    sides = [
        (side, bus)
        for side, bus in [("right", follower.bus_right), ("left", follower.bus_left)]
        if "gripper" in bus.motors
    ]
    if not sides:
        print("      Grippers disabled (dexterous-hand session) — skipping gripper test")
        return
    print("      Testing grippers (open → hold)...")

    start_obs = follower.get_observation()
    start_pos = {
        side: np.rad2deg(start_obs.get(f"{side}_gripper.pos", 0.0))
        for side, _bus in sides
    }

    # Interpolate grippers from current position to open over open_seconds
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < open_seconds:
        frac = min(1.0, (time.perf_counter() - t0) / open_seconds)
        for side, bus in sides:
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
        for _side, bus in sides:
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


def _apply_joints(follower, left_deg, right_deg, left_grip_deg, right_grip_deg, cfg: dict,
                  gravity: dict) -> dict:
    """Send one batched MIT command round per bus (7 joints + gripper when
    commanded) and return the decoded status replies keyed by side-prefixed
    motor name (e.g. "left_joint_1") with degree units, as sync_read_all_states.
    Batch replies ARE fresh motor states at command time — the caller merges
    them into the observation cache instead of a separate read round."""
    commands_right: dict = {}
    for i, motor in enumerate(follower.bus_right.motors):
        if i >= 7:
            break
        if i < len(right_deg):
            commands_right[motor] = (
                _get_kp(cfg, motor), _get_kd(cfg, motor),
                float(right_deg[i]), 0.0, gravity.get(f"right_{motor}", 0.0),
            )
    if right_grip_deg is not None and "gripper" in follower.bus_right.motors:
        commands_right["gripper"] = (
            _get_kp(cfg, "gripper"), _get_kd(cfg, "gripper"),
            right_grip_deg, 0.0, 0.0,
        )

    commands_left: dict = {}
    for i, motor in enumerate(follower.bus_left.motors):
        if i >= 7:
            break
        if i < len(left_deg):
            commands_left[motor] = (
                _get_kp(cfg, motor), _get_kd(cfg, motor),
                float(left_deg[i]), 0.0, gravity.get(f"left_{motor}", 0.0),
            )
    if left_grip_deg is not None and "gripper" in follower.bus_left.motors:
        commands_left["gripper"] = (
            _get_kp(cfg, "gripper"), _get_kd(cfg, "gripper"),
            left_grip_deg, 0.0, 0.0,
        )

    states: dict = {}
    if commands_right:
        for m, st in follower.bus_right._mit_control_batch(commands_right).items():
            states[f"right_{m}"] = st
    if commands_left:
        for m, st in follower.bus_left._mit_control_batch(commands_left).items():
            states[f"left_{m}"] = st
    return states


def _get_kp(cfg: dict, motor: str) -> float:
    kp = cfg.get("kp", {})
    return kp.get(motor, 25.0)


def _get_kd(cfg: dict, motor: str) -> float:
    kd = cfg.get("kd", {})
    return kd.get(motor, 0.3)


def _validate_gains(cfg: dict) -> None:
    """Fail fast if configured gains exceed the Damiao MIT wire ranges.

    The protocol packs kp into 0..500 and kd into 0..5 (12 bits); anything
    beyond is clamped at encode time. Refuse to start instead of letting the
    file say one thing while the wire carries another — the configuration is
    the single source of truth for gains.
    """
    from lerobot.motors.damiao.damiao import MIT_KD_RANGE, MIT_KP_RANGE
    bad = []
    for motor, v in cfg.get("kp", {}).items():
        if not MIT_KP_RANGE[0] <= float(v) <= MIT_KP_RANGE[1]:
            bad.append(f"kp.{motor}={v} (allowed {MIT_KP_RANGE[0]}..{MIT_KP_RANGE[1]})")
    for motor, v in cfg.get("kd", {}).items():
        if not MIT_KD_RANGE[0] <= float(v) <= MIT_KD_RANGE[1]:
            bad.append(f"kd.{motor}={v} (allowed {MIT_KD_RANGE[0]}..{MIT_KD_RANGE[1]})")
    if bad:
        raise ValueError(
            "Gains outside the Damiao MIT protocol range — fix config/robot.yaml: "
            + "; ".join(bad))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def run(cfg: dict, ws_uri: str, *, start_publisher: bool = True) -> None:
    _validate_gains(cfg)
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
    state_pub = None  # in-process joint-state publisher
    gravity_stop = None  # set to stop the background gravity thread
    gravity_thread = None
    try:
        # 1. Connect + calibrate
        follower_config = OpenArmsFollowerConfig(
            port_left=can_left, port_right=can_right, can_interface=can_iface,
            id="openarms_follower", disable_torque_on_disconnect=True, max_relative_target=5.0,
            left_gripper_enabled=cfg.get("left_gripper_enabled", True),
            right_gripper_enabled=cfg.get("right_gripper_enabled", True),
        )
        print("\n[1/3] Connecting follower arm...")
        follower = OpenArmsFollower(follower_config)

        # Auto-copy meshes from the IK adaptor if missing here.
        # Both adaptors use the same mesh files — no need to download twice.
        _mesh_src = (
            Path(__file__).resolve().parent.parent / "openarm-ik-ros2-adaptor"
            / "webui-monitor" / "3d_assets" / "urdf" / "meshes"
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
                print("      Meshes should be at ../openarm-ik-ros2-adaptor/webui-monitor/3d_assets/urdf/meshes/")
                print("      They are auto-copied to lerobot on first start.")
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

            # Start joint-state publishing (in-process: single CAN socket
            # owner — no stale-frame backlog). If it cannot start (e.g. ROS2
            # not sourced), continue without publishing rather than risking
            # the control loop.
            if start_publisher:
                try:
                    from joint_state_publisher import ArmJointStatePublisher
                    state_pub = ArmJointStatePublisher(cfg)
                    state_pub.start()
                    print("  [publisher] In-process joint-state publisher started")
                except Exception as e:
                    print(f"  [publisher] Unavailable ({e}) — continuing without state publishing")
                    state_pub = None

            pub_err = {"n": 0, "last": 0.0}

            def _publish_state(obs: dict) -> None:
                """Best-effort joint-state publish, stamped at call time.

                Observability must never stop the arm: any failure on this
                path is logged (throttled) and the control loop continues.
                Silence instead of exceptions is caught by the collector's
                stream tracker (state stream goes stale/absent).
                """
                if state_pub is None:
                    return
                try:
                    state_pub.publish(obs, state_pub.stamp_now())
                except Exception as e:
                    pub_err["n"] += 1
                    now_m = time.monotonic()
                    if now_m - pub_err["last"] >= 5.0:
                        print(f"  [publisher] publish failed x{pub_err['n']} "
                              f"({type(e).__name__}: {e}) — continuing")
                        pub_err["last"] = now_m

            count = 0
            t0 = time.perf_counter()
            last_print = t0
            target_period = 1.0 / fps
            latest = {"left": [], "right": [], "left_gripper_deg": None, "right_gripper_deg": None}
            last_msg_time = time.perf_counter()
            now = t0  # initialized up front: the first-iteration stream-timeout
            # path below references `now` before the end-of-loop assignment
            prev_cmd: dict = {}  # seeded from the arm's ACTUAL pose (below)
            # Observation cache (get_observation layout, radians): seeded by a
            # full read, then updated in place by the batched MIT replies —
            # no separate read round in the steady-state loop.
            last_obs: dict | None = None

            # Gravity feed-forward runs in a background thread at a slow rate:
            # pinocchio RNE over 14 joints every 33 ms cycle is wasted work on
            # a slowly-varying quantity. The loop reads the cached snapshot
            # under a lock; the worker recomputes on a COPY of the latest obs.
            gravity_hz = float(cfg.get("gravity_hz", 5.0))
            gravity_lock = threading.Lock()
            gravity_cache: dict = {
                "torque": _compute_gravity(follower, follower.get_observation())}
            gravity_stop = threading.Event()

            def _gravity_worker() -> None:
                while not gravity_stop.is_set():
                    obs = last_obs
                    if obs is not None:
                        try:
                            torques = _compute_gravity(follower, dict(obs))
                            with gravity_lock:
                                gravity_cache["torque"] = torques
                        except Exception:
                            pass  # keep last good snapshot
                    gravity_stop.wait(1.0 / gravity_hz)

            gravity_thread = threading.Thread(
                target=_gravity_worker, daemon=True, name="gravity-comp")
            gravity_thread.start()

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
                        _publish_state(follower_obs)
                        with gravity_lock:
                            gravity = gravity_cache["torque"]

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
                            last_obs = None  # force a fresh observation read

                        if last_obs is None:
                            last_obs = follower.get_observation()
                        if not prev_cmd:
                            # Seed clamp tracking from the arm's ACTUAL pose —
                            # otherwise the first streamed frame is sent
                            # unclamped and the arm jumps from home to the
                            # leader pose at full speed (abrupt start move).
                            # Seeding ramps it to the stream at
                            # max_joint_delta_deg per cycle instead.
                            for side, bus in [("left", follower.bus_left), ("right", follower.bus_right)]:
                                prev_cmd[f"{side}_joints"] = [
                                    np.rad2deg(last_obs.get(f"{side}_{motor}.pos", 0.0))
                                    for motor in list(bus.motors)[:7]
                                ]
                                if "gripper" in bus.motors:
                                    prev_cmd[f"{side}_gripper"] = np.rad2deg(
                                        last_obs.get(f"{side}_gripper.pos", 0.0))

                        # Clamp per-joint deltas for safety — no joint can change
                        # more than MAX_JOINT_DELTA_DEG per cycle (~90 deg/s effective)
                        left_deg, right_deg, left_grip, right_grip = _clamp_joint_deltas(
                            latest["left"], latest["right"],
                            # Ignore gripper commands for sides whose gripper
                            # motor is disabled (dexterous-hand session).
                            latest.get("left_gripper_deg") if "gripper" in follower.bus_left.motors else None,
                            latest.get("right_gripper_deg") if "gripper" in follower.bus_right.motors else None,
                            prev_cmd, cfg,
                        )
                        # Send MIT impedance commands — one batched round per bus.
                        # Gravity feed-forward uses the latest decoded bus states
                        # (first cycle after connect/resume reads a full
                        # observation); the batch replies are this cycle's fresh
                        # joint state and feed publishing + the next gravity pass.
                        with gravity_lock:
                            gravity = gravity_cache["torque"]
                        for key, st in _apply_joints(
                                follower, left_deg, right_deg, left_grip, right_grip,
                                cfg, gravity).items():
                            last_obs[f"{key}.pos"] = np.deg2rad(st["position"])
                            last_obs[f"{key}.vel"] = np.deg2rad(st["velocity"])
                            last_obs[f"{key}.torque"] = st["torque"]
                        _publish_state(last_obs)

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
        # Stop background workers first — they touch the CAN bus / ROS graph
        # and must be quiet before the arm moves home and disconnects.
        # Every step tolerates a second Ctrl+C arriving mid-cleanup
        # (KeyboardInterrupt is BaseException, not Exception).
        if gravity_stop is not None:
            gravity_stop.set()
        if gravity_thread is not None:
            try:
                gravity_thread.join(timeout=2.0)  # daemon thread: dies with process regardless
            except (Exception, KeyboardInterrupt):
                pass
        if state_pub is not None:
            try:
                state_pub.stop()
            except (Exception, KeyboardInterrupt):
                pass

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
    parser.add_argument("--no-publisher", action="store_true",
                        help="Disable joint-state publishing")
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
