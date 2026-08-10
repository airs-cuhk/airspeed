"""Solver loop — 50 Hz async loop that reads targets and broadcasts snapshots."""

from __future__ import annotations

import asyncio
import json
import math
import time
from typing import Any

from aiohttp import web

from server.config_loader import AppConfig
from server.ik_service import IKService, parse_target
from server.snapshot import build_snapshot
from server.target_buffer import TargetBuffer
from server.vr_subscriber import VRSubscriber
from server.vr_normalizer import VRNormalizer
from server.ros2_publisher import ArmCommandPublisher

# 50 Hz = 20 ms period
SOLVE_PERIOD_S = 1.0 / 50.0


async def solver_loop(
    app: web.Application,
    ik_service: IKService,
    buffer: TargetBuffer,
    config: AppConfig,
    vr_subscriber: VRSubscriber | None = None,
    vr_normalizer: VRNormalizer | None = None,
    ros2_publisher: ArmCommandPublisher | None = None,
) -> None:
    """50 Hz loop: read latest target → solve → broadcast snapshot.

    Target sources (priority order):
    1. VR normalizer (active calibration state)
    2. TargetBuffer (legacy drag)
    3. Idle (no target)
    """
    print("[solver_loop] Starting 50 Hz solver loop")

    while True:
        t_start = time.monotonic()

        # Check for reset_home request
        if app.get("reset_home_requested"):
            app["reset_home_requested"] = False
            ik_service.reset_to_home()
            # Pin origin + calibrate from current VR positions (same as B-button)
            if vr_normalizer is not None and vr_subscriber is not None:
                head_pose = vr_subscriber.data_store.get_head_pose()
                if head_pose is not None:
                    vr_normalizer.pin_and_calibrate(
                        head_pose, vr_subscriber.data_store, ik_service.home_fk
                    )
                else:
                    vr_normalizer._reset(hard=True)

        control_source = "idle"
        target_L = None
        target_R = None
        left_trigger = 0.0
        right_trigger = 0.0

        # Priority 1: VR normalizer
        if vr_normalizer is not None and vr_subscriber is not None:
            prev_state = vr_normalizer.state.value
            vr_result = vr_normalizer.normalize(vr_subscriber.data_store)
            # B-button pin also resets arms to home
            if vr_result.state.value == "ready" and prev_state != "ready":
                ik_service.reset_to_home()
            if vr_result.state.value == "active":
                target_L = vr_result.left
                target_R = vr_result.right
                left_trigger = vr_result.left_trigger
                right_trigger = vr_result.right_trigger
                control_source = "vr"

        # Gripper command in degrees: trigger=0 → open (-65°), trigger=1 → closed (0°).
        # Degrees are the motor-control unit (WebSocket → arm_controller → damiao).
        # The ROS2 command topic gets radians instead (math.radians at publish time)
        # so the recorded command matches the recorded joint_state units.
        left_gripper_deg = -65.0 * (1.0 - left_trigger)
        right_gripper_deg = -65.0 * (1.0 - right_trigger)

        # Priority 2: Legacy target buffer
        if control_source == "idle":
            target_data = buffer.read()
            if target_data is not None:
                left_data = target_data.get("left", {})
                right_data = target_data.get("right", {})
                target_L = parse_target(left_data)
                target_R = parse_target(right_data)
                if target_L is not None or target_R is not None:
                    control_source = "drag"

        # Solve if we have a target
        if target_L is not None or target_R is not None:
            result = await ik_service.solve_targets(target_L, target_R)
            app["solve_index"] = app.get("solve_index", 0) + 1
        else:
            result = None

        # Build VR status (with poses mapped to robot base frame)
        vr_status = None
        if vr_subscriber is not None:
            raw_status = vr_subscriber.data_store.get_status()
            # Map pose positions/orientations from VR native to robot base frame,
            # applying origin rebasing if the normalizer has pinned an origin.
            mapped_poses = _map_vr_poses(
                raw_status.get("poses", {}), config, vr_normalizer
            )
            vr_status = {
                **raw_status,
                "poses": mapped_poses,
                "ros2_installed": vr_subscriber.ros2_installed,
                "subscriber_running": vr_subscriber.is_running,
                "calibration": _map_calibration(
                    vr_normalizer.get_status() if vr_normalizer else {},
                    config,
                ),
            }

        # Override finger joints for visualization/snapshot only.
        # The real control path uses 7 arm joints (radians) plus the separate
        # gripper degree fields computed above.
        # trigger=0 → gripper open → URDF 0.044m, trigger=1 → closed → URDF 0.0m
        if (
            result is not None
            and result.success
            and len(result.joint_radians_left) == 9
            and len(result.joint_radians_right) == 9
        ):
            result.joint_radians_left[7] = 0.044 * (1.0 - left_trigger)
            result.joint_radians_left[8] = 0.044 * (1.0 - left_trigger)
            result.joint_radians_right[7] = 0.044 * (1.0 - right_trigger)
            result.joint_radians_right[8] = 0.044 * (1.0 - right_trigger)

        # Build and broadcast snapshot
        snapshot = build_snapshot(
            solve_result=result,
            config=config,
            solve_index=app.get("solve_index", 0),
            solver_status={
                "warmup_complete": ik_service.solver.warmup_complete,
                "warmup_error": ik_service.solver.warmup_error,
            },
            ik_service=ik_service,
            vr_status=vr_status,
            control_source=control_source,
        )

        await _broadcast(app, snapshot)

        # Publish joint commands to ROS2 + WS arm
        if (
            result is not None
            and result.success
            and len(result.joint_radians_left) == 9
            and len(result.joint_radians_right) == 9
        ):
            # ROS2 path: all values in radians so the recorded
            # ik_*_joint_commands match the arm_*_joint_state units.
            # Gripper sessions: 8 values (7 arm + gripper). Dexterous-hand
            # sessions (gripper_enabled: false): 7 arm joints only — the
            # trigger drives the ROH hand, not an OpenArm gripper.
            if ros2_publisher is not None:
                if config.robot.gripper_enabled:
                    left_cmd = result.joint_radians_left[:7] + [math.radians(left_gripper_deg)]
                    right_cmd = result.joint_radians_right[:7] + [math.radians(right_gripper_deg)]
                else:
                    left_cmd = result.joint_radians_left[:7]
                    right_cmd = result.joint_radians_right[:7]
                ros2_publisher.publish(
                    left_joints=left_cmd,
                    right_joints=right_cmd,
                    left_target_xyz=result.target_left_position if result.target_left_active else None,
                    left_target_quat_xyzw=_wxyz_to_xyzw(result.target_left_quaternion_wxyz) if result.target_left_active else None,
                    right_target_xyz=result.target_right_position if result.target_right_active else None,
                    right_target_quat_xyzw=_wxyz_to_xyzw(result.target_right_quaternion_wxyz) if result.target_right_active else None,
                )
            # WS control path: 7 arm joints + separate gripper degree fields.
            # The full 18-DOF joint_commands array is kept only for snapshot/visualization.
            joint_commands = result.joint_radians_left + result.joint_radians_right
            await _broadcast_arm(
                app,
                result.joint_radians_left[:7],
                result.joint_radians_right[:7],
                left_gripper_deg,
                right_gripper_deg,
            )

        # Maintain 50 Hz cadence
        elapsed = time.monotonic() - t_start
        sleep_time = SOLVE_PERIOD_S - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)


async def _broadcast(app: web.Application, snapshot: dict[str, Any]) -> None:
    """Send snapshot JSON to all connected WebSocket clients."""
    websockets: set[web.WebSocketResponse] = app.get("websockets", set())
    data = json.dumps(snapshot)
    for ws in list(websockets):
        if not ws.closed:
            try:
                await ws.send_str(data)
            except Exception:
                pass


async def _broadcast_arm(
    app: web.Application,
    left_joints: list[float],
    right_joints: list[float],
    left_gripper_deg: float = 0.0,
    right_gripper_deg: float = 0.0,
) -> None:
    """Send joint commands + gripper state to all arm WebSocket clients.

    Args:
        left_joints: 7 arm joint positions in radians.
        right_joints: 7 arm joint positions in radians.
        left_gripper_deg: Left gripper target in degrees (0 = closed, -65 = open).
        right_gripper_deg: Right gripper target in degrees.
    """
    arm_websockets: set[web.WebSocketResponse] = app.get("arm_websockets", set())
    if not arm_websockets:
        return
    data = json.dumps({
        "type": "arm_commands",
        "joints": left_joints + right_joints,
        "left": left_joints,
        "right": right_joints,
        "left_gripper_deg": left_gripper_deg,
        "right_gripper_deg": right_gripper_deg,
    })
    for ws in list(arm_websockets):
        if not ws.closed:
            try:
                await ws.send_str(data)
            except Exception:
                pass


def _map_vr_poses(
    raw_poses: dict[str, Any],
    config: AppConfig,
    vr_normalizer: VRNormalizer | None = None,
) -> dict[str, Any]:
    """Map VR pose positions and orientations from native frame to robot base frame.

    Two-stage pipeline:
    1. Rebase into pinned VR origin frame (if normalizer has pinned an origin)
    2. Apply axis_mapping matrix (native → robot base frame)

    This ensures the webui-monitor and telemetry see coordinates in the pinned frame,
    matching what the IK solver receives.
    """
    import jax.numpy as jnp
    import jaxlie

    mat = jnp.array(config.vr.axis_mapping.native_to_intermediate)
    scale = config.vr.axis_mapping.position_scale
    origin = vr_normalizer._vr_origin if vr_normalizer is not None else None

    mapped: dict[str, Any] = {}
    for side, pose_data in raw_poses.items():
        entry = dict(pose_data)
        pos = pose_data.get("position")
        quat = pose_data.get("orientation_wxyz")

        if pos is not None and not pose_data.get("stale", True):
            # Stage 1: Rebase into pinned origin frame
            if origin is not None and quat is not None:
                controller_se3 = jaxlie.SE3.from_rotation_and_translation(
                    jaxlie.SO3(jnp.array(quat)),
                    jnp.array(pos),
                )
                rebased = origin.inverse() @ controller_se3
                pos = rebased.translation().tolist()
                quat = rebased.rotation().wxyz.tolist()

            # Stage 2: Apply axis mapping
            mapped_pos = (mat @ jnp.array(pos) * scale).tolist()
            entry["position"] = mapped_pos

            if quat is not None:
                rotation = jaxlie.SO3(jnp.array(quat))
                basis = jaxlie.SO3.from_matrix(mat)
                mapped_rot = basis @ rotation @ basis.inverse()
                entry["orientation_wxyz"] = mapped_rot.wxyz.tolist()

        mapped[side] = entry
    return mapped


def _wxyz_to_xyzw(wxyz: list[float]) -> list[float]:
    """Convert quaternion from WXYZ (scalar-first) to XYZW (scalar-last)."""
    return [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]


def _map_calibration(
    cal: dict[str, Any],
    config: AppConfig,
) -> dict[str, Any]:
    """Map VR origin position/orientation from VR native to robot base frame."""
    if not cal.get("origin_position"):
        return cal
    import jax.numpy as jnp
    import jaxlie

    mapped = dict(cal)
    mat = jnp.array(config.vr.axis_mapping.native_to_intermediate)
    scale = config.vr.axis_mapping.position_scale

    pos = jnp.array(cal["origin_position"])
    mapped["origin_position"] = (mat @ pos * scale).tolist()

    if cal.get("origin_quaternion_wxyz"):
        rot = jaxlie.SO3(jnp.array(cal["origin_quaternion_wxyz"]))
        basis = jaxlie.SO3.from_matrix(mat)
        mapped_rot = basis @ rot @ basis.inverse()
        mapped["origin_quaternion_wxyz"] = mapped_rot.wxyz.tolist()

    return mapped
