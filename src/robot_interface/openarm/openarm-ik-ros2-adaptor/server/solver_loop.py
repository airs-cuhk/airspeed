"""Solver loop — 50 Hz async loop that reads VR targets, solves IK, publishes to ROS2 and WS arm."""

from __future__ import annotations

import asyncio
import json
import time
from typing import Any

from aiohttp import web

from server.config_loader import AppConfig
from server.ik_service import IKService, parse_target
from server.vr_subscriber import VRSubscriber
from server.vr_normalizer import VRNormalizer
from server.ros2_publisher import ArmCommandPublisher

SOLVE_PERIOD_S = 1.0 / 50.0


async def solver_loop(
    app: web.Application,
    ik_service: IKService,
    config: AppConfig,
    vr_subscriber: VRSubscriber | None = None,
    vr_normalizer: VRNormalizer | None = None,
    ros2_publisher: ArmCommandPublisher | None = None,
) -> None:
    """50 Hz loop: read latest VR target → solve IK → publish joint commands."""
    print("[solver_loop] Starting 50 Hz solver loop")

    while True:
        t_start = time.monotonic()

        # Check for reset_home request
        if app.get("reset_home_requested"):
            app["reset_home_requested"] = False
            ik_service.reset_to_home()
            if vr_normalizer is not None and vr_subscriber is not None:
                head_pose = vr_subscriber.data_store.get_head_pose()
                if head_pose is not None:
                    vr_normalizer.pin_and_calibrate(
                        head_pose, vr_subscriber.data_store, ik_service.home_fk
                    )
                else:
                    vr_normalizer._reset()

        target_L = None
        target_R = None
        left_trigger = 0.0
        right_trigger = 0.0

        # VR normalizer provides SE3 targets
        if vr_normalizer is not None and vr_subscriber is not None:
            prev_state = vr_normalizer.state.value
            vr_result = vr_normalizer.normalize(vr_subscriber.data_store)
            if vr_result.state.value == "ready" and prev_state != "ready":
                ik_service.reset_to_home()
            if vr_result.state.value == "active":
                target_L = vr_result.left
                target_R = vr_result.right
                left_trigger = vr_result.left_trigger
                right_trigger = vr_result.right_trigger

        # Solve if we have a target
        if target_L is not None or target_R is not None:
            result = await ik_service.solve_targets(target_L, target_R)
            app["solve_index"] = app.get("solve_index", 0) + 1
        else:
            result = None

        # Override finger joints with trigger-based gripper
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

        # Publish joint commands to ROS2 + WS arm
        if (
            result is not None
            and result.success
            and len(result.joint_radians_left) == 9
            and len(result.joint_radians_right) == 9
        ):
            joint_commands = result.joint_radians_left + result.joint_radians_right
            if ros2_publisher is not None:
                ros2_publisher.publish(
                    left_joints=result.joint_radians_left[:8],
                    right_joints=result.joint_radians_right[:8],
                    left_target_xyz=result.target_left_position if result.target_left_active else None,
                    left_target_quat_xyzw=_wxyz_to_xyzw(result.target_left_quaternion_wxyz) if result.target_left_active else None,
                    right_target_xyz=result.target_right_position if result.target_right_active else None,
                    right_target_quat_xyzw=_wxyz_to_xyzw(result.target_right_quaternion_wxyz) if result.target_right_active else None,
                )
            await _broadcast_arm(app, joint_commands, left_trigger, right_trigger)

        # Maintain 50 Hz cadence
        elapsed = time.monotonic() - t_start
        if (sleep_time := SOLVE_PERIOD_S - elapsed) > 0:
            await asyncio.sleep(sleep_time)


async def _broadcast_arm(
    app: web.Application,
    joint_commands: list[float],
    left_trigger: float = 0.0,
    right_trigger: float = 0.0,
) -> None:
    """Send joint commands + gripper state to arm WebSocket clients."""
    arm_websockets: set[web.WebSocketResponse] = app.get("arm_websockets", set())
    if not arm_websockets:
        return
    left_gripper_deg = -65.0 * (1.0 - left_trigger)
    right_gripper_deg = -65.0 * (1.0 - right_trigger)
    data = json.dumps({
        "type": "arm_commands",
        "joints": joint_commands,
        "left": joint_commands[:9],
        "right": joint_commands[9:],
        "left_gripper_deg": left_gripper_deg,
        "right_gripper_deg": right_gripper_deg,
    })
    for ws in list(arm_websockets):
        if not ws.closed:
            try:
                await ws.send_str(data)
            except Exception:
                pass


def _wxyz_to_xyzw(wxyz: list[float]) -> list[float]:
    return [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]
