"""Snapshot builder — constructs the JSON snapshot sent to the webui-monitor."""

from __future__ import annotations

import time
from typing import Any

from server.config_loader import AppConfig
from server.ik_service import IKService, SolveResult


def build_snapshot(
    solve_result: SolveResult | None,
    config: AppConfig,
    solve_index: int,
    solver_status: dict[str, Any],
    ik_service: IKService | None = None,
    vr_status: dict[str, Any] | None = None,
    control_source: str = "idle",
) -> dict[str, Any]:
    """Build the full-snapshot JSON for the webui-monitor."""
    now_ns = time.time_ns()

    # When idle, use the IK service's current joint state (home after reset)
    idle_joints: list[float] = []
    if ik_service is not None and solve_result is None:
        idle_joints = ik_service.home_joint_radians

    snapshot: dict[str, Any] = {
        "type": "snapshot",
        "meta": {
            "version": config.meta.version,
            "timestamp_ns": now_ns,
            "solve_index": solve_index,
        },
        "solver": solver_status,
        "control_source": control_source,
        "arms": {
            "left": _arm_state(solve_result, "left", idle_joints, ik_service),
            "right": _arm_state(solve_result, "right", idle_joints, ik_service),
        },
    }

    # VR status
    if vr_status is not None:
        snapshot["vr_status"] = vr_status

    # Add audit summary if we have a solve result
    if solve_result is not None:
        snapshot["audit"] = {
            "last_solve_time_ms": solve_result.solve_time_ms,
            "total_solves": solve_index,
        }

    return snapshot


def _arm_state(
    result: SolveResult | None,
    side: str,
    idle_joints: list[float] | None = None,
    ik_service: IKService | None = None,
) -> dict[str, Any]:
    """Extract arm state from a SolveResult for one side."""
    if result is None:
        # Use idle joints from ik_service (home after reset, 9 per side)
        if idle_joints and len(idle_joints) >= 18:
            is_left = side == "left"
            joint_rads = idle_joints[:9] if is_left else idle_joints[9:18]
        else:
            joint_rads = []

        # Show FK home EE position as target when calibrated (READY state)
        target_pos = [0, 0, 0]
        target_quat = [1, 0, 0, 0]
        fk_pos = [0, 0, 0]
        fk_quat = [1, 0, 0, 0]
        if ik_service is not None and side in ik_service.home_fk:
            home_ee = ik_service.home_fk[side]
            fk_pos = home_ee.translation().tolist()
            fk_quat = home_ee.rotation().wxyz.tolist()
            target_pos = fk_pos
            target_quat = fk_quat

        return {
            "target_active": False,
            "target_position": target_pos,
            "target_quaternion_wxyz": target_quat,
            "joint_radians": joint_rads,
            "fk_position": fk_pos,
            "fk_quaternion_wxyz": fk_quat,
            "position_error_mm": 0.0,
            "orientation_error_deg": 0.0,
            "solve_time_ms": 0.0,
            "success": True,
            "failure_reason": None,
        }

    is_left = side == "left"
    return {
        "target_active": result.target_left_active if is_left else result.target_right_active,
        "target_position": result.target_left_position if is_left else result.target_right_position,
        "target_quaternion_wxyz": result.target_left_quaternion_wxyz if is_left else result.target_right_quaternion_wxyz,
        "joint_radians": result.joint_radians_left if is_left else result.joint_radians_right,
        "fk_position": result.fk_left_position if is_left else result.fk_right_position,
        "fk_quaternion_wxyz": result.fk_left_quaternion_wxyz if is_left else result.fk_right_quaternion_wxyz,
        "position_error_mm": result.position_error_mm.get(side, 0.0),
        "orientation_error_deg": result.orientation_error_deg.get(side, 0.0),
        "solve_time_ms": result.solve_time_ms,
        "success": result.success,
        "failure_reason": result.failure_reason,
    }
