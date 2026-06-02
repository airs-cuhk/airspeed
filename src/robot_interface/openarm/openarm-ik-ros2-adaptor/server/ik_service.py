"""IK service — wraps OpenArmRobot + PyrokiSolver with config-driven cost weights."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any

import jax.numpy as jnp
import jaxlie

from server.config_loader import AppConfig
from server.audit_logger import AuditLogger


@dataclass
class SolveResult:
    success: bool
    failure_reason: str | None = None
    target_left_active: bool = False
    target_left_position: list[float] = field(default_factory=lambda: [0, 0, 0])
    target_left_quaternion_wxyz: list[float] = field(default_factory=lambda: [1, 0, 0, 0])
    target_right_active: bool = False
    target_right_position: list[float] = field(default_factory=lambda: [0, 0, 0])
    target_right_quaternion_wxyz: list[float] = field(default_factory=lambda: [1, 0, 0, 0])
    q_input: list[float] = field(default_factory=list)
    q_output: list[float] = field(default_factory=list)
    joint_radians_left: list[float] = field(default_factory=list)
    joint_radians_right: list[float] = field(default_factory=list)
    fk_left_position: list[float] = field(default_factory=lambda: [0, 0, 0])
    fk_left_quaternion_wxyz: list[float] = field(default_factory=lambda: [1, 0, 0, 0])
    fk_right_position: list[float] = field(default_factory=lambda: [0, 0, 0])
    fk_right_quaternion_wxyz: list[float] = field(default_factory=lambda: [1, 0, 0, 0])
    position_error_mm: dict[str, float] = field(default_factory=dict)
    orientation_error_deg: dict[str, float] = field(default_factory=dict)
    solve_time_ms: float = 0.0


class IKService:
    def __init__(self, config: AppConfig) -> None:
        from openarm_ik_solver import OpenArmRobot, PyrokiSolver
        import numpy as np

        self.config = config
        self.robot = OpenArmRobot(solver_config=config.solver)
        self.solver = PyrokiSolver(self.robot, solver_config=config.solver)

        # Use HOME_POSITION from config (degrees→radians) as initial joint state
        home_rad = config.robot.home_position_rad
        self.q_current = np.array(home_rad, dtype=np.float32)
        self._home_rad = home_rad
        self.audit_log = AuditLogger(max_entries=10000)

        # Compute FK at home for end-effector poses
        self._home_fk: dict[str, jaxlie.SE3] = {}
        self._compute_home_fk()

    def _compute_home_fk(self) -> None:
        """Compute forward kinematics at home position for both end effectors."""
        import numpy as np
        q_home = self._home_to_solver_order(np.array(self._home_rad, dtype=np.float32))
        fk = self.robot.forward_kinematics(q_home)
        for side in ("left", "right"):
            if fk.get(side) is not None:
                self._home_fk[side] = fk[side]

    @property
    def home_fk(self) -> dict[str, jaxlie.SE3]:
        """FK end-effector poses at the home configuration."""
        return self._home_fk

    @property
    def home_joint_radians(self) -> list[float]:
        """Home joint angles split per-side for frontend display.

        Returns [left_9, right_9] where each side has 7 arm + 2 finger joints.
        """
        home = list(self._home_rad)
        # Config order: [L1..7, L_grip, R1..7, R_grip]
        left_arm = home[:7]
        left_grip = home[7]
        right_arm = home[8:15]
        right_grip = home[15]
        left = left_arm + [left_grip, 0.0]    # 7 arm + 2 finger
        right = right_arm + [right_grip, 0.0]  # 7 arm + 2 finger
        return left + right

    def _home_to_solver_order(self, q_config: "np.ndarray") -> "np.ndarray":
        """Reorder from config order to URDF/solver actuated joint order.

        Config:  [L1..7, L_grip, R1..7, R_grip]
        Solver:  [L1..7, R1..7, L_finger1, R_finger1]
        """
        import numpy as np
        return np.array([
            *q_config[:7],    # L1..7
            *q_config[8:15],  # R1..7
            q_config[7],      # L_grip → L_finger1
            q_config[15],     # R_grip → R_finger1
        ], dtype=np.float32)

    def _solver_to_config_order(self, q_solver: "np.ndarray") -> "np.ndarray":
        """Reorder from URDF/solver order back to config order.

        Solver:  [L1..7, R1..7, L_finger1, R_finger1]
        Config:  [L1..7, L_grip, R1..7, R_grip]
        """
        import numpy as np
        return np.array([
            *q_solver[:7],    # L1..7
            q_solver[14],     # L_finger1 → L_grip
            *q_solver[7:14],  # R1..7
            q_solver[15],     # R_finger1 → R_grip
        ], dtype=np.float32)

    def reset_to_home(self) -> None:
        """Reset q_current to the configured HOME_POSITION."""
        import numpy as np
        self.q_current = np.array(self._home_rad, dtype=np.float32)

    async def solve_targets(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
    ) -> SolveResult:
        """Solve IK for the given targets, running the solver in an executor."""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None, self._solve_sync, target_L, target_R
        )

    def _solve_sync(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
    ) -> SolveResult:
        """Synchronous solve — called from executor to avoid blocking event loop."""
        import numpy as np
        t0 = time.perf_counter()

        # Choose warm-start: fixed home (deterministic) or previous output (responsive)
        if self.config.solver.use_home_warm_start:
            q_input = np.array(self._home_rad, dtype=np.float32)
        else:
            q_input = self.q_current
        q_input_list = q_input.tolist()

        try:
            # Solver expects URDF order; q_input is in config order
            q_solver_in = self._home_to_solver_order(q_input)
            q_out = self.solver.solve(target_L, target_R, None, q_solver_in)
            solve_time_ms = (time.perf_counter() - t0) * 1000
            q_output_list = q_out.tolist()

            # FK verification (solver output is already in solver order)
            fk = self.robot.forward_kinematics(q_out)

            # Only update q_current when NOT using fixed home warm-start
            if not self.config.solver.use_home_warm_start:
                self.q_current = self._solver_to_config_order(q_out)

            # Compute errors and FK data
            pos_err: dict[str, float] = {}
            ori_err: dict[str, float] = {}
            fk_pos: dict[str, list[float]] = {}
            fk_quat: dict[str, list[float]] = {}

            for side, target in [("left", target_L), ("right", target_R)]:
                fk_se3 = fk.get(side)
                if fk_se3 is not None:
                    fk_pos[side] = fk_se3.translation().tolist()
                    fk_quat[side] = fk_se3.rotation().wxyz.tolist()

                    if target is not None:
                        pos_diff = jnp.linalg.norm(
                            fk_se3.translation() - target.translation()
                        ) * 1000
                        pos_err[side] = float(pos_diff)

                        q_err = fk_se3.rotation().inverse() @ target.rotation()
                        angle = 2 * jnp.arccos(
                            jnp.clip(abs(q_err.wxyz[0]), 0, 1)
                        )
                        ori_err[side] = float(jnp.degrees(angle))
                    else:
                        pos_err[side] = 0.0
                        ori_err[side] = 0.0

            # Split joints for frontend: solver order [L1..7, R1..7, L_f1, R_f1]
            # Frontend expects [L1..7, L_f1, L_f2, R1..7, R_f1, R_f2]
            sol = q_output_list
            left_arm = sol[:7]      # L1..7
            right_arm = sol[7:14]   # R1..7
            left_f1 = sol[14]       # L_finger1
            right_f1 = sol[15]      # R_finger1
            joint_left = left_arm + [left_f1, 0.0]     # 9 values
            joint_right = right_arm + [right_f1, 0.0]   # 9 values

            result = SolveResult(
                success=True,
                failure_reason=None,
                target_left_active=target_L is not None,
                target_left_position=(
                    target_L.translation().tolist()
                    if target_L
                    else [0, 0, 0]
                ),
                target_left_quaternion_wxyz=(
                    target_L.rotation().wxyz.tolist()
                    if target_L
                    else [1, 0, 0, 0]
                ),
                target_right_active=target_R is not None,
                target_right_position=(
                    target_R.translation().tolist()
                    if target_R
                    else [0, 0, 0]
                ),
                target_right_quaternion_wxyz=(
                    target_R.rotation().wxyz.tolist()
                    if target_R
                    else [1, 0, 0, 0]
                ),
                q_input=q_input_list,
                q_output=q_output_list,
                joint_radians_left=joint_left,
                joint_radians_right=joint_right,
                fk_left_position=fk_pos.get("left", [0, 0, 0]),
                fk_left_quaternion_wxyz=fk_quat.get("left", [1, 0, 0, 0]),
                fk_right_position=fk_pos.get("right", [0, 0, 0]),
                fk_right_quaternion_wxyz=fk_quat.get("right", [1, 0, 0, 0]),
                position_error_mm=pos_err,
                orientation_error_deg=ori_err,
                solve_time_ms=solve_time_ms,
            )

        except Exception as exc:
            solve_time_ms = (time.perf_counter() - t0) * 1000
            result = SolveResult(
                success=False,
                failure_reason=str(exc),
                q_input=q_input_list,
                solve_time_ms=solve_time_ms,
            )

        # Record audit entry
        self.audit_log.append(
            solve_index=self.audit_log.count + 1,
            target_left=_target_to_dict(target_L),
            target_right=_target_to_dict(target_R),
            q_input=q_input_list,
            q_output=result.q_output,
            solve_time_ms=result.solve_time_ms,
            success=result.success,
            failure_reason=result.failure_reason,
            fk_left_position=result.fk_left_position,
            fk_left_quaternion_wxyz=result.fk_left_quaternion_wxyz,
            fk_right_position=result.fk_right_position,
            fk_right_quaternion_wxyz=result.fk_right_quaternion_wxyz,
            position_error_mm=result.position_error_mm,
            orientation_error_deg=result.orientation_error_deg,
        )

        return result


def parse_target(arm_data: dict[str, Any]) -> jaxlie.SE3 | None:
    """Parse an arm target dict into a jaxlie SE3, or None if inactive."""
    if not arm_data.get("active", False):
        return None
    pos = arm_data.get("position", [0, 0, 0])
    quat_wxyz = arm_data.get("quaternion_wxyz", [1, 0, 0, 0])
    rotation = jaxlie.SO3(jnp.array(quat_wxyz))
    translation = jnp.array(pos)
    return jaxlie.SE3.from_rotation_and_translation(rotation, translation)


def _target_to_dict(target: jaxlie.SE3 | None) -> dict[str, Any]:
    if target is None:
        return {"active": False, "position": None, "quaternion_wxyz": None}
    return {
        "active": True,
        "position": target.translation().tolist(),
        "quaternion_wxyz": target.rotation().wxyz.tolist(),
    }
