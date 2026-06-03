"""VR pose normalizer — origin pinning, calibration, and axis mapping."""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field
from typing import Any

import jax.numpy as jnp
import jaxlie

from server.config_loader import VRConfig
from server.vr_subscriber import VRDataStore, VRPose, VRButtonState


class CalibrationState(enum.Enum):
    WAITING = "waiting"
    READY = "ready"
    ACTIVE = "active"


@dataclass
class NormalizedTarget:
    """Output of normalization: SE3 target in robot base frame."""
    left: jaxlie.SE3 | None = None
    right: jaxlie.SE3 | None = None
    state: CalibrationState = CalibrationState.WAITING
    left_trigger: float = 0.0
    right_trigger: float = 0.0


def _quat_wxyz_to_so3(wxyz: list[float]) -> jaxlie.SO3:
    """Convert [w, x, y, z] quaternion to jaxlie SO3."""
    return jaxlie.SO3(jnp.array(wxyz))


def _pose_to_se3(pose: VRPose) -> jaxlie.SE3:
    """Convert VRPose to jaxlie SE3."""
    rotation = _quat_wxyz_to_so3(pose.orientation_wxyz)
    translation = jnp.array(pose.position)
    return jaxlie.SE3.from_rotation_and_translation(rotation, translation)


def _yaw_from_se3(pose: jaxlie.SE3) -> float:
    """Extract yaw (rotation around Y axis in VR space) from SE3.

    VR space convention: Y is up, ground plane is XZ.
    """
    rot = pose.rotation()
    forward = rot @ jnp.array([0.0, 0.0, -1.0])
    forward_xz = jnp.array([forward[0], forward[2]])
    norm = jnp.linalg.norm(forward_xz)
    if norm < 1e-6:
        return 0.0
    return float(math.atan2(-float(forward_xz[0]), -float(forward_xz[1])))


def _yaw_only_rotation(yaw: float) -> jaxlie.SO3:
    """Create a Y-axis rotation from yaw angle (VR convention: Y is up)."""
    half = yaw / 2.0
    return jaxlie.SO3(jnp.array([math.cos(half), 0.0, math.sin(half), 0.0]))


def _apply_axis_mapping(
    position: list[float],
    rotation: jaxlie.SO3,
    config: VRConfig,
) -> tuple[list[float], jaxlie.SO3]:
    """Apply axis mapping from VR native to robot base frame."""
    mat = jnp.array(config.axis_mapping.native_to_intermediate)
    pos = mat @ jnp.array(position) * config.axis_mapping.position_scale
    basis_quat = jaxlie.SO3.from_matrix(mat)
    mapped_rot = basis_quat @ rotation @ basis_quat.inverse()
    return pos.tolist(), mapped_rot


def _detect_button(
    button_state: VRButtonState | None,
    button_name: str,
) -> bool:
    """Detect if a specific button is pressed from button state array.

    Pico controller button indices (from dry VR bridge):
    Index 0: Trigger/gripper, 1: Grip/middle finger, 2: ?, 3: Thumbstick, 4: A, last: B
    """
    if button_state is None or not button_state.buttons:
        return False
    buttons = button_state.buttons
    name = button_name.upper()
    if name == "A":
        return len(buttons) > 4 and buttons[4] > 0.5
    if name == "B":
        return len(buttons) > 0 and buttons[-1] > 0.5
    return False


def _trigger_value(button_state: VRButtonState | None) -> float:
    """Extract analog trigger value (index 0) from button state, clamped [0, 1]."""
    if button_state is None or not button_state.buttons:
        return 0.0
    return min(max(button_state.buttons[0], 0.0), 1.0)


class VRNormalizer:
    """VR pose normalization pipeline.

    State machine: WAITING → (B) → READY → (A) → ACTIVE
    - WAITING: No origin pinned
    - READY: Origin pinned + anchors set, markers rebase but NO IK solving
    - ACTIVE: IK solving enabled, producing delta targets

    B-button / Reset Home: pin origin + calibrate anchors → READY
    A-button: start IK solving → ACTIVE
    """

    def __init__(self, config: VRConfig) -> None:
        self.config = config
        self.state = CalibrationState.WAITING
        self._vr_origin: jaxlie.SE3 | None = None
        self._calibration_anchor: dict[str, jaxlie.SE3] = {}
        self._home_ee: dict[str, jaxlie.SE3] = {}
        self._home_ee_source: dict[str, jaxlie.SE3] = {}
        self._last_active_s: float = 0.0
        self._prev_pin_pressed = False
        self._prev_start_pressed = False
        self._needs_pin = True  # B must be pressed before first A

    def set_home_ee(self, home_ee: dict[str, jaxlie.SE3]) -> None:
        """Set the FK home end-effector poses (from IKService)."""
        self._home_ee_source = home_ee

    def normalize(self, data_store: VRDataStore) -> NormalizedTarget:
        """Run the full normalization pipeline on latest VR data."""
        now = time.monotonic()

        # Check staleness — revert to WAITING if no VR data for > timeout
        if self.state != CalibrationState.WAITING:
            if (now - self._last_active_s) > self.config.calibration.stale_timeout_s:
                self._reset()
                return NormalizedTarget(state=CalibrationState.WAITING,
                                        left_trigger=left_trigger, right_trigger=right_trigger)

        # Get head pose for pinning
        head_pose = data_store.get_head_pose()
        button_states = data_store.get_button_states()

        # Extract trigger analog values (index 0) for gripper control
        left_trigger = _trigger_value(button_states.get("left"))
        right_trigger = _trigger_value(button_states.get("right"))

        # Refresh activity timestamp when VR data is flowing
        if head_pose is not None:
            self._last_active_s = now

        # Detect button presses (rising edge) on right controller only
        pin_pressed_now = False
        start_pressed_now = False
        for side in ("right",):
            bs = button_states.get(side)
            if _detect_button(bs, self.config.calibration.pin_button):
                pin_pressed_now = True
            if _detect_button(bs, self.config.calibration.start_button):
                start_pressed_now = True

        pin_rising = pin_pressed_now and not self._prev_pin_pressed
        start_rising = start_pressed_now and not self._prev_start_pressed
        self._prev_pin_pressed = pin_pressed_now
        self._prev_start_pressed = start_pressed_now

        # B-button: pin origin + calibrate anchors → READY
        if pin_rising and head_pose is not None:
            self._pin_and_calibrate(head_pose, data_store, self._home_ee_source)
            self._needs_pin = False
            self._last_active_s = now
            print("[CALIBRATION] Origin pinned — press A to start IK solving")

        # A-button: start IK solving → ACTIVE (requires B first)
        if start_rising:
            if self._needs_pin:
                print("[CALIBRATION] WARNING: Press B first to pin origin and normalize position, then press A")
            elif self.state == CalibrationState.READY:
                self.state = CalibrationState.ACTIVE
                self._needs_pin = True  # require B again after next stop/restart
                self._last_active_s = now
                print("[CALIBRATION] IK solving ACTIVE — press B to re-pin at any time")

        # Produce targets only if ACTIVE
        if self.state == CalibrationState.ACTIVE:
            result = self._produce_targets(data_store)
            result.left_trigger = left_trigger
            result.right_trigger = right_trigger
            self._last_active_s = now
            return result

        return NormalizedTarget(state=self.state,
                                left_trigger=left_trigger, right_trigger=right_trigger)

    def pin_and_calibrate(
        self,
        head_pose: VRPose,
        data_store: VRDataStore,
        home_ee: dict[str, jaxlie.SE3] | None = None,
    ) -> None:
        """Public entry point for reset_home: pin origin + calibrate anchors."""
        self._pin_and_calibrate(head_pose, data_store, home_ee)

    def _pin_and_calibrate(
        self,
        head_pose: VRPose,
        data_store: VRDataStore,
        home_ee: dict[str, jaxlie.SE3] | None = None,
    ) -> None:
        """Pin origin from head, snapshot current handles as anchors, set READY."""
        head_se3 = _pose_to_se3(head_pose)
        pos = head_se3.translation()
        ground_pos = jnp.array([pos[0], 0.0, pos[2]])
        yaw = _yaw_from_se3(head_se3)
        yaw_rotation = _yaw_only_rotation(yaw)
        self._vr_origin = jaxlie.SE3.from_rotation_and_translation(
            yaw_rotation, ground_pos
        )

        # Snapshot current handle positions as calibration anchors
        self._calibration_anchor.clear()
        poses = data_store.get_latest_poses()
        for side, pose in poses.items():
            rebased = self._rebase_pose(pose)
            self._calibration_anchor[side] = rebased

        # Store FK home end-effector poses as the base for IK targets
        self._home_ee = home_ee or {}

        self.state = CalibrationState.READY

    def _rebase_pose(self, pose: VRPose) -> jaxlie.SE3:
        """Rebase a controller pose into the pinned VR origin frame."""
        if self._vr_origin is None:
            return _pose_to_se3(pose)
        controller_se3 = _pose_to_se3(pose)
        return self._vr_origin.inverse() @ controller_se3

    def _produce_targets(self, data_store: VRDataStore) -> NormalizedTarget:
        """Produce SE3 targets: axis-map the VR delta, then compose with FK home.

        Pipeline per the teleop-to-robot mapping spec:
        1. Compute delta from calibration anchor (identity when hands still)
        2. Axis-map delta: position = mat @ pos, rotation = basis @ rot @ basis⁻¹
        3. Compose with FK home EE: target = FK_home @ delta
        """
        poses = data_store.get_latest_poses()
        left_target = None
        right_target = None

        for side in ("left", "right"):
            if side not in poses or side not in self._calibration_anchor:
                continue

            rebased = self._rebase_pose(poses[side])
            anchor = self._calibration_anchor[side]

            # Compute delta from anchor (identity when hands haven't moved)
            delta = anchor.inverse() @ rebased
            delta_pos = delta.translation()
            delta_rot = delta.rotation()

            # Apply axis mapping: VR native → ROS REP-103 intermediate
            # Position: mat @ pos * scale
            # Rotation: basis @ rot @ basis⁻¹ (conjugation)
            mapped_pos, mapped_rot = _apply_axis_mapping(
                delta_pos.tolist(), delta_rot, self.config
            )
            delta_se3 = jaxlie.SE3.from_rotation_and_translation(
                mapped_rot,
                jnp.array(mapped_pos),
            )

            # Compose with FK home: delta is already in robot base frame,
            # so add position (no rotation) and compose rotation in base frame.
            home_ee = self._home_ee.get(side)
            if home_ee is not None:
                target = jaxlie.SE3.from_rotation_and_translation(
                    delta_se3.rotation() @ home_ee.rotation(),
                    home_ee.translation() + delta_se3.translation(),
                )
            else:
                target = delta_se3

            if side == "left":
                left_target = target
            else:
                right_target = target

        return NormalizedTarget(
            left=left_target,
            right=right_target,
            state=self.state,
        )

    def _reset(self) -> None:
        """Reset to WAITING state."""
        self.state = CalibrationState.WAITING
        self._needs_pin = True
        self._vr_origin = None
        self._calibration_anchor.clear()
        self._home_ee.clear()
        self._prev_pin_pressed = False
        self._prev_start_pressed = False

    def get_status(self) -> dict[str, Any]:
        """Return current calibration state for snapshot."""
        status: dict[str, Any] = {
            "state": self.state.value,
            "origin_pinned": self._vr_origin is not None,
            "calibrated_sides": list(self._calibration_anchor.keys()),
        }
        if self._vr_origin is not None:
            status["origin_position"] = self._vr_origin.translation().tolist()
            status["origin_quaternion_wxyz"] = self._vr_origin.rotation().wxyz.tolist()
        return status
