"""Canonical ROH hand driver — hardware side, ROS-free and testable.

Owns one ROH hand over SocketCAN via the vendored RohGestureController.
All topic-facing values are radians (0 = open); SDK raw counts never leave
this module.

Ported from the partner's roh_hand_vr_controller.py (115 stack), keeping the
safety semantics (D7):
- firmware stall protection configured at connect
- send-delta threshold: commands smaller than the threshold are not resent
  (avoids CAN spam)
- contact-current stop with latch: when any finger current crosses the
  lowest enabled threshold while closing/holding, the hand is stopped and
  further closing commands are rejected until the operator commands the
  TRIPPED fingers back below their recorded stop positions (hysteresis) or
  fully open — fingers holding a constant pose (e.g. thumb_root in the
  five-finger ready/grasp poses) do not block the release
- a short monitor cooldown after large pose jumps avoids false stops from
  transient currents

The VR gesture state machine lives in the separate VR->hand controller
(S5); this driver only executes radian commands and enforces safety.
"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# Vendored SDK + wrapper (module-local path, no env exports needed).
_TP = Path(__file__).resolve().parent / "third_party"
sys.path.insert(0, str(_TP / "ohand_serial_sdk_python-main" / "src"))
sys.path.insert(0, str(_TP / "wrappers"))

from roh_gesture_controller import RohGestureController  # noqa: E402

from roh_units import (  # noqa: E402
    COUNTS_CLOSED,
    NUM_FINGERS,
    counts_to_rad,
    rad_to_counts,
    validate_command,
)

logger = logging.getLogger(__name__)

_ALL_FINGER_BITS = 0x3F
_MODE_CHANGE_COOLDOWN_S = 0.25


@dataclass
class SafetyConfig:
    speed: int = 120
    current_limit_ma: int = 250
    contact_stop_enabled: bool = True
    contact_current_ma: int = 250
    firmware_stall_enabled: bool = True
    firmware_stall_speed: int = 16
    firmware_stall_current_ma: int = 200
    firmware_stall_after_ms: int = 300
    firmware_stall_retry_ms: int = 500
    per_finger_stop: bool = False


@dataclass
class HandDriverConfig:
    side: str = "left"
    can_channel: str = "can2"
    can_bitrate: int = 1000000
    hand_id: int = 0x02
    master_id: int = 0x01
    dry_run: bool = False
    send_delta_threshold: float = 0.02
    monitor_interval_s: float = 0.05
    closed_rad: list[float] = field(default_factory=lambda: [1.6] * NUM_FINGERS)
    safety: SafetyConfig = field(default_factory=SafetyConfig)


class RohHandDriver:
    """Drive one ROH hand in radians with ported safety logic."""

    def __init__(self, config: HandDriverConfig) -> None:
        self.config = config
        self._ctrl = RohGestureController(
            can_channel=config.can_channel,
            can_bitrate=config.can_bitrate,
            hand_id=config.hand_id,
            master_id=config.master_id,
            dry_run=config.dry_run,
        )
        self._can_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._monitor_stop = threading.Event()
        self._monitor_thread: Optional[threading.Thread] = None

        self.connected = False
        self.last_error: Optional[str] = None

        self._last_target_counts: Optional[list[int]] = None
        self._closing_or_holding = False
        self._latched = False
        self._latched_fingers: Optional[list[int]] = None
        self._stop_target_counts: Optional[list[int]] = None
        self._cooldown_deadline = 0.0
        self.consecutive_read_failures = 0
        self._mon_last_positions: Optional[list[int]] = None

        self._delta_counts = max(int(config.send_delta_threshold * COUNTS_CLOSED), 1)

    # ---- lifecycle ---------------------------------------------------------

    def connect(self) -> None:
        """Connect to the hand and configure firmware stall protection."""
        with self._can_lock:
            self._ctrl.connect()
            if self.config.safety.firmware_stall_enabled:
                s = self.config.safety
                self._ctrl.configure_stall_protection(
                    speed=s.firmware_stall_speed,
                    stop_current_ma=s.firmware_stall_current_ma,
                    stop_after_ms=s.firmware_stall_after_ms,
                    retry_after_ms=s.firmware_stall_retry_ms,
                )
        self.connected = True
        self._monitor_stop.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name=f"roh-monitor-{self.config.side}",
            daemon=True,
        )
        self._monitor_thread.start()
        logger.info(
            "ROH driver connected: side=%s channel=%s dry_run=%s",
            self.config.side, self.config.can_channel, self.config.dry_run,
        )

    def close(self) -> None:
        """Stop monitoring, stop the motors, and release the CAN channel."""
        self._monitor_stop.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=2.0)
            self._monitor_thread = None
        with self._can_lock:
            try:
                self._ctrl.safe_stop()
            except Exception:
                pass
            try:
                self._ctrl.close()
            except Exception:
                pass
        self.connected = False

    def __enter__(self) -> "RohHandDriver":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- command path ------------------------------------------------------

    def command_radians(self, positions_rad: list[float]) -> bool:
        """Execute a 6-finger radian command. Returns True if sent to the hand.

        Rejected (returns False) when:
        - the contact latch is active and the command does not retreat the
          latched (tripped) fingers below their recorded stop positions
          (hysteresis = 2x send-delta), or
        - the command is below the send-delta threshold (CAN spam guard).
        """
        cfg = self.config
        validate_command(positions_rad, cfg.closed_rad)
        target = [rad_to_counts(r, cr) for r, cr in zip(positions_rad, cfg.closed_rad)]

        with self._state_lock:
            if self._latched:
                release_counts = 2 * self._delta_counts
                stop = self._stop_target_counts or [0] * NUM_FINGERS
                # Only the tripped fingers must retreat; fingers that held a
                # constant pose through the grasp (e.g. thumb_root in five
                # mode) are not part of the contact and must not block the
                # release. No recorded mask (e.g. latch set externally) falls
                # back to requiring all fingers to retreat.
                masked = (
                    self._latched_fingers
                    if self._latched_fingers
                    else range(NUM_FINGERS)
                )
                retreating = all(
                    target[i] <= stop[i] - release_counts for i in masked
                ) or all(t == 0 for t in target)
                if not retreating:
                    return False
                self._latched = False
                self._latched_fingers = None
                self._stop_target_counts = None
                logger.info("ROH (%s): contact latch released by retreat command", cfg.side)

            last = self._last_target_counts
            if last is not None and max(
                abs(target[i] - last[i]) for i in range(NUM_FINGERS)
            ) < self._delta_counts:
                return False  # below send-delta threshold — not resent

            closing = last is None or any(target[i] > last[i] for i in range(NUM_FINGERS))
            big_jump = last is None or max(
                abs(target[i] - last[i]) for i in range(NUM_FINGERS)
            ) >= 4 * self._delta_counts

        with self._can_lock:
            self._ctrl.send_positions(target, speed=cfg.safety.speed)

        with self._state_lock:
            self._last_target_counts = list(target)
            self._closing_or_holding = closing
            if big_jump:
                self._cooldown_deadline = time.monotonic() + _MODE_CHANGE_COOLDOWN_S
        return True

    def open_hand(self) -> None:
        """Command the hand fully open and clear any contact latch."""
        with self._state_lock:
            self._latched = False
            self._latched_fingers = None
            self._stop_target_counts = None
        with self._can_lock:
            self._ctrl.send_positions([0] * NUM_FINGERS, speed=self.config.safety.speed)
        with self._state_lock:
            self._last_target_counts = [0] * NUM_FINGERS
            self._closing_or_holding = False

    # ---- telemetry path ----------------------------------------------------

    def read_joint_state(self) -> tuple[Optional[list[float]], Optional[list[float]]]:
        """Read (positions_rad, currents_ma). None for the whole tuple element
        on read failure — callers must not fabricate values. Per-finger current
        read failures surface as NaN in the currents list."""
        with self._can_lock:
            try:
                positions = self._ctrl.read_positions().get("current")
                currents = self._ctrl.read_currents()
            except Exception as exc:
                self.consecutive_read_failures += 1
                self.last_error = f"telemetry read failed: {exc}"
                logger.warning("ROH (%s): %s", self.config.side, self.last_error)
                return None, None

        if positions is None or any(p is None for p in positions):
            self.consecutive_read_failures += 1
            return None, None
        self.consecutive_read_failures = 0

        positions_rad = [
            counts_to_rad(int(p), cr) for p, cr in zip(positions, self.config.closed_rad)
        ]
        currents_ma = [
            float(c) if c is not None else math.nan for c in (currents or [])
        ]
        if len(currents_ma) != NUM_FINGERS:
            currents_ma = None
        return positions_rad, currents_ma

    # ---- safety monitor ----------------------------------------------------

    @property
    def contact_latched(self) -> bool:
        with self._state_lock:
            return self._latched

    def _contact_threshold_ma(self) -> int:
        s = self.config.safety
        threshold = 0
        if s.contact_stop_enabled and s.contact_current_ma > 0:
            threshold = s.contact_current_ma
        if s.current_limit_ma > 0:
            threshold = min(threshold, s.current_limit_ma) if threshold > 0 else s.current_limit_ma
        return threshold

    def _monitor_loop(self) -> None:
        interval = self.config.monitor_interval_s
        while not self._monitor_stop.is_set():
            with self._state_lock:
                armed = (
                    self.connected
                    and self._closing_or_holding
                    and not self._latched
                    and time.monotonic() >= self._cooldown_deadline
                )
            if armed and self._contact_threshold_ma() > 0:
                try:
                    with self._can_lock:
                        currents = self._ctrl.read_currents()
                        positions = self._ctrl.read_positions().get("current")
                except Exception:
                    currents, positions = [], None
                finger_indices = self._evaluate_poll(currents, positions)
                if finger_indices:
                    self._contact_stop(finger_indices, currents, self._contact_threshold_ma())
            self._monitor_stop.wait(interval)

    # Position change (counts) below this while over-current means the finger
    # is blocked; above it the current is just free-space motion current.
    _STALL_EPSILON_COUNTS = 200

    def _evaluate_poll(self, currents, positions) -> list[int]:
        """Return the tripped finger indices (empty when nothing trips).

        Trips only when a finger is over the current threshold AND its
        position is not advancing across polls — that is the contact/stall
        signature. A finger sweeping freely can draw more than the threshold
        (e.g. thumb_root on the long ready-pose sweep, ~540 mA measured on
        hardware 2026-08-11); stopping there would false-latch the hand on
        every startup.
        """
        threshold = self._contact_threshold_ma()
        if (
            threshold <= 0
            or not positions
            or any(p is None for p in positions)
        ):
            self._mon_last_positions = None
            return []
        positions = [int(p) for p in positions]
        prev = self._mon_last_positions
        self._mon_last_positions = positions
        if prev is None:
            return []  # need two polls to judge progress
        tripped = []
        for idx, value in enumerate(currents):
            if value is None or value < threshold:
                continue
            if abs(positions[idx] - prev[idx]) <= self._STALL_EPSILON_COUNTS:
                tripped.append(idx)
        return tripped

    def _contact_stop(self, finger_indices: list[int], currents, threshold: int) -> None:
        s = self.config.safety
        finger_bits = (
            sum(1 << i for i in finger_indices) if s.per_finger_stop else _ALL_FINGER_BITS
        )
        try:
            with self._can_lock:
                self._ctrl.safe_stop(finger_bits)
        finally:
            with self._state_lock:
                self._latched = True
                self._latched_fingers = list(finger_indices)
                self._stop_target_counts = (
                    list(self._last_target_counts)
                    if self._last_target_counts is not None else None
                )
                self._closing_or_holding = False
            logger.warning(
                "ROH (%s): contact stop — fingers %s currents %s mA >= %d mA; "
                "latched until tripped fingers retreat",
                self.config.side,
                finger_indices,
                {i: currents[i] for i in finger_indices},
                threshold,
            )

    # ---- status ------------------------------------------------------------

    def get_status(self) -> dict:
        with self._state_lock:
            return {
                "side": self.config.side,
                "connected": self.connected,
                "dry_run": self.config.dry_run,
                "can_channel": self.config.can_channel,
                "contact_latched": self._latched,
                "latched_fingers": (
                    list(self._latched_fingers)
                    if self._latched_fingers is not None else None
                ),
                "closing_or_holding": self._closing_or_holding,
                "last_target_counts": (
                    list(self._last_target_counts)
                    if self._last_target_counts is not None else None
                ),
                "consecutive_read_failures": self.consecutive_read_failures,
                "last_error": self.last_error,
            }
