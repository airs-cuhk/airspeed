#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
roh_gesture_controller.py

Merged wrapper for the OYMotion ROH-LiteS001 (CAN version) dexterous hand on
Ubuntu SocketCAN.  The new wrappers2.0 implementation is used as the base
because it provides resistance-aware guarded moves, current/force telemetry,
and firmware stall protection.  Legacy gesture names and a few extra poses
from the original wrapper are preserved as aliases or additional entries.

Default ROH channel is ``can2`` (verified PEAK PCAN-USB adapter, USB id
``0c72:000c``).  Pass ``can_channel=...`` to use a different interface.

The manufacturer OHand Python SDK is used unmodified; the SocketCAN backend
is imported directly from ``ohand.interface.can.socet_can_interface``.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


# Make the manufacturer SDK importable when this file is run directly.
SDK_SRC = Path(__file__).resolve().parents[1] / "ohand_serial_sdk_python-main" / "src"
if str(SDK_SRC) not in sys.path:
    sys.path.insert(0, str(SDK_SRC))

from ohand.OHandSerialAPI import OHandSerialAPI
from ohand.constants import (
    HAND_CMD_GET_FINGER_FORCE,
    HAND_PROTOCOL_UART,
    HAND_RESP_SUCCESS,
    HAND_RESP_UNMATCHED_CMD,
    MAX_FORCE_ENTRIES,
)


NUM_MOTORS = 6
ALL_FINGER_BITS = (1 << NUM_MOTORS) - 1
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky", "thumb_root")

DEFAULT_CAN_CHANNEL = "can2"
DEFAULT_CAN_BITRATE = 1000000
DEFAULT_HAND_ID = 0x02
DEFAULT_MASTER_ID = 0x01
DEFAULT_COMMAND_TIMEOUT_MS = 500

DEFAULT_SPEED = 120
DEFAULT_SLOW_SPEED = 30
DEFAULT_SETTLE_MS = 1500
DEFAULT_RESISTANCE_CURRENT_MA = 450
DEFAULT_SLOW_CURRENT_MA = 320
DEFAULT_RESISTANCE_FORCE_MN = 0
DEFAULT_SLOW_FORCE_MN = 0
DEFAULT_RESISTANCE_HOLD_MS = 120
DEFAULT_MONITOR_INTERVAL_MS = 50
DEFAULT_POSITION_STALL_HOLD_MS = 300
DEFAULT_POSITION_STALL_MIN_DELTA = 20
DEFAULT_POSITION_STALL_ERROR_COUNTS = 2500


# Precision pinch biomechanics.  The thumb root (motor 5) is already rotated
# into opposition toward the index finger in the pre-pinch pose, with only
# slight thumb flexion (motor 0).  During closure the index finger (motor 1)
# flexes the most, the thumb flexes moderately, and the thumb root receives
# only a small additional rotation.  This produces a pad-to-pad precision pinch
# rather than the power-grasp thumb posture used for whole-hand closure.
PINCH_PRE_POSITION = (15000, 0, 0, 0, 0, 45000)      # thumb slightly flexed, index open, root opposed
PINCH_MAX_POSITION = (35000, 48000, 0, 0, 0, 56000)  # thumb moderate, index prioritized, root slightly more opposed

# Canonical gesture target positions.  Position 0 is open/spread, 65535 is closed.
GESTURE_POSITIONS: Dict[str, Tuple[int, ...]] = {
    "open": (0, 0, 0, 0, 0, 0),
    "spread": (0, 0, 0, 0, 0, 0),
    "rest": (20000, 10000, 10000, 10000, 10000, 0),
    "five_finger_ready_grasp": (0, 0, 0, 0, 0, 65000),
    "five_finger_grasp": (45000, 65535, 65535, 65535, 65535, 65000),
    "fist": (45000, 65535, 65535, 65535, 65535, 0),
    "two_finger_ready_grasp": PINCH_PRE_POSITION,
    "two_finger_grasp": PINCH_MAX_POSITION,
    "pinch": PINCH_MAX_POSITION,
    "extend_index": (45000, 0, 65535, 65535, 65535, 0),
    "point": (45000, 0, 65535, 65535, 65535, 0),
    "victory": (45000, 0, 0, 65535, 65535, 0),
    # Extra pose carried over from the original wrapper.
    "middle_finger_up": (45000, 45000, 0, 45000, 45000, 0),
}

# Legacy / alternate names that map to canonical gestures above.
GESTURE_ALIASES: Dict[str, str] = {
    "five_finger_open": "open",
    "point_index_finger": "point",
    "extend_index_finger": "point",
}

GESTURE_DESCRIPTIONS: Dict[str, str] = {
    "open": "All fingers fully open / spread.",
    "spread": "Alias for open.",
    "rest": "Relaxed, slightly curled posture.",
    "five_finger_ready_grasp": "Safe first pose: fingers partly curled, thumb to the side.",
    "five_finger_grasp": "Whole-hand soft grasp / fist, thumb to the side.",
    "fist": "Alias for five_finger_grasp.",
    "two_finger_ready_grasp": "Pinch-ready: index forward, other fingers tucked, thumb opposed.",
    "two_finger_grasp": "Thumb-index pinch, other fingers tucked.",
    "pinch": "Alias for two_finger_grasp.",
    "extend_index": "Index extended, other fingers flexed, thumb tucked.",
    "point": "Alias for extend_index.",
    "victory": "Index and middle extended, ring/pinky flexed.",
    "middle_finger_up": "Middle extended, other fingers flexed/tucked.",
}


def scan_can_channels() -> List[Dict[str, str]]:
    """Map each Linux canX interface to its USB vendor/product and state.

    Returns a list of dicts with keys: channel, state, vendor, product,
    usb_id, device, is_roh_adapter.  This helps locate the ROH PEAK PCAN-USB
    adapter without sending any CAN frames.
    """
    import glob

    results: List[Dict[str, str]] = []
    for iface_path in sorted(glob.glob("/sys/class/net/can*")):
        channel = Path(iface_path).name
        device_link = Path(iface_path) / "device"
        if not device_link.exists():
            continue
        device_path = device_link.resolve()
        vendor = product = "unknown"
        usb_id = "unknown"
        # Walk up to the USB device directory (it has idVendor/idProduct).
        for parent in [device_path, *device_path.parents]:
            vendor_file = parent / "idVendor"
            product_file = parent / "idProduct"
            if vendor_file.exists() and product_file.exists():
                vendor = vendor_file.read_text().strip()
                product = product_file.read_text().strip()
                usb_id = f"{vendor}:{product}"
                break

        state = "unknown"
        operstate_file = Path(iface_path) / "operstate"
        if operstate_file.exists():
            state = operstate_file.read_text().strip()

        is_roh = usb_id.lower() == "0c72:000c"
        results.append(
            {
                "channel": channel,
                "state": state,
                "vendor": vendor,
                "product": product,
                "usb_id": usb_id,
                "device": str(device_path),
                "is_roh_adapter": str(is_roh),
            }
        )
    return results


@dataclass(frozen=True)
class GestureStep:
    positions: Tuple[int, ...]
    speed: int = DEFAULT_SPEED
    settle_ms: int = DEFAULT_SETTLE_MS


@dataclass(frozen=True)
class ResistanceEvent:
    signal: str
    finger_id: int
    value: int
    threshold: int
    action: str
    elapsed_ms: int

    @property
    def finger_name(self) -> str:
        if 0 <= self.finger_id < len(FINGER_NAMES):
            return FINGER_NAMES[self.finger_id]
        return f"finger_{self.finger_id}"

    def as_dict(self) -> Dict[str, object]:
        return {
            "signal": self.signal,
            "finger_id": self.finger_id,
            "finger_name": self.finger_name,
            "value": self.value,
            "threshold": self.threshold,
            "action": self.action,
            "elapsed_ms": self.elapsed_ms,
        }

    def __str__(self) -> str:
        return (
            f"{self.signal} resistance on {self.finger_name}: "
            f"{self.value} >= {self.threshold}; action={self.action}"
        )


class ResistanceDetected(RuntimeError):
    def __init__(self, event: ResistanceEvent):
        super().__init__(str(event))
        self.event = event


class RohGestureController:
    """CAN/SocketCAN wrapper for ROH hand gestures with resistance stop."""

    def __init__(
        self,
        can_channel: str = DEFAULT_CAN_CHANNEL,
        can_bitrate: int = DEFAULT_CAN_BITRATE,
        hand_id: int = DEFAULT_HAND_ID,
        master_id: int = DEFAULT_MASTER_ID,
        command_timeout_ms: int = DEFAULT_COMMAND_TIMEOUT_MS,
        dry_run: bool = False,
    ):
        self.can_channel = can_channel
        self.can_bitrate = can_bitrate
        self.hand_id = hand_id
        self.master_id = master_id
        self.command_timeout_ms = command_timeout_ms
        self.dry_run = dry_run
        self.bus = None
        self.api: Optional[OHandSerialAPI] = None

    def __enter__(self) -> "RohGestureController":
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def connect(self) -> "RohGestureController":
        if self.dry_run:
            return self

        try:
            from ohand.interface.can.socet_can_interface import (
                CAN_Init,
                delay_milli_seconds_impl,
                get_milli_seconds_impl,
                recv_data_impl,
                send_data_impl,
            )
        except ImportError as exc:
            raise RuntimeError(
                "Failed to import the ROH SocketCAN backend. Install the SDK "
                "dependencies first: python3 -m pip install -e ROH/ohand_serial_sdk_python-main"
            ) from exc

        can_port = self._can_channel_to_port(self.can_channel)
        self.bus = CAN_Init(port_name=can_port, baudrate=self.can_bitrate)
        if self.bus is None:
            raise RuntimeError(f"Failed to open SocketCAN channel {self.can_channel}")

        self.api = OHandSerialAPI(
            self.bus,
            HAND_PROTOCOL_UART,
            self.master_id,
            send_data_impl,
            recv_data_impl,
        )
        self.api.HAND_SetTimerFunction(get_milli_seconds_impl, delay_milli_seconds_impl)
        self.api.HAND_SetCommandTimeOut(self.command_timeout_ms)
        return self

    def close(self) -> None:
        if self.bus is not None and hasattr(self.bus, "shutdown"):
            self.bus.shutdown()
        self.bus = None
        self.api = None

    def perform(self, name: str, speed: int = DEFAULT_SPEED, settle_ms: int = DEFAULT_SETTLE_MS) -> None:
        for step in self.gesture_steps(name, speed=speed, settle_ms=settle_ms):
            self.send_positions(step.positions, speed=step.speed)
            if step.settle_ms > 0:
                time.sleep(step.settle_ms / 1000.0)

    def perform_guarded(
        self,
        name: str,
        speed: int = DEFAULT_SPEED,
        slow_speed: int = DEFAULT_SLOW_SPEED,
        current_limit_ma: int = DEFAULT_RESISTANCE_CURRENT_MA,
        slow_current_ma: int = DEFAULT_SLOW_CURRENT_MA,
        force_limit_mn: int = DEFAULT_RESISTANCE_FORCE_MN,
        slow_force_mn: int = DEFAULT_SLOW_FORCE_MN,
        hold_ms: int = DEFAULT_RESISTANCE_HOLD_MS,
        interval_ms: int = DEFAULT_MONITOR_INTERVAL_MS,
        position_stall_enabled: bool = False,
        position_stall_hold_ms: int = DEFAULT_POSITION_STALL_HOLD_MS,
        position_stall_min_delta: int = DEFAULT_POSITION_STALL_MIN_DELTA,
        position_stall_error_counts: int = DEFAULT_POSITION_STALL_ERROR_COUNTS,
        settle_ms: int = DEFAULT_SETTLE_MS,
    ) -> None:
        for step in self.gesture_steps(name, speed=speed, settle_ms=settle_ms):
            self.move_guarded(
                step.positions,
                speed=step.speed,
                slow_speed=slow_speed,
                duration_ms=step.settle_ms,
                current_limit_ma=current_limit_ma,
                slow_current_ma=slow_current_ma,
                force_limit_mn=force_limit_mn,
                slow_force_mn=slow_force_mn,
                hold_ms=hold_ms,
                interval_ms=interval_ms,
                position_stall_enabled=position_stall_enabled,
                position_stall_hold_ms=position_stall_hold_ms,
                position_stall_min_delta=position_stall_min_delta,
                position_stall_error_counts=position_stall_error_counts,
            )

    def move_guarded(
        self,
        positions: Sequence[int],
        speed: int = DEFAULT_SPEED,
        slow_speed: int = DEFAULT_SLOW_SPEED,
        duration_ms: int = DEFAULT_SETTLE_MS,
        current_limit_ma: int = DEFAULT_RESISTANCE_CURRENT_MA,
        slow_current_ma: int = DEFAULT_SLOW_CURRENT_MA,
        force_limit_mn: int = DEFAULT_RESISTANCE_FORCE_MN,
        slow_force_mn: int = DEFAULT_SLOW_FORCE_MN,
        hold_ms: int = DEFAULT_RESISTANCE_HOLD_MS,
        interval_ms: int = DEFAULT_MONITOR_INTERVAL_MS,
        position_stall_enabled: bool = False,
        position_stall_hold_ms: int = DEFAULT_POSITION_STALL_HOLD_MS,
        position_stall_min_delta: int = DEFAULT_POSITION_STALL_MIN_DELTA,
        position_stall_error_counts: int = DEFAULT_POSITION_STALL_ERROR_COUNTS,
    ) -> None:
        slowed = False
        target = tuple(_clamp_int(value, 0, 65535) for value in positions)

        self.send_positions(target, speed=speed)

        def slow_down(event: ResistanceEvent) -> None:
            nonlocal slowed
            if slowed:
                return
            slowed = True
            print(f"Soft resistance detected, slowing down: {event}")
            self.send_positions(target, speed=slow_speed)

        if duration_ms <= 0:
            return

        event = self.monitor_resistance(
            duration_ms=duration_ms,
            current_limit_ma=current_limit_ma,
            slow_current_ma=slow_current_ma,
            force_limit_mn=force_limit_mn,
            slow_force_mn=slow_force_mn,
            hold_ms=hold_ms,
            interval_ms=interval_ms,
            on_soft=slow_down,
            target_positions=target,
            position_stall_enabled=position_stall_enabled,
            position_stall_hold_ms=position_stall_hold_ms,
            position_stall_min_delta=position_stall_min_delta,
            position_stall_error_counts=position_stall_error_counts,
        )
        if event is not None:
            self.safe_stop()
            raise ResistanceDetected(event)

    def monitor_resistance(
        self,
        duration_ms: int,
        current_limit_ma: int = DEFAULT_RESISTANCE_CURRENT_MA,
        slow_current_ma: int = DEFAULT_SLOW_CURRENT_MA,
        force_limit_mn: int = DEFAULT_RESISTANCE_FORCE_MN,
        slow_force_mn: int = DEFAULT_SLOW_FORCE_MN,
        hold_ms: int = DEFAULT_RESISTANCE_HOLD_MS,
        interval_ms: int = DEFAULT_MONITOR_INTERVAL_MS,
        on_soft: Optional[Callable[[ResistanceEvent], None]] = None,
        target_positions: Optional[Sequence[int]] = None,
        position_stall_enabled: bool = False,
        position_stall_hold_ms: int = DEFAULT_POSITION_STALL_HOLD_MS,
        position_stall_min_delta: int = DEFAULT_POSITION_STALL_MIN_DELTA,
        position_stall_error_counts: int = DEFAULT_POSITION_STALL_ERROR_COUNTS,
    ) -> Optional[ResistanceEvent]:
        if duration_ms <= 0:
            return None

        hard_since: Dict[Tuple[str, int], float] = {}
        soft_seen: set[Tuple[str, int]] = set()
        stall_since: Dict[int, float] = {}
        last_positions: Optional[List[Optional[int]]] = None
        start_time = time.monotonic()
        deadline = start_time + max(duration_ms, 0) / 1000.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            elapsed_ms = int((now - start_time) * 1000)

            currents = self.read_currents() if current_limit_ma > 0 or slow_current_ma > 0 else []
            hard_event = self._check_soft_and_hard_thresholds(
                signal="current_ma",
                values=currents,
                soft_threshold=slow_current_ma,
                hard_threshold=current_limit_ma,
                hard_since=hard_since,
                soft_seen=soft_seen,
                hold_ms=hold_ms,
                elapsed_ms=elapsed_ms,
                on_soft=on_soft,
            )
            if hard_event is not None:
                return hard_event

            forces = self.read_force_maxima() if force_limit_mn > 0 or slow_force_mn > 0 else []
            hard_event = self._check_soft_and_hard_thresholds(
                signal="force_mn",
                values=forces,
                soft_threshold=slow_force_mn,
                hard_threshold=force_limit_mn,
                hard_since=hard_since,
                soft_seen=soft_seen,
                hold_ms=hold_ms,
                elapsed_ms=elapsed_ms,
                on_soft=on_soft,
            )
            if hard_event is not None:
                return hard_event

            if position_stall_enabled and target_positions is not None:
                positions = self.read_positions().get("current", [])
                stall_event = self._check_position_stall(
                    current_positions=positions,
                    target_positions=target_positions,
                    last_positions=last_positions,
                    stall_since=stall_since,
                    hold_ms=position_stall_hold_ms,
                    min_delta=position_stall_min_delta,
                    error_counts=position_stall_error_counts,
                    elapsed_ms=elapsed_ms,
                    now=now,
                )
                last_positions = list(positions) if positions else last_positions
                if stall_event is not None:
                    return stall_event

            time.sleep(max(interval_ms, 1) / 1000.0)

        return None

    def _check_position_stall(
        self,
        current_positions: Sequence[Optional[int]],
        target_positions: Sequence[int],
        last_positions: Optional[Sequence[Optional[int]]],
        stall_since: Dict[int, float],
        hold_ms: int,
        min_delta: int,
        error_counts: int,
        elapsed_ms: int,
        now: float,
    ) -> Optional[ResistanceEvent]:
        """Detect blocked closing by position, independent of current feedback.

        A finger is considered stalled only when it is still far from the target,
        the target is in the closing direction, and the measured position has not
        changed enough for ``hold_ms``. This is intentionally conservative so an
        already-open/settled finger does not look like a fault.
        """
        if len(target_positions) != NUM_MOTORS or len(current_positions) != NUM_MOTORS:
            stall_since.clear()
            return None

        for finger_id, current in enumerate(current_positions):
            target = target_positions[finger_id]
            if current is None:
                stall_since.pop(finger_id, None)
                continue

            # Closing is increasing logical position for these ROH gestures.
            remaining = target - current
            if remaining <= max(error_counts, 0):
                stall_since.pop(finger_id, None)
                continue

            if last_positions is None or finger_id >= len(last_positions):
                stall_since[finger_id] = now
                continue

            previous = last_positions[finger_id]
            if previous is None:
                stall_since[finger_id] = now
                continue

            moved = abs(current - previous)
            if moved >= max(min_delta, 0):
                stall_since[finger_id] = now
                continue

            start = stall_since.setdefault(finger_id, now)
            if int((now - start) * 1000) >= hold_ms:
                return ResistanceEvent(
                    signal="position_stall",
                    finger_id=finger_id,
                    value=int(remaining),
                    threshold=error_counts,
                    action="stop",
                    elapsed_ms=elapsed_ms,
                )

        return None

    def send_positions(self, positions: Sequence[int], speed: int | Sequence[int] = DEFAULT_SPEED) -> None:
        pos = _normalize_positions(positions)
        speeds = _normalize_speeds(speed)
        if self.dry_run:
            print(f"[dry-run] HAND_SetFingerPosAll hand=0x{self.hand_id:02X} pos={pos} speed={speeds}")
            return

        api = self._require_api()
        remote_err: List[int] = []
        err = api.HAND_SetFingerPosAll(self.hand_id, pos, speeds, NUM_MOTORS, remote_err)
        self._raise_if_error("HAND_SetFingerPosAll", err, remote_err)

    def safe_stop(self, finger_bits: int = ALL_FINGER_BITS) -> None:
        if self.dry_run:
            print(f"[dry-run] HAND_FingerStop hand=0x{self.hand_id:02X} bits=0x{finger_bits:02X}")
            return

        api = self._require_api()
        remote_err: List[int] = []
        err = api.HAND_FingerStop(self.hand_id, finger_bits, remote_err)
        self._raise_if_error("HAND_FingerStop", err, remote_err)

    def configure_stall_protection(
        self,
        speed: int = 16,
        stop_current_ma: int = 200,
        stop_after_ms: int = 300,
        retry_after_ms: int = 500,
    ) -> None:
        if self.dry_run:
            print(
                "[dry-run] HAND_SetFingerStopParams "
                f"speed={speed} current={stop_current_ma} stop_after={stop_after_ms} retry={retry_after_ms}"
            )
            return

        api = self._require_api()
        for finger_id in range(NUM_MOTORS):
            remote_err: List[int] = []
            err = api.HAND_SetFingerStopParams(
                self.hand_id,
                finger_id,
                _clamp_int(speed, 0, 65535),
                _clamp_int(stop_current_ma, 0, 65535),
                _clamp_int(stop_after_ms, 0, 65535),
                _clamp_int(retry_after_ms, 0, 65535),
                remote_err,
            )
            self._raise_if_error(f"HAND_SetFingerStopParams[{finger_id}]", err, remote_err)

    def reset_force(self) -> None:
        if self.dry_run:
            print(f"[dry-run] HAND_ResetForce hand=0x{self.hand_id:02X}")
            return

        api = self._require_api()
        remote_err: List[int] = []
        err = api.HAND_ResetForce(self.hand_id, remote_err)
        self._raise_if_error("HAND_ResetForce", err, remote_err)

    def read_currents(self) -> List[Optional[int]]:
        if self.dry_run:
            return [0] * NUM_MOTORS

        api = self._require_api()
        values: List[Optional[int]] = []
        for finger_id in range(NUM_MOTORS):
            current = [0]
            remote_err: List[int] = []
            err, value = api.HAND_GetFingerCurrent(self.hand_id, finger_id, current, remote_err)
            values.append(value if err == HAND_RESP_SUCCESS else None)
        return values

    def read_positions(self) -> Dict[str, List[Optional[int]]]:
        if self.dry_run:
            return {"target": [None] * NUM_MOTORS, "current": [None] * NUM_MOTORS}

        api = self._require_api()
        target = [0] * NUM_MOTORS
        current = [0] * NUM_MOTORS
        motor_count = [NUM_MOTORS]
        remote_err: List[int] = []
        err, target, current = api.HAND_GetFingerPosAll(self.hand_id, target, current, motor_count, remote_err)
        if err != HAND_RESP_SUCCESS:
            return {"target": [None] * NUM_MOTORS, "current": [None] * NUM_MOTORS}
        return {"target": list(target), "current": list(current)}

    def read_force_values(self, finger_id: int) -> Optional[List[int]]:
        if self.dry_run:
            return [0]

        api = self._require_api()
        data = bytearray([_clamp_int(finger_id, 0, 255)])
        out = bytearray(2 + MAX_FORCE_ENTRIES * 2)
        remote_err: List[int] = []
        err = api.HAND_SendCmd(self.hand_id, HAND_CMD_GET_FINGER_FORCE, data, len(data))
        if err == HAND_RESP_SUCCESS:
            err = api.HAND_GetResponse(
                self.hand_id,
                HAND_CMD_GET_FINGER_FORCE,
                self.command_timeout_ms,
                out,
                remote_err,
            )
        if err != HAND_RESP_SUCCESS or len(out) < 2 or out[0] != finger_id:
            return None

        entry_count = min(out[1], MAX_FORCE_ENTRIES, max((len(out) - 2) // 2, 0))
        values = []
        for index in range(entry_count):
            low = out[2 + 2 * index]
            high = out[3 + 2 * index]
            values.append(low | (high << 8))
        return values

    def read_force_maxima(self) -> List[Optional[int]]:
        maxima: List[Optional[int]] = []
        for finger_id in range(NUM_MOTORS):
            values = self.read_force_values(finger_id)
            if not values:
                maxima.append(None)
                continue
            valid_values = [value for value in values if value < 65535]
            maxima.append(max(valid_values) if valid_values else None)
        return maxima

    def read_telemetry(self) -> Dict[str, object]:
        positions = self.read_positions()
        return {
            "current_ma": self.read_currents(),
            "force_mn": self.read_force_maxima(),
            "target_pos": positions["target"],
            "current_pos": positions["current"],
        }

    @classmethod
    def gesture_names(cls) -> List[str]:
        return sorted(GESTURE_POSITIONS.keys())

    @classmethod
    def list_gestures(cls) -> List[str]:
        """Legacy-compatible name list, including aliases."""
        names = set(GESTURE_POSITIONS.keys()) | set(GESTURE_ALIASES.keys())
        return sorted(names)

    @classmethod
    def describe(cls, name: str) -> str:
        """Legacy-compatible description, including aliases."""
        canonical = GESTURE_ALIASES.get(name, name)
        if canonical in GESTURE_DESCRIPTIONS:
            desc = GESTURE_DESCRIPTIONS[canonical]
            if name in GESTURE_ALIASES:
                return f"Alias for '{canonical}'. {desc}"
            return desc
        return "No description available."

    @classmethod
    def gesture_steps(
        cls,
        name: str,
        speed: int = DEFAULT_SPEED,
        settle_ms: int = DEFAULT_SETTLE_MS,
    ) -> Tuple[GestureStep, ...]:
        key = name.lower()
        key = GESTURE_ALIASES.get(key, key)
        if key not in GESTURE_POSITIONS:
            known = ", ".join(cls.gesture_names())
            raise ValueError(f"Unknown gesture '{name}'. Known gestures: {known}")
        return (GestureStep(GESTURE_POSITIONS[key], _clamp_int(speed, 0, 255), settle_ms),)

    def _check_soft_and_hard_thresholds(
        self,
        signal: str,
        values: Sequence[Optional[int]],
        soft_threshold: int,
        hard_threshold: int,
        hard_since: Dict[Tuple[str, int], float],
        soft_seen: set[Tuple[str, int]],
        hold_ms: int,
        elapsed_ms: int,
        on_soft: Optional[Callable[[ResistanceEvent], None]],
    ) -> Optional[ResistanceEvent]:
        now = time.monotonic()
        for finger_id, value in enumerate(values):
            key = (signal, finger_id)
            if value is None:
                hard_since.pop(key, None)
                continue

            if soft_threshold > 0 and value >= soft_threshold and key not in soft_seen:
                soft_seen.add(key)
                if on_soft is not None:
                    on_soft(
                        ResistanceEvent(
                            signal=signal,
                            finger_id=finger_id,
                            value=value,
                            threshold=soft_threshold,
                            action="slow",
                            elapsed_ms=elapsed_ms,
                        )
                    )

            if hard_threshold <= 0:
                continue

            if value >= hard_threshold:
                start = hard_since.setdefault(key, now)
                if int((now - start) * 1000) >= hold_ms:
                    return ResistanceEvent(
                        signal=signal,
                        finger_id=finger_id,
                        value=value,
                        threshold=hard_threshold,
                        action="stop",
                        elapsed_ms=elapsed_ms,
                    )
            else:
                hard_since.pop(key, None)
        return None

    def _require_api(self) -> OHandSerialAPI:
        if self.api is None:
            raise RuntimeError("ROH controller is not connected. Use connect() or a context manager.")
        return self.api

    def _raise_if_error(self, operation: str, err: int, remote_err: Sequence[int]) -> None:
        if err == HAND_RESP_SUCCESS:
            return
        extra = f", remote_err={list(remote_err)}" if remote_err else ""
        hint = ""
        if err == HAND_RESP_UNMATCHED_CMD:
            hint = (
                " (hint: another ROH controller may be using the same CAN channel; "
                "only one process may control the hand on a given interface)"
            )
        raise RuntimeError(f"{operation} failed: err={err}{extra}{hint}")

    @staticmethod
    def _can_channel_to_port(channel: str) -> str:
        text = str(channel).strip().lower()
        if text.startswith("can"):
            text = text[3:]
        if not text:
            raise ValueError(f"Invalid CAN channel: {channel!r}")
        return text


def _normalize_positions(positions: Sequence[int]) -> List[int]:
    if len(positions) != NUM_MOTORS:
        raise ValueError(f"Expected {NUM_MOTORS} motor positions, got {len(positions)}")
    return [_clamp_int(value, 0, 65535) for value in positions]


def _normalize_speeds(speed: int | Sequence[int]) -> List[int]:
    if isinstance(speed, Iterable) and not isinstance(speed, (bytes, bytearray, str)):
        speeds = list(speed)
        if len(speeds) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor speeds, got {len(speeds)}")
        return [_clamp_int(value, 0, 255) for value in speeds]
    return [_clamp_int(int(speed), 0, 255)] * NUM_MOTORS


def _clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))
