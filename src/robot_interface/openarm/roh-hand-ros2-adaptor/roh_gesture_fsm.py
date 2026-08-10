"""VR → ROH hand gesture state machine (pure logic, ROS-free).

Ported from the partner's RohHandVRController._tick (115 stack) with two
canonical changes:
- input is sensor_msgs/Joy ``axes`` (indices verified in pipeline step 01:
  0=trigger, 3=thumbstick click, 4=A, 5=B) instead of Float32MultiArray
- output is a 6-finger RADIAN command (0 = open) instead of raw SDK counts;
  conversion happens here via per-finger closed_rad so the driver stays
  unit-agnostic

Semantics preserved:
- trigger interpolates ready→grasp pose for the current mode
- thumbstick click (rising edge) cycles five → two → point; point holds the
  point pose and ignores the trigger
- open button (default right B) opens the hand and latches; clear button
  (default right A) releases the latch
- resistance/contact latching is NOT here — the driver enforces it and
  releases on retreat commands, which the falling trigger naturally produces
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from roh_units import NUM_FINGERS, counts_to_rad

_MODE_GESTURES = {
    "five": ("five_finger_ready_grasp", "five_finger_grasp"),
    "two": ("two_finger_ready_grasp", "two_finger_grasp"),
    "point": (None, "point"),
}
_CYCLE_ORDER = ["five", "two", "point"]
_BUTTON_PRESS_THRESHOLD = 0.5

# Fallback poses in counts, used only if a gesture name is missing from the
# wrapper's GESTURE_POSITIONS (same fallbacks as the 115 controller).
_FALLBACK_COUNTS = {
    "open": (0, 0, 0, 0, 0, 0),
    "point": (45000, 0, 65535, 65535, 65535, 0),
}


@dataclass
class VrFsmConfig:
    side: str = "left"
    trigger_index: int = 0
    gesture_cycle_index: int = 3
    open_button_side: str = "right"
    open_button_index: int = 5
    clear_latch_button_side: str = "right"
    clear_latch_button_index: int = 4
    trigger_deadzone: float = 0.03
    default_mode: str = "five"
    closed_rad: list[float] | None = None


class RohGestureFSM:
    """Button states in, radian hand command out (None = hold current pose)."""

    def __init__(self, config: VrFsmConfig, gesture_positions: dict[str, tuple[int, ...]]) -> None:
        self.config = config
        if config.closed_rad is None or len(config.closed_rad) != NUM_FINGERS:
            raise ValueError("closed_rad must have 6 entries")
        self.mode = config.default_mode if config.default_mode in _MODE_GESTURES else "five"
        self._open_latched = False
        self._prev_open = False
        self._prev_clear = False
        self._prev_cycle = False
        self._gestures_rad = {
            name: self._counts_to_rad(tuple(counts))
            for name, counts in gesture_positions.items()
            if counts is not None and len(counts) == NUM_FINGERS
        }

    def _counts_to_rad(self, counts: tuple[int, ...]) -> list[float]:
        return [counts_to_rad(c, cr) for c, cr in zip(counts, self.config.closed_rad)]

    def _gesture(self, name: str) -> list[float]:
        if name in self._gestures_rad:
            return self._gestures_rad[name]
        return self._counts_to_rad(_FALLBACK_COUNTS[name])

    @staticmethod
    def _btn(buttons: list[float], index: int) -> float:
        return buttons[index] if 0 <= index < len(buttons) else 0.0

    @property
    def open_latched(self) -> bool:
        return self._open_latched

    def tick(
        self,
        own_buttons: list[float],
        other_buttons: list[float],
    ) -> tuple[list[float] | None, str | None]:
        """Advance one control tick.

        own_buttons: Joy axes of this hand's VR controller side.
        other_buttons: Joy axes of the OTHER side (open/clear buttons, which
        default to the right controller for both hands).

        Returns (command_rad | None, event | None). command_rad is a 6-float
        radian command to publish; None means "hold current pose, publish
        nothing". event is a short log string for mode/latch transitions.
        """
        cfg = self.config
        open_buttons = other_buttons if cfg.open_button_side != cfg.side else own_buttons
        clear_buttons = other_buttons if cfg.clear_latch_button_side != cfg.side else own_buttons

        open_pressed = self._btn(open_buttons, cfg.open_button_index) > _BUTTON_PRESS_THRESHOLD
        clear_pressed = self._btn(clear_buttons, cfg.clear_latch_button_index) > _BUTTON_PRESS_THRESHOLD
        cycle_pressed = self._btn(own_buttons, cfg.gesture_cycle_index) > _BUTTON_PRESS_THRESHOLD
        trigger = min(max(self._btn(own_buttons, cfg.trigger_index), 0.0), 1.0)
        if trigger < cfg.trigger_deadzone:
            trigger = 0.0

        open_rising = open_pressed and not self._prev_open
        clear_rising = clear_pressed and not self._prev_clear
        cycle_rising = cycle_pressed and not self._prev_cycle
        self._prev_open, self._prev_clear, self._prev_cycle = (
            open_pressed, clear_pressed, cycle_pressed)

        if open_rising:
            self._open_latched = True
            return self._gesture("open"), f"open latched ({cfg.side})"

        if clear_rising:
            self._open_latched = False
            return None, "open latch cleared"

        if self._open_latched:
            return None, None

        if cycle_rising:
            idx = _CYCLE_ORDER.index(self.mode)
            self.mode = _CYCLE_ORDER[(idx + 1) % len(_CYCLE_ORDER)]
            if self.mode == "point":
                return self._gesture("point"), f"mode -> point ({cfg.side})"
            ready_name, _ = _MODE_GESTURES[self.mode]
            return self._gesture(ready_name), f"mode -> {self.mode} ({cfg.side})"

        if self.mode == "point":
            return None, None

        ready_name, grasp_name = _MODE_GESTURES[self.mode]
        ready = self._gesture(ready_name)
        grasp = self._gesture(grasp_name)
        command = [ready[i] + trigger * (grasp[i] - ready[i]) for i in range(NUM_FINGERS)]
        if not all(math.isfinite(v) for v in command):  # defensive: never emit NaN
            return None, "non-finite command suppressed"
        return command, None
