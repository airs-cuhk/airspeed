"""Recording control router — single switch for all three control modes.

manual_ui button ───┐
CLI service call ───┼──→ invoke_action("toggle") ──→ state machine transition
teleop binding ─────┘

Toggle: one button starts AND stops recording.
Delete: after stopping, press delete to move episode to .trash/.
        If not pressed before the next toggle-start, the episode is kept.
"""

from __future__ import annotations

import time as _time
from dataclasses import dataclass
from typing import Any, Callable

from core.config import RecordingControlConfig, RecordingControlMode
from core.runtime.recording_state import (
    RecordingLifecycle,
    RecordingStateError,
    RecordingStateMachine,
)


class RecordingControlError(ValueError):
    """Raised when recording control evaluation fails."""


@dataclass(frozen=True)
class RecordingControlResult:
    accepted: bool
    action: str | None = None
    message: str = ""
    episode_id: str | None = None

    @property
    def triggered(self) -> bool:
        return self.accepted and self.action is not None


class RecordingControlRouter:
    """Route episode control through the mode selected in session config."""

    def __init__(
        self,
        config: RecordingControlConfig,
        state_machine: RecordingStateMachine,
        *,
        on_delete_requested: Callable[[], bool] | None = None,
    ) -> None:
        self._config = config
        self._mode = config.mode
        self._state = state_machine
        self._toggle_debounce_s = config.toggle_debounce_s
        self._last_toggle_stop_time: float = 0.0
        self._pending_episode: bool = False
        self._on_delete_requested = on_delete_requested or (lambda: False)
        self._binding_states: dict[str, bool] = {
            action: False for action, _ in config.bindings
        }

    @property
    def mode(self) -> RecordingControlMode:
        return self._mode

    @property
    def pending_episode(self) -> bool:
        """True after toggle-stop, before next toggle-start or delete."""
        return self._pending_episode

    @property
    def last_episode_kept(self) -> bool:
        """After leaving PENDING: True if episode was kept, False if trashed."""
        return not self._pending_episode

    # -- unified entry point (all modes call this) --

    def invoke_action(
        self, action: str, *, source: RecordingControlMode | str,
    ) -> RecordingControlResult:
        normalized_source = RecordingControlMode(source)
        if normalized_source != self._mode:
            return RecordingControlResult(
                accepted=False,
                message=(
                    f"recording control mode {self._mode.value} does not accept "
                    f"{normalized_source.value} actions"
                ),
            )
        return self._invoke(action)

    # -- service mode --

    def handle_service_action(self, action: str) -> RecordingControlResult:
        return self.invoke_action(action, source=RecordingControlMode.SERVICE)

    def handle_service_delete(self) -> RecordingControlResult:
        return self.invoke_action("delete", source=RecordingControlMode.SERVICE)

    # -- manual UI mode --

    def handle_manual_ui_action(self, action: str) -> RecordingControlResult:
        return self.invoke_action(action, source=RecordingControlMode.MANUAL_UI)

    def handle_manual_ui_delete(self) -> RecordingControlResult:
        return self.invoke_action("delete", source=RecordingControlMode.MANUAL_UI)

    # -- device binding mode --

    def handle_stream_message(
        self, stream_name: str, message: Any,
    ) -> RecordingControlResult:
        if self._mode != RecordingControlMode.DEVICE_BINDING:
            return RecordingControlResult(accepted=False)

        result = RecordingControlResult(accepted=False)
        for action, binding in self._config.bindings:
            if binding.stream_name != stream_name:
                continue
            active = self._binding_is_active(binding, message)
            if not active:
                if self._binding_states.get(action, False):
                    self._binding_states[action] = False
                continue
            if self._binding_states.get(action, False):
                continue  # still held — debounce
            self._binding_states[action] = True

            if action == "toggle":
                result = self._handle_toggle()
            elif action == "delete":
                result = self._handle_delete()

        return result

    def _handle_toggle(self) -> RecordingControlResult:
        if self._state.is_recording:
            # Stop: save episode, enter PENDING, record debounce timestamp
            result = self._invoke("stop")
            if result.accepted:
                self._pending_episode = True
                self._last_toggle_stop_time = _time.time()
            return result
        else:
            # Start: check debounce window first
            elapsed = _time.time() - self._last_toggle_stop_time
            if self._pending_episode and elapsed < self._toggle_debounce_s:
                return RecordingControlResult(
                    accepted=False,
                    message=f"debounce: wait {self._toggle_debounce_s - elapsed:.1f}s before starting",
                )
            self._pending_episode = False
            return self._invoke("start")

    def _handle_delete(self) -> RecordingControlResult:
        if not self._pending_episode:
            return RecordingControlResult(accepted=False, message="no pending episode to delete")
        ok = self._on_delete_requested()
        if ok:
            self._pending_episode = False
            return RecordingControlResult(accepted=True, action="delete", message="episode moved to trash")
        return RecordingControlResult(accepted=False, message="delete failed")

    # -- internal --

    def _invoke(self, action: str) -> RecordingControlResult:
        normalized = _normalize_action(action)

        if normalized == "delete":
            return self._handle_delete()

        if normalized == "toggle":
            return self._handle_toggle()

        if normalized == "start":
            if self._state.lifecycle == RecordingLifecycle.FAILED:
                return RecordingControlResult(accepted=False, message="recorder is in FAILED state")
            if self._state.is_recording:
                return RecordingControlResult(accepted=False, message="already recording")
            try:
                self._state.start_episode(_generate_episode_id())
            except RecordingStateError as exc:
                return RecordingControlResult(accepted=False, message=str(exc))
            self._pending_episode = False
            return RecordingControlResult(
                accepted=True, action="start",
                episode_id=self._state.active_episode_id,
                message=f"started {self._state.active_episode_id}",
            )

        if not self._state.is_recording:
            return RecordingControlResult(accepted=False, message="not recording")

        if normalized == "stop":
            self._state.end_episode(success=True, reason="operator_stop")
            return RecordingControlResult(accepted=True, action="stop", message="stopped")
        if normalized == "save":
            self._state.end_episode(success=True, reason="completed")
            return RecordingControlResult(accepted=True, action="save", message="saved")
        if normalized == "abort":
            self._state.abort_episode()
            self._pending_episode = False
            return RecordingControlResult(accepted=True, action="abort", message="aborted")

        return RecordingControlResult(accepted=False, message=f"unknown action: {action}")

    def _binding_is_active(self, binding, message: Any) -> bool:
        values = _extract_numeric_sequence(message, binding.field_name)
        if binding.button_index is not None:
            if binding.button_index >= len(values):
                return False
            value = values[binding.button_index]
        else:
            value = values[0] if len(values) == 1 else 0.0
        threshold = 0.0 if binding.threshold is None else float(binding.threshold)
        return value >= threshold if binding.threshold is not None else value > 0.0


def _normalize_action(action: str) -> str:
    action = action.strip()
    if action not in {"start", "stop", "abort", "save", "toggle", "delete"}:
        raise RecordingControlError(f"invalid action: {action!r}")
    return action


def _extract_numeric_sequence(message: Any, field_name: str) -> list[float]:
    value = getattr(message, field_name, None)
    if value is None and isinstance(message, dict):
        value = message.get(field_name)
    if value is None and field_name == "buttons":
        value = getattr(message, "data", None)
        if value is None and isinstance(message, dict):
            value = message.get("data")
    if value is None:
        return []
    if hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
        return [float(v) for v in value]
    return [float(value)]


def _generate_episode_id() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).strftime("episode-%Y%m%dT%H%M%S%fZ")


__all__ = ["RecordingControlError", "RecordingControlResult", "RecordingControlRouter"]
