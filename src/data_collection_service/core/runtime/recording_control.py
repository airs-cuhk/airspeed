"""Recording control router — single switch for all three control modes.

manual_ui button ───┐
CLI service call ───┼──→ invoke_action("start") ──→ state machine transition
teleop binding ─────┘
"""

from __future__ import annotations

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
    ) -> None:
        self._config = config
        self._mode = config.mode
        self._state = state_machine
        self._binding_states: dict[str, bool] = {
            action: False for action, _ in config.bindings
        }

    @property
    def mode(self) -> RecordingControlMode:
        return self._mode

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

    # -- manual UI mode --

    def handle_manual_ui_action(self, action: str) -> RecordingControlResult:
        return self.invoke_action(action, source=RecordingControlMode.MANUAL_UI)

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
                # Reset debounce on release
                if self._binding_states.get(action, False):
                    self._binding_states[action] = False
                continue
            if self._binding_states.get(action, False):
                continue  # debounce: wait for release
            self._binding_states[action] = True
            if action == "start" and self._state.is_recording:
                continue
            if action in ("stop", "save", "abort") and not self._state.is_recording:
                continue
            result = self.invoke_action(action, source=RecordingControlMode.DEVICE_BINDING)

        return result

    # -- internal --

    def _invoke(self, action: str) -> RecordingControlResult:
        normalized = _normalize_action(action)

        if normalized == "start":
            if self._state.lifecycle == RecordingLifecycle.FAILED:
                return RecordingControlResult(accepted=False, message="recorder is in FAILED state")
            if self._state.is_recording:
                return RecordingControlResult(accepted=False, message="already recording")
            try:
                self._state.start_episode(_generate_episode_id())
            except RecordingStateError as exc:
                return RecordingControlResult(accepted=False, message=str(exc))
            return RecordingControlResult(
                accepted=True, action="start",
                episode_id=self._state.active_episode_id,
                message=f"started {self._state.active_episode_id}",
            )

        if not self._state.is_recording:
            return RecordingControlResult(accepted=False, message="not recording")

        if normalized == "stop":
            self._state.end_episode(success=False, reason="operator_stop")
            return RecordingControlResult(accepted=True, action="stop", message="stopped")
        if normalized == "save":
            self._state.end_episode(success=True, reason="completed")
            return RecordingControlResult(accepted=True, action="save", message="saved")
        if normalized == "abort":
            self._state.abort_episode()
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
    if action not in {"start", "stop", "abort", "save"}:
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
