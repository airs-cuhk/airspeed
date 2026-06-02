"""Recording state machine — single source of truth for episode lifecycle."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Mapping


class RecordingLifecycle(str, Enum):
    IDLE = "idle"
    RECORDING = "recording"
    FAILED = "failed"


class RecordingStateError(ValueError):
    """Raised when a recording state transition is invalid."""


EpisodeStartHandler = Callable[[str], Any]
EpisodeEndHandler = Callable[[bool, str], Any]


@dataclass(frozen=True)
class RecordingHealthSnapshot:
    lifecycle: RecordingLifecycle
    active_episode_id: str | None = None
    step_count: int = 0


class RecordingStateMachine:
    """Owns the recording lifecycle. All control modes call the same methods."""

    def __init__(
        self,
        start_handler: EpisodeStartHandler,
        end_handler: EpisodeEndHandler,
    ) -> None:
        self._lifecycle = RecordingLifecycle.IDLE
        self._active_episode_id: str | None = None
        self._start_handler = start_handler
        self._end_handler = end_handler

    # -- queries --

    @property
    def lifecycle(self) -> RecordingLifecycle:
        return self._lifecycle

    @property
    def is_idle(self) -> bool:
        return self._lifecycle == RecordingLifecycle.IDLE

    @property
    def is_recording(self) -> bool:
        return self._lifecycle == RecordingLifecycle.RECORDING

    @property
    def active_episode_id(self) -> str | None:
        return self._active_episode_id

    @property
    def health_snapshot(self) -> RecordingHealthSnapshot:
        return RecordingHealthSnapshot(
            lifecycle=self._lifecycle,
            active_episode_id=self._active_episode_id,
        )

    # -- actions (the single control switch) --

    def start_episode(self, episode_id: str) -> None:
        if self._lifecycle == RecordingLifecycle.FAILED:
            raise RecordingStateError("cannot start: recorder is in FAILED state")
        if self._lifecycle == RecordingLifecycle.RECORDING:
            raise RecordingStateError(f"cannot start: already recording {self._active_episode_id}")
        self._start_handler(episode_id)
        self._lifecycle = RecordingLifecycle.RECORDING
        self._active_episode_id = episode_id

    def end_episode(self, success: bool, reason: str) -> None:
        if self._lifecycle != RecordingLifecycle.RECORDING:
            raise RecordingStateError("cannot end: not recording")
        self._lifecycle = RecordingLifecycle.IDLE
        self._active_episode_id = None
        self._end_handler(success, reason)

    def abort_episode(self) -> None:
        self.end_episode(success=False, reason="operator_abort")

    def fail(self) -> None:
        self._lifecycle = RecordingLifecycle.FAILED
        self._active_episode_id = None


__all__ = [
    "RecordingHealthSnapshot",
    "RecordingLifecycle",
    "RecordingStateError",
    "RecordingStateMachine",
]
