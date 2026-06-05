"""Target buffer — atomic latest-wins buffer for SE3 targets from webui-monitor."""

from __future__ import annotations

import threading
from typing import Any


class TargetBuffer:
    """Thread-safe buffer that always holds the most recent target.

    Write: called from WebSocket handler (any thread).
    Read: called from solver loop (asyncio thread).
    Last write wins — older targets are silently discarded.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        self._has_new = False

    def write(self, target: dict[str, Any]) -> None:
        """Store a new target, replacing any previous one."""
        with self._lock:
            self._latest = target
            self._has_new = True

    def read(self) -> dict[str, Any] | None:
        """Return the latest target (or None if nothing was written)."""
        with self._lock:
            return self._latest

    def consume(self) -> dict[str, Any] | None:
        """Return the latest target and clear the new-flag."""
        with self._lock:
            self._has_new = False
            return self._latest

    @property
    def has_new(self) -> bool:
        with self._lock:
            return self._has_new
