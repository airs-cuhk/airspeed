"""Thread-safe StreamTracker — MultiThreadedExecutor compatible.

Same API as runtime/stream_tracker.py, with threading.Lock on metrics dicts.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading
import time as _time
from typing import Any


class StreamStatus(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    STALE = "stale"
    ABSENT = "absent"
    INVALID = "invalid"


@dataclass(frozen=True)
class StreamMetrics:
    messages_received: int = 0
    messages_valid: int = 0
    messages_invalid: int = 0
    last_timestamp_ns: int | None = None
    last_timestamp_age_ms: float | None = None
    first_timestamp_ns: int | None = None
    observed_rate_hz: float | None = None


class StreamTracker:
    """Tracks per-stream metrics and derives status — thread-safe."""

    def __init__(self, stream_names: list[str], expected_periods_ms: dict[str, float]) -> None:
        self._metrics: dict[str, dict[str, Any]] = {
            name: {
                "received": 0, "valid": 0, "invalid": 0,
                "last_ts_ns": None, "last_mono_ns": None,
                "first_ts_ns": None, "first_mono_ns": None,
            }
            for name in stream_names
        }
        self._periods_ms = expected_periods_ms
        self._prev_ts_ns: dict[str, list[int]] = {name: [] for name in stream_names}
        self._lock = threading.Lock()

    def record_valid(self, name: str, timestamp_ns: int) -> None:
        with self._lock:
            m = self._metrics[name]
            m["received"] += 1
            m["valid"] += 1
            self._update_timing(name, m, timestamp_ns)

    def record_invalid(self, name: str, timestamp_ns: int | None) -> None:
        with self._lock:
            m = self._metrics[name]
            m["received"] += 1
            m["invalid"] += 1
            if timestamp_ns is not None:
                self._update_timing(name, m, timestamp_ns)

    def reset(self) -> None:
        with self._lock:
            for name in self._metrics:
                self._metrics[name] = {
                    "received": 0, "valid": 0, "invalid": 0,
                    "last_ts_ns": None, "last_mono_ns": None,
                    "first_ts_ns": None, "first_mono_ns": None,
                }
                self._prev_ts_ns[name] = []

    def snapshot(self) -> dict[str, tuple[StreamStatus, StreamMetrics]]:
        with self._lock:
            now_mono = _time.monotonic_ns()
            result: dict[str, tuple[StreamStatus, StreamMetrics]] = {}
            for name, m in self._metrics.items():
                metrics = StreamMetrics(
                    messages_received=m["received"],
                    messages_valid=m["valid"],
                    messages_invalid=m["invalid"],
                    last_timestamp_ns=m["last_ts_ns"],
                    last_timestamp_age_ms=(
                        (now_mono - m["last_mono_ns"]) / 1e6
                        if m["last_mono_ns"] is not None else None
                    ),
                    first_timestamp_ns=m["first_ts_ns"],
                    observed_rate_hz=self._compute_rate(name),
                )
                status = self._derive_status(name, metrics)
                result[name] = (status, metrics)
            return result

    def _update_timing(self, name: str, m: dict, ts_ns: int) -> None:
        now_mono = _time.monotonic_ns()
        if m["first_ts_ns"] is None:
            m["first_ts_ns"] = ts_ns
            m["first_mono_ns"] = now_mono
        m["last_ts_ns"] = ts_ns
        m["last_mono_ns"] = now_mono
        buf = self._prev_ts_ns[name]
        buf.append(ts_ns)
        if len(buf) > 20:
            buf.pop(0)

    def _derive_status(self, name: str, m: StreamMetrics) -> StreamStatus:
        if m.messages_received == 0:
            return StreamStatus.ABSENT
        if m.messages_valid == 0 and m.messages_received > 0:
            return StreamStatus.INVALID
        if m.observed_rate_hz is not None and m.observed_rate_hz > 0:
            observed_period_ms = 1000.0 / m.observed_rate_hz
        else:
            observed_period_ms = self._periods_ms.get(name, 500.0)
        stale_threshold_ms = max(5 * observed_period_ms, 1000.0)
        if m.last_timestamp_age_ms is not None and m.last_timestamp_age_ms > stale_threshold_ms:
            return StreamStatus.STALE
        if m.messages_invalid > 0:
            return StreamStatus.DEGRADED
        return StreamStatus.HEALTHY

    def _compute_rate(self, name: str) -> float | None:
        buf = self._prev_ts_ns[name]
        if len(buf) < 2:
            return None
        diffs = [(buf[i] - buf[i - 1]) / 1e9 for i in range(1, len(buf))]
        if not diffs:
            return None
        avg_diff = sum(diffs) / len(diffs)
        return 1.0 / avg_diff if avg_diff > 0 else None


__all__ = ["StreamMetrics", "StreamStatus", "StreamTracker"]
