"""Audit logger — in-memory ring buffer of per-cycle solver I/O."""

from __future__ import annotations

import csv
import io
import json
import time
from collections import deque
from typing import Any


class AuditLogger:
    def __init__(self, max_entries: int = 10000) -> None:
        self._entries: deque[dict[str, Any]] = deque(maxlen=max_entries)
        self.count: int = 0

    def append(self, **kwargs: Any) -> None:
        entry = {"timestamp_ns": time.time_ns(), **kwargs}
        self._entries.append(entry)
        self.count += 1

    def get_entries(self) -> list[dict[str, Any]]:
        return list(self._entries)

    def to_json(self) -> str:
        return json.dumps(list(self._entries), indent=2)

    def to_csv(self) -> str:
        if not self._entries:
            return ""
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=self._entries[0].keys())
        writer.writeheader()
        writer.writerows(self._entries)
        return output.getvalue()
