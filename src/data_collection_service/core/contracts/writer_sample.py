"""WriterSample: what the adapter produces, what the writer consumes.

The node shell passes this directly through — it does not inspect payloads.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WriterSample:
    """One validated message, ready for the AIRS writer.

    The node shell does not open this envelope. It passes it to the writer
    verbatim. The adapter owns the decision of what goes in it.
    """

    stream_name: str
    timestamp_ns: int
    values: tuple[float, ...] | None = None   # for vector streams
    image_data: bytes | None = None            # for image streams


__all__ = ["WriterSample"]
