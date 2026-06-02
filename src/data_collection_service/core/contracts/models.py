"""Canonical adapter-boundary contracts for the data collection service."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Any, Mapping


SCHEMA_VERSION = "1.0"


class ContractError(ValueError):
    """Raised when a canonical contract instance is invalid."""


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class CanonicalStreamFamily(str, Enum):
    TELEOP = "teleop"
    ROBOT = "robot"
    SENSOR = "sensor"
    ACTION = "action"


class CanonicalTimeDomain(str, Enum):
    ROS_HEADER = "ros_header"
    ROS_RECEIVE = "ros_receive"
    SYSTEM_CLOCK = "system_clock"
    STEADY_CLOCK = "steady_clock"


_ALLOWED_FAMILIES = frozenset(f.value for f in CanonicalStreamFamily)
_ALLOWED_DOMAINS = frozenset(d.value for d in CanonicalTimeDomain)


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CanonicalStreamIdentity:
    """Transport-agnostic identity for one configured ingress stream."""

    stream_name: str
    source_family: str

    def __post_init__(self) -> None:
        _require_non_empty("CanonicalStreamIdentity.stream_name", self.stream_name)
        _require_non_empty("CanonicalStreamIdentity.source_family", self.source_family)
        if self.source_family not in _ALLOWED_FAMILIES:
            raise ContractError(
                f"CanonicalStreamIdentity.source_family must be one of "
                f"{sorted(_ALLOWED_FAMILIES)}; got {self.source_family!r}"
            )

    def to_dict(self) -> dict[str, str]:
        return {"stream_name": self.stream_name, "source_family": self.source_family}


@dataclass(frozen=True)
class AdapterBoundarySample:
    """Canonical envelope emitted by every adapter before storage.

    This is the single contract that every ingress path must cross.
    Raw ROS2/websocket/device-specific objects must not pass beyond this point.
    """

    stream: CanonicalStreamIdentity
    timestamp: datetime
    payload: dict[str, Any]
    time_domain: str
    source_timestamp: datetime | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.stream, CanonicalStreamIdentity):
            raise ContractError("AdapterBoundarySample.stream must be a CanonicalStreamIdentity")
        _require_datetime(self.timestamp, "AdapterBoundarySample.timestamp")
        _require_datetime(self.source_timestamp, "AdapterBoundarySample.source_timestamp",
                          optional=True)
        _require_non_empty("AdapterBoundarySample.time_domain", self.time_domain)
        if self.time_domain not in _ALLOWED_DOMAINS:
            raise ContractError(
                f"AdapterBoundarySample.time_domain must be one of "
                f"{sorted(_ALLOWED_DOMAINS)}; got {self.time_domain!r}"
            )
        if not isinstance(self.payload, Mapping) or not self.payload:
            raise ContractError("AdapterBoundarySample.payload must be a non-empty mapping")
        for key in self.payload:
            if not isinstance(key, str) or not key.strip():
                raise ContractError("AdapterBoundarySample.payload keys must be non-empty strings")
        object.__setattr__(self, "payload", dict(self.payload))

        if self.time_domain == CanonicalTimeDomain.ROS_HEADER:
            if self.source_timestamp is None:
                raise ContractError(
                    "AdapterBoundarySample.source_timestamp is required when "
                    "time_domain=ros_header"
                )
            if self.timestamp != self.source_timestamp:
                raise ContractError(
                    "AdapterBoundarySample.timestamp must equal source_timestamp "
                    "when time_domain=ros_header"
                )

    @property
    def stream_name(self) -> str:
        return self.stream.stream_name

    @property
    def source_family(self) -> str:
        return self.stream.source_family

    def to_dict(self) -> dict[str, Any]:
        return {
            "stream": self.stream.to_dict(),
            "timestamp": self.timestamp.isoformat(),
            "payload": dict(self.payload),
            "time_domain": self.time_domain,
            "source_timestamp": (
                self.source_timestamp.isoformat() if self.source_timestamp is not None else None
            ),
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _require_non_empty(field: str, value: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f"{field} must be a non-empty string")


def _require_datetime(
    value: datetime | None, field: str, *, optional: bool = False,
) -> None:
    if optional and value is None:
        return
    if not isinstance(value, datetime):
        raise ContractError(f"{field} must be a datetime")
    if value.tzinfo is None or value.utcoffset() is None:
        raise ContractError(f"{field} must be timezone-aware")


__all__ = [
    "AdapterBoundarySample",
    "CanonicalStreamFamily",
    "CanonicalStreamIdentity",
    "CanonicalTimeDomain",
    "ContractError",
    "SCHEMA_VERSION",
]
