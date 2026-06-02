"""Shared helpers for ROS2 message extraction and canonical sample building."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping, Sequence

from core.config import StreamConfig, TimeDomain
from core.contracts import (
    AdapterBoundarySample,
    CanonicalStreamIdentity,
    ContractError,
)


class AdapterError(ValueError):
    """Raised when an adapter cannot map a raw source message."""


def build_boundary_sample(
    stream: StreamConfig, *, payload: Mapping[str, Any],
    received_at: datetime, source_timestamp: datetime | None,
) -> AdapterBoundarySample:
    if stream.time_domain == TimeDomain.ROS_HEADER and source_timestamp is None:
        raise AdapterError(f"{stream.name} requires header timestamp for ros_header")
    timestamp = source_timestamp if stream.time_domain == TimeDomain.ROS_HEADER else received_at
    try:
        return AdapterBoundarySample(
            stream=CanonicalStreamIdentity(stream.name, stream.source),
            timestamp=timestamp,
            payload=dict(payload),
            time_domain=stream.time_domain.value,
            source_timestamp=source_timestamp,
        )
    except ContractError as exc:
        raise AdapterError(str(exc)) from exc


def extract_header_timestamp(message: Any) -> datetime | None:
    header = _get_member(message, "header")
    if header is None:
        return None
    stamp = _get_member(header, "stamp")
    if stamp is None:
        return None
    if isinstance(stamp, datetime):
        return stamp
    sec = _get_member(stamp, "sec")
    nanosec = _get_member(stamp, "nanosec")
    if sec is None:
        return None
    nanosec = nanosec or 0
    return datetime.fromtimestamp(float(sec) + float(nanosec) / 1e9, tz=timezone.utc)


def extract_pose_payload(message: Any, *, fallback_frame_id: str | None = None) -> dict[str, Any]:
    pose = _get_member(message, "pose") or message
    position = _get_member(pose, "position")
    orientation = _get_member(pose, "orientation") or _get_member(pose, "rotation")
    if position is None or orientation is None:
        raise AdapterError("pose message must define position and orientation")
    payload: dict[str, Any] = {
        "position": {"x": _get_float(position, "x"), "y": _get_float(position, "y"), "z": _get_float(position, "z")},
        "orientation": {"x": _get_float(orientation, "x"), "y": _get_float(orientation, "y"),
                        "z": _get_float(orientation, "z"), "w": _get_float(orientation, "w")},
    }
    frame_id = _extract_frame_id(message) or fallback_frame_id
    if frame_id is not None:
        payload["frame_id"] = frame_id
    return payload


def extract_numeric_sequence(message: Any, *, field_name: str = "data") -> Sequence[float]:
    data = _get_member(message, field_name) if field_name else message
    if data is None:
        raise AdapterError(f"{field_name} must be a numeric sequence")
    if isinstance(data, (bytes, bytearray)):
        return list(data)
    if isinstance(data, Sequence) and not isinstance(data, str):
        return [float(v) for v in data]
    if hasattr(data, "__float__"):
        return [float(data)]
    raise AdapterError(f"{field_name} must be a numeric sequence")


def extract_image_payload(message: Any, *, fallback_frame_id: str | None = None) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "height": _get_int(message, "height"),
        "width": _get_int(message, "width"),
        "encoding": _get_str(message, "encoding"),
        "is_bigendian": _get_int(message, "is_bigendian"),
        "step": _get_int(message, "step"),
        "data": _extract_image_data(_get_member(message, "data")),
    }
    frame_id = _extract_frame_id(message) or fallback_frame_id
    if frame_id is not None:
        payload["frame_id"] = frame_id
    return payload


def _extract_image_data(value: Any) -> bytes:
    if isinstance(value, (bytes, bytearray)):
        return bytes(value)
    try:
        view = memoryview(value)
        if view.itemsize == 1 and view.format in {"B", "b", "c"}:
            return view.tobytes()
    except TypeError:
        pass
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return bytes(value)
    raise AdapterError("image data must be bytes or a numeric sequence")


def _extract_frame_id(message: Any) -> str | None:
    header = _get_member(message, "header")
    if header is None:
        return None
    frame_id = _get_member(header, "frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        return None
    return frame_id


def _get_member(value: Any, name: str) -> Any:
    if isinstance(value, Mapping):
        return value.get(name)
    return getattr(value, name, None)


def _get_float(obj: Any, name: str) -> float:
    v = _get_member(obj, name)
    if v is None:
        raise AdapterError(f"missing field: {name}")
    return float(v)


def _get_int(obj: Any, name: str) -> int:
    v = _get_member(obj, name)
    if v is None:
        raise AdapterError(f"missing field: {name}")
    if isinstance(v, bool) or not isinstance(v, int):
        raise AdapterError(f"{name} must be an integer")
    return v


def _get_str(obj: Any, name: str) -> str:
    v = _get_member(obj, name)
    if not isinstance(v, str) or not v.strip():
        raise AdapterError(f"{name} must be a non-empty string")
    return v


__all__ = [
    "AdapterError",
    "build_boundary_sample",
    "extract_header_timestamp",
    "extract_image_payload",
    "extract_numeric_sequence",
    "extract_pose_payload",
]
