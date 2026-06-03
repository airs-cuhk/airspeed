"""Config-driven adapter registry for per-stream message adaptation."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Mapping

from core.config import SessionConfig, StreamConfig
from core.contracts import WriterSample
from core.schema import (
    AdapterBoundaryValidationError,
    AdapterBoundaryValidator,
    AdapterPayloadProfileRegistry,
)
from core.adapters.common import (
    AdapterError,
    build_boundary_sample,
    extract_header_timestamp,
    extract_image_payload,
    extract_numeric_sequence,
    extract_pose_payload,
)

PayloadBuilder = Callable[[Any, StreamConfig], Mapping[str, Any]]


@dataclass(frozen=True)
class AdapterBinding:
    source: str
    message_type: str
    name: str
    payload_profile: str
    payload_builder: PayloadBuilder

    def __post_init__(self) -> None:
        for field, val in (("source", self.source), ("message_type", self.message_type),
                           ("name", self.name), ("payload_profile", self.payload_profile)):
            if not isinstance(val, str) or not val.strip():
                raise AdapterError(f"AdapterBinding.{field} must be a non-empty string")
        if not callable(self.payload_builder):
            raise AdapterError("AdapterBinding.payload_builder must be callable")


class ConfiguredStreamAdapter:
    def __init__(self, stream: StreamConfig, binding: AdapterBinding,
                 boundary_validator: AdapterBoundaryValidator) -> None:
        if stream.source != binding.source:
            raise AdapterError(f"{stream.name} source mismatch: {stream.source!r} vs {binding.source!r}")
        if stream.message_type != binding.message_type:
            raise AdapterError(f"{stream.name} msg_type mismatch: {stream.message_type!r} vs {binding.message_type!r}")
        self.stream = stream
        self.binding = binding
        self._validator = boundary_validator

    def register_with(self, writer: Any) -> None:
        """Tell the writer what this stream needs. Derived from the YAML fields contract."""
        if self.stream.message_type == "sensor_msgs/Image":
            reencode = (self.stream.image_encoding is not None
                        and self.stream.image_encoding.value == "jpeg")
            writer.register_image_stream(
                self.stream.name, width=640, height=480, channels=3,
                reencode_to_jpeg=reencode,
            )
        elif self.stream.fields:
            # If any field is a sequence, dims must be determined at runtime
            has_sequence = any(f.type == "sequence" for f in self.stream.fields)
            if has_sequence:
                writer.register_vector_stream(self.stream.name, dims=0)
            else:
                writer.register_vector_stream(
                    self.stream.name, dims=len(self.stream.fields),
                    columns=tuple(f.path for f in self.stream.fields),
                )
        else:
            writer.register_vector_stream(self.stream.name, dims=0)

    def adapt(self, message: Any, *, received_at: datetime) -> WriterSample:
        payload = self.binding.payload_builder(message, self.stream)
        sample = build_boundary_sample(
            self.stream, payload=payload, received_at=received_at,
            source_timestamp=extract_header_timestamp(message),
        )
        try:
            validated = self._validator.validate_sample(sample, profile_name=self.binding.payload_profile)
        except AdapterBoundaryValidationError as exc:
            raise AdapterError(str(exc)) from exc

        ts_ns = int(validated.timestamp.timestamp() * 1e9)
        return _writer_sample_from_payload(
            self.stream.name, validated.payload, ts_ns,
        )


class AdapterRegistry:
    def __init__(self, bindings: tuple[AdapterBinding, ...] | None = None,
                 *, profile_registry: AdapterPayloadProfileRegistry | None = None) -> None:
        source = bindings if bindings is not None else _default_bindings()
        self._profile_registry = profile_registry or AdapterPayloadProfileRegistry.with_defaults()
        self._boundary_validator = AdapterBoundaryValidator(self._profile_registry)
        by_name: dict[str, AdapterBinding] = {}
        by_sig: dict[tuple[str, str], dict[str, AdapterBinding]] = {}
        for b in source:
            if b.name in by_name:
                raise AdapterError(f"duplicate binding name: {b.name!r}")
            by_name[b.name] = b
            by_sig.setdefault((b.source, b.message_type), {})[b.name] = b
        self._by_name = by_name
        self._by_sig = by_sig

    @classmethod
    def with_defaults(cls) -> AdapterRegistry:
        return cls()

    @property
    def supported_bindings(self) -> tuple[tuple[str, str, str], ...]:
        return tuple(sorted((b.source, b.message_type, b.name) for b in self._by_name.values()))

    def resolve(self, stream: StreamConfig) -> ConfiguredStreamAdapter:
        binding = self._resolve_binding(stream)
        return ConfiguredStreamAdapter(stream, binding, self._boundary_validator)

    def resolve_session(self, config: SessionConfig) -> dict[str, ConfiguredStreamAdapter]:
        return {name: self.resolve(stream) for name, stream in config.streams}

    def _resolve_binding(self, stream: StreamConfig) -> AdapterBinding:
        matches = self._by_sig.get((stream.source, stream.message_type), {})
        names = sorted(matches)
        if not names:
            raise AdapterError(
                f"no adapter for stream {stream.name!r} source={stream.source!r} "
                f"msg_type={stream.message_type!r}; add a binding in _default_bindings()"
            )
        if len(names) > 1:
            raise AdapterError(
                f"ambiguous bindings for {stream.name!r}: {names}"
            )
        return matches[names[0]]


def _default_bindings() -> tuple[AdapterBinding, ...]:
    return (
        AdapterBinding("teleop", "geometry_msgs/PoseStamped", "teleop_pose", "teleop_pose", _pose_payload),
        AdapterBinding("teleop", "std_msgs/Float32MultiArray", "teleop_buttons", "teleop_buttons", _buttons_payload),
        AdapterBinding("robot", "std_msgs/Float32MultiArray", "robot_joint_positions", "robot_joint_positions", _joint_payload),
        AdapterBinding("robot", "sensor_msgs/JointState", "robot_joint_state", "robot_joint_positions", _joint_state_payload),
        AdapterBinding("robot", "geometry_msgs/PoseStamped", "robot_pose", "robot_pose", _pose_payload),
        AdapterBinding("sensor", "sensor_msgs/Image", "sensor_rgb_image", "sensor_rgb_image", _image_payload),
        AdapterBinding("sensor", "std_msgs/Float32MultiArray", "sensor_array", "sensor_array", _sensor_array_payload),
    )


def _pose_payload(msg: Any, stream: StreamConfig) -> dict[str, Any]:
    return extract_pose_payload(msg, fallback_frame_id=stream.frame_id)


def _buttons_payload(msg: Any, _: StreamConfig) -> dict[str, Any]:
    return {"buttons": extract_numeric_sequence(msg)}


def _joint_payload(msg: Any, _: StreamConfig) -> dict[str, Any]:
    return {"joint_positions": extract_numeric_sequence(msg)}


def _joint_state_payload(msg: Any, _: StreamConfig) -> dict[str, Any]:
    return {"joint_positions": list(msg.position)}


def _image_payload(msg: Any, stream: StreamConfig) -> dict[str, Any]:
    return extract_image_payload(msg, fallback_frame_id=stream.frame_id)


def _sensor_array_payload(msg: Any, _: StreamConfig) -> dict[str, Any]:
    return {"data": extract_numeric_sequence(msg)}


def _writer_sample_from_payload(
    stream_name: str, payload: dict[str, Any], ts_ns: int,
) -> WriterSample:
    """Generic: walk the payload dict and produce a WriterSample.

    Does not know about message types. If the payload contains a 'data' key
    with bytes, it's an image. Otherwise, collect all leaf scalar values into
    a flat tuple.
    """
    # Image: the 'data' field holds raw bytes
    data_bytes = payload.get("data")
    if isinstance(data_bytes, (bytes, bytearray)):
        return WriterSample(
            stream_name=stream_name, timestamp_ns=ts_ns,
            image_data=bytes(data_bytes),
        )

    # Vector: flatten all leaf scalar values depth-first
    values = _collect_scalars(payload)
    return WriterSample(
        stream_name=stream_name, timestamp_ns=ts_ns,
        values=tuple(values),
    )


def _collect_scalars(obj: Any) -> list[float]:
    """Recursively collect all scalar numeric values from a nested dict.

    Walks depth-first in sorted key order — guarantees deterministic field
    ordering regardless of how the upstream dict was constructed. This is
    the generic flattening step: no per-message-type logic, just "give me
    every number in this payload."
    """
    result: list[float] = []
    if isinstance(obj, dict):
        for _key in sorted(obj):  # sorted = deterministic field order
            result.extend(_collect_scalars(obj[_key]))
    elif isinstance(obj, (list, tuple)):
        for item in obj:
            result.extend(_collect_scalars(item))
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        result.append(float(obj))
    return result


__all__ = ["AdapterBinding", "AdapterRegistry", "ConfiguredStreamAdapter"]
