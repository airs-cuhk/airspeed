"""Typed session configuration for AIRS-standard data collection runs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import yaml


SCHEMA_VERSION = "1.0"


class SessionConfigError(ValueError):
    """Raised when a session configuration payload is invalid."""


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class TimeDomain(str, Enum):
    ROS_HEADER = "ros_header"
    ROS_RECEIVE = "ros_receive"
    SYSTEM_CLOCK = "system_clock"
    STEADY_CLOCK = "steady_clock"


class QoSReliability(str, Enum):
    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"


class QoSDurability(str, Enum):
    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


class QoSHistory(str, Enum):
    KEEP_LAST = "keep_last"
    KEEP_ALL = "keep_all"


class RecordingControlMode(str, Enum):
    SERVICE = "service"
    DEVICE_BINDING = "device_binding"
    MANUAL_UI = "manual_ui"


class ImageEncoding(str, Enum):
    JPEG = "jpeg"
    RAW = "raw"


# ---------------------------------------------------------------------------
# Field rule — per-stream canonical contract
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FieldRule:
    """One field in a stream's canonical payload contract."""

    path: str
    type: str
    required: bool = True

    _ALLOWED_TYPES = frozenset({
        "float64", "float32", "int32", "uint32", "uint64",
        "string", "bytes", "bool", "sequence",
    })

    def __post_init__(self) -> None:
        if not isinstance(self.path, str) or not self.path.strip():
            raise SessionConfigError("FieldRule.path must be a non-empty string")
        if self.type not in self._ALLOWED_TYPES:
            raise SessionConfigError(
                f"FieldRule.type must be one of {sorted(self._ALLOWED_TYPES)}; "
                f"got {self.type!r}"
            )

    def to_dict(self) -> dict[str, Any]:
        return {"path": self.path, "type": self.type, "required": self.required}


# ---------------------------------------------------------------------------
# Stream sub-models
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StreamQoS:
    reliability: QoSReliability = QoSReliability.BEST_EFFORT
    durability: QoSDurability = QoSDurability.VOLATILE
    history: QoSHistory = QoSHistory.KEEP_LAST
    depth: int = 10

    def __post_init__(self) -> None:
        if not isinstance(self.depth, int) or self.depth < 1:
            raise SessionConfigError("StreamQoS.depth must be an integer >= 1")

    def to_dict(self) -> dict[str, Any]:
        return {
            "reliability": self.reliability.value,
            "durability": self.durability.value,
            "history": self.history.value,
            "depth": self.depth,
        }


@dataclass(frozen=True)
class StreamConfig:
    """One configured stream in a collection session."""

    name: str
    source: str
    topic: str
    message_type: str
    qos: StreamQoS = field(default_factory=StreamQoS)
    time_domain: TimeDomain = TimeDomain.ROS_RECEIVE
    expected_rate_hz: float | None = None
    frame_id: str | None = None
    image_encoding: ImageEncoding | None = None
    fields: tuple[FieldRule, ...] = ()
    notes: str | None = None

    _ALLOWED_SOURCES = frozenset({"teleop", "robot", "sensor"})
    _IMAGE_MESSAGE_TYPE = "sensor_msgs/Image"

    def __post_init__(self) -> None:
        _require_non_empty("StreamConfig.name", self.name)
        _require_non_empty("StreamConfig.topic", self.topic)
        _require_non_empty("StreamConfig.message_type", self.message_type)
        if self.source not in self._ALLOWED_SOURCES:
            raise SessionConfigError(
                f"StreamConfig.source must be one of {sorted(self._ALLOWED_SOURCES)}; "
                f"got {self.source!r}"
            )
        if not isinstance(self.qos, StreamQoS):
            raise SessionConfigError("StreamConfig.qos must be a StreamQoS")
        if self.image_encoding is not None and not isinstance(self.image_encoding, ImageEncoding):
            raise SessionConfigError("StreamConfig.image_encoding must be an ImageEncoding or None")
        has_image_encoding = self.image_encoding is not None
        is_image_stream = self.message_type == self._IMAGE_MESSAGE_TYPE
        if has_image_encoding and not is_image_stream:
            raise SessionConfigError(
                f"image_encoding is only valid for {self._IMAGE_MESSAGE_TYPE} streams; "
                f"{self.name} has message_type={self.message_type!r}"
            )
        if not isinstance(self.fields, tuple):
            object.__setattr__(self, "fields", tuple(self.fields))

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "source": self.source,
            "topic": self.topic,
            "message_type": self.message_type,
            "qos": self.qos.to_dict(),
            "time_domain": self.time_domain.value,
        }
        if self.expected_rate_hz is not None:
            payload["expected_rate_hz"] = self.expected_rate_hz
        if self.frame_id is not None:
            payload["frame_id"] = self.frame_id
        if self.image_encoding is not None:
            payload["image_encoding"] = self.image_encoding.value
        if self.fields:
            payload["fields"] = [f.to_dict() for f in self.fields]
        if self.notes is not None:
            payload["notes"] = self.notes
        return payload


# ---------------------------------------------------------------------------
# Session sub-models
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DeviceEntry:
    device_id: str
    role: str
    driver_version: str | None = None
    firmware_version: str | None = None
    calibration_ref: str | None = None
    serial_number: str | None = None

    def __post_init__(self) -> None:
        _require_non_empty("DeviceEntry.device_id", self.device_id)
        _require_non_empty("DeviceEntry.role", self.role)

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {"device_id": self.device_id, "role": self.role}
        if self.driver_version is not None:
            d["driver_version"] = self.driver_version
        if self.firmware_version is not None:
            d["firmware_version"] = self.firmware_version
        if self.calibration_ref is not None:
            d["calibration_ref"] = self.calibration_ref
        if self.serial_number is not None:
            d["serial_number"] = self.serial_number
        return d


@dataclass(frozen=True)
class RecordingControlBinding:
    stream_name: str
    button_index: int | None = None
    threshold: float | None = None
    field_name: str = "buttons"

    def __post_init__(self) -> None:
        _require_non_empty("RecordingControlBinding.stream_name", self.stream_name)
        if self.button_index is not None and (self.button_index < 0):
            raise SessionConfigError("RecordingControlBinding.button_index must be >= 0")
        if self.threshold is not None and not isinstance(self.threshold, (int, float)):
            raise SessionConfigError("RecordingControlBinding.threshold must be numeric")

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {"stream_name": self.stream_name, "field_name": self.field_name}
        if self.button_index is not None:
            d["button_index"] = self.button_index
        if self.threshold is not None:
            d["threshold"] = float(self.threshold)
        return d


@dataclass(frozen=True)
class RecordingControlConfig:
    mode: RecordingControlMode = RecordingControlMode.SERVICE
    toggle_debounce_s: float = 0.5
    bindings: tuple[tuple[str, RecordingControlBinding], ...] = ()

    _ALLOWED_BINDING_ACTIONS = frozenset({"toggle", "delete"})

    def __post_init__(self) -> None:
        if not isinstance(self.mode, RecordingControlMode):
            raise SessionConfigError("RecordingControlConfig.mode must be a RecordingControlMode")
        if self.toggle_debounce_s < 0:
            raise SessionConfigError("RecordingControlConfig.toggle_debounce_s must be >= 0")
        if self.mode == RecordingControlMode.DEVICE_BINDING:
            if not self.bindings:
                raise SessionConfigError(
                    "RecordingControlConfig.bindings must define at least one action "
                    "when mode=device_binding"
                )
            for action, _ in self.bindings:
                if action not in self._ALLOWED_BINDING_ACTIONS:
                    raise SessionConfigError(
                        f"RecordingControlConfig.bindings action must be one of "
                        f"{sorted(self._ALLOWED_BINDING_ACTIONS)}; got {action!r}"
                    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode.value,
            "toggle_debounce_s": self.toggle_debounce_s,
            "bindings": {action: b.to_dict() for action, b in self.bindings},
        }


@dataclass(frozen=True)
class StorageConfig:
    root: str = "data/episodes"
    format: str = "hdf5"
    compression: str | None = "gzip"
    config_hash_algorithm: str = "sha256"

    _ALLOWED_HASH_ALGORITHMS = frozenset({"sha256"})

    def __post_init__(self) -> None:
        _require_non_empty("StorageConfig.root", self.root)
        if self.format != "hdf5":
            raise SessionConfigError("StorageConfig.format must be 'hdf5'")
        if self.config_hash_algorithm not in self._ALLOWED_HASH_ALGORITHMS:
            raise SessionConfigError(
                f"StorageConfig.config_hash_algorithm must be one of "
                f"{sorted(self._ALLOWED_HASH_ALGORITHMS)}"
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "root": self.root,
            "format": self.format,
            "compression": self.compression,
            "config_hash_algorithm": self.config_hash_algorithm,
        }


@dataclass(frozen=True)
class SessionMeta:
    name: str
    task_id: str
    operator_id: str
    devices: tuple[tuple[str, DeviceEntry], ...] = ()
    recording_control: RecordingControlConfig = field(default_factory=RecordingControlConfig)
    notes: str | None = None

    def __post_init__(self) -> None:
        _require_non_empty("SessionMeta.name", self.name)
        _require_non_empty("SessionMeta.task_id", self.task_id)
        _require_non_empty("SessionMeta.operator_id", self.operator_id)
        if not isinstance(self.recording_control, RecordingControlConfig):
            raise SessionConfigError(
                "SessionMeta.recording_control must be a RecordingControlConfig"
            )

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "name": self.name,
            "task_id": self.task_id,
            "operator_id": self.operator_id,
            "recording_control": self.recording_control.to_dict(),
        }
        if self.devices:
            d["devices"] = {name: dev.to_dict() for name, dev in self.devices}
        if self.notes is not None:
            d["notes"] = self.notes
        return d


# ---------------------------------------------------------------------------
# Top-level session config
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SessionConfig:
    schema_version: str
    session: SessionMeta
    storage: StorageConfig
    streams: tuple[tuple[str, StreamConfig], ...]

    _ROOT_KEYS = frozenset({"schema_version", "session", "storage", "streams"})
    _STREAM_KEYS = frozenset({
        "source", "topic", "message_type", "qos", "time_domain",
        "expected_rate_hz", "frame_id", "image_encoding", "fields", "notes",
    })
    _SESSION_KEYS = frozenset({
        "name", "task_id", "operator_id", "devices", "recording_control", "notes",
    })

    def __post_init__(self) -> None:
        _require_non_empty("SessionConfig.schema_version", self.schema_version)

        time_domains: set[TimeDomain] = set()
        for _, stream in self.streams:
            if not isinstance(stream, StreamConfig):
                raise SessionConfigError("SessionConfig.streams must contain StreamConfig values")
            time_domains.add(stream.time_domain)

        if not self.streams:
            raise SessionConfigError("SessionConfig.streams must define at least one stream")

    def _streams_dict(self) -> dict[str, StreamConfig]:
        return dict(self.streams)

    def _devices_dict(self) -> dict[str, DeviceEntry]:
        return dict(self.session.devices)

    def compute_config_hash(self) -> str:
        canonical = self.to_dict()
        payload = json.dumps(canonical, sort_keys=True, default=str)
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "session": self.session.to_dict(),
            "storage": self.storage.to_dict(),
            "streams": {name: s.to_dict() for name, s in self.streams},
        }


# ---------------------------------------------------------------------------
# YAML loading
# ---------------------------------------------------------------------------


class _UniqueKeyLoader(yaml.SafeLoader):
    """YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False,
) -> dict[Any, Any]:
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise SessionConfigError(f"duplicate config key: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping,
)


def load_session_config(
    config_path: str | Path,
    *,
    relative_path_root: Path | None = None,
) -> SessionConfig:
    resolved = Path(config_path).expanduser()
    if relative_path_root is not None and not resolved.is_absolute():
        resolved = (relative_path_root / resolved).resolve()
    else:
        resolved = resolved.resolve()

    if not resolved.exists():
        raise SessionConfigError(f"session config file not found: {resolved}")

    raw = yaml.load(resolved.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    if not isinstance(raw, Mapping):
        raise SessionConfigError("session config must be a YAML mapping at the top level")

    return _parse_config(raw, source_path=str(resolved))


def load_session_config_dict(payload: Mapping[str, Any]) -> SessionConfig:
    if not isinstance(payload, Mapping):
        raise SessionConfigError("session config dict must be a mapping")
    return _parse_config(dict(payload), source_path="<dict>")


def _parse_config(raw: dict[str, Any], *, source_path: str) -> SessionConfig:
    unknown = set(raw) - SessionConfig._ROOT_KEYS
    if unknown:
        raise SessionConfigError(
            f"unknown top-level config keys: {sorted(unknown)}"
        )

    schema_version = _require_str(raw, "schema_version")

    session_raw = _require_mapping(raw, "session")
    unknown_session = set(session_raw) - SessionConfig._SESSION_KEYS
    if unknown_session:
        raise SessionConfigError(
            f"unknown session config keys: {sorted(unknown_session)}"
        )

    devices_raw = session_raw.get("devices")
    devices: tuple[tuple[str, DeviceEntry], ...] = ()
    if devices_raw is not None:
        devices = _parse_devices(devices_raw)

    rc_raw = session_raw.get("recording_control")
    recording_control = RecordingControlConfig()
    if rc_raw is not None:
        recording_control = _parse_recording_control(rc_raw)

    session = SessionMeta(
        name=_require_str(session_raw, "name"),
        task_id=_require_str(session_raw, "task_id"),
        operator_id=_require_str(session_raw, "operator_id"),
        devices=devices,
        recording_control=recording_control,
        notes=session_raw.get("notes"),
    )

    storage_raw = _require_mapping(raw, "storage")
    storage = StorageConfig(
        root=str(storage_raw.get("root", "data/episodes")),
        format=str(storage_raw.get("format", "hdf5")),
        compression=storage_raw.get("compression", "gzip"),
        config_hash_algorithm=str(storage_raw.get("config_hash_algorithm", "sha256")),
    )

    streams_raw = _require_mapping(raw, "streams")
    streams = _parse_streams(streams_raw)

    return SessionConfig(
        schema_version=schema_version,
        session=session,
        storage=storage,
        streams=streams,
    )


def _parse_devices(raw: Any) -> tuple[tuple[str, DeviceEntry], ...]:
    if not isinstance(raw, Mapping):
        raise SessionConfigError("session.devices must be a mapping")
    entries: list[tuple[str, DeviceEntry]] = []
    for name, entry_raw in raw.items():
        if not isinstance(entry_raw, Mapping):
            raise SessionConfigError(f"devices.{name} must be a mapping")
        entries.append((
            str(name),
            DeviceEntry(
                device_id=str(entry_raw.get("device_id", name)),
                role=str(entry_raw.get("role", "")),
                driver_version=_optional_str(entry_raw, "driver_version"),
                firmware_version=_optional_str(entry_raw, "firmware_version"),
                calibration_ref=_optional_str(entry_raw, "calibration_ref"),
                serial_number=_optional_str(entry_raw, "serial_number"),
            ),
        ))
    return tuple(entries)


def _parse_recording_control(raw: Any) -> RecordingControlConfig:
    if not isinstance(raw, Mapping):
        raise SessionConfigError("session.recording_control must be a mapping")
    mode_str = str(raw.get("mode", "service"))
    try:
        mode = RecordingControlMode(mode_str)
    except ValueError:
        raise SessionConfigError(
            f"session.recording_control.mode must be one of "
            f"{[m.value for m in RecordingControlMode]}; got {mode_str!r}"
        )
    bindings_raw = raw.get("bindings")
    bindings: list[tuple[str, RecordingControlBinding]] = []
    if bindings_raw is not None:
        if not isinstance(bindings_raw, Mapping):
            raise SessionConfigError("session.recording_control.bindings must be a mapping")
        for action, binding_raw in bindings_raw.items():
            if not isinstance(binding_raw, Mapping):
                raise SessionConfigError(f"bindings.{action} must be a mapping")
            bindings.append((
                str(action),
                RecordingControlBinding(
                    stream_name=str(binding_raw.get("stream_name", "")),
                    button_index=binding_raw.get("button_index"),
                    threshold=binding_raw.get("threshold"),
                    field_name=str(binding_raw.get("field_name", "buttons")),
                ),
            ))
    toggle_debounce_s = float(raw.get("toggle_debounce_s", 0.5))
    return RecordingControlConfig(mode=mode, toggle_debounce_s=toggle_debounce_s, bindings=tuple(bindings))


def _parse_streams(raw: Any) -> tuple[tuple[str, StreamConfig], ...]:
    if not isinstance(raw, Mapping):
        raise SessionConfigError("streams must be a mapping")
    streams: list[tuple[str, StreamConfig]] = []
    for name, stream_raw in raw.items():
        streams.append((str(name), _parse_stream(str(name), stream_raw)))
    return tuple(streams)


def _parse_stream(name: str, raw: Any) -> StreamConfig:
    if not isinstance(raw, Mapping):
        raise SessionConfigError(f"streams.{name} must be a mapping")

    unknown = set(raw) - SessionConfig._STREAM_KEYS
    if unknown:
        raise SessionConfigError(
            f"streams.{name}: unknown stream config keys: {sorted(unknown)}"
        )

    qos_raw = raw.get("qos")
    qos = StreamQoS()
    if qos_raw is not None:
        qos = _parse_qos(qos_raw)

    td_str = str(raw.get("time_domain", "ros_receive"))
    try:
        time_domain = TimeDomain(td_str)
    except ValueError:
        raise SessionConfigError(
            f"streams.{name}.time_domain must be one of "
            f"{[t.value for t in TimeDomain]}; got {td_str!r}"
        )

    ie_str = raw.get("image_encoding")
    image_encoding: ImageEncoding | None = None
    if ie_str is not None:
        try:
            image_encoding = ImageEncoding(str(ie_str))
        except ValueError:
            raise SessionConfigError(
                f"streams.{name}.image_encoding must be one of "
                f"{[e.value for e in ImageEncoding]}; got {ie_str!r}"
            )

    fields_raw = raw.get("fields")
    fields: list[FieldRule] = []
    if fields_raw is not None:
        if not isinstance(fields_raw, list):
            raise SessionConfigError(f"streams.{name}.fields must be a list")
        for i, f in enumerate(fields_raw):
            if not isinstance(f, Mapping):
                raise SessionConfigError(f"streams.{name}.fields[{i}] must be a mapping")
            fields.append(FieldRule(
                path=_require_str(f, "path", scope=f"streams.{name}.fields[{i}]"),
                type=_require_str(f, "type", scope=f"streams.{name}.fields[{i}]"),
                required=bool(f.get("required", True)),
            ))

    rate_hz_raw = raw.get("expected_rate_hz")
    expected_rate_hz: float | None = None
    if rate_hz_raw is not None:
        try:
            expected_rate_hz = float(rate_hz_raw)
            if expected_rate_hz <= 0:
                raise SessionConfigError(f"streams.{name}.expected_rate_hz must be > 0")
        except (TypeError, ValueError):
            raise SessionConfigError(f"streams.{name}.expected_rate_hz must be a positive number")

    return StreamConfig(
        name=name,
        source=_require_str(raw, "source", scope=f"streams.{name}"),
        topic=_require_str(raw, "topic", scope=f"streams.{name}"),
        message_type=_require_str(raw, "message_type", scope=f"streams.{name}"),
        qos=qos,
        time_domain=time_domain,
        expected_rate_hz=expected_rate_hz,
        frame_id=_optional_str(raw, "frame_id"),
        image_encoding=image_encoding,
        fields=tuple(fields),
        notes=_optional_str(raw, "notes"),
    )


def _parse_qos(raw: Any) -> StreamQoS:
    if not isinstance(raw, Mapping):
        raise SessionConfigError("stream qos must be a mapping")
    reliability = QoSReliability.BEST_EFFORT
    if "reliability" in raw:
        try:
            reliability = QoSReliability(str(raw["reliability"]))
        except ValueError:
            raise SessionConfigError(f"invalid qos reliability: {raw['reliability']!r}")
    durability = QoSDurability.VOLATILE
    if "durability" in raw:
        try:
            durability = QoSDurability(str(raw["durability"]))
        except ValueError:
            raise SessionConfigError(f"invalid qos durability: {raw['durability']!r}")
    history = QoSHistory.KEEP_LAST
    if "history" in raw:
        try:
            history = QoSHistory(str(raw["history"]))
        except ValueError:
            raise SessionConfigError(f"invalid qos history: {raw['history']!r}")
    depth = int(raw.get("depth", 10))
    return StreamQoS(reliability=reliability, durability=durability, history=history, depth=depth)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _require_non_empty(field: str, value: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise SessionConfigError(f"{field} must be a non-empty string")


def _require_str(raw: Mapping[str, Any], key: str, *, scope: str = "") -> str:
    prefix = f"{scope}: " if scope else ""
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise SessionConfigError(f"{prefix}{key} must be a non-empty string")
    return value


def _require_mapping(raw: Mapping[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key)
    if not isinstance(value, Mapping):
        raise SessionConfigError(f"{key} must be a mapping")
    return dict(value)


def _optional_str(raw: Mapping[str, Any], key: str) -> str | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        return None
    return value


__all__ = [
    "DeviceEntry",
    "FieldRule",
    "ImageEncoding",
    "QoSDurability",
    "QoSHistory",
    "QoSReliability",
    "RecordingControlBinding",
    "RecordingControlConfig",
    "RecordingControlMode",
    "SCHEMA_VERSION",
    "SessionConfig",
    "SessionConfigError",
    "SessionMeta",
    "StorageConfig",
    "StreamConfig",
    "StreamQoS",
    "TimeDomain",
    "load_session_config",
    "load_session_config_dict",
]
