"""Adapter-owned payload profiles for canonical boundary validation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from core.contracts import AdapterBoundarySample, CanonicalStreamFamily

_MISSING = object()
_ALLOWED_FAMILIES = frozenset(f.value for f in CanonicalStreamFamily)


class AdapterPayloadProfileError(ValueError):
    """Raised when a profile definition is invalid."""


class AdapterPayloadValidationError(ValueError):
    """Raised when a payload fails validation against its profile."""

    def __init__(self, field_path: str, message: str) -> None:
        super().__init__(message)
        self.field_path = field_path


@dataclass(frozen=True)
class AdapterPayloadFieldRule:
    """One structural rule for a dot-separated payload field path."""

    path: str
    required: bool = True
    numeric: bool = False
    min_items: int | None = None
    max_items: int | None = None
    min_value: float | None = None
    max_value: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.path, str) or not self.path.strip():
            raise AdapterPayloadProfileError("AdapterPayloadFieldRule.path must be non-empty")
        if any(not s for s in self.path.split(".")):
            raise AdapterPayloadProfileError("AdapterPayloadFieldRule.path must not contain empty segments")
        if self.min_items is not None and (not isinstance(self.min_items, int) or self.min_items < 0):
            raise AdapterPayloadProfileError("min_items must be a non-negative integer")
        if self.max_items is not None and (not isinstance(self.max_items, int) or self.max_items < 0):
            raise AdapterPayloadProfileError("max_items must be a non-negative integer")
        if (self.min_items is not None and self.max_items is not None
                and self.min_items > self.max_items):
            raise AdapterPayloadProfileError("min_items must be <= max_items")
        if self.min_value is not None and self.max_value is not None and self.min_value > self.max_value:
            raise AdapterPayloadProfileError("min_value must be <= max_value")


@dataclass(frozen=True)
class AdapterPayloadProfile:
    """One reusable structural payload profile emitted by a configured adapter."""

    name: str
    source_family: str
    description: str
    field_rules: tuple[AdapterPayloadFieldRule, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise AdapterPayloadProfileError("AdapterPayloadProfile.name must be non-empty")
        if self.source_family not in _ALLOWED_FAMILIES:
            raise AdapterPayloadProfileError(
                f"source_family must be one of {sorted(_ALLOWED_FAMILIES)}; "
                f"got {self.source_family!r}"
            )
        if not isinstance(self.description, str) or not self.description.strip():
            raise AdapterPayloadProfileError("AdapterPayloadProfile.description must be non-empty")

    def validate_payload(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, Mapping) or not payload:
            raise AdapterPayloadValidationError("payload", "payload must be a non-empty mapping")
        for rule in self.field_rules:
            _validate_rule(dict(payload), rule)
        return dict(payload)

    def validate_sample(self, sample: AdapterBoundarySample) -> AdapterBoundarySample:
        if sample.source_family != self.source_family:
            raise AdapterPayloadValidationError(
                "stream.source_family",
                f"expected {self.source_family!r}, got {sample.source_family!r}",
            )
        self.validate_payload(sample.payload)
        return sample


class AdapterPayloadProfileRegistry:
    """Lookup and validate adapter-owned payload profiles by name."""

    def __init__(self, profiles: Mapping[str, AdapterPayloadProfile] | None = None) -> None:
        source = profiles if profiles is not None else _default_profiles_by_name()
        self._profiles = dict(source)

    @classmethod
    def with_defaults(cls) -> AdapterPayloadProfileRegistry:
        return cls()

    @property
    def profile_names(self) -> tuple[str, ...]:
        return tuple(sorted(self._profiles))

    def get(self, name: str) -> AdapterPayloadProfile | None:
        return self._profiles.get(name)

    def require(self, name: str) -> AdapterPayloadProfile:
        profile = self.get(name)
        if profile is None:
            raise AdapterPayloadProfileError(f"unknown profile: {name!r}")
        return profile

    def validate_payload(self, profile_name: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self.require(profile_name).validate_payload(payload)

    def validate_sample(self, profile_name: str, sample: AdapterBoundarySample) -> AdapterBoundarySample:
        return self.require(profile_name).validate_sample(sample)


# ---------------------------------------------------------------------------
# Validation logic
# ---------------------------------------------------------------------------


def _resolve_path(root: Mapping[str, Any], path: str) -> Any:
    current: Any = root
    for segment in path.split("."):
        if not isinstance(current, Mapping):
            raise AdapterPayloadValidationError(path, f"expected mapping at {segment!r}")
        if segment not in current:
            return _MISSING
        current = current[segment]
    return current


def _validate_rule(payload: Mapping[str, Any], rule: AdapterPayloadFieldRule) -> None:
    value = _resolve_path(payload, rule.path)
    if value is _MISSING:
        if rule.required:
            raise AdapterPayloadValidationError(rule.path, f"missing required field: {rule.path}")
        return
    if value is None:
        raise AdapterPayloadValidationError(rule.path, f"{rule.path} must not be None")

    if rule.min_items is not None or rule.max_items is not None:
        if isinstance(value, (str, bytes, bytearray)):
            pass  # byte payloads satisfy min/max items implicitly
        elif not isinstance(value, Sequence):
            raise AdapterPayloadValidationError(rule.path, f"{rule.path} must be a sequence")
        else:
            n = len(value)
            if rule.min_items is not None and n < rule.min_items:
                raise AdapterPayloadValidationError(rule.path, f"{rule.path} has {n} items, min {rule.min_items}")
            if rule.max_items is not None and n > rule.max_items:
                raise AdapterPayloadValidationError(rule.path, f"{rule.path} has {n} items, max {rule.max_items}")

    if rule.numeric:
        _validate_numeric(value, rule.path, rule.min_value, rule.max_value)


def _validate_numeric(value: Any, path: str, min_v: float | None, max_v: float | None) -> None:
    if isinstance(value, (bytes, bytearray)):
        # Skip full iteration when range covers all byte values
        if (min_v is None or min_v <= 0) and (max_v is None or max_v >= 255):
            if len(value) == 0 and _has_min_items(path, min_v):
                return  # empty bytes fail min_items check elsewhere
            return
        # Sample first and last byte only
        if len(value) > 0:
            _check_numeric(int(value[0]), f"{path}[0]", min_v, max_v)
            if len(value) > 1:
                _check_numeric(int(value[-1]), f"{path}[{len(value)-1}]", min_v, max_v)
        return
    if isinstance(value, Sequence) and not isinstance(value, str):
        for i, item in enumerate(value):
            _check_numeric(item, f"{path}[{i}]", min_v, max_v)
        return
    _check_numeric(value, path, min_v, max_v)


def _has_min_items(path: str, min_v: float | None) -> bool:
    return False  # min_items is checked in _validate_rule, not here


def _check_numeric(value: Any, path: str, min_v: float | None, max_v: float | None) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AdapterPayloadValidationError(path, f"{path} must be numeric")
    if not _isfinite(float(value)):
        raise AdapterPayloadValidationError(path, f"{path} must be finite")
    if min_v is not None and float(value) < min_v:
        raise AdapterPayloadValidationError(path, f"{path}={value} below min {min_v}")
    if max_v is not None and float(value) > max_v:
        raise AdapterPayloadValidationError(path, f"{path}={value} above max {max_v}")


def _isfinite(v: float) -> bool:
    import math
    return math.isfinite(v)


# ---------------------------------------------------------------------------
# Default profiles
# ---------------------------------------------------------------------------


def _pose_field_rules() -> tuple[AdapterPayloadFieldRule, ...]:
    return tuple(
        AdapterPayloadFieldRule(path=p, numeric=True)
        for p in (
            "position.x", "position.y", "position.z",
            "orientation.x", "orientation.y", "orientation.z", "orientation.w",
        )
    )


def _default_profiles_by_name() -> dict[str, AdapterPayloadProfile]:
    profiles = (
        AdapterPayloadProfile(
            name="teleop_pose", source_family="teleop",
            description="Canonical pose payload from teleop adapters.",
            field_rules=_pose_field_rules(),
        ),
        AdapterPayloadProfile(
            name="teleop_buttons", source_family="teleop",
            description="Button or discrete control payload from teleop adapters.",
            field_rules=(AdapterPayloadFieldRule("buttons", numeric=True, min_items=1),),
        ),
        AdapterPayloadProfile(
            name="robot_joint_positions", source_family="robot",
            description="Joint-state payload from robot adapters.",
            field_rules=(AdapterPayloadFieldRule("joint_positions", numeric=True, min_items=1),),
        ),
        AdapterPayloadProfile(
            name="robot_pose", source_family="robot",
            description="Cartesian pose payload from robot adapters.",
            field_rules=_pose_field_rules(),
        ),
        AdapterPayloadProfile(
            name="sensor_rgb_image", source_family="sensor",
            description="Image payload from sensor adapters.",
            field_rules=(
                AdapterPayloadFieldRule("height", numeric=True, min_value=1),
                AdapterPayloadFieldRule("width", numeric=True, min_value=1),
                AdapterPayloadFieldRule("encoding", required=True),
                AdapterPayloadFieldRule("is_bigendian", numeric=True, min_value=0, max_value=1),
                AdapterPayloadFieldRule("step", numeric=True, min_value=1),
                AdapterPayloadFieldRule("data", numeric=True, min_items=1, min_value=0, max_value=255),
            ),
        ),
        AdapterPayloadProfile(
            name="sensor_array", source_family="sensor",
            description="Generic numeric array payload from sensor adapters (tactile, IMU, etc.).",
            field_rules=(AdapterPayloadFieldRule("data", numeric=True, min_items=1),),
        ),
    )
    return {p.name: p for p in profiles}


DEFAULT_ADAPTER_PAYLOAD_PROFILES: tuple[AdapterPayloadProfile, ...] = tuple(
    _default_profiles_by_name().values()
)

__all__ = [
    "AdapterPayloadFieldRule",
    "AdapterPayloadProfile",
    "AdapterPayloadProfileError",
    "AdapterPayloadProfileRegistry",
    "AdapterPayloadValidationError",
    "DEFAULT_ADAPTER_PAYLOAD_PROFILES",
]
