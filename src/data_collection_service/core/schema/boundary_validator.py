"""Validates adapter boundary samples against declared payload profiles."""

from __future__ import annotations

from typing import Any, Mapping

from core.contracts import AdapterBoundarySample
from core.schema.adapter_profiles import (
    AdapterPayloadProfileRegistry,
    AdapterPayloadValidationError,
)


class AdapterBoundaryValidationError(ValueError):
    """Raised when an adapter boundary sample fails profile validation."""

    def __init__(self, message: str, *, field_path: str | None = None) -> None:
        super().__init__(message)
        self.field_path = field_path


class AdapterBoundaryValidator:
    """Validate AdapterBoundarySample payloads against registered profiles."""

    def __init__(self, profile_registry: AdapterPayloadProfileRegistry) -> None:
        if not isinstance(profile_registry, AdapterPayloadProfileRegistry):
            raise ValueError("profile_registry must be an AdapterPayloadProfileRegistry")
        self._registry = profile_registry

    def validate_sample(
        self, sample: AdapterBoundarySample, *, profile_name: str,
    ) -> AdapterBoundarySample:
        try:
            return self._registry.validate_sample(profile_name, sample)
        except AdapterPayloadValidationError as exc:
            raise AdapterBoundaryValidationError(
                str(exc), field_path=getattr(exc, "field_path", None),
            ) from exc


__all__ = ["AdapterBoundaryValidationError", "AdapterBoundaryValidator"]
