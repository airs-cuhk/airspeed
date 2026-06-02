"""Test payload profile validation edge cases — NaN, Inf, empty sequences."""
from datetime import datetime, timezone
from core.contracts import AdapterBoundarySample, CanonicalStreamIdentity
from core.schema import (
    AdapterPayloadProfileRegistry, AdapterBoundaryValidator,
    AdapterBoundaryValidationError,
)

UTC = timezone.utc
TS = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)


def _sample(family, payload):
    return AdapterBoundarySample(
        stream=CanonicalStreamIdentity("s", family),
        timestamp=TS, payload=payload, time_domain="ros_receive",
    )


def test_rejects_nan_in_position():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    try:
        val.validate_sample(_sample("teleop", {
            "position": {"x": float("nan"), "y": 0.2, "z": 0.3},
            "orientation": {"x": 0, "y": 0.1, "z": 0.2, "w": 0.9},
        }), profile_name="teleop_pose")
        assert False
    except AdapterBoundaryValidationError:
        pass


def test_rejects_inf_in_orientation():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    try:
        val.validate_sample(_sample("teleop", {
            "position": {"x": 0.1, "y": 0.2, "z": 0.3},
            "orientation": {"x": float("inf"), "y": 0.1, "z": 0.2, "w": 0.9},
        }), profile_name="teleop_pose")
        assert False
    except AdapterBoundaryValidationError:
        pass


def test_rejects_empty_button_sequence():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    try:
        val.validate_sample(_sample("teleop", {"buttons": []}), profile_name="teleop_buttons")
        assert False
    except AdapterBoundaryValidationError:
        pass


def test_rejects_zero_size_image():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    try:
        val.validate_sample(_sample("sensor", {
            "height": 0, "width": 640, "encoding": "rgb8",
            "is_bigendian": 0, "step": 1920, "data": b"\x00",
        }), profile_name="sensor_rgb_image")
        assert False
    except AdapterBoundaryValidationError:
        pass
