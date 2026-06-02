from datetime import datetime, timezone
from core.contracts import AdapterBoundarySample, CanonicalStreamIdentity
from core.schema import (
    AdapterPayloadProfileRegistry, AdapterBoundaryValidator,
    AdapterBoundaryValidationError,
)

UTC = timezone.utc
TS = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)


def _make_sample(family, payload, time_domain="ros_receive"):
    return AdapterBoundarySample(
        stream=CanonicalStreamIdentity("s", family),
        timestamp=TS, payload=payload, time_domain=time_domain,
        source_timestamp=TS if time_domain == "ros_header" else None,
    )


def test_validates_teleop_pose():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    result = val.validate_sample(_make_sample("teleop", {
        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
        "orientation": {"x": 0, "y": 0.1, "z": 0.2, "w": 0.9},
    }), profile_name="teleop_pose")
    assert result.source_family == "teleop"


def test_rejects_missing_position_x():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    try:
        val.validate_sample(_make_sample("teleop", {
            "position": {"y": 0.2, "z": 0.3},
            "orientation": {"x": 0, "y": 0.1, "z": 0.2, "w": 0.9},
        }), profile_name="teleop_pose")
        assert False
    except AdapterBoundaryValidationError as e:
        assert e.field_path == "position.x"


def test_rejects_wrong_family():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    try:
        val.validate_sample(_make_sample("sensor", {
            "position": {"x": 0.1, "y": 0.2, "z": 0.3},
            "orientation": {"x": 0, "y": 0.1, "z": 0.2, "w": 0.9},
        }), profile_name="teleop_pose")
        assert False
    except AdapterBoundaryValidationError:
        pass


def test_validates_image():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    result = val.validate_sample(_make_sample("sensor", {
        "height": 480, "width": 640, "encoding": "rgb8",
        "is_bigendian": 0, "step": 1920, "data": b"\x00" * 100,
    }), profile_name="sensor_rgb_image")
    assert result.source_family == "sensor"


def test_validates_buttons():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    result = val.validate_sample(_make_sample("teleop", {
        "buttons": [0.0, 1.0, 0.5],
    }), profile_name="teleop_buttons")
    assert result.source_family == "teleop"


def test_validates_joints():
    reg = AdapterPayloadProfileRegistry.with_defaults()
    val = AdapterBoundaryValidator(reg)
    result = val.validate_sample(_make_sample("robot", {
        "joint_positions": [0.0, 0.1, 0.2],
    }), profile_name="robot_joint_positions")
    assert result.source_family == "robot"
