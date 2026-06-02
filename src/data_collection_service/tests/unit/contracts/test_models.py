from datetime import datetime, timezone
from core.contracts import (
    AdapterBoundarySample, CanonicalStreamIdentity, CanonicalStreamFamily,
    CanonicalTimeDomain, ContractError,
)

UTC = timezone.utc


def test_stream_identity_valid():
    sid = CanonicalStreamIdentity("leader_pose", "teleop")
    assert sid.stream_name == "leader_pose"
    assert sid.source_family == "teleop"
    assert sid.to_dict() == {"stream_name": "leader_pose", "source_family": "teleop"}


def test_stream_identity_rejects_empty_name():
    try:
        CanonicalStreamIdentity("", "teleop")
        assert False
    except ContractError:
        pass


def test_stream_identity_rejects_bad_family():
    try:
        CanonicalStreamIdentity("x", "bad")
        assert False
    except ContractError:
        pass


def test_family_enum_values():
    assert CanonicalStreamFamily.TELEOP.value == "teleop"
    assert CanonicalStreamFamily.ROBOT.value == "robot"
    assert CanonicalStreamFamily.SENSOR.value == "sensor"
    assert CanonicalStreamFamily.ACTION.value == "action"


def test_time_domain_enum_values():
    assert CanonicalTimeDomain.ROS_HEADER.value == "ros_header"
    assert CanonicalTimeDomain.ROS_RECEIVE.value == "ros_receive"
    assert CanonicalTimeDomain.SYSTEM_CLOCK.value == "system_clock"
    assert CanonicalTimeDomain.STEADY_CLOCK.value == "steady_clock"


def test_boundary_sample_valid():
    ts = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)
    sid = CanonicalStreamIdentity("s", "teleop")
    sample = AdapterBoundarySample(
        stream=sid, timestamp=ts, payload={"x": 1},
        time_domain="ros_header", source_timestamp=ts,
    )
    assert sample.stream_name == "s"
    assert sample.timestamp == ts
    assert sample.to_dict()["stream"] == sid.to_dict()


def test_boundary_sample_rejects_empty_payload():
    ts = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)
    sid = CanonicalStreamIdentity("s", "teleop")
    try:
        AdapterBoundarySample(stream=sid, timestamp=ts, payload={}, time_domain="ros_receive")
        assert False
    except ContractError:
        pass


def test_boundary_sample_requires_source_ts_for_ros_header():
    ts = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)
    sid = CanonicalStreamIdentity("s", "teleop")
    try:
        AdapterBoundarySample(stream=sid, timestamp=ts, payload={"x": 1}, time_domain="ros_header")
        assert False
    except ContractError:
        pass


def test_boundary_sample_source_ts_optional_for_ros_receive():
    ts = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)
    sid = CanonicalStreamIdentity("s", "teleop")
    s = AdapterBoundarySample(stream=sid, timestamp=ts, payload={"x": 1}, time_domain="ros_receive")
    assert s.source_timestamp is None
