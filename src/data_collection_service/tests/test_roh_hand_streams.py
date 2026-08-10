"""Tests for the canonical ROH hand stream contract.

Pins the JointState/ros_header contract for the hand session — the inverse of
the retired 115 test (test_roh_f32_streams.py), which pinned header-less
Float32MultiArray streams with a ros_receive time domain. Header-less
messages must be rejected; hand streams are ordinary JointState streams.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

from core.adapters import AdapterError, AdapterRegistry
from core.config import load_session_config, load_session_config_dict

HAND_SESSION = (
    Path(__file__).resolve().parent.parent
    / "config/session_vr_ik_roh_hand_button_control.yaml"
)


def _hand_config():
    return load_session_config_dict({
        "schema_version": "1.0",
        "session": {"name": "test", "task_id": "t", "operator_id": "o"},
        "storage": {"root": "data/episodes", "format": "hdf5"},
        "streams": {
            "hand_left_joint_state": {
                "source": "robot",
                "topic": "/roh/left/joint_state",
                "message_type": "sensor_msgs/JointState",
                "columns": ["thumb", "index", "middle", "ring", "pinky", "thumb_root"],
                "time_domain": "ros_header",
                "fields": [{"path": "position", "type": "sequence"}],
            },
        },
    })


def _joint_state_msg(positions, *, with_header: bool = True):
    msg = SimpleNamespace(position=list(positions), name=[], velocity=[], effort=[])
    if with_header:
        msg.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=1700000000, nanosec=500000000),
            frame_id="",
        )
    return msg


def test_jointstate_hand_stream_adapts_with_header_time():
    adapters = AdapterRegistry.with_defaults().resolve_session(_hand_config())
    adapter = adapters["hand_left_joint_state"]
    assert adapter.effective_columns() == (
        "thumb", "index", "middle", "ring", "pinky", "thumb_root",
    )

    msg = _joint_state_msg([0.0, 0.1, 0.2, 0.3, 0.4, 0.5])
    sample = adapter.adapt(msg, received_at=datetime.now(timezone.utc))
    assert sample.timestamp_ns == 1700000000_500000000
    assert sample.values == (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)
    assert sample.image_data is None


def test_headerless_jointstate_rejected_ros_header_only_contract():
    adapters = AdapterRegistry.with_defaults().resolve_session(_hand_config())
    adapter = adapters["hand_left_joint_state"]
    msg = _joint_state_msg([0.0] * 6, with_header=False)
    with pytest.raises(AdapterError, match="requires a header timestamp"):
        adapter.adapt(msg, received_at=datetime.now(timezone.utc))


def test_ros_receive_time_domain_rejected_by_config_loader():
    """The non-canonical receipt-time domain must not come back."""
    with pytest.raises(Exception, match="time_domain|ros_receive"):
        load_session_config_dict({
            "schema_version": "1.0",
            "session": {"name": "t", "task_id": "t", "operator_id": "o"},
            "storage": {"root": "data/episodes", "format": "hdf5"},
            "streams": {
                "s": {
                    "source": "robot",
                    "topic": "/roh/left/joint_state",
                    "message_type": "sensor_msgs/JointState",
                    "columns": ["a"],
                    "time_domain": "ros_receive",
                    "fields": [{"path": "position", "type": "sequence"}],
                },
            },
        })


def test_hand_session_file_resolves_canonical_contract():
    """The live hand-session YAML: 18 streams, 4 hand streams at 6 columns,
    ros_header everywhere, no Float32MultiArray, arms at 7 columns."""
    cfg = load_session_config(str(HAND_SESSION))
    streams = dict(cfg.streams)

    assert len(streams) == 18

    hand = {n: s for n, s in streams.items() if n.startswith("hand_")}
    assert len(hand) == 4
    for s in hand.values():
        assert s.message_type == "sensor_msgs/JointState"
        assert s.time_domain.value == "ros_header"
        assert len(s.columns) == 6
    assert {s.topic for s in hand.values()} == {
        "/roh/left/joint_state", "/roh/right/joint_state",
        "/roh/left/joint_command", "/roh/right/joint_command",
    }

    for name in ("arm_left_joint_state", "arm_right_joint_state",
                 "ik_left_joint_commands", "ik_right_joint_commands"):
        assert len(streams[name].columns) == 7, name

    assert all(s.time_domain.value == "ros_header" for s in streams.values())
    assert all(
        s.message_type != "std_msgs/Float32MultiArray" for s in streams.values()
    )
