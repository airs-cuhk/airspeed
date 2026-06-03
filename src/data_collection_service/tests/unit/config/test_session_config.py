from core.config import (
    SessionConfigError, load_session_config_dict, FieldRule,
    ImageEncoding, SessionConfig, StreamConfig,
)


def test_loads_profile_mixed():
    cfg = load_session_config_dict(_mixed_payload())
    assert cfg.schema_version == "1.0"
    assert cfg.session.name == "mixed_collection"
    assert len(cfg.streams) == 5


def test_field_rule_roundtrip():
    f = FieldRule(path="position.x", type="float64", required=True)
    assert f.path == "position.x"
    assert f.type == "float64"
    assert f.required is True
    d = f.to_dict()
    assert d["path"] == "position.x"


def test_image_encoding_on_non_image_rejected():
    payload = _mixed_payload()
    payload["streams"]["leader_pose"]["image_encoding"] = "jpeg"
    try:
        load_session_config_dict(payload)
        assert False, "should have raised"
    except SessionConfigError:
        pass


def test_image_encoding_on_image_accepted():
    payload = _mixed_payload()
    payload["streams"]["front_rgb"]["image_encoding"] = "jpeg"
    cfg = load_session_config_dict(payload)
    rgb = dict(cfg.streams)["front_rgb"]
    assert rgb.image_encoding == ImageEncoding.JPEG


def test_config_hash_stable():
    a = load_session_config_dict(_mixed_payload())
    b = load_session_config_dict(_mixed_payload())
    assert a.compute_config_hash() == b.compute_config_hash()


def test_unknown_top_level_key_rejected():
    payload = _mixed_payload()
    payload["sync"] = {"tolerance_ms": 50}
    try:
        load_session_config_dict(payload)
        assert False
    except SessionConfigError:
        pass


def test_missing_stream_field_rejected():
    payload = _mixed_payload()
    del payload["streams"]
    try:
        load_session_config_dict(payload)
        assert False
    except SessionConfigError:
        pass


def test_stream_fields_tuple():
    cfg = load_session_config_dict(_mixed_payload())
    for _, s in cfg.streams:
        assert isinstance(s.fields, tuple)
        for f in s.fields:
            assert isinstance(f, FieldRule)


def test_to_dict_roundtrip():
    cfg = load_session_config_dict(_mixed_payload())
    d = cfg.to_dict()
    assert d["schema_version"] == "1.0"
    assert "streams" in d
    assert "session" in d
    assert "storage" in d


def _mixed_payload():
    return {
        "schema_version": "1.0",
        "session": {"name": "mixed_collection", "task_id": "t", "operator_id": "op",
                     "recording_control": {"mode": "manual_ui"}},
        "storage": {"root": "data/episodes"},
        "streams": {
            "leader_pose": {"source": "teleop", "topic": "/lp", "message_type": "geometry_msgs/PoseStamped",
                            "time_domain": "ros_header", "frame_id": "robot_world",
                            "fields": [{"path": "position.x", "type": "float64"}]},
            "leader_buttons": {"source": "teleop", "topic": "/lb", "message_type": "std_msgs/Float32MultiArray",
                               "fields": [{"path": "data", "type": "sequence"}]},
            "arm_joints": {"source": "robot", "topic": "/aj", "message_type": "std_msgs/Float32MultiArray",
                           "fields": [{"path": "data", "type": "sequence"}]},
            "wrist_pose": {"source": "robot", "topic": "/wp", "message_type": "geometry_msgs/PoseStamped",
                           "time_domain": "ros_header", "frame_id": "robot_world"},
            "front_rgb": {"source": "sensor", "topic": "/frgb", "message_type": "sensor_msgs/Image",
                          "time_domain": "ros_header", "image_encoding": "raw",
                          "fields": [{"path": "height", "type": "uint32"}, {"path": "data", "type": "bytes"}]},
        },
    }
