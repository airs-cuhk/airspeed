"""Integration test: config → mock messages → AIRS HDF5 → validator."""

import tempfile, os
from datetime import datetime, timezone
from types import SimpleNamespace

from core.config import load_session_config_dict
from core.adapters import AdapterRegistry
from core.storage import AirsHdf5Writer
from core.validation import validate_dataset


UTC = timezone.utc


def _mixed_payload():
    return {
        "schema_version": "1.0",
        "session": {"name": "integration_test", "task_id": "t", "operator_id": "op"},
        "storage": {"root": "data/episodes"},
        "streams": {
            "leader_pose": {
                "source": "teleop", "topic": "/lp", "message_type": "geometry_msgs/PoseStamped",
                "time_domain": "ros_header", "frame_id": "robot_world",
                "fields": [{"path": "position.x", "type": "float64"}],
            },
            "arm_joints": {
                "source": "robot", "topic": "/aj", "message_type": "std_msgs/Float32MultiArray",
                "fields": [{"path": "data", "type": "sequence"}],
            },
            "front_rgb": {
                "source": "sensor", "topic": "/frgb", "message_type": "sensor_msgs/Image",
                "time_domain": "ros_header", "image_encoding": "raw",
                "fields": [{"path": "height", "type": "uint32"}, {"path": "data", "type": "bytes"}],
            },
        },
    }


def test_end_to_end_mixed_pipeline():
    config = load_session_config_dict(_mixed_payload())
    registry = AdapterRegistry.with_defaults()
    adapters = registry.resolve_session(config)

    d = tempfile.mkdtemp()
    try:
        w = AirsHdf5Writer(d, description=config.session.name,
                           robot_type="test_bot", series_number="S001")
        w.open_episode("ep001")
        w.register_vector_stream("leader_pose", 7,
                                 columns=("px", "py", "pz", "qx", "qy", "qz", "qw"))
        w.register_vector_stream("arm_joints", 3, columns=("j0", "j1", "j2"))
        w.register_image_stream("front_rgb", width=64, height=48, channels=3, encoding="rgb8")

        ts = datetime(2026, 5, 20, 12, 0, tzinfo=UTC)
        for i in range(10):
            ts_ns = int(ts.timestamp() * 1e9) + int(i * 1e9 / 30)

            pose_msg = SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=int(ts.timestamp()), nanosec=0),
                    frame_id="robot_world",
                ),
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.1, y=0.2, z=float(i) * 0.01),
                    orientation=SimpleNamespace(x=0.0, y=0.1, z=0.2, w=0.9),
                ),
            )
            result = adapters["leader_pose"].adapt(pose_msg, received_at=ts)
            w.append_vector("leader_pose", result.values, ts_ns)

            joint_msg = SimpleNamespace(data=[float(i), i * 0.1, i * 0.2])
            result = adapters["arm_joints"].adapt(joint_msg, received_at=ts)
            w.append_vector("arm_joints", result.values, ts_ns)

            img_msg = SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=int(ts.timestamp()), nanosec=0),
                    frame_id="camera_front_optical_frame",
                ),
                height=48, width=64, encoding="rgb8", is_bigendian=0, step=192,
                data=b"\x00" * (48 * 64 * 3),
            )
            result = adapters["front_rgb"].adapt(img_msg, received_at=ts)
            w.append_image("front_rgb", result.image_data, ts_ns)

        path = w.close_episode(sample_rate=30.0)
        report = validate_dataset(path)
        assert report.is_valid, f"Errors: {report.errors}"
        assert len(report.errors) == 0
    finally:
        import shutil; shutil.rmtree(d)
