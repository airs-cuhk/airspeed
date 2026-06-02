"""ROS2 node shell — MultiThreadedExecutor variant (bandwidth-optimized).

Replaces rclpy.spin() with MultiThreadedExecutor(num_threads=4) so camera
callbacks and vector-stream callbacks run in parallel. Requires thread-safe
writer and stream tracker.

To use: replace the launch command's module path:
    python3 -m core.runtime.threaded.ros2_collection_node \
        --session-config config/session/profile_mixed.yaml

Or in the launch file, change:
    cmd=["python3","-m","core.runtime.threaded.ros2_collection_node", ...]

Original (single-threaded) remains at:
    core.runtime.ros2_collection_node
"""

from __future__ import annotations

import argparse
import sys
import time as _time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image as RosImage, JointState
    from std_msgs.msg import Float32MultiArray
    from std_srvs.srv import SetBool, Trigger
except ImportError:
    rclpy = None
    Node = object
    MultiThreadedExecutor = object
    PoseStamped, RosImage, Float32MultiArray, JointState = object, object, object, object
    Trigger, SetBool = object, object
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy = object, object, object, object

from core.config import (
    QoSDurability, QoSHistory, QoSReliability,
    RecordingControlMode, StreamConfig,
    load_session_config,
)
from core.adapters import AdapterRegistry
from core.runtime.recording_control import RecordingControlRouter
from core.runtime.recording_state import (
    RecordingLifecycle, RecordingStateMachine,
)
# Thread-safe variants
from core.runtime.threaded.stream_tracker import StreamTracker
from core.storage.threaded import AirsHdf5Writer


def _resolve_message_class(type_name: str):
    """Dynamically import a ROS2 message class from its string name.

    E.g. 'sensor_msgs/JointState' → sensor_msgs.msg.JointState.
    No hardcoded message type list — any ROS2 message in the session YAML works.
    """
    pkg, _, cls = type_name.partition("/")
    if not pkg or not cls:
        return None
    module_name = f"{pkg}.msg"
    try:
        module = __import__(module_name, fromlist=[cls])
        return getattr(module, cls, None)
    except ImportError:
        return None

DEFAULT_UI_HOST = "127.0.0.1"
DEFAULT_UI_PORT = 8765
DEFAULT_NUM_THREADS = 4


class PlatformCollectionNode(Node):
    """ROS2 shell — MultiThreadedExecutor variant. Same logic as base."""

    def __init__(
        self, *, session_config: str | Path, output_dir: str | None = None,
        operator_ui_host: str = DEFAULT_UI_HOST,
        operator_ui_port: int = DEFAULT_UI_PORT,
    ) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is not installed; source a ROS2 environment first")
        super().__init__("platform_collection")

        config_path = Path(session_config).expanduser().resolve()
        self._config = load_session_config(config_path)

        out_dir = Path(output_dir) if output_dir else Path(self._config.storage.root)
        self._writer = AirsHdf5Writer(
            out_dir,
            description=self._config.session.name,
            robot_type=_robot_type_from_config(self._config),
            series_number=self._config.session.operator_id,
        )

        def _start(episode_id: str) -> None:
            self.get_logger().info(f"EPISODE START: {episode_id}")
            self._writer.open_episode(episode_id)
            for adapter in self._adapters.values():
                adapter.register_with(self._writer)
            self._stream_tracker.reset()

        def _end(success: bool, reason: str) -> None:
            path = self._writer.close_episode(
                sample_rate=self._compute_sample_rate(),
                success=success,
                termination_reason=reason,
            )
            self.get_logger().info(f"episode closed: {path} success={success} reason={reason}")

        self._state_machine = RecordingStateMachine(start_handler=_start, end_handler=_end)
        self._stream_tracker = StreamTracker(
            stream_names=[name for name, _ in self._config.streams],
            expected_periods_ms={name: 500.0 for name, _ in self._config.streams},
        )
        self._control_router = RecordingControlRouter(
            self._config.session.recording_control, self._state_machine,
        )
        self._adapter_registry = AdapterRegistry.with_defaults()
        self._adapters = self._adapter_registry.resolve_session(self._config)

        self._create_subscriptions()
        self._create_services()
        self._start_manual_ui(operator_ui_host, operator_ui_port)

        self.get_logger().info(
            f"node initialized (threaded): {len(self._adapters)} streams, "
            f"output={out_dir}, control={self._control_router.mode.value}"
        )

    # -- subscriptions --

    def _create_subscriptions(self) -> None:
        for name, stream in self._config.streams:
            msg_cls = _resolve_message_class(stream.message_type)
            if msg_cls is None:
                self.get_logger().warn(f"skipping {name}: unsupported type {stream.message_type}")
                continue
            qos = _qos_profile_from_stream(stream)
            self.create_subscription(msg_cls, stream.topic, self._make_handler(name), qos)
            self.get_logger().info(f"  {name} → {stream.topic}")

    def _make_handler(self, stream_name: str):
        def handler(msg):
            self._handle_message(stream_name, msg)
        return handler

    def _handle_message(self, stream_name: str, msg: Any) -> None:
        # Route to recording control (device_binding mode) — no-op in other modes
        self._control_router.handle_stream_message(stream_name, msg)

        adapter = self._adapters.get(stream_name)
        if adapter is None:
            return
        try:
            now = datetime.now(timezone.utc)
            sample = adapter.adapt(msg, received_at=now)
            self._stream_tracker.record_valid(stream_name, sample.timestamp_ns)
            if not self._state_machine.is_recording:
                return
            if sample.image_data is not None:
                self._writer.append_image(stream_name, sample.image_data, sample.timestamp_ns)
            elif sample.values is not None:
                self._writer.append_vector(stream_name, sample.values, sample.timestamp_ns)
        except Exception as exc:
            self._stream_tracker.record_invalid(stream_name, None)
            self.get_logger().error(f"{stream_name} ingest failed: {exc}")

    # -- services, UI, helpers (identical to base) --

    def _create_services(self) -> None:
        self.create_service(Trigger, "/platform_collection/start_episode", self._svc_start)
        self.create_service(SetBool, "/platform_collection/end_episode", self._svc_end)
        self.create_service(Trigger, "/platform_collection/abort_episode", self._svc_abort)

    def _svc_start(self, req, resp):
        self.get_logger().info("ACTION: start_episode requested")
        result = self._control_router.handle_service_action("start")
        resp.success = result.accepted
        resp.message = result.message
        self.get_logger().info(f"ACTION: start_episode → accepted={result.accepted} {result.message}")
        return resp

    def _svc_end(self, req, resp):
        action = "save" if req.data else "stop"
        self.get_logger().info(f"ACTION: end_episode ({action}) requested")
        result = self._control_router.handle_service_action(action)
        resp.success = result.accepted
        resp.message = result.message
        self.get_logger().info(f"ACTION: end_episode → accepted={result.accepted} {result.message}")
        return resp

    def _svc_abort(self, req, resp):
        self.get_logger().info("ACTION: abort_episode requested")
        result = self._control_router.handle_service_action("abort")
        resp.success = result.accepted
        resp.message = result.message
        self.get_logger().info(f"ACTION: abort_episode → accepted={result.accepted} {result.message}")
        return resp

    def _start_manual_ui(self, host: str, port: int) -> None:
        try:
            from core.runtime.manual_operator_ui import ManualOperatorUI
            self._ui = ManualOperatorUI(
                state_machine=self._state_machine,
                control_router=self._control_router,
                stream_tracker=self._stream_tracker,
                host=host, port=port,
                logger=self.get_logger().info,
            )
            self._ui.start()
            self.get_logger().info(f"manual UI at http://{host}:{port}")
        except Exception as exc:
            self.get_logger().warn(f"manual UI failed to start: {exc}")

    def _compute_sample_rate(self) -> float:
        return 0.0

    def shutdown(self) -> None:
        if self._state_machine.is_recording:
            self._state_machine.abort_episode()
        if hasattr(self, "_ui"):
            try:
                self._ui.stop()
            except Exception:
                pass


def _qos_profile_from_stream(stream: StreamConfig) -> QoSProfile:
    return QoSProfile(
        depth=stream.qos.depth,
        reliability={QoSReliability.BEST_EFFORT: ReliabilityPolicy.BEST_EFFORT,
                      QoSReliability.RELIABLE: ReliabilityPolicy.RELIABLE}[stream.qos.reliability],
        durability={QoSDurability.VOLATILE: DurabilityPolicy.VOLATILE,
                    QoSDurability.TRANSIENT_LOCAL: DurabilityPolicy.TRANSIENT_LOCAL}[stream.qos.durability],
        history={QoSHistory.KEEP_LAST: HistoryPolicy.KEEP_LAST,
                 QoSHistory.KEEP_ALL: HistoryPolicy.KEEP_ALL}[stream.qos.history],
    )


def _robot_type_from_config(config) -> str:
    roles = sorted({d.role for _, d in config.session.devices})
    return "_".join(roles) if roles else "unknown"


# -- CLI entry point (MultiThreadedExecutor) --


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Platform collection node (threaded)")
    parser.add_argument("--session-config", required=True, help="Path to session YAML")
    parser.add_argument("--output-dir", default=None, help="Output directory for episodes")
    parser.add_argument("--operator-ui-host", default=DEFAULT_UI_HOST)
    parser.add_argument("--operator-ui-port", type=int, default=DEFAULT_UI_PORT)
    parser.add_argument("--num-threads", type=int, default=DEFAULT_NUM_THREADS,
                        help=f"Executor thread count (default: {DEFAULT_NUM_THREADS})")
    args = parser.parse_args(argv)

    if rclpy is None:
        raise SystemExit("rclpy not installed; source a ROS2 environment first")
    rclpy.init(args=None)
    node = PlatformCollectionNode(
        session_config=args.session_config, output_dir=args.output_dir,
        operator_ui_host=args.operator_ui_host, operator_ui_port=args.operator_ui_port,
    )

    executor = MultiThreadedExecutor(num_threads=args.num_threads)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["PlatformCollectionNode", "main"]
