"""YAML configuration loader with typed access."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import math

import yaml


@dataclass
class MetaConfig:
    project: str
    version: str
    description: str
    urdf_reference: str
    ik_solver_package: str


@dataclass
class RobotConfig:
    urdf_path: str
    default_pose: dict[str, float]
    end_effectors: dict[str, str]
    home_position_deg: dict[str, float]
    home_position_rad: list[float] = field(init=False)

    def __post_init__(self) -> None:
        """Convert home_position_deg (degrees) to a flat radians list.

        Order: left_joint_1..7, left_gripper, right_joint_1..7, right_gripper
        """
        order = [
            "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4",
            "left_joint_5", "left_joint_6", "left_joint_7", "left_gripper",
            "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4",
            "right_joint_5", "right_joint_6", "right_joint_7", "right_gripper",
        ]
        self.home_position_rad = [
            math.radians(self.home_position_deg[name]) for name in order
        ]


@dataclass
class SolverConfig:
    max_iterations: int
    linear_solver: str
    cost_weights: dict[str, float]
    self_collision_margin_m: float
    use_home_warm_start: bool = False


@dataclass
class ServerConfig:
    host: str
    port: int
    static_routes: list[dict[str, str]]
    cors: dict[str, Any]


@dataclass
class SpaceConfig:
    frames: dict[str, dict[str, str]]
    transforms: dict[str, Any]


@dataclass
class ROS2TopicsConfig:
    left_pose: str
    right_pose: str
    head_pose: str
    left_buttons: str
    right_buttons: str
    raw_data: str


@dataclass
class QoSConfig:
    reliability: str
    durability: str
    history: str
    history_depth: int


@dataclass
class ROS2Config:
    topics: ROS2TopicsConfig
    qos: QoSConfig
    node_name: str
    spin_thread_daemon: bool


@dataclass
class AxisMappingConfig:
    native_to_intermediate: list[list[float]]
    intermediate_to_robot: str
    position_scale: float

    def get_rotation_matrix(self) -> list[list[float]]:
        """Return the 3x3 axis mapping matrix."""
        return self.native_to_intermediate


@dataclass
class CalibrationConfig:
    stale_timeout_s: float
    pin_button: str
    start_button: str


@dataclass
class PublisherConfig:
    enabled: bool
    topic: str
    node_name: str
    qos: QoSConfig


@dataclass
class VRConfig:
    ros2: ROS2Config
    axis_mapping: AxisMappingConfig
    calibration: CalibrationConfig
    publisher: PublisherConfig | None = None


@dataclass
class AppConfig:
    meta: MetaConfig
    robot: RobotConfig
    solver: SolverConfig
    server: ServerConfig
    space: SpaceConfig
    vr: VRConfig


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with open(path) as f:
        data = yaml.safe_load(f)
    if data is None:
        raise ValueError(f"Empty config file: {path}")
    return data


def _require(data: dict[str, Any], key: str, context: str) -> Any:
    """Validate a required field exists and is not empty/None."""
    if key not in data or data[key] is None:
        raise ValueError(f"Missing required field '{key}' in {context}")
    value = data[key]
    if isinstance(value, str) and not value.strip():
        raise ValueError(f"Empty required field '{key}' in {context}")
    return value


def _parse_vr_config(data: dict[str, Any]) -> VRConfig:
    """Parse and validate vr.yaml into VRConfig."""
    ros2 = _require(data, "ros2", "vr.yaml")
    topics = _require(ros2, "topics", "vr.yaml/ros2")
    qos_data = _require(ros2, "qos", "vr.yaml/ros2")
    axis_data = _require(data, "axis_mapping", "vr.yaml")
    cal_data = _require(data, "calibration", "vr.yaml")

    publisher_data = data.get("publisher")
    publisher: PublisherConfig | None = None
    if publisher_data is not None:
        pub_qos_data = _require(publisher_data, "qos", "vr.yaml/publisher")
        publisher = PublisherConfig(
            enabled=_require(publisher_data, "enabled", "vr.yaml/publisher"),
            topic=_require(publisher_data, "topic", "vr.yaml/publisher"),
            node_name=_require(publisher_data, "node_name", "vr.yaml/publisher"),
            qos=QoSConfig(
                reliability=_require(pub_qos_data, "reliability", "vr.yaml/publisher/qos"),
                durability=_require(pub_qos_data, "durability", "vr.yaml/publisher/qos"),
                history=_require(pub_qos_data, "history", "vr.yaml/publisher/qos"),
                history_depth=_require(pub_qos_data, "history_depth", "vr.yaml/publisher/qos"),
            ),
        )

    return VRConfig(
        ros2=ROS2Config(
            topics=ROS2TopicsConfig(
                left_pose=_require(topics, "left_pose", "vr.yaml/ros2/topics"),
                right_pose=_require(topics, "right_pose", "vr.yaml/ros2/topics"),
                head_pose=_require(topics, "head_pose", "vr.yaml/ros2/topics"),
                left_buttons=_require(topics, "left_buttons", "vr.yaml/ros2/topics"),
                right_buttons=_require(topics, "right_buttons", "vr.yaml/ros2/topics"),
                raw_data=_require(topics, "raw_data", "vr.yaml/ros2/topics"),
            ),
            qos=QoSConfig(
                reliability=_require(qos_data, "reliability", "vr.yaml/ros2/qos"),
                durability=_require(qos_data, "durability", "vr.yaml/ros2/qos"),
                history=_require(qos_data, "history", "vr.yaml/ros2/qos"),
                history_depth=_require(qos_data, "history_depth", "vr.yaml/ros2/qos"),
            ),
            node_name=_require(ros2, "node_name", "vr.yaml/ros2"),
            spin_thread_daemon=_require(ros2, "spin_thread_daemon", "vr.yaml/ros2"),
        ),
        axis_mapping=AxisMappingConfig(
            native_to_intermediate=_require(axis_data, "native_to_intermediate", "vr.yaml/axis_mapping"),
            intermediate_to_robot=_require(axis_data, "intermediate_to_robot", "vr.yaml/axis_mapping"),
            position_scale=_require(axis_data, "position_scale", "vr.yaml/axis_mapping"),
        ),
        calibration=CalibrationConfig(
            stale_timeout_s=_require(cal_data, "stale_timeout_s", "vr.yaml/calibration"),
            pin_button=_require(cal_data, "pin_button", "vr.yaml/calibration"),
            start_button=_require(cal_data, "start_button", "vr.yaml/calibration"),
        ),
        publisher=publisher,
    )


def load_config(config_dir: Path, solver_config_path: Path | None = None) -> AppConfig:
    """Load and validate all YAML config files."""
    if not config_dir.is_dir():
        raise FileNotFoundError(f"Config directory not found: {config_dir}")

    meta_data = _load_yaml(config_dir / "meta.yaml")
    robot_data = _load_yaml(config_dir / "robot.yaml")
    solver_data = _load_yaml(solver_config_path if solver_config_path else config_dir / "solver_smooth.yaml")
    server_data = _load_yaml(config_dir / "server.yaml")
    space_data = _load_yaml(config_dir / "space.yaml")
    vr_data = _load_yaml(config_dir / "vr.yaml")

    return AppConfig(
        meta=MetaConfig(
            project=_require(meta_data, "project", "meta.yaml"),
            version=_require(meta_data, "version", "meta.yaml"),
            description=_require(meta_data, "description", "meta.yaml"),
            urdf_reference=_require(meta_data, "urdf_reference", "meta.yaml"),
            ik_solver_package=_require(meta_data, "ik_solver_package", "meta.yaml"),
        ),
        robot=RobotConfig(
            urdf_path=_require(robot_data, "urdf_path", "robot.yaml"),
            default_pose=_require(robot_data, "joints", "robot.yaml")["default_pose"],
            end_effectors=_require(robot_data, "end_effectors", "robot.yaml"),
            home_position_deg=_require(robot_data, "home_position_deg", "robot.yaml"),
        ),
        solver=SolverConfig(
            max_iterations=_require(solver_data, "max_iterations", "solver.yaml"),
            linear_solver=_require(solver_data, "linear_solver", "solver.yaml"),
            cost_weights=_require(solver_data, "cost_weights", "solver.yaml"),
            self_collision_margin_m=_require(solver_data, "self_collision_margin_m", "solver.yaml"),
            use_home_warm_start=solver_data.get("use_home_warm_start", False),
        ),
        server=ServerConfig(
            host=_require(server_data, "host", "server.yaml"),
            port=_require(server_data, "port", "server.yaml"),
            static_routes=server_data.get("static_routes", []),
            cors=_require(server_data, "cors", "server.yaml"),
        ),
        space=SpaceConfig(
            frames=_require(space_data, "frames", "space.yaml"),
            transforms=_require(space_data, "transforms", "space.yaml"),
        ),
        vr=_parse_vr_config(vr_data),
    )
