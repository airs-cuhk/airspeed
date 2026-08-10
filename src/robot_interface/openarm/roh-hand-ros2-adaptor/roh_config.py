"""YAML loader for the canonical roh_hand.yaml (roh_hands list only)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from roh_hand_driver import HandDriverConfig, SafetyConfig
from roh_units import NUM_FINGERS


@dataclass
class Ros2InterfaceConfig:
    joint_state_topic: str
    joint_command_topic: str
    publish_rate_hz: float = 30.0
    joint_name_prefix: str = "roh"


@dataclass
class NodeHandConfig:
    driver: HandDriverConfig
    ros2: Ros2InterfaceConfig
    finger_names: list[str] = field(default_factory=list)


def _int(value, default: int) -> int:
    if value is None:
        return default
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def _bool(value, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def load_hand_configs(path: str | Path) -> list[NodeHandConfig]:
    """Parse roh_hand.yaml; returns one NodeHandConfig per ENABLED hand."""
    with open(path) as f:
        data = yaml.safe_load(f)
    if not data or not isinstance(data.get("roh_hands"), list):
        raise ValueError(f"{path}: expected a 'roh_hands' list")

    configs: list[NodeHandConfig] = []
    for item in data["roh_hands"]:
        if not _bool(item.get("enabled"), False):
            continue
        side = str(item.get("side", "left")).lower()
        safety_data = item.get("safety") or {}
        sd = SafetyConfig()
        safety = SafetyConfig(
            speed=_int(safety_data.get("speed"), sd.speed),
            current_limit_ma=_int(safety_data.get("current_limit_ma"), sd.current_limit_ma),
            contact_stop_enabled=_bool(safety_data.get("contact_stop_enabled"), sd.contact_stop_enabled),
            contact_current_ma=_int(safety_data.get("contact_current_ma"), sd.contact_current_ma),
            firmware_stall_enabled=_bool(safety_data.get("firmware_stall_enabled"), sd.firmware_stall_enabled),
            firmware_stall_speed=_int(safety_data.get("firmware_stall_speed"), sd.firmware_stall_speed),
            firmware_stall_current_ma=_int(safety_data.get("firmware_stall_current_ma"), sd.firmware_stall_current_ma),
            firmware_stall_after_ms=_int(safety_data.get("firmware_stall_after_ms"), sd.firmware_stall_after_ms),
            firmware_stall_retry_ms=_int(safety_data.get("firmware_stall_retry_ms"), sd.firmware_stall_retry_ms),
            per_finger_stop=_bool(safety_data.get("per_finger_stop"), sd.per_finger_stop),
        )
        closed_rad = [float(v) for v in (item.get("calibration") or {}).get("closed_rad", [1.6] * NUM_FINGERS)]
        if len(closed_rad) != NUM_FINGERS:
            raise ValueError(f"{path}: side={side} closed_rad must have {NUM_FINGERS} entries")

        driver = HandDriverConfig(
            side=side,
            can_channel=str(item.get("can_channel", "can2")),
            can_bitrate=_int(item.get("can_bitrate"), 1000000),
            hand_id=_int(item.get("hand_id"), 0x02),
            master_id=_int(item.get("master_id"), 0x01),
            dry_run=_bool(item.get("dry_run"), False),
            send_delta_threshold=float(item.get("send_delta_threshold", 0.02)),
            monitor_interval_s=float(item.get("monitor_interval_s", 0.05)),
            closed_rad=closed_rad,
            safety=safety,
        )

        ros2_data = item.get("ros2") or {}
        ros2 = Ros2InterfaceConfig(
            joint_state_topic=str(ros2_data.get("joint_state_topic", f"/roh/{side}/joint_state")),
            joint_command_topic=str(ros2_data.get("joint_command_topic", f"/roh/{side}/joint_command")),
            publish_rate_hz=float(ros2_data.get("publish_rate_hz", 30.0)),
            joint_name_prefix=str(ros2_data.get("joint_name_prefix", f"roh_{side}")),
        )
        configs.append(NodeHandConfig(driver=driver, ros2=ros2))
    return configs
