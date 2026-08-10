#!/usr/bin/env python3
"""Step 06 — hand-session YAML variant resolves through the real loader (D1/D8).

Evidence over assumption: the session config is loaded with the data-collection
service's own load_session_config, not parsed by hand. Also gates the IK
adaptor's gripper_enabled flag (7-column commands in hand sessions).
"""

import py_compile
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import DATA_COLLECTION, SESSION_CONFIG_HAND, SRC

STEP = "06_session_config"
IK_DIR = SRC / "robot_interface/openarm/openarm-ik-ros2-adaptor"


def body(gs):
    gs.gate("hand session YAML exists", SESSION_CONFIG_HAND.exists(),
            measured=str(SESSION_CONFIG_HAND))

    sys.path.insert(0, str(DATA_COLLECTION))
    from core.config import load_session_config

    cfg = load_session_config(str(SESSION_CONFIG_HAND))
    streams = dict(cfg.streams)

    gs.gate("session resolves 18 streams through the real loader",
            len(streams) == 18, measured=len(streams), threshold=18)

    hand_streams = {n: s for n, s in streams.items() if n.startswith("hand_")}
    gs.gate("4 hand streams (2 state + 2 command)",
            len(hand_streams) == 4, measured=sorted(hand_streams))

    gs.gate("hand streams are sensor_msgs/JointState with ros_header",
            all(s.message_type == "sensor_msgs/JointState"
                and s.time_domain.value == "ros_header"
                for s in hand_streams.values()),
            measured={n: (s.message_type, s.time_domain.value)
                      for n, s in hand_streams.items()})

    gs.gate("hand streams have exactly 6 columns (fingers)",
            all(len(s.columns) == 6 for s in hand_streams.values()),
            measured={n: len(s.columns) for n, s in hand_streams.items()}, threshold=6)

    gs.gate("hand topics match the driver node contract",
            {s.topic for s in hand_streams.values()} == {
                "/roh/left/joint_state", "/roh/right/joint_state",
                "/roh/left/joint_command", "/roh/right/joint_command"},
            measured=sorted(s.topic for s in hand_streams.values()))

    arm = [streams["arm_left_joint_state"], streams["arm_right_joint_state"],
           streams["ik_left_joint_commands"], streams["ik_right_joint_commands"]]
    gs.gate("arm + ik streams are 7 columns (gripper column dropped)",
            all(len(s.columns) == 7 for s in arm),
            measured={s.name: len(s.columns) for s in arm}, threshold=7)

    gs.gate("no stream uses a non-ros_header time domain",
            all(s.time_domain.value == "ros_header" for s in streams.values()),
            measured={n for n, s in streams.items() if s.time_domain.value != "ros_header"})

    gs.gate("no Float32MultiArray anywhere in the hand session",
            all(s.message_type != "std_msgs/Float32MultiArray" for s in streams.values()),
            measured={s.message_type for s in streams.values()})

    # -- IK adaptor gripper flag
    import yaml
    ik_robot = yaml.safe_load((IK_DIR / "config/robot.yaml").read_text())
    gs.gate("IK robot.yaml carries gripper_enabled (default true)",
            ik_robot.get("gripper_enabled") is True,
            measured=ik_robot.get("gripper_enabled"))

    loader_src = (IK_DIR / "server/config_loader.py").read_text()
    solver_src = (IK_DIR / "server/solver_loop.py").read_text()
    ws_src = (IK_DIR / "server/ws_handler.py").read_text()
    gs.gate("config_loader parses gripper_enabled into RobotConfig",
            "gripper_enabled: bool = True" in loader_src
            and 'gripper_enabled=bool(robot_data.get("gripper_enabled", True))' in loader_src,
            measured="flag in dataclass + parse")
    gs.gate("solver_loop publishes 7 joints when gripper disabled",
            "if config.robot.gripper_enabled:" in solver_src
            and "left_cmd = result.joint_radians_left[:7]" in solver_src,
            measured="conditional 7/8-column publish")
    gs.gate("ws_handler trims finger joint names when gripper disabled",
            "if not config.robot.gripper_enabled:" in ws_src
            and "left_joint_names = left_joint_names[:7]" in ws_src,
            measured="name lists trimmed to 7")

    for path in (IK_DIR / "server/config_loader.py", IK_DIR / "server/solver_loop.py",
                 IK_DIR / "server/ws_handler.py"):
        py_compile.compile(str(path), doraise=True)
        gs.gate(f"py_compile: {path.name}", True, measured=path.name)


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["05_vr_controller"], body=body))
