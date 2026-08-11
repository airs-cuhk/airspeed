#!/usr/bin/env python3
"""Step 04 — gripper_enabled flags ported to canonical (D8).

Hand sessions run arms as 7-joint: the gripper motor is skipped at bus build,
calibration, gripper test, command, and joint-state publishing. Gates are
source-level plus py_compile (lerobot/rclpy deps are not installed on the dev
machine); behavior is covered by the pytest suite in step 08.
"""

import py_compile
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import OPENARM_ADAPTOR

STEP = "04_gripper_flags"
CFG_SRC = OPENARM_ADAPTOR / "lerobot/robots/openarms/config_openarms_follower.py"
FOLLOWER_SRC = OPENARM_ADAPTOR / "lerobot/robots/openarms/openarms_follower.py"
ARM_CTRL = OPENARM_ADAPTOR / "arm_controller.py"
JS_PUB = OPENARM_ADAPTOR / "joint_state_publisher.py"
ROBOT_YAML = OPENARM_ADAPTOR / "config/robot.yaml"


def body(gs):
    cfg = CFG_SRC.read_text()
    fol = FOLLOWER_SRC.read_text()
    ctrl = ARM_CTRL.read_text()
    jsp = JS_PUB.read_text()

    gs.gate("config dataclass carries left/right_gripper_enabled (default True)",
            "left_gripper_enabled: bool = True" in cfg
            and "right_gripper_enabled: bool = True" in cfg,
            measured="both flags present")

    gs.gate("follower builds motors via _build_side_motors(gripper_enabled)",
            "_build_side_motors" in fol
            and 'if motor_name == "gripper" and not gripper_enabled' in fol
            and "motors_right = _build_side_motors(config.right_gripper_enabled)" in fol
            and "motors_left = _build_side_motors(config.left_gripper_enabled)" in fol,
            measured="_build_side_motors wired to both flags")

    gs.gate("calibration splits guarded by bus motor membership",
            fol.count('in self.bus_right.motors') >= 2
            and fol.count('in self.bus_left.motors') >= 2
            and 'in motors_right' in fol and 'in motors_left' in fol,
            measured="guards in __init__, calibrate, send_action")

    gs.gate("arm_controller passes flags to OpenArmsFollowerConfig",
            'left_gripper_enabled=cfg.get("left_gripper_enabled", True)' in ctrl
            and 'right_gripper_enabled=cfg.get("right_gripper_enabled", True)' in ctrl,
            measured="flags plumbed from robot.yaml")

    gs.gate("_test_grippers skips sides without a gripper motor",
            'if "gripper" in bus.motors' in ctrl
            and "Grippers disabled (dexterous-hand session)" in ctrl,
            measured="bus-membership guard + skip message")

    gs.gate("_apply_joints only commands gripper when motor exists",
            'if right_grip_deg is not None and "gripper" in follower.bus_right.motors' in ctrl
            and 'if left_grip_deg is not None and "gripper" in follower.bus_left.motors' in ctrl,
            measured="both sides guarded")

    gs.gate("WS gripper commands ignored for disabled sides",
            'latest.get("left_gripper_deg") if "gripper" in follower.bus_left.motors else None' in ctrl
            and 'latest.get("right_gripper_deg") if "gripper" in follower.bus_right.motors else None' in ctrl,
            measured="_clamp_joint_deltas call sites gated")

    gs.gate("joint_state_publisher trims gripper names/values when disabled",
            "self._left_gripper_enabled" in jsp
            and '"gripper" not in n' in jsp
            and "if gripper_enabled:" in jsp,
            measured="name trim + conditional gripper append")

    for path in (CFG_SRC, FOLLOWER_SRC, ARM_CTRL, JS_PUB):
        py_compile.compile(str(path), doraise=True)
        gs.gate(f"py_compile: {path.name}", True, measured=str(path.name))

    import yaml
    robot = yaml.safe_load(ROBOT_YAML.read_text())
    gs.gate("robot.yaml carries the flags (booleans; false = hand session on 115)",
            isinstance(robot.get("left_gripper_enabled"), bool)
            and isinstance(robot.get("right_gripper_enabled"), bool),
            measured=(robot.get("left_gripper_enabled"),
                      robot.get("right_gripper_enabled")))


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["03_driver_node"], body=body))
