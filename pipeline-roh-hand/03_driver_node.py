#!/usr/bin/env python3
"""Step 03 — ROH driver module: units, config, safety logic (dry-run).

Gates the canonical roh-hand-ros2-adaptor module without hardware:
- unit conversion roundtrip + bounds
- config loads with canonical topics/rates/calibration
- driver dry-run: send-delta gating, contact latch reject/retreat semantics
- rclpy node file parses (rclpy itself is not required on the dev machine)
"""

import ast
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import ROH_MODULE, FINGER_JOINTS

STEP = "03_driver_node"
sys.path.insert(0, str(ROH_MODULE))


def body(gs):
    # -- files exist
    for name in ("roh_units.py", "roh_hand_driver.py", "roh_config.py",
                 "roh_hand_node.py", "config/roh_hand.yaml",
                 "launch/start.sh", "README.md"):
        gs.gate(f"module file exists: {name}", (ROH_MODULE / name).exists(),
                measured=str(ROH_MODULE / name))

    # -- units
    import roh_units as u

    gs.gate("counts 0 -> 0 rad", u.counts_to_rad(0, 1.6) == 0.0, measured=0.0)
    gs.gate("counts 65535 -> closed_rad", abs(u.counts_to_rad(65535, 1.6) - 1.6) < 1e-12,
            measured=u.counts_to_rad(65535, 1.6), threshold=1.6)
    rt = [u.counts_to_rad(u.rad_to_counts(r, 1.6), 1.6) for r in (0.0, 0.4, 0.8, 1.6)]
    gs.gate("rad->counts->rad roundtrip within one count",
            all(abs(a - b) <= 1.6 / 65535 + 1e-9 for a, b in zip(rt, (0.0, 0.4, 0.8, 1.6))),
            measured=rt)
    for bad in (float("nan"), float("inf"), 7.0):
        try:
            u.rad_to_counts(bad, 1.6)
            ok = False
        except ValueError:
            ok = True
        gs.gate(f"rad_to_counts rejects {bad}", ok, measured=bad)
    try:
        u.validate_command([0.0] * 5, [1.6] * 6)
        ok = False
    except ValueError:
        ok = True
    gs.gate("validate_command rejects wrong length", ok, measured="5 positions")

    # -- config
    from roh_config import load_hand_configs

    hands = load_hand_configs(ROH_MODULE / "config/roh_hand.yaml")
    gs.gate("two enabled hands parsed", len(hands) == 2, measured=len(hands), threshold=2)
    left = next(h for h in hands if h.driver.side == "left")
    right = next(h for h in hands if h.driver.side == "right")
    gs.gate("CAN channels can2/can3", left.driver.can_channel == "can2"
            and right.driver.can_channel == "can3",
            measured=(left.driver.can_channel, right.driver.can_channel))
    gs.gate("canonical topics per hand",
            left.ros2.joint_state_topic == "/roh/left/joint_state"
            and left.ros2.joint_command_topic == "/roh/left/joint_command"
            and right.ros2.joint_state_topic == "/roh/right/joint_state",
            measured=(left.ros2.joint_state_topic, right.ros2.joint_state_topic))
    gs.gate("publish rate 30 Hz (D6)",
            all(h.ros2.publish_rate_hz == 30.0 for h in hands),
            measured=[h.ros2.publish_rate_hz for h in hands], threshold=30.0)
    gs.gate("closed_rad has 6 entries per hand",
            all(len(h.driver.closed_rad) == 6 for h in hands),
            measured=[len(h.driver.closed_rad) for h in hands], threshold=6)
    gs.gate("safety ported (contact 250 mA, stall 200 mA, delta 0.02, monitor 50 ms)",
            left.driver.safety.contact_current_ma == 250
            and left.driver.safety.firmware_stall_current_ma == 200
            and abs(left.driver.send_delta_threshold - 0.02) < 1e-9
            and abs(left.driver.monitor_interval_s - 0.05) < 1e-9,
            measured=(left.driver.safety.contact_current_ma,
                      left.driver.safety.firmware_stall_current_ma,
                      left.driver.send_delta_threshold,
                      left.driver.monitor_interval_s))

    # -- driver dry-run behavior
    from roh_hand_driver import HandDriverConfig, RohHandDriver

    cfg = HandDriverConfig(dry_run=True, side="left")
    drv = RohHandDriver(cfg)
    drv.connect()
    gs.gate("dry-run connect succeeds", drv.connected, measured=drv.get_status()["connected"])

    sent1 = drv.command_radians([0.5] * 6)
    sent2 = drv.command_radians([0.5] * 6)  # below send-delta -> not resent
    sent3 = drv.command_radians([0.9] * 6)  # above threshold -> sent
    gs.gate("send-delta gating (send, skip, send)",
            sent1 is True and sent2 is False and sent3 is True,
            measured=(sent1, sent2, sent3))

    # simulate contact latch and check reject/retreat semantics
    drv._latched = True
    drv._stop_target_counts = drv._last_target_counts
    rejected = drv.command_radians([1.0] * 6)
    retreat = drv.command_radians([0.0] * 6)
    gs.gate("contact latch rejects closing, accepts full-open retreat",
            rejected is False and retreat is True and drv.contact_latched is False,
            measured=(rejected, retreat, drv.contact_latched))

    status = drv.get_status()
    gs.gate("status dict exposes latch/failure counters",
            "contact_latched" in status and "consecutive_read_failures" in status,
            measured=sorted(status.keys()))
    drv.close()
    gs.gate("dry-run close succeeds", not drv.connected, measured=drv.connected)

    # -- node parses without importing rclpy (not installed on dev machine)
    tree = ast.parse((ROH_MODULE / "roh_hand_node.py").read_text())
    gs.gate("roh_hand_node.py parses", True, measured="ast.parse OK")

    # -- finger naming constant consistent across module and pipeline
    fingers = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "FINGERS" for t in node.targets
        ):
            fingers = ast.literal_eval(node.value)
    gs.gate("node finger order matches canonical layout",
            fingers == FINGER_JOINTS, measured=fingers, threshold=FINGER_JOINTS)


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["02_vendor_sdk"], body=body))
