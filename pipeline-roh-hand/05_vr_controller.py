#!/usr/bin/env python3
"""Step 05 — VR→hand controller: Joy FSM port (D2/D15).

Gates the pure state machine end-to-end (no ROS): trigger interpolation in
radians, gesture mode cycle, open/clear latch semantics, stale-button
fallback, and config wiring of the vr: blocks.
"""

import ast
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import ROH_MODULE

STEP = "05_vr_controller"
sys.path.insert(0, str(ROH_MODULE))
sys.path.insert(0, str(ROH_MODULE / "third_party/ohand_serial_sdk_python-main/src"))
sys.path.insert(0, str(ROH_MODULE / "third_party/wrappers"))


def body(gs):
    for name in ("roh_gesture_fsm.py", "roh_vr_controller_node.py",
                 "launch/start_vr_controller.sh"):
        gs.gate(f"module file exists: {name}", (ROH_MODULE / name).exists(),
                measured=name)

    from roh_gesture_controller import GESTURE_POSITIONS
    from roh_config import load_hand_configs
    from roh_gesture_fsm import RohGestureFSM
    from roh_units import counts_to_rad

    hands = load_hand_configs(ROH_MODULE / "config/roh_hand.yaml")
    gs.gate("both hands carry enabled vr: blocks",
            len(hands) == 2 and all(h.vr is not None and h.vr.enabled for h in hands),
            measured=[h.vr is not None for h in hands])
    left = next(h for h in hands if h.driver.side == "left")
    gs.gate("vr config ports partner indices (0=trigger, 3=cycle, 4=A, 5=B)",
            left.vr.fsm.trigger_index == 0 and left.vr.fsm.gesture_cycle_index == 3
            and left.vr.fsm.open_button_index == 5 and left.vr.fsm.clear_latch_button_index == 4
            and left.vr.fsm.open_button_side == "right",
            measured=(left.vr.fsm.trigger_index, left.vr.fsm.gesture_cycle_index,
                      left.vr.fsm.open_button_index, left.vr.fsm.clear_latch_button_index))
    gs.gate("vr control rate 20 Hz (partner value)",
            left.vr.control_rate_hz == 20.0, measured=left.vr.control_rate_hz)

    closed_rad = left.driver.closed_rad
    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))

    # -- trigger interpolation (mode five, default)
    ready = GESTURE_POSITIONS["five_finger_ready_grasp"]
    grasp = GESTURE_POSITIONS["five_finger_grasp"]
    cmd, _ = fsm.tick([0.0] * 6, [0.0] * 6)
    expect = [counts_to_rad(c, cr) for c, cr in zip(ready, closed_rad)]
    gs.gate("trigger=0 -> ready pose in radians",
            cmd is not None and all(abs(a - b) < 1e-9 for a, b in zip(cmd, expect)),
            measured=cmd[:2], threshold=expect[:2])

    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))
    cmd, _ = fsm.tick([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6)
    expect = [counts_to_rad(c, cr) for c, cr in zip(grasp, closed_rad)]
    gs.gate("trigger=1 -> grasp pose in radians",
            cmd is not None and all(abs(a - b) < 1e-9 for a, b in zip(cmd, expect)),
            measured=cmd[:2], threshold=expect[:2])

    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))
    cmd, _ = fsm.tick([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6)
    mid = [counts_to_rad(r + 0.5 * (g - r), cr)
           for r, g, cr in zip(ready, grasp, closed_rad)]
    # tolerance = one count in radians (interpolation in rad vs counts space)
    tol1 = max(closed_rad) / 65535 + 1e-9
    gs.gate("trigger=0.5 -> midpoint interpolation",
            cmd is not None and all(abs(a - b) < tol1 for a, b in zip(cmd, mid)),
            measured=cmd[:2], threshold=mid[:2])

    # -- deadzone: trigger below 0.03 reads as 0
    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))
    cmd, _ = fsm.tick([0.02, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6)
    expect = [counts_to_rad(c, cr) for c, cr in zip(ready, closed_rad)]
    gs.gate("trigger below deadzone -> ready pose",
            cmd is not None and all(abs(a - b) < 1e-9 for a, b in zip(cmd, expect)),
            measured=cmd[0])

    # -- gesture cycle: five -> two -> point -> five, rising edge only
    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))
    click = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    cmd, event = fsm.tick(click, [0.0] * 6)
    gs.gate("click rising edge -> mode two + ready pose",
            fsm.mode == "two" and cmd is not None and "two" in (event or ""),
            measured=(fsm.mode, event))
    cmd2, _ = fsm.tick(click, [0.0] * 6)  # held, not rising -> trigger interp in mode two
    gs.gate("held click does not re-cycle",
            fsm.mode == "two" and cmd2 is not None, measured=fsm.mode)
    fsm.tick([0.0] * 6, [0.0] * 6)  # release
    cmd, event = fsm.tick(click, [0.0] * 6)
    gs.gate("second click -> point mode holds point pose",
            fsm.mode == "point" and cmd is not None and "point" in (event or ""),
            measured=(fsm.mode, event))
    cmd, _ = fsm.tick([0.0] * 6, [0.0] * 6)
    gs.gate("point mode ignores trigger (no command)",
            cmd is None, measured=cmd)

    # -- open latch: right B opens + latches; right A clears
    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))
    b_press = [0.0] * 5 + [1.0]
    cmd, event = fsm.tick([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], b_press)
    gs.gate("right B rising -> open command + latch set",
            cmd is not None and all(abs(v) < 1e-9 for v in cmd) and fsm.open_latched,
            measured=(cmd, fsm.open_latched))
    cmd, _ = fsm.tick([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6)
    gs.gate("latched hand ignores trigger",
            cmd is None and fsm.open_latched, measured=(cmd, fsm.open_latched))
    a_press = [0.0] * 4 + [1.0, 0.0]
    cmd, event = fsm.tick([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], a_press)
    gs.gate("right A rising -> latch cleared",
            not fsm.open_latched and "cleared" in (event or ""),
            measured=(fsm.open_latched, event))
    cmd, _ = fsm.tick([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6)
    gs.gate("trigger control resumes after clear",
            cmd is not None, measured=cmd is not None)

    # -- empty (stale/absent) buttons behave as all-released
    fsm = RohGestureFSM(left.vr.fsm, dict(GESTURE_POSITIONS))
    cmd, _ = fsm.tick([], [])
    expect = [counts_to_rad(c, cr) for c, cr in zip(ready, closed_rad)]
    gs.gate("absent buttons -> trigger 0 (ready pose, hand opens)",
            cmd is not None and all(abs(a - b) < 1e-9 for a, b in zip(cmd, expect)),
            measured=cmd[0])

    # -- radian sanity: every emitted command is finite and within ±2π
    gs.gate("all gesture outputs within radian sanity bound",
            all(abs(v) <= 6.29 for g in fsm._gestures_rad.values() for v in g),
            measured=max(abs(v) for g in fsm._gestures_rad.values() for v in g),
            threshold="<= 2π")

    # -- node file parses (rclpy not installed on dev machine)
    ast.parse((ROH_MODULE / "roh_vr_controller_node.py").read_text())
    gs.gate("roh_vr_controller_node.py parses", True, measured="ast.parse OK")


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["04_gripper_flags"], body=body))
