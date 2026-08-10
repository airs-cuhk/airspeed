#!/usr/bin/env python3
"""Step 02 — Vendor the OHand SDK + gesture wrapper into the repo (D12).

Source (read-only): 115:/home/intern/airspeed/ROH/{ohand_serial_sdk_python-main,
wrappers/roh_gesture_controller.py} — mirrored under
src/robot_interface/openarm/roh-hand-ros2-adaptor/third_party/ so the wrapper's
relative SDK path (parents[1]/ohand_serial_sdk_python-main/src) resolves
unchanged. No PYTHONPATH exports, no ancestor walking.

Gates: files present, wrapper + SDK import cleanly, finger naming constant
matches the canonical 6-joint layout.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import ROH_MODULE, FINGER_JOINTS

STEP = "02_vendor_sdk"
TP = ROH_MODULE / "third_party"


def body(gs):
    sdk_src = TP / "ohand_serial_sdk_python-main/src"
    wrapper = TP / "wrappers/roh_gesture_controller.py"

    gs.gate("SDK source tree vendored", (sdk_src / "ohand/OHandSerialAPI.py").exists()
            and (sdk_src / "ohand/constants.py").exists()
            and (sdk_src / "ohand/interface/can/socet_can_interface.py").exists(),
            measured=str(sdk_src))
    gs.gate("gesture wrapper vendored", wrapper.exists(), measured=str(wrapper))
    gs.gate("no venv/firmware bulk vendored",
            not (TP / "roh_venv").exists() and not (TP / "roh_lites_firmware-main").exists(),
            measured="roh_venv and firmware excluded")

    sys.path.insert(0, str(sdk_src))
    sys.path.insert(0, str(TP / "wrappers"))
    import ohand.constants  # noqa: E402
    gs.gate("ohand SDK imports without PYTHONPATH tricks", True,
            measured="ohand.constants imported from vendored tree")

    import roh_gesture_controller  # noqa: E402
    gs.gate("wrapper imports cleanly", True, measured="roh_gesture_controller imported")
    gs.gate(
        "wrapper finger layout matches canonical 6-joint layout",
        list(roh_gesture_controller.FINGER_NAMES) == FINGER_JOINTS,
        measured=list(roh_gesture_controller.FINGER_NAMES), threshold=FINGER_JOINTS)
    gs.gate(
        "wrapper motor count is 6",
        roh_gesture_controller.NUM_MOTORS == 6,
        measured=roh_gesture_controller.NUM_MOTORS, threshold=6)
    gs.gate(
        "wrapper defaults target can2 @ 1 Mbps, hand_id 0x02",
        roh_gesture_controller.DEFAULT_CAN_CHANNEL == "can2"
        and roh_gesture_controller.DEFAULT_CAN_BITRATE == 1000000
        and roh_gesture_controller.DEFAULT_HAND_ID == 0x02,
        measured=(roh_gesture_controller.DEFAULT_CAN_CHANNEL,
                  roh_gesture_controller.DEFAULT_CAN_BITRATE,
                  roh_gesture_controller.DEFAULT_HAND_ID))


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["01_verify_joy_mapping"], body=body))
