"""All paths, constants, and conventions for the ROH integration pipeline.

Unit/representation decisions (stated once, here, per pipeline methodology):
- Finger joint positions/commands on topics are RADIANS (0 = fully open).
- Finger joint currents ride in JointState.effort, documented as mA.
- SDK boundary values are raw counts 0..65535 (0 = open, 65535 = closed);
  conversion happens only inside the ROH driver module.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"

# Implementation targets
VR_BRIDGE_SERVER = SRC / "teleoperation_interface/vr-standard-ros2-bridge-adaptor/vr_bridge_server.py"
IK_VR_NORMALIZER = SRC / "robot_interface/openarm/openarm-ik-ros2-adaptor/server/vr_normalizer.py"
OPENARM_ADAPTOR = SRC / "robot_interface/openarm/openarm-control-ros2-adaptor"
ROH_MODULE = SRC / "robot_interface/openarm/roh-hand-ros2-adaptor"
ROH_CONFIG = ROH_MODULE / "config/roh_hand.yaml"
DATA_COLLECTION = SRC / "data_collection_service"
SESSION_CONFIG_GRIPPER = DATA_COLLECTION / "config/session_vr_ik_robot_button_control.yaml"
SESSION_CONFIG_HAND = DATA_COLLECTION / "config/session_vr_ik_roh_hand_button_control.yaml"

# Remote (read-only verification only — never send control commands)
REMOTE_HOST = "airspeed115"
REMOTE_REPO = "/home/intern/airspeed-canonical"
REMOTE_OLD_STACK = "/home/intern/airspeed"  # reference source for the partner's ROH code

# Hardware constants
CAN_CHANNELS = {"left": "can2", "right": "can3"}
HAND_PUBLISH_HZ = 30.0
FINGER_JOINTS = ["thumb", "index", "middle", "ring", "pinky", "thumb_root"]
COUNTS_OPEN, COUNTS_CLOSED = 0, 65535

# VR Joy axes mapping (verified from source by step 01 — do not hand-edit;
# if this changes, step 01 must be re-run and the artifact regenerated)
JOY_AXES = {
    0: "trigger (analog 0..1)",
    1: "grip / middle finger",
    2: "reserved",
    3: "thumbstick click",
    4: "A button",
    5: "B button",
}
