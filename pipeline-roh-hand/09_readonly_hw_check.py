#!/usr/bin/env python3
"""Step 09 — read-only hardware verification on airspeed115 (S9).

NON-INVASIVE ONLY: CAN interfaces checked, vendored module imported on 115,
and each hand is CONNECTED (socket open) and READ (positions/currents).
Nothing is ever written: no send_positions, no safe_stop, no stall-protection
configuration. The robot is unsecured; control validation happens later with
an operator present.

Remote probes run on 115 inside /home/intern/airspeed-canonical with the
existing ROH venv python (has python-can).
"""

import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step

STEP = "09_readonly_hw_check"
HOST = "airspeed115"
REPO = "/home/intern/airspeed-canonical"
VENV_PY = "/home/intern/airspeed/.venv/bin/python"

# Read-only probe: connect (socket open only) -> read -> close. No writes.
PROBE = r"""
import json, sys
sys.path.insert(0, "src/robot_interface/openarm/roh-hand-ros2-adaptor")
sys.path.insert(0, "src/robot_interface/openarm/roh-hand-ros2-adaptor/third_party/ohand_serial_sdk_python-main/src")
sys.path.insert(0, "src/robot_interface/openarm/roh-hand-ros2-adaptor/third_party/wrappers")
from roh_gesture_controller import RohGestureController
from roh_units import counts_to_rad

result = {}
for side, channel in (("left", "can2"), ("right", "can3")):
    ctrl = RohGestureController(can_channel=channel)
    try:
        ctrl.connect()  # opens the CAN socket only — sends nothing
        positions = ctrl.read_positions().get("current")
        currents = ctrl.read_currents()
        result[side] = {
            "positions_counts": positions,
            "currents_ma": currents,
            "positions_rad": (
                [round(counts_to_rad(int(p), 1.6), 4) for p in positions]
                if positions and all(p is not None for p in positions) else None
            ),
        }
    except Exception as exc:
        result[side] = {"error": f"{type(exc).__name__}: {exc}"}
    finally:
        try:
            ctrl.close()  # socket shutdown only
        except Exception:
            pass
print("PROBE_JSON:" + json.dumps(result))
"""


def _ssh(gs, cmd: str, timeout: int = 60) -> tuple[int, str]:
    r = subprocess.run(
        ["ssh", "-n", "-o", "ConnectTimeout=10", HOST, cmd],
        capture_output=True, text=True, timeout=timeout,
    )
    out = (r.stdout + r.stderr).strip()
    gs.note("ssh$ %s -> rc=%d", cmd[:80], r.returncode)
    return r.returncode, out


def body(gs):
    # -- repo synced on 115
    rc, out = _ssh(gs, f"git -C {REPO} log --oneline -1")
    gs.gate("115 repo present and synced", rc == 0 and "03753e4" in out,
            measured=out)

    # -- CAN interfaces up and healthy
    rc, out = _ssh(gs, "ip -details link show can2; ip -details link show can3")
    gs.gate("can2/can3 UP, ERROR-ACTIVE, 1 Mbps",
            out.count("state UP") == 2
            and out.count("ERROR-ACTIVE") == 2
            and out.count("bitrate 1000000") == 2,
            measured=[l.strip() for l in out.splitlines() if "state" in l or "bitrate" in l])

    # -- no other process owns the CAN buses (single-owner rule)
    rc, out = _ssh(gs, "ps aux | grep -E 'roh|arm_controller|vr_bridge' | grep -v grep || true")
    gs.gate("no old-stack process owns the CAN buses",
            rc == 0 and not out.strip(), measured=out.strip() or "none")

    # -- module imports on 115
    rc, out = _ssh(
        gs,
        f"cd {REPO} && {VENV_PY} -c \""
        "import sys; sys.path.insert(0, 'src/robot_interface/openarm/roh-hand-ros2-adaptor');"
        "import roh_units, roh_gesture_fsm, roh_hand_driver, roh_config;"
        "cfg = roh_config.load_hand_configs('src/robot_interface/openarm/roh-hand-ros2-adaptor/config/roh_hand.yaml');"
        "print('hands:', len(cfg))\"",
    )
    gs.gate("canonical ROH module imports + config loads on 115",
            rc == 0 and "hands: 2" in out, measured=out)

    # -- read-only telemetry probe on both hands
    probe_b64 = __import__("base64").b64encode(PROBE.encode()).decode()
    rc, out = _ssh(
        gs,
        f"cd {REPO} && echo {probe_b64} | base64 -d | {VENV_PY} -",
        timeout=120,
    )
    marker = "PROBE_JSON:"
    gs.gate("read-only probe executed on 115", rc == 0 and marker in out,
            measured=out[-300:] if rc != 0 else "probe ran")
    probe = json.loads(out.split(marker, 1)[1].splitlines()[0])

    for side in ("left", "right"):
        entry = probe[side]
        ok = (
            "error" not in entry
            and entry.get("positions_counts") is not None
            and all(p is not None for p in entry["positions_counts"])
        )
        gs.gate(f"{side} hand answered position reads (CAN + SDK live)",
                ok, measured=entry)
        if ok:
            counts = [int(p) for p in entry["positions_counts"]]
            gs.gate(f"{side} positions within raw-count bounds (0..65535)",
                    all(0 <= p <= 65535 for p in counts),
                    measured=counts)
            gs.gate(f"{side} radian conversion within sanity bound (|x| <= 2π)",
                    all(abs(v) <= 6.29 for v in entry["positions_rad"]),
                    measured=entry["positions_rad"])
            currents = entry.get("currents_ma") or []
            gs.gate(f"{side} currents readable and below contact threshold (250 mA)",
                    len(currents) == 6
                    and all(c is None or c < 250 for c in currents),
                    measured=currents, threshold="< 250 mA")


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["08_pytest_suite"], body=body))
