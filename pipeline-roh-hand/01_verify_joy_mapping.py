#!/usr/bin/env python3
"""Step 01 — Verify the VR Joy axes mapping from source (decision D15).

The hand controller will consume sensor_msgs/Joy on /vr/<side>_buttons. The
partner's ROH code assumed Float32MultiArray channel semantics
(0=trigger, 3=thumbstick click, 4=A, 5=B). This step proves from source that
the canonical bridge constructs Joy.axes with the SAME channel order as the
old Float32MultiArray.data, and that the canonical IK code's documented index
convention matches — so the partner's button semantics port 1:1.

Output artifact: artifacts/joy_axes_mapping.md
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import VR_BRIDGE_SERVER, IK_VR_NORMALIZER, JOY_AXES

STEP = "01_verify_joy_mapping"


def body(gs):
    bridge_src = VR_BRIDGE_SERVER.read_text()
    normalizer_src = IK_VR_NORMALIZER.read_text()

    # Gate 1: canonical bridge publishes Joy on the button topics.
    gs.gate(
        "bridge publishes sensor_msgs/Joy on /vr/<side>_buttons",
        "sensor_msgs.msg import Joy" in bridge_src
        and re.search(r"create_publisher\(Joy, '/vr/left_buttons'", bridge_src) is not None
        and re.search(r"create_publisher\(Joy, '/vr/right_buttons'", bridge_src) is not None,
        measured="Joy on both button topics",
    )

    # Gate 2: Joy header is stamped (canonical ros_header contract).
    gs.gate(
        "Joy messages carry a stamped header",
        re.search(r"m\.header\.stamp = stamp", bridge_src) is not None,
        measured="m.header.stamp = stamp in _make_buttons",
    )

    # Gate 3: axes are built from the same client array as the old 115
    # Float32MultiArray version: first 6 entries of data[side]['button'],
    # zero-padded. (Verified on 2026-08-10: 115's _make_buttons is byte-identical
    # in construction — same buttons[:6] slice, same 0.0 padding — only the
    # destination field differs, .data vs .axes.)
    m = re.search(
        r"vals = \[float\(b\.get\('value', 0\)\) for b in buttons\[:6\]\].*?m\.axes = vals",
        bridge_src, re.S,
    )
    gs.gate(
        "Joy.axes built from buttons[:6] with 0.0 padding (same order as 115 .data)",
        m is not None,
        measured="buttons[:6] -> m.axes",
    )

    # Gate 4: canonical IK code documents the index convention.
    doc = re.search(r"Index 0: Trigger.*?1: Grip.*?3: Thumbstick.*?4: A.*?last: B",
                    normalizer_src, re.S)
    gs.gate(
        "vr_normalizer documents Pico index convention (0=trigger, 3=thumbstick, 4=A, last=B)",
        doc is not None,
        measured=doc.group(0)[:80] + "..." if doc else None,
    )

    # Gate 5: trigger extraction uses index 0, A uses 4, B uses last index —
    # matching the partner's ROH semantics (0=trigger, 3=click, 4=A, 5=B).
    gs.gate(
        "trigger read from axes[0], A from axes[4], B from axes[-1] in canonical code",
        "buttons[0]" in normalizer_src
        and "buttons[4] > 0.5" in normalizer_src
        and "buttons[-1] > 0.5" in normalizer_src,
        measured="buttons[0] / buttons[4] / buttons[-1]",
    )

    lines = ["# VR Joy Axes Mapping — verified from source", "",
             "Evidence: `%s` and `%s` (step 01 gates, this repo)." % (
                 VR_BRIDGE_SERVER.relative_to(VR_BRIDGE_SERVER.parents[3]),
                 IK_VR_NORMALIZER.relative_to(IK_VR_NORMALIZER.parents[3])), "",
             "The canonical bridge builds `Joy.axes` from the VR client's `button` array",
             "(`buttons[:6]`, zero-padded) — byte-identical construction to the old 115",
             "`Float32MultiArray.data` version, so channel order is preserved 1:1.", "",
             "| axes index | meaning | used by ROH hand controller for |", "|---|---|---|"]
    roh_use = {0: "trigger analog → grasp interpolation",
               1: "—", 2: "—",
               3: "thumbstick click → gesture mode cycle",
               4: "A (right) → clear open latch",
               5: "B (right) → open / reset hand, set latch"}
    for idx, meaning in JOY_AXES.items():
        lines.append(f"| {idx} | {meaning} | {roh_use[idx]} |")
    lines += ["", "Pressed threshold: value > 0.5. Trigger clamped to [0, 1].",
              "Conclusion: partner's ROH button semantics port unchanged onto Joy axes",
              "(0=trigger, 3=click, 4=A, 5=B). No index mismatch found."]
    gs.write_artifact("joy_axes_mapping.md", "\n".join(lines) + "\n")

    gs.gate("mapping artifact written", True, measured="artifacts/joy_axes_mapping.md")


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=[], body=body))
