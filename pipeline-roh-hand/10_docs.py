#!/usr/bin/env python3
"""Step 10 — documentation updated and consistent (S10).

Gates that the docs a newcomer reads first reflect the implementation:
CAN setup script referenced, SDK can0 constraint documented, CONTEXT.md
glossary entries, root README hand-session section, plan doc results.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import REPO_ROOT, ROH_MODULE

STEP = "10_docs"
PLAN = REPO_ROOT / "docs/roh-hand-canonical-integration-plan.md"


def body(gs):
    roh_readme = (ROH_MODULE / "README.md").read_text()
    gs.gate("ROH README references setup_can.sh and the can0 SDK constraint",
            "setup_can.sh" in roh_readme and "cannot open `can0`" in roh_readme,
            measured="setup script + constraint documented")
    gs.gate("ROH README documents single-owner rule and left/right caveat",
            "Single-owner rule" in roh_readme and "identical" in roh_readme,
            measured="operator caveats present")

    setup = ROH_MODULE / "launch/setup_can.sh"
    gs.gate("setup_can.sh exists and is executable",
            setup.exists() and setup.stat().st_mode & 0o111,
            measured=oct(setup.stat().st_mode & 0o777))

    context = (REPO_ROOT / "CONTEXT.md").read_text()
    gs.gate("CONTEXT.md has Hand session + Contact latch glossary entries",
            "**Hand session**" in context and "**Contact latch**" in context,
            measured="glossary updated")
    gs.gate("CONTEXT.md pins radian/finger layout in the glossary",
            "/roh/<side>/joint_state" in context and "thumb_root" in context,
            measured="topic + finger order documented")

    readme = (REPO_ROOT / "README.md").read_text()
    gs.gate("root README documents the hand session launch path",
            "roh-hand-ros2-adaptor/launch/setup_can.sh" in readme
            and "session_vr_ik_roh_hand_button_control.yaml" in readme,
            measured="launch + session config referenced")

    plan = PLAN.read_text()
    gs.gate("plan doc carries implementation results + hardware findings",
            "## Implementation results (2026-08-10" in plan
            and "cannot open `can0`" in plan
            and "setup_can.sh" in plan,
            measured="results + findings recorded")
    gs.gate("plan doc lists remaining operator-present work",
            "Not yet done" in plan and "closed_rad" in plan,
            measured="calibration + control validation pending items")

    manifest = (REPO_ROOT / "pipeline-roh-hand/MANIFEST.md").read_text()
    gs.gate("MANIFEST unchanged in purpose (immutable anchor)",
            "Purpose." in manifest and "Invariants" in manifest,
            measured="manifest intact")


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["09_readonly_hw_check"], body=body))
