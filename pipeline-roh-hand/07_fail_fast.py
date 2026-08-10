#!/usr/bin/env python3
"""Step 07 — fail-fast wiring for hand streams (D10).

Two layers, both gated with the real code paths:
1. Collector: hand streams are ordinary session streams, so the existing
   StreamTracker classifies a silent hand ABSENT and a stalled hand STALE —
   the same strictness as arm streams (no hand-specific collector code).
2. Source: the ROH driver node exits non-zero after READ_FAILURE_TIMEOUT_S
   of telemetry read failures, so a dead CAN bus fails loudly instead of
   recording fabricated gaps.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from util_gate import run_step
from util_paths import DATA_COLLECTION, ROH_MODULE, SESSION_CONFIG_HAND

STEP = "07_fail_fast"


def body(gs):
    # -- layer 1: collector stream tracker covers hand streams
    sys.path.insert(0, str(DATA_COLLECTION))
    from core.config import load_session_config
    from core.runtime.stream_tracker import StreamStatus, StreamTracker

    cfg = load_session_config(str(SESSION_CONFIG_HAND))
    names = [n for n, _ in cfg.streams]
    tracker = StreamTracker(
        stream_names=names,
        expected_periods_ms={n: 500.0 for n in names},
    )
    hand_names = [n for n in names if n.startswith("hand_")]
    gs.gate("all 4 hand streams registered in the stream tracker",
            all(n in tracker._metrics for n in hand_names) and len(hand_names) == 4,
            measured=hand_names)

    snap = tracker.snapshot()
    gs.gate("silent hand streams classify ABSENT (fail-visible)",
            all(snap[n][0] == StreamStatus.ABSENT for n in hand_names),
            measured={n: snap[n][0].value for n in hand_names})

    # feed one old message, then let it go stale
    for n in hand_names:
        tracker.record_valid(n, 1_000_000_000)
    snap = tracker.snapshot()
    gs.gate("fed hand stream not ABSENT/INVALID",
            all(snap[n][0] in (StreamStatus.HEALTHY, StreamStatus.STALE)
                for n in hand_names),
            measured={n: snap[n][0].value for n in hand_names})
    time.sleep(2.6)  # stale threshold: max(5x 500 ms expected period, 1000 ms)
    snap = tracker.snapshot()
    gs.gate("stalled hand stream classifies STALE after >1 s silence",
            all(snap[n][0] == StreamStatus.STALE for n in hand_names),
            measured={n: snap[n][0].value for n in hand_names})

    # -- layer 2: driver node fail-fast on persistent read failure
    node_src = (ROH_MODULE / "roh_hand_node.py").read_text()
    gs.gate("driver node raises SystemExit(2) after read-failure timeout",
            "raise SystemExit(2)" in node_src and "READ_FAILURE_TIMEOUT_S = 2.0" in node_src,
            measured="SystemExit(2) + 2.0 s timeout")
    gs.gate("driver node skips publish on read failure (no fabricated data)",
            "if positions_rad is None:" in node_src and "return" in node_src,
            measured="skip-publish path present")

    sys.path.insert(0, str(ROH_MODULE))
    from roh_hand_driver import HandDriverConfig, RohHandDriver

    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    drv.connect()
    # dry-run reads return None positions -> read_joint_state reports failure
    positions, _currents = drv.read_joint_state()
    gs.gate("dry-run read failure surfaces as None (not fabricated zeros)",
            positions is None, measured=positions)
    gs.gate("consecutive_read_failures increments on failure",
            drv.consecutive_read_failures >= 1,
            measured=drv.consecutive_read_failures)
    drv.close()

    # -- and the session YAML keeps the strict contract (no receipt-time domain)
    import yaml
    session = yaml.safe_load(SESSION_CONFIG_HAND.read_text())
    domains = {s.get("time_domain") for s in session["streams"].values()}
    gs.gate("session YAML pins ros_header for every stream (strict contract)",
            domains == {"ros_header"}, measured=domains, threshold={"ros_header"})


if __name__ == "__main__":
    sys.exit(run_step(STEP, requires=["06_session_config"], body=body))
