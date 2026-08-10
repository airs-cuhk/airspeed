"""Gate/logging/timing/STATUS framework for the ROH integration pipeline.

Every step script follows the same lifecycle:
    gs = StepRun("01_verify_joy_mapping", requires=[])      # predecessor gate
    gs.gate("check name", ok, measured=..., threshold=...)  # hard boolean checks
    gs.finish()                                             # writes STATUS PASS

On any GateError or uncaught exception the step writes STATUS FAIL with the
exact reason and exits non-zero. Logs tee to logs/<step>/step.log; per-gate
and total wall-clock times land in logs/<step>/timing.json.
"""

from __future__ import annotations

import json
import logging
import sys
import time
import traceback
from pathlib import Path

PIPELINE_DIR = Path(__file__).resolve().parent
LOGS_ROOT = PIPELINE_DIR / "logs"


class GateError(RuntimeError):
    """Raised when a hard gate fails; aborts the step immediately."""


def _json_default(obj):
    if isinstance(obj, (set, frozenset)):
        return sorted(obj)
    if isinstance(obj, tuple):
        return list(obj)
    return str(obj)


class StepRun:
    def __init__(self, step: str, requires: list[str] | None = None):
        self.step = step
        self.log_dir = LOGS_ROOT / step
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log = logging.getLogger(step)
        self.log.setLevel(logging.DEBUG)
        self.log.handlers.clear()
        self.log.propagate = False
        fmt = logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s")
        fh = logging.FileHandler(self.log_dir / "step.log", mode="w")
        fh.setFormatter(fmt)
        ch = logging.StreamHandler()
        ch.setFormatter(fmt)
        self.log.addHandler(fh)
        self.log.addHandler(ch)
        self._t0 = time.monotonic()
        self.timing: dict = {"step": step, "gates": []}

        # Predecessor gate: every required step must have STATUS PASS.
        for req in requires or []:
            status_file = LOGS_ROOT / req / "STATUS.txt"
            status = status_file.read_text().strip() if status_file.exists() else "MISSING"
            if status != "PASS":
                self._abort(f"predecessor gate failed: {req} STATUS={status} (expected PASS)")
            self.log.info("predecessor %s STATUS=PASS", req)

    def gate(self, name: str, ok: bool, measured=None, threshold=None) -> None:
        """Hard boolean check. Failure aborts the step with a precise reason."""
        t = time.monotonic() - self._t0
        entry = {"gate": name, "elapsed_s": round(t, 3), "result": "PASS" if ok else "FAIL"}
        if measured is not None:
            entry["measured"] = measured
        if threshold is not None:
            entry["threshold"] = threshold
        self.timing["gates"].append(entry)
        if ok:
            self.log.info("GATE PASS: %s (measured=%r threshold=%r)", name, measured, threshold)
        else:
            self._abort(f"GATE FAIL: {name} (measured={measured!r} threshold={threshold!r})")

    def note(self, msg: str, *args) -> None:
        self.log.info(msg, *args)

    def write_artifact(self, name: str, content: str) -> Path:
        art_dir = PIPELINE_DIR / "artifacts"
        art_dir.mkdir(parents=True, exist_ok=True)
        path = art_dir / name
        path.write_text(content)
        self.log.info("artifact written: %s", path)
        return path

    def _abort(self, reason: str) -> None:
        self.log.error("%s", reason)
        self._write_status(f"FAIL: {reason}")
        raise GateError(reason)

    def _write_status(self, text: str) -> None:
        self.timing["total_s"] = round(time.monotonic() - self._t0, 3)
        (self.log_dir / "timing.json").write_text(
            json.dumps(self.timing, indent=2, default=_json_default))
        (self.log_dir / "STATUS.txt").write_text(text + "\n")

    def finish(self) -> None:
        self._write_status("PASS")
        self.log.info("STEP %s PASS (%.2fs)", self.step, self.timing["total_s"])


def run_step(step: str, requires: list[str] | None, body) -> int:
    """Entry point wrapper: run body(gs), translate failures to exit codes."""
    try:
        gs = StepRun(step, requires=requires)
        body(gs)
        gs.finish()
        return 0
    except GateError:
        return 1
    except Exception:  # noqa: BLE001 — any crash is a step FAIL with trace
        reason = f"uncaught exception:\n{traceback.format_exc()}"
        try:
            gs.log.error("%s", reason)  # type: ignore[has-type]
            gs._write_status(f"FAIL: {reason.splitlines()[-1]}")
        except Exception:
            (LOGS_ROOT / step).mkdir(parents=True, exist_ok=True)
            (LOGS_ROOT / step / "STATUS.txt").write_text(f"FAIL: {reason}\n")
        print(reason, file=sys.stderr)
        return 1
