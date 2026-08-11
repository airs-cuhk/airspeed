"""Unit tests for the canonical ROH hand driver, FSM, and unit conversion.

Runs without ROS2 and without hardware (dry_run=True). Mirrors the pipeline
step 03/05 gates so the logic is also covered by plain pytest.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

MODULE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(MODULE))
sys.path.insert(0, str(MODULE / "third_party/ohand_serial_sdk_python-main/src"))
sys.path.insert(0, str(MODULE / "third_party/wrappers"))

from roh_gesture_controller import GESTURE_POSITIONS  # noqa: E402
from roh_gesture_fsm import RohGestureFSM, VrFsmConfig  # noqa: E402
from roh_hand_driver import HandDriverConfig, RohHandDriver  # noqa: E402
from roh_units import counts_to_rad, rad_to_counts, validate_command  # noqa: E402

CLOSED_RAD = [1.6] * 6


# ---- units ---------------------------------------------------------------

def test_counts_rad_boundaries():
    assert counts_to_rad(0, 1.6) == 0.0
    assert counts_to_rad(65535, 1.6) == pytest.approx(1.6)
    assert rad_to_counts(0.0, 1.6) == 0
    assert rad_to_counts(1.6, 1.6) == 65535


def test_rad_to_counts_rejects_non_finite_and_insane():
    for bad in (float("nan"), float("inf"), 7.0):
        with pytest.raises(ValueError):
            rad_to_counts(bad, 1.6)


def test_validate_command_rejects_wrong_length():
    with pytest.raises(ValueError):
        validate_command([0.0] * 5, CLOSED_RAD)


# ---- driver (dry-run) -----------------------------------------------------

def test_send_delta_gating():
    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    drv.connect()
    assert drv.command_radians([0.5] * 6) is True
    assert drv.command_radians([0.5] * 6) is False  # below threshold, not resent
    assert drv.command_radians([0.9] * 6) is True
    drv.close()


def test_contact_latch_reject_and_retreat():
    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    drv.connect()
    drv.command_radians([0.9] * 6)
    drv._latched = True
    drv._stop_target_counts = list(drv._last_target_counts)
    assert drv.command_radians([1.0] * 6) is False  # closing rejected
    assert drv.command_radians([0.0] * 6) is True   # full-open retreat accepted
    assert drv.contact_latched is False
    drv.close()


def test_contact_latch_releases_on_tripped_finger_retreat():
    """Five-mode scenario: thumb_root holds the same pose in ready and grasp,
    so it can never retreat below its recorded stop target. A latch tripped
    by the closing fingers must release on trigger release without requiring
    thumb_root to retreat."""
    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    drv.connect()
    grasp = [0.9] * 5 + [1.55]          # thumb_root constant (ready == grasp)
    drv.command_radians(grasp)
    drv._latched = True
    drv._latched_fingers = [0, 1, 2, 3, 4]  # tripped by monitor, thumb_root not involved
    drv._stop_target_counts = list(drv._last_target_counts)
    assert drv.command_radians([0.88] * 5 + [1.55]) is False  # still pushing, rejected
    assert drv.command_radians([0.0] * 5 + [1.55]) is True    # trigger released, accepted
    assert drv.contact_latched is False
    drv.close()


def test_read_failure_surfaces_as_none_not_fabricated():
    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    drv.connect()
    positions, _ = drv.read_joint_state()  # dry-run positions are None
    assert positions is None
    assert drv.consecutive_read_failures >= 1
    drv.close()


def test_monitor_ignores_motion_current_during_free_sweep():
    """Over-threshold current while the finger is advancing must NOT trip —
    long free-space sweeps (e.g. thumb_root ready pose) draw >250 mA."""
    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    currents = [0, 0, 0, 0, 0, 541]
    assert drv._evaluate_poll(currents, [0, 0, 0, 0, 0, 1000]) == []  # first poll seeds
    assert drv._evaluate_poll(currents, [0, 0, 0, 0, 0, 5000]) == []  # advancing
    drv.close()


def test_monitor_trips_on_stalled_overcurrent_finger():
    """Over-threshold current with no position progress = contact/stall."""
    drv = RohHandDriver(HandDriverConfig(dry_run=True))
    currents = [0, 0, 0, 0, 0, 541]
    assert drv._evaluate_poll(currents, [0, 0, 0, 0, 0, 5000]) == []  # seed
    idx = drv._evaluate_poll(currents, [0, 0, 0, 0, 0, 5050])  # stalled (delta 50 < 200)
    assert idx == [5]
    drv.close()


# ---- FSM ------------------------------------------------------------------

def _fsm() -> RohGestureFSM:
    return RohGestureFSM(VrFsmConfig(side="left", closed_rad=CLOSED_RAD),
                         dict(GESTURE_POSITIONS))


def test_trigger_interpolates_ready_to_grasp_in_radians():
    fsm = _fsm()
    cmd, _ = fsm.tick([1.0, 0, 0, 0, 0, 0], [0.0] * 6)
    grasp = GESTURE_POSITIONS["five_finger_grasp"]
    expect = [counts_to_rad(c, cr) for c, cr in zip(grasp, CLOSED_RAD)]
    assert cmd == pytest.approx(expect)


def test_gesture_cycle_and_point_holds():
    fsm = _fsm()
    click = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    fsm.tick(click, [0.0] * 6)
    assert fsm.mode == "two"
    fsm.tick([0.0] * 6, [0.0] * 6)  # release
    fsm.tick(click, [0.0] * 6)
    assert fsm.mode == "point"
    cmd, _ = fsm.tick([1.0, 0, 0, 0, 0, 0], [0.0] * 6)
    assert cmd is None  # trigger ignored in point mode


def test_open_latch_and_clear():
    fsm = _fsm()
    cmd, _ = fsm.tick([1.0, 0, 0, 0, 0, 0], [0.0] * 5 + [1.0])  # right B
    assert cmd == pytest.approx([0.0] * 6)
    assert fsm.open_latched
    cmd, _ = fsm.tick([1.0, 0, 0, 0, 0, 0], [0.0] * 6)
    assert cmd is None  # trigger ignored while latched
    fsm.tick([0.0] * 6, [0.0] * 4 + [1.0, 0.0])  # right A clears
    assert not fsm.open_latched
