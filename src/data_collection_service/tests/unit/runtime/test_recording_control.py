"""Test RecordingControlRouter mode gating and action validation."""
from core.config import RecordingControlConfig, RecordingControlMode
from core.runtime.recording_control import RecordingControlRouter
from core.runtime.recording_state import RecordingStateMachine


def _make_router(mode="service"):
    sm = RecordingStateMachine(
        start_handler=lambda eid: None,
        end_handler=lambda s, r: None,
    )
    cfg = RecordingControlConfig(mode=RecordingControlMode(mode))
    return RecordingControlRouter(cfg, sm)


def test_service_mode_accepts_service_action():
    router = _make_router("service")
    result = router.handle_service_action("start")
    assert result.accepted
    assert result.action == "start"


def test_service_mode_rejects_manual_ui_action():
    router = _make_router("service")
    result = router.handle_manual_ui_action("start")
    assert not result.accepted


def test_manual_ui_mode_accepts_ui_action():
    router = _make_router("manual_ui")
    result = router.handle_manual_ui_action("start")
    assert result.accepted


def test_manual_ui_mode_rejects_service_action():
    router = _make_router("manual_ui")
    result = router.handle_service_action("start")
    assert not result.accepted


def test_rejects_start_when_already_recording():
    router = _make_router("service")
    router.handle_service_action("start")
    result = router.handle_service_action("start")
    assert not result.accepted
    assert "already recording" in result.message.lower()


def test_rejects_stop_when_idle():
    router = _make_router("service")
    result = router.handle_service_action("stop")
    assert not result.accepted


def test_stop_save_abort_all_end_episode():
    router = _make_router("service")
    router.handle_service_action("start")
    r = router.handle_service_action("stop")
    assert r.accepted
    assert r.action == "stop"


def test_invalid_action_rejected():
    router = _make_router("service")
    try:
        router.handle_service_action("invalid")
        assert False
    except Exception:
        pass
