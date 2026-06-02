from core.runtime.recording_state import (
    RecordingLifecycle, RecordingStateError, RecordingStateMachine,
)


def test_initial_state_is_idle():
    started = []
    ended = []
    sm = RecordingStateMachine(
        start_handler=lambda eid: started.append(eid),
        end_handler=lambda s, r: ended.append((s, r)),
    )
    assert sm.is_idle
    assert not sm.is_recording
    assert sm.lifecycle == RecordingLifecycle.IDLE


def test_start_transitions_to_recording():
    started = []
    sm = RecordingStateMachine(
        start_handler=lambda eid: started.append(eid),
        end_handler=lambda s, r: None,
    )
    sm.start_episode("ep001")
    assert sm.is_recording
    assert sm.active_episode_id == "ep001"
    assert len(started) == 1


def test_rejects_start_when_recording():
    sm = RecordingStateMachine(
        start_handler=lambda eid: None,
        end_handler=lambda s, r: None,
    )
    sm.start_episode("ep001")
    try:
        sm.start_episode("ep002")
        assert False
    except RecordingStateError:
        pass


def test_rejects_end_when_idle():
    sm = RecordingStateMachine(
        start_handler=lambda eid: None,
        end_handler=lambda s, r: None,
    )
    try:
        sm.end_episode(True, "done")
        assert False
    except RecordingStateError:
        pass


def test_end_returns_to_idle():
    ended = []
    sm = RecordingStateMachine(
        start_handler=lambda eid: None,
        end_handler=lambda s, r: ended.append((s, r)),
    )
    sm.start_episode("ep001")
    sm.end_episode(True, "completed")
    assert sm.is_idle
    assert len(ended) == 1
    assert ended[0] == (True, "completed")


def test_abort_calls_end_with_false():
    ended = []
    sm = RecordingStateMachine(
        start_handler=lambda eid: None,
        end_handler=lambda s, r: ended.append((s, r)),
    )
    sm.start_episode("ep001")
    sm.abort_episode()
    assert sm.is_idle
    assert ended[0] == (False, "operator_abort")


def test_health_snapshot():
    sm = RecordingStateMachine(
        start_handler=lambda eid: None,
        end_handler=lambda s, r: None,
    )
    snap = sm.health_snapshot
    assert snap.lifecycle == RecordingLifecycle.IDLE
    sm.start_episode("ep001")
    snap = sm.health_snapshot
    assert snap.lifecycle == RecordingLifecycle.RECORDING
    assert snap.active_episode_id == "ep001"
