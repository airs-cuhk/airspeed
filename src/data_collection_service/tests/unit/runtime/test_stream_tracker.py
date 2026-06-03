"""Test StreamTracker status derivation and metrics."""
from core.runtime.stream_tracker import StreamTracker, StreamStatus
import time


def test_initial_status_is_absent():
    t = StreamTracker(["s1"], {"s1": 100.0})
    snap = t.snapshot()
    assert snap["s1"][0] == StreamStatus.ABSENT


def test_single_message_is_healthy():
    t = StreamTracker(["s1"], {"s1": 100.0})
    t.record_valid("s1", 1_700_000_000_000_000_000)
    snap = t.snapshot()
    assert snap["s1"][0] == StreamStatus.HEALTHY
    assert snap["s1"][1].messages_received == 1
    assert snap["s1"][1].messages_valid == 1


def test_stale_after_period_expires():
    t = StreamTracker(["s1"], {"s1": 50.0})  # 50 ms period, stale = max(5*50, 1000) = 1000ms
    base = 1_700_000_000_000_000_000
    for i in range(5):
        t.record_valid("s1", base + int(i * 5e7))  # 50ms apart
    time.sleep(1.2)  # past 1000ms absolute stale floor
    snap = t.snapshot()
    assert snap["s1"][0] == StreamStatus.STALE


def test_invalid_when_all_rejected():
    t = StreamTracker(["s1"], {"s1": 100.0})
    t.record_invalid("s1", None)
    t.record_invalid("s1", None)
    snap = t.snapshot()
    assert snap["s1"][0] == StreamStatus.INVALID
    assert snap["s1"][1].messages_received == 2
    assert snap["s1"][1].messages_valid == 0


def test_degraded_when_some_invalid():
    t = StreamTracker(["s1"], {"s1": 100.0})
    t.record_valid("s1", 1_700_000_000_000_000_000)
    t.record_invalid("s1", None)
    snap = t.snapshot()
    assert snap["s1"][0] == StreamStatus.DEGRADED


def test_reset_clears_metrics():
    t = StreamTracker(["s1"], {"s1": 100.0})
    t.record_valid("s1", 1_700_000_000_000_000_000)
    t.reset()
    snap = t.snapshot()
    assert snap["s1"][0] == StreamStatus.ABSENT
    assert snap["s1"][1].messages_received == 0


def test_rate_computation():
    t = StreamTracker(["s1"], {"s1": 100.0})
    base = 1_700_000_000_000_000_000
    for i in range(10):
        t.record_valid("s1", base + int(i * 1e8))  # 100 ms apart → 10 Hz
    snap = t.snapshot()
    rate = snap["s1"][1].observed_rate_hz
    assert rate is not None
    assert 9.0 < rate < 11.0  # ~10 Hz


def test_multi_stream_independence():
    t = StreamTracker(["a", "b"], {"a": 100.0, "b": 100.0})
    t.record_valid("a", 1_700_000_000_000_000_000)
    snap = t.snapshot()
    assert snap["a"][0] == StreamStatus.HEALTHY
    assert snap["b"][0] == StreamStatus.ABSENT
