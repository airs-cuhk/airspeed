# ROH Contact Latch — Constant-Pose Finger Deadlock, Fix Spec (2026-08-12)

Symptom: grasping a plastic bottle (five-finger mode), the hand squeezes,
the contact safety stop trips, and the hand **cannot be reopened by
releasing the VR trigger** — only the B button (full open) recovers it.

## Root cause

`src/robot_interface/openarm/roh-hand-ros2-adaptor/roh_hand_driver.py`

The contact-stop latch release check (`command_radians`, lines 182–204)
requires **every tripped finger** to be commanded back below its recorded
stop target minus hysteresis:

```python
release_counts = 2 * self._delta_counts          # ≈ 2621 counts (0.02 send-delta × 2)
retreating = all(target[i] <= stop[i] - release_counts for i in masked) \
             or all(t == 0 for t in target)
```

`masked` = `_latched_fingers` (the fingers that tripped). The escape only
works if the trigger can actually move each tripped finger's target down.

But the five-finger poses (`third_party/wrappers/roh_gesture_controller.py:81-82`)
hold thumb_root constant across the whole trigger range:

- `five_finger_ready_grasp = (0, 0, 0, 0, 0, 65000)`
- `five_finger_grasp      = (45000, 65535, 65535, 65535, 65535, 65000)`

When the bottle squeeze makes thumb_root itself trip (blocked, holding a
constant pose, current ≥ 250 mA — exactly the stall signature `_evaluate_poll`
detects, `roh_hand_driver.py:312-341`), its recorded stop target is 65000 and
every trigger-derived command also targets 65000:

```
65000 <= 65000 - 2621  →  False   (forever)
```

The latch is mathematically unreleaseable via the trigger; only `all(t == 0)`
(full open, the B button) clears it. The driver docstring's claim that
constant-pose fingers "do not block the release" covers only the case where
the constant-pose finger is *not* in the tripped set — the mask does not
help when thumb_root itself trips, which on a wrap-around grasp is the most
likely finger to trip.

## Chosen fix (to implement)

**Latch only fingers whose command was actively closing at trip time.**

A finger holding a fixed target (thumb_root at 65000) was not being driven
further when it tripped — its over-current is incidental squeeze from the
other fingers/object. It should not enter `_latched_fingers`, so it cannot
deadlock the release.

### Implementation sketch (`roh_hand_driver.py`)

1. Track which fingers the last accepted command *increased* (in counts):
   in `command_radians`, alongside `_last_target_counts`, store
   `_last_closing_mask: list[bool]` — `target[i] > last[i]` per finger
   (False for all fingers on the first command after connect/open).
2. In `_contact_stop`, filter the tripped indices through that mask:

   ```python
   active = [i for i in finger_indices
             if self._last_closing_mask and self._last_closing_mask[i]]
   latching = active or list(finger_indices)  # fallback: latch all tripped
   ```

   The fallback preserves current behavior when no closing-mask information
   exists (e.g. latch tripped without a prior command), so safety is never
   weakened — a genuinely driven finger is always latched.
3. `_latched_fingers = latching`. Stop semantics are unchanged:
   `safe_stop` still stops all fingers (`per_finger_stop: false`), only the
   *release mask* narrows.
4. Update the module docstring (lines 12–17): constant-pose fingers that
   trip while holding no longer enter the release mask.

### Tests (`tests/test_roh_hand_driver.py`)

Existing harness already covers latch/retreat; add:

- thumb_root (index 5) trips with a constant 65000 target while index
  closes → latch records only index; a trigger-release command (ready pose)
  is accepted.
- finger that was actively closing when it tripped still latches and still
  demands the 2× send-delta retreat (regression guard).
- fallback path: trip with no closing mask → all tripped fingers latched.

## Rejected alternatives

- **Pose tweak (`five_finger_grasp` thumb_root 65000 → 64000), proposed
  2026-08-12 — rejected.** The 1000-count delta is below the 2621-count
  hysteresis, and the sign is wrong: releasing the trigger would command
  thumb_root *more* closed than at trip, so the latch still never releases
  and the finger pushes further into the object. Even the correct-sign
  variant (ready 62000 / grasp 65000) only releases for trips at trigger
  ≥ 0.87 and changes validated grasp mechanics for every object.
- **Treat "not more closed than at trip" as non-blocking** — viable, but
  keeps a tripped constant-pose finger in the latch set and weakens the
  hysteresis semantics; the actively-closing filter matches the stated
  intent ("contact stop while closing") more directly.

## Out of scope (not changed)

- Firmware stall protection (200 mA / 300 ms / 500 ms retry) — unchanged.
- The 250 mA trip threshold and 200-count stall epsilon — unchanged.
- The separate VR open latch (B sets / A clears, `roh_gesture_fsm.py`) —
  unrelated, unchanged.

---

## Addendum: redundant safety layers (first-scan finding, recorded for further fixes)

From the independent safety-stack review (same day, before this spec). The
latch-release deadlock above is the *release-side* bug; there is also a
*trip-side* redundancy that makes the latch fire during routine holds.

### The five active layers

1. Firmware stall protection (in-hand): 200 mA, 300 ms, retry 500 ms, speed 16.
2. Driver contact latch: 250 mA + no-position-advance, 50 ms poll →
   `safe_stop` of ALL six fingers (`per_finger_stop: false`) + latch.
3. Send-delta threshold (2% anti-CAN-spam).
4. 0.25 s monitor cooldown after big pose jumps.
5. VR-side staleness gate + B/A open latch.

### The bad interaction

Layers 1 and 2 supervise the same phenomenon with thresholds too close
(200 vs 250 mA), so the firmware always acts first. During a normal object
hold the firmware stall fires **by design** (that is how the hand holds
without overheating), and its stop→retry cycle then produces exactly the
driver's trip signature: position static + current spike ≥ 250 mA. Result:
**every routine hold eventually latches the driver**, and the latch stops
all six fingers, not just the contacting one. Combined with the constant-
pose deadlock above, a normal bottle grasp becomes a whole-hand freeze.

### Secondary release-side friction

The retreat check compares against the *commanded* target at trip time (not
the fingers' actual contact position) with a 2× send-delta hysteresis, so a
partial trigger release does not release the latch; and a light re-squeeze
while still in contact re-trips within ~100 ms.

### Candidate directions (deferred — decide after the constant-pose fix lands)

- Make one layer the supervisor: firmware-primary (driver monitor demoted
  to telemetry/warnings; set firmware threshold deliberately, e.g. 250 mA)
  or driver-primary (firmware raised to a ~400 mA last-resort backstop).
- Consider `per_finger_stop: true` so a single contacting finger does not
  freeze the whole hand.
- If the driver latch is kept, base the release on actual positions (or the
  trigger value, as in the original 115 controller) rather than commanded
  targets.
