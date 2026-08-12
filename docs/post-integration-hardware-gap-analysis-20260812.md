# Post-Integration Hardware Gap Analysis (2026-08-12)

What actually broke between the gated-pipeline integration (all 10 steps PASS,
2026-08-10) and real robot testing on 115 (2026-08-11/12). Each entry:
symptom → root cause → fix → why the pipeline didn't catch it.

The pipeline verified everything verifiable without hardware and an operator.
Every remaining gap was an assumption about the **physical platform or the
deployment environment** — invisible to source-level gates and dry-run tests.

## 1. CAN link mode: arms need CAN FD, not plain CAN

- **Symptom**: arm controller reports joints not connected; 0 RX packets.
- **Root cause**: arm channels were brought up as classic CAN, but the Damiao
  buses speak CAN FD (1M nominal / 5M data). The old README's bring-up line
  (copied into `setup_can.sh`) was wrong/stale.
- **Fix**: `fd on dbitrate 5000000` for can0/can1 (`13a9b8d`).
- **Why missed**: no gate exercised the real link layer; the old stack's docs
  were trusted over executed behavior (methodology anti-pattern #1).

## 2. USB enumeration instability + SDK port-0 constraint

- **Symptom**: one hand unreachable; channels swapped between adapters.
- **Root cause**: USB enumeration order is unstable across replugs; the OHand
  SDK cannot open `can0` (ports 1..16 only).
- **Fix**: `setup_can.sh` pins interface names by USB vendor/model ID.
- **Why missed**: the dev machine has no CAN hardware; enumeration order was
  an implicit assumption baked into config (`can2`/`can3`).

## 3. Contact-stop false trip on motion current

- **Symptom**: hand latched at startup during the ready-pose sweep; every
  further command rejected.
- **Root cause**: current-only threshold (250 mA) cannot distinguish
  free-space motion current (~540 mA measured on a long sweep) from contact.
- **Fix**: stall signature — trip only when over-current AND position not
  advancing (`a40b660`).
- **Why missed**: dry-run returns zero currents; no gate could observe real
  motor current profiles. Safety semantics were ported from docs/code, not
  measured data.

## 4. Latch release deadlock (thumb_root)

- **Symptom**: once latched, the hand could not be released by trigger
  release in five-mode.
- **Root cause**: release required *all* fingers to retreat below their stop
  positions; `thumb_root` holds an identical pose in ready and grasp, so it
  can never retreat.
- **Fix**: per-finger latch mask — only tripped fingers must retreat
  (`13a1c28`).
- **Why missed**: the unit test set the latch state directly rather than
  simulating the realistic grasp geometry. A content-level test (real pose
  pairs) caught it only when run by the operator.

## 5. Abrupt arm start on stream connect

- **Symptom**: arm jumps from home to leader pose at full speed when the IK
  stream starts or resumes.
- **Root cause**: slew-clamp tracking (`prev_cmd`) started empty; the first
  streamed frame bypassed the clamp.
- **Fix**: seed clamp tracking from the arm's actual observed pose
  (`13a1c28`).
- **Why missed**: needs the real closed loop (WS stream + physical arm) to
  manifest; the code path "worked" in every static check.

## 6. Startup wrist tremble

- **Symptom**: visible trembling during the home move, settling once the
  control loop starts.
- **Root cause**: three stacked factors — wrist gains tuned for the stock
  gripper (2–5× old-stack values), gravity URDF unaware of the ROH payload,
  and 50 Hz full-RNE gravity recomputation dithering torque. Analysis:
  `docs/arm-startup-trembling-analysis-20260811.md`.
- **Fix**: factor 3 fixed by caching gravity at 5 Hz in `_move_to_home`
  (`fe3ab82`); gains iterated on hardware and settled at a ζ-preserved
  softer wrist tune (kp 60/24/24/24, `e61cd9a`). URDF payload correction
  deliberately deferred.
- **Why missed**: dynamics. No static gate sees a control loop ringing.

## 7. Wrist IK branch flips

- **Symptom**: wrist twist → arm suddenly swings to another joint
  configuration; frontend looks continuous.
- **Root cause**: solver finds the same EE pose in a different joint branch
  at the wrist limits (J5 ±90°, J6 ±45°); nothing enforces joint-space
  continuity on the output path; home parked J5 exactly on the ±90° limit.
  Confirmed in the audit log (J6 jumps 23–27° with ~2 mm task error; one
  231° J1 wrap event). Analysis: `docs/wrist-ik-branch-flip-20260812.md`.
- **Fix so far**: home J5 rolled back to 0 (`aeb376c`). Proposed but not yet
  implemented: output-side branch-continuity guard (reject >15°/solve jumps
  with converged task error; keep warm start anchored to the accepted
  branch).
- **Why missed**: emerges only at envelope edges with a human operator
  twisting a real controller.

## 8. Deployment environment leakage

- **Symptoms**: `ModuleNotFoundError: lerobot.robots.openarms` on 115;
  service would have imported the old stack's `core` package.
- **Root causes**: operator shells export `LEROBOT_SRC` pointing at a stock
  upstream lerobot (no openarms fork); the copied venv carried editable
  installs pointing at the old repo paths.
- **Fixes**: `arm_controller.py` falls back to the bundled fork when
  LEROBOT_SRC lacks openarms (`39c436e`); dedicated canonical venv with
  editable links stripped.
- **Why missed**: the dev machine has neither the env var nor the old repo.
  Environment state on the target is invisible to the pipeline.

## 9. Privilege/transport interaction (sudo + FastDDS)

- **Symptom**: nodes run with sudo work on CAN but commands/states silently
  stop flowing to user-space processes.
- **Root cause**: FastDDS shared-memory transport is per-user; root-owned
  nodes can't exchange DDS data with the user-space VR bridge/collector.
- **Fix**: no-sudo guard + one-command stack launcher (`start_roh_stack.sh`,
  `13a1c28`).
- **Why missed**: nothing in the pipeline runs as root; DDS transport
  semantics only bite in the real multi-process deployment.

## 10. VR bridge fragility (adb)

- **Symptoms**: `no permissions` device errors; lost reverse tunnels; IK
  frontend unreachable from the headset (`ws://localhost:5200`).
- **Root causes**: udev rule ordering (`60-libgphoto2` overrode a `51-` rule;
  fixed as `99-`); the PICO re-enumerates on sleep/wake, dropping device-node
  perms and adb reverse mappings; the IK port (5200) was never tunneled, and
  "localhost" in the headset browser is the headset.
- **Fixes**: `99-pico-vr.rules` + plugdev group; adb reverse for both ports;
  a self-healing watchdog (added `3fa27cb`/`fffbc73`, later removed on 115
  during testing — currently the tunnel is *not* self-healing; revisit if
  disconnects recur).
- **Why missed**: USB/udev/adb behavior only exists on the target with the
  physical headset.

## Meta-lessons for the next pipeline

1. **Budget a hardware-shakeout stage explicitly.** Steps 01–10 verified
   logic; stage "H" (operator-present hardware validation) found 10 issues
   in two days. Plan for it instead of treating it as an afterthought.
2. **Every assumption about the physical layer needs a runtime gate**: link
   mode (CAN FD), channel identity (USB-ID pinning), current profiles, and
   joint-limit proximity at home are all checkable at startup — cheap gates
   that would have caught #1, #2, and part of #7 before the operator did.
3. **Deployment environment is part of the system.** Env vars, venv links,
   user/group membership, and udev rules on the target should be asserted by
   a preflight step, not discovered by a crash.
4. **Safety thresholds ported from another codebase are hypotheses.** The
   250 mA contact threshold was plausible on paper and wrong in free-space
   motion; calibrate from measured data before trusting it.
5. **The frontend agreeing with reality is not a gate.** Visualization can
   be continuous while joint space flips; only recorded telemetry (audit
   log) settles such questions — instrument first, then judge.
