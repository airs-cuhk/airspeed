# ROH Hand Integration — Canonical Merge Plan

Date: 2026-08-10 (decisions refined via structured interview same day)
Status: plan, decisions locked — ready for implementation
Source: investigation of `airspeed115:/home/intern/airspeed/src` vs local `src/`
(full unified diff archived at `/tmp/src-diff-115-vs-local.txt` on the dev machine)

## Background

A partner integrated the ROH dexterous hand (OYMotion OHand / ROH-LiteS001,
SocketCAN can2/can3) into airspeed on machine `airspeed115`, based on an older
airspeed version. The goal is to merge that work into current canonical `main`
in a generic way: the hand's ROS topics must follow the canonical messaging
rules, and the data-collection service must configure the same topics without
core changes.

## Locked decisions (interview 2026-08-10)

| # | Decision | Choice |
|---|----------|--------|
| D1 | Grippers vs hands | **Mutually exclusive sessions** — a session uses arm grippers XOR ROH hands, never both |
| D2 | Hand command path | **Standalone VR→hand controller** consuming `sensor_msgs/Joy`, publishing canonical `JointState` commands (not through the IK/solver chain) |
| D3 | Process ownership | ROH driver runs as its **own ROS2 node/process** owning can2/can3 |
| D4 | Units on topics | **Radians** with per-finger calibration ranges in `roh_hand.yaml`; counts↔radians at the driver edge |
| D5 | Telemetry streams | Positions + currents only — **forces stream dropped**; currents ride in `joint_state.effort` (documented as mA) |
| D6 | Publish rate | **30 Hz** for all hand streams (matches the proven 115 rate) |
| D7 | Safety logic | **Port in full** — contact-stop latch, stall protection, send-delta threshold, all config-driven from `roh_hand.yaml` |
| D8 | Gripper column in hand sessions | **Config-flag disable** — port 115's `gripper_enabled` flags into canonical lerobot config / `arm_controller.py`; arms run 7 joints in hand sessions |
| D9 | Single-hand sessions | **Supported, config-driven** — `roh_hands` list entries individually enabled/disabled |
| D10 | Episode failure policy | **Fail fast** — hand stream timeout/CAN error fails the episode, same strictness as arm streams |
| D11 | Package location | **Submodule of the openarm adaptor** (`robot_interface/openarm/...`), but running as its own node per D3 |
| D12 | SDK dependency | **Vendor into the repo** (`third_party/` under the ROH module) — OHand SDK + `RohGestureController` wrapper; removes fragile PYTHONPATH/ancestor-walking |
| D13 | Porting workflow | Development on local branch **`feature/roh-hand-canonical-integration`** (one commit per feature stage); synced **one-directional local → 115** to `115:/home/intern/airspeed-canonical` via git bundle over SSH per `docs/git-bundle-sync-to-226.md` (ff-only, never force). Bootstrapped 2026-08-10; 115's `origin` URL is pinned to the stable bundle path `/tmp/airspeed-canonical-sync.bundle` |
| D14 | Old stack on 115 | **Keep `/home/intern/airspeed` as-is** during the port; CAN contention handled by operator convention (never run both stacks simultaneously) until the user confirms retirement timing |
| D15 | Joy button indices | **Verify from `vr_bridge_server.py` source** before porting; map partner's semantics (axes 0=trigger, 3=joystick click, 4=A, 5=B) and document any mismatch |

## Why the 115 integration diverged

The 115 work is functionally solid (safety logic, gesture state machine, CAN
handling), but instead of conforming to the existing airspeed contract it added
a **parallel message/type system** around it:

- Header-less `std_msgs/Float32MultiArray` topics
  (`/vr/*_buttons`, `/roh/<side>/currents|forces|positions_current`, 30 Hz)
- New time domains `ros_receive` / `system` / `steady` plus header-less
  fallback logic in `core/adapters/common.py`
- Three extra adapter bindings (`teleop_f32_buttons`, `robot_f32_array`,
  `sensor_f32_array`) and payload profiles (`robot_array`, `sensor_array`)
  that exist only on 115
- A duplicated `vr_subscriber.py` in the control adaptor (canonical VR
  subscription already lives in the IK adaptor, now on `sensor_msgs/Joy`)
- A subprocess `arm_state_publisher.py` (canonical main uses the in-process
  `joint_state_publisher.py`)
- A session YAML with 7-column arm streams — incompatible with canonical
  8-column streams (7 arm joints + gripper, radians)

So the merge is not "copy the files over" — it is **re-expressing the ROH
device in the canonical contract**, then porting the safety/gesture logic into
that shape.

## Canonical rules to conform to (current main)

1. Every stream stamps with `ros_header` — the message must carry a Header.
   No receipt-time fallbacks.
2. Joint positions/commands are `sensor_msgs/JointState`; arms use 8 columns
   (7 joints + gripper), **radians** at the topic boundary.
3. VR buttons are `sensor_msgs/Joy` (channels in `axes`, stamped header),
   published by `vr_bridge_server.py`.
4. Devices are declared in the session YAML; adapters resolve from message
   type — no per-device adapter code in the core.
5. Driver/SDK units (raw counts, mA, mN) are converted at the driver edge and
   never leak onto topics.

## Target integration shape

Each ROH hand is its **own device** (D3, D11): a standalone ROS2 node living
in a submodule of the openarm adaptor, owning its CAN bus, publishing and
subscribing stamped standard messages. Hands are not folded into the arm's
gripper slot.

Topic/message mapping (115 → canonical, reflects D4/D5):

| 115 (current on airspeed115) | Canonical target |
|---|---|
| `/roh/<side>/positions_current` — Float32MultiArray, raw counts 0..65535 | `/roh/<side>/joint_state` — `sensor_msgs/JointState`, 6 named finger joints (thumb, index, middle, ring, pinky, thumb_root), **radians**, stamped, 30 Hz |
| `/roh/<side>/currents` — mA | `effort` field of the same `/roh/<side>/joint_state` (documented as mA) |
| `/roh/<side>/forces` — mN | **dropped** (D5) |
| internal raw-count command targets | subscribe `/roh/<side>/joint_command` — `JointState`, radians; convert to counts at the driver edge |
| `/vr/<side>_buttons` Float32MultiArray | canonical `sensor_msgs/Joy` (`axes`), duplicate `vr_subscriber.py` deleted; index mapping verified from `vr_bridge_server.py` source first (D15) |

Consequence for the data-collection service: **zero core changes**. No
`ros_receive` domain, no `f32` adapters, no new profiles. The session YAML
gains hand streams using the existing `robot_joint_state` profile with
`ros_header`, 6 columns per hand (D5). `test_roh_f32_streams.py` (which
currently pins the Float32MultiArray/`ros_receive` contract) gets inverted:
header-less input must raise `AdapterError`.

## What to keep from the 115 code (the real value)

Per D7, all of it — config-driven from `roh_hand.yaml`:

- Contact-current safe-stop with trigger-release latch (250 mA, 50 ms poll,
  deadzone hysteresis)
- Send-delta threshold (~2% of range) to avoid CAN spam
- Gesture mode cycle (five → two → point) and open/clear latch button
  semantics (trigger idx 0, joystick click idx 3, right-B open idx 5,
  right-A clear idx 4 — subject to D15 verification)
- Firmware stall-protection setup (200 mA / 300 ms / retry 500 ms / speed 16)
  and the `_can_lock` serialization of all CAN calls
- `roh_hand.yaml` parameter set — kept as the device config, referenced from
  `robot.yaml` like other device configs; per-finger radian calibration
  ranges added here (D4)
- `tools/profile_hand_telemetry_rate.py` (after fixing stale
  `airspeedbox`/ROS Humble paths)
- The `gripper_enabled` flags in `config_openarms_follower.py` /
  `openarms_follower.py` — ported to canonical to implement D8

## What to drop

- `ros_receive` / `system` / `steady` time domains and the fallback in
  `common.py` — the core stays strict
- The three `f32` adapter bindings and `robot_array` / `sensor_array`
  profiles in `registry.py` / `adapter_profiles.py`
- The duplicated `vr_subscriber.py`, `arm_state_publisher.py`, and the
  `config/vr.yaml` copy
- The `/roh/<side>/forces` stream (D5)
- ROH-specific PYTHONPATH exports in `launch/start.sh` — replaced by the
  vendored SDK (D12)

## Execution order

1. **Verify Joy axes mapping** (D15) — read `vr_bridge_server.py`, confirm
   channel order, document the hand button mapping.
2. **Vendor the SDK** (D12) — copy OHand SDK + `RohGestureController` into
   the ROH submodule's `third_party/`; remove sys.path walking.
3. **Driver node** (D3, D4, D7) — ROH submodule under the openarm adaptor:
   standalone node per hand (or one node managing both), counts↔radians at
   the boundary, full safety logic, `JointState` in/out at 30 Hz.
4. **Gripper disable flags** (D8) — port `gripper_enabled` into canonical
   `config_openarms_follower.py` / `arm_controller.py`; hand sessions run
   arms as 7-joint.
5. **VR mapping** (D2) — standalone VR→hand controller consuming Joy,
   publishing `/roh/<side>/joint_command`.
6. **Session config** — canonical `session_vr_ik_robot_button_control.yaml`
   gains a hand-session variant: hand streams (`ros_header`, 6 columns each)
   + 7-column arm streams. Gripper and hand setups are two session variants
   of one codebase (D1).
7. **Fail-fast wiring** (D10) — hand stream timeout/CAN error propagates to
   episode failure exactly like arm stream failures.
8. **Tests** — rewrite `test_roh_f32_streams.py` to pin the
   JointState/`ros_header` contract; update the stream-resolution test for
   the new stream set; single-hand config case (D9).
9. **Deploy to 115** (D13) — ✅ done 2026-08-10: branch
   `feature/roh-hand-canonical-integration` created locally;
   `115:/home/intern/airspeed-canonical` bootstrapped from a full-history
   bundle (`main` + feature branch only — local `legacy-v1.2` history is
   incomplete and breaks whole-repo bundles). Thereafter sync via git bundle
   (local → 115, ff-only); old folder untouched (D14).
10. **Hardware validation** — run on 115 with hands; only then discuss
    retiring the old stack (user confirms timing). During development the
    hardware may be powered but **no control commands are sent** — only
    read-only/non-invasive checks (CAN presence, SDK import, telemetry
    reads), since the robot is unsecured and VR is not connected.
11. **Docs** — `CONTEXT.md` + README updates: hand device, CAN layout (arms
    can0/can1, hands can2/can3, single-owner rule), radian calibration, the
    115 sync procedure (bundle doc generalized from 226).

## Operational notes for 115 (D13/D14)

- Two stacks coexist on 115: old `/home/intern/airspeed` and new
  `/home/intern/airspeed-canonical`. **Never run both simultaneously** —
  CAN channels can2/can3 accept a single owner (second owner gets
  `HAND_RESP_UNMATCHED_CMD` err=6). Enforcement is by operator convention
  until the old stack is retired (user will confirm when).
- 115 clone is on branch `feature/roh-hand-canonical-integration` tracking
  `origin/feature/roh-hand-canonical-integration`; its `origin` URL is the
  stable bundle path `/tmp/airspeed-canonical-sync.bundle`.
- Sync procedure (run from local repo root):

  ```bash
  BASE=$(ssh airspeed115 'git -C /home/intern/airspeed-canonical rev-parse HEAD')
  git bundle create /tmp/airspeed-canonical-sync.bundle \
    "$BASE"..feature/roh-hand-canonical-integration
  git bundle verify /tmp/airspeed-canonical-sync.bundle
  scp /tmp/airspeed-canonical-sync.bundle airspeed115:/tmp/
  ssh airspeed115 'cd /home/intern/airspeed-canonical && \
    git fetch origin && \
    git merge --ff-only origin/feature/roh-hand-canonical-integration && \
    git log --oneline -3'
  ```

  Code flows local → 115 only; no tracked-source edits on 115; always
  `--ff-only` — if the merge can't fast-forward, stop and inspect.

## Unrelated 115 changes (handle separately)

- Worth porting back: UI hardening in `manual_operator_ui.py` /
  `run_global_config.sh` (wildcard→localhost URL, `Cache-Control: no-store`,
  dual-stack bind, browser auto-open); pinned RealSense serials in
  `camera.yaml`; `eclipse-zenoh` dependency.
- Superseded by local tuning: 7-joint gains (kp 60×4/6/8/6, kd ~9), 30 Hz
  control rate, 1.5 s stream timeout, 7-column arm streams.
- ✅ Adopted 2026-08-11: the 115 home pose (`robot_shared.yaml`: j4=90°,
  j5=∓90° mirrored, j7=0) replaced the old j4=50/j5=0/j7=∓50 pose on the
  feature branch — it is the verified-safe home for the current mount.

---

## Implementation results (2026-08-10, branch `feature/roh-hand-canonical-integration`)

All 9 pipeline steps PASS (traces in `pipeline-roh-hand/logs/`):

1. **Joy mapping verified from source** — canonical bridge builds `Joy.axes`
   identically to the old `Float32MultiArray.data`; indices 0=trigger,
   3=thumbstick click, 4=A, 5=B confirmed 1:1. Artifact:
   `pipeline-roh-hand/artifacts/joy_axes_mapping.md`.
2. **SDK vendored** — `roh-hand-ros2-adaptor/third_party/` (OHand SDK +
   gesture wrapper), imports without PYTHONPATH tricks.
3. **Driver node** — `roh_hand_driver.py` (ROS-free, dry-run testable) +
   `roh_hand_node.py`: JointState radians at 30 Hz, currents in `effort`,
   full safety (contact latch, stall protection, send-delta), SystemExit(2)
   after 2 s of read failures.
4. **gripper_enabled flags** — ported to lerobot config/follower,
   `arm_controller.py`, `joint_state_publisher.py`, `robot.yaml`.
5. **VR→hand controller** — `roh_gesture_fsm.py` (pure state machine,
   20 Hz) + `roh_vr_controller_node.py` publishing
   `/roh/<side>/joint_command`.
6. **Hand session YAML** — `session_vr_ik_roh_hand_button_control.yaml`
   resolves 18 streams through the real loader: 4 hand streams (6 cols,
   JointState, ros_header), arms/IK at 7 cols, no Float32MultiArray. IK
   adaptor gained its own `gripper_enabled` flag.
7. **Fail-fast** — silent hand streams classify ABSENT/STALE in the existing
   tracker (same strictness as arms); driver node exits non-zero on
   persistent CAN read failure.
8. **Tests** — `test_roh_hand_streams.py` pins the inverted contract
   (header-less input raises AdapterError; `ros_receive` rejected by the
   loader); 9 ROH unit tests; suite shows no new failures (15 pre-existing
   env-limited failures identical on unmodified main).
9. **Read-only hardware verification on 115** — both hands answered position
   and current reads over CAN (positions ≈ open, currents < 250 mA). No
   control commands were sent.

### Hardware findings (new, not in the original plan)

- **The OHand SDK cannot open `can0`** (`CAN_Init` rejects port 0 — valid
  ports are 1..16). USB enumeration is unstable: after a replug the ROH
  dongles landed on can0/can1, which simultaneously breaks the arms and
  makes one hand unreachable by the SDK.
- **Fix: `launch/setup_can.sh`** pins interface names by USB ID (arms
  0c72:0011 → can0/can1, ROH dongles 0c72:000c → can2/can3) and brings the
  buses up. Run with sudo after any replug/reboot.
- The two ROH dongles are identical (no unique serial exposed): left/right
  assignment follows enumeration order — verify per-hand before a session.

### Not yet done (needs operator-present hardware session)

- Control-path validation on hardware (VR trigger → hand motion), including
  contact-latch behavior under real load.
- Per-finger `closed_rad` calibration (placeholders at 1.6 rad).
- left/right dongle identity verification (which physical hand is can2).
- Gripper-session regression test on hardware (flags default true; code
  paths unchanged but unverified since the port).
