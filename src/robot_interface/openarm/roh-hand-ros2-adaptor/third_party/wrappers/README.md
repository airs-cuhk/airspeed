# ROH-LiteS001 Gesture Wrappers

Terminal-command gesture presets for the OYMotion **ROH-LiteS001 (CAN version)**
dexterous hand. These wrappers sit on top of the manufacturer OHand Python SDK
**without modifying it**, and talk to the hand over the configured CAN interface
(default `can2`, the verified ROH PEAK PCAN-USB adapter, USB id `0c72:000c`).

> They never touch `can0` / `can1` (OpenArm / PCAN-USB Pro FD adapters).

## Files

| File | Purpose |
|---|---|
| `roh_gesture_controller.py` | Core controller: gestures, telemetry, resistance-aware guarded moves, firmware stall protection, CAN channel scan. |
| `gesture_cli.py` | Terminal CLI: `list`, `scan`, `run`, `run-guarded`, `stop`, `telemetry`, `configure-stall`, `grasp`. |
| `roh_read_force.py` | Read-only current / force telemetry for threshold tuning. |
| `roh_current_probe.py` | Read-only left/right current probe for comparing `can3` and `can2` without moving either hand. |
| `roh_dual_read_diagnostic.py` | Read-only dual-hand diagnostic: samples `can3` and `can2` in separate threads and records SDK/CAN failure details. |
| `roh_finger_current_sweep.py` | Active single-finger current sweep for comparing resistance response between two physical hands. |
| `roh_config.py` | Legacy compatibility config (kept for external scripts that import it). |
| `README.md` | This file. |

No manufacturer SDK file was modified. SocketCAN is obtained by importing the
SDK's existing `ohand.interface.can.socet_can_interface` module directly
(the SDK's default package imports the pcan backend, which we bypass).

## Prerequisites

1. Verify the ROH adapter is present:
   ```bash
   lsusb | grep PEAK
   # expect: ID 0c72:000c PEAK System PCAN-USB
   ```
2. Bring up the configured CAN interface (`can2` by default) at 1 Mbps:
   ```bash
   sudo ip link set can2 down
   sudo ip link set can2 type can bitrate 1000000 restart-ms 100
   sudo ip link set can2 up
   ```
   If your adapter appears on a different interface, pass `--can-channel canX` or
   edit `DEFAULT_CAN_CHANNEL` in `roh_gesture_controller.py`.
3. Activate the isolated ROH venv (scrubs ROS2 PYTHONPATH leakage):
   ```bash
   source ~/airspeed/ROH/activate_roh_env.sh
   cd ~/airspeed/ROH/wrappers
   ```

## Gestures

Canonical gesture names:

| Name | Description |
|---|---|
| `open` / `spread` | All fingers fully open. |
| `rest` | Relaxed, slightly curled posture. |
| `five_finger_ready_grasp` | Safe first pose: fingers partly curled, thumb to the side. |
| `five_finger_grasp` / `fist` | Whole-hand soft grasp / fist, thumb to the side. |
| `two_finger_ready_grasp` | Pre-pinch pose: thumb root opposed toward index, slight thumb flex, index open. |
| `two_finger_grasp` / `pinch` | Precision thumb–index pinch: index flexes most, thumb flexes moderately, minimal extra thumb-root rotation. |
| `extend_index` / `point` | Index extended, other fingers flexed. |
| `victory` | Index and middle extended, ring/pinky flexed. |
| `middle_finger_up` | Middle extended, other fingers flexed/tucked. |

Motor map: `0=thumb flex, 1=index, 2=middle, 3=ring, 4=pinky, 5=thumb root`.
Position `0`=open, `65535`=closed; speed `0-255`.

## Usage

```bash
# List CAN interfaces and identify the ROH adapter
python gesture_cli.py scan

# List gesture presets
python gesture_cli.py list

# Run a preset (real motion)
python gesture_cli.py run five_finger_ready_grasp --yes
python gesture_cli.py run five_finger_grasp --yes

# Whole-hand grasp stops on normal object contact by default.
# Lower --contact-current-ma if you still hear stalled-motor current noise;
# raise it if the hand stops before holding the object firmly.
python gesture_cli.py run five_finger_grasp --yes --contact-current-ma 180

# Two-finger pinch runs slowly and stops each finger independently on force.
python gesture_cli.py run pinch --yes
python gesture_cli.py run two_finger_grasp --yes

# Run with resistance stop: slow at --slow-current-ma, stop if --current-limit-ma
# stays above threshold for --hold-ms.
python gesture_cli.py run-guarded five_finger_grasp --yes \
  --slow-current-ma 320 \
  --current-limit-ma 450 \
  --hold-ms 120 \
  --slow-speed 30

# Read telemetry (safe, hand does not move)
python gesture_cli.py telemetry
python gesture_cli.py telemetry --watch

# Read telemetry with the dedicated read-only tool
python roh_read_force.py
python roh_read_force.py --watch
python roh_read_force.py --json

# Compare left can3 and right can2 currents without commanding motion
python roh_current_probe.py \
  --left-channel can3 \
  --right-channel can2 \
  --seconds 90 \
  --rate-hz 2 \
  --csv /tmp/roh_current_probe.csv

# Active resistance comparison after adapter swap:
# can3 = good right hand, can2 = suspect left hand.
python roh_finger_current_sweep.py \
  --left-channel can3 \
  --right-channel can2 \
  --left-label good_right_hand_on_can3 \
  --right-label bad_left_hand_on_can2 \
  --mode sequential \
  --finger middle \
  --open-first \
  --speed 20 \
  --step-counts 1000 \
  --current-stop-ma 150 \
  --jsonl /tmp/roh_middle_sweep.jsonl \
  --csv /tmp/roh_middle_sweep.csv \
  --yes

# Stop all motors
python gesture_cli.py stop

# Configure firmware stall protection
python gesture_cli.py configure-stall --yes \
  --stall-current-ma 200 \
  --stall-after-ms 300 \
  --stall-retry-ms 500

# Interactive arrow-key grasp
python gesture_cli.py grasp --yes

# Interactive two-finger pinch
python gesture_cli.py pinch --yes
```

### Interactive grasp (`grasp`)

- `UP` / `DOWN` : close / open the hand in increments.
- `HOME`        : fully open.
- `END`         : fully close.
- `q`           : quit and stop all motors.

The hand starts fully open with the thumb root already opposed. Each keypress
sends a new position command immediately.

#### Per-finger force stop

By default `grasp` monitors per-finger force and stops individual fingers when
they hit the threshold, even if you keep pressing `UP`:

```bash
python gesture_cli.py grasp --yes --force-limit-mn 500
```

- `--force-limit-mn 500` (default): any finger whose force reaches 500 mN is
  held at its current position; other fingers continue closing.
- `--force-limit-mn 0`: disables force monitoring.

A stopped finger is released again as soon as you open the hand (`DOWN` / `HOME`).
The terminal shows live force readings and which fingers are currently stopped.

If your hand does not return valid force values, `--force-limit-mn` has no
effect; use current-based guarded moves (`run-guarded`) instead.

### Interactive two-finger pinch (`pinch`)

```bash
python gesture_cli.py pinch --yes
```

- `UP` / `DOWN` : pinch closer / open pinch.
- `HOME`        : fully open.
- `END`         : fully close.
- `q`           : quit and stop motors.

Only the thumb and index move; the other fingers stay tucked. The thumb root
starts already opposed toward the index finger in a dedicated pre-pinch pose
(`two_finger_ready_grasp`). During closure the index finger flexes the most,
the thumb flexes moderately, and the thumb root rotates only slightly more,
bringing the thumb and index pads together for a precision pinch.

By default `pinch` monitors thumb and index force and stops each independently
when it hits the threshold:

```bash
python gesture_cli.py pinch --yes --force-limit-mn 500
```

- `--force-limit-mn 500` (default): thumb or index reaching 500 mN is held at
  its current position; the other finger continues.
- `--force-limit-mn 0`: disables force monitoring.

A stopped finger is released again when you open the pinch (`DOWN` / `HOME`).

## Force and current telemetry

Two tools read telemetry without moving the hand:

- `python gesture_cli.py telemetry [--watch]` — prints a JSON summary with
  target positions, per-finger current (`mA`), and per-finger maximum force
  (`mN`).
- `python roh_read_force.py [--watch]` — human-readable per-finger output,
  including the raw force value array for each finger.
- `python roh_current_probe.py --left-channel can3 --right-channel can2` —
  read-only JSON-lines/CSV comparison of both hands, including per-finger current
  read errors so `None` can be separated from a real low current reading.
- `python roh_dual_read_diagnostic.py --left-channel can3 --right-channel can2` —
  deeper read-only diagnosis for SDK/CAN failures such as timeout, bus-off, and
  remote errors.
- `python roh_finger_current_sweep.py --finger middle ...` — active comparison:
  moves one selected finger slowly and logs current/position while stopping at a
  configurable current threshold.

Typical workflow for tuning resistance stop:

1. Run the hand into a light object and watch the current/force values:
   ```bash
   python gesture_cli.py telemetry --watch
   ```
2. Note the maximum current/force just before the hand stalls.
3. Set `--current-limit-ma` a little above that value, and `--slow-current-ma`
   below it, then use `run-guarded`.

> The wrapper uses motor current as the primary blocking signal because it is
> available through `HAND_GetFingerCurrent` on all tested firmware. Optional
> force feedback can be enabled with `--force-limit-mn` if the hand returns
> valid force values. Force is read with `HAND_SendCmd`/`HAND_GetResponse` so no
> SDK file is modified.

## How resistance stop works

A guarded move does:

1. Send the requested target position at normal speed.
2. Poll current and optional force telemetry.
3. If a soft threshold is crossed, resend the same target with a lower speed.
4. If a hard threshold stays above limit for the hold time, call
   `HAND_FingerStop` and raise `ResistanceDetected`.

For cases where a motor current reading is missing or too low to be trusted, add
`--position-stall-stop` to `run` or `run-guarded`. This also stops if a closing
finger remains far from its target while the measured position stops changing:

```bash
python gesture_cli.py --can-channel can3 run-guarded five_finger_grasp --yes \
  --position-stall-stop \
  --current-limit-ma 0 \
  --force-limit-mn 0 \
  --settle-ms 3000
```

## Shutdown / safe-stop

In order of escalation:

1. **Stop motors (software):**
   ```bash
   python gesture_cli.py stop
   ```
   Sends `HAND_FingerStop` for all six motors.

2. **Ctrl-C during a run:** the controller's context manager closes the bus.

3. **Open the hand to a relaxed posture** before powering down:
   ```bash
   python gesture_cli.py run open --yes
   ```

4. **Hard stop — disconnect 24V power.** If the hand behaves abnormally
   (continuous buzzer, stalled motor noise, overheating, smoke), **cut the 24V
   supply immediately.** This is the ultimate safe-stop; there is no software
   substitute for removing power.

> Note: the default `--hand-id` is `0x02` (SDK default). If your unit reports a
> different node id, pass `--hand-id 0xNN`.

## Troubleshooting

If you see `HAND_SetFingerPosAll failed: err=3` (timeout):

1. Check the CAN interface is UP and ERROR-ACTIVE:
   ```bash
   ip -details link show can2
   ```
2. Listen for replies while sending a stop command:
   ```bash
   candump can2   # in one terminal
   python gesture_cli.py stop --yes   # in another
   ```
3. Verify the hand is powered and the hand node ID is correct.
4. Power-cycle the 24V supply if the hand appears stuck.
