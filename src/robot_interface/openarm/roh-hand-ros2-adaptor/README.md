# ROH Hand ROS2 Adaptor (canonical)

Standalone ROS2 driver for the OYMotion ROH-LiteS001 dexterous hand, following
the airspeed canonical messaging contract:

- **`/roh/<side>/joint_state`** (pub) — stamped `sensor_msgs/JointState` at
  30 Hz; `position` = radians (0 = open), `effort` = finger currents in mA
  (NaN = per-finger read failure). Joints: `roh_<side>_<finger>` with fingers
  `thumb, index, middle, ring, pinky, thumb_root`.
- **`/roh/<side>/joint_command`** (sub) — `sensor_msgs/JointState`, `position`
  in radians. Commands pass through the driver's safety gates (send-delta
  threshold, contact latch) before reaching the hand.

SDK raw counts (0..65535) are converted at the driver boundary and never
appear on topics. Header-less messages are not used anywhere; the
data-collection service consumes these streams with `ros_header` like any
other device — no core changes.

## Layout

- `roh_hand_node.py` — rclpy glue (one process, all enabled hands)
- `roh_hand_driver.py` — hardware driver + safety (ROS-free, unit-testable)
- `roh_config.py` — `roh_hand.yaml` loader
- `roh_units.py` — counts ↔ radians conversion (+ sanity validation)
- `config/roh_hand.yaml` — per-hand config (CAN, topics, calibration, safety)
- `third_party/` — vendored OHand SDK + `RohGestureController` wrapper

## Safety (ported in full from the 115 stack, D7)

- Firmware stall protection configured at connect (200 mA / 300 ms / retry
  500 ms / speed 16).
- Send-delta threshold: changes < 2% of full scale are not resent (CAN spam
  guard).
- Contact-current stop: while closing or holding, currents are polled every
  50 ms; a finger ≥ 250 mA **whose position is not advancing** (blocked —
  the contact/stall signature) triggers `safe_stop` and latches the hand.
  Over-threshold current while the finger still advances is free-space
  motion current and does not trip (long sweeps like the thumb_root ready
  pose draw ~540 mA). Further closing commands are rejected until the
  operator commands a retreat below the recorded stop position (hysteresis)
  or fully open.
- Fail-fast: telemetry read failures for > 2 s exit the node non-zero (D10).

## CAN bus setup (required before starting)

**Use the pinning script** — USB enumeration is unstable and the OHand SDK
cannot open `can0` (port 0 invalid, SDK accepts can1..can16 only). The script
renames interfaces by USB ID so the mapping always lands on the canonical
channels:

```bash
sudo src/robot_interface/openarm/roh-hand-ros2-adaptor/launch/setup_can.sh
# arms (PEAK PCAN-USB Pro FD, 0c72:0011) -> can0/can1
# hands (PEAK PCAN-USB,      0c72:000c) -> can2/can3, 1 Mbps, restart-ms 100
```

Hands: left = can2, right = can3. The two ROH dongles are identical — the
left/right assignment follows enumeration order; verify per-hand before a
session. **Single-owner rule:** only one process may own a CAN channel —
never run this adaptor alongside the old 115 ROH stack.

## Run

```bash
# One command for the whole ROH hand stack (driver + VR→hand controller):
src/robot_interface/openarm/roh-hand-ros2-adaptor/launch/start_roh_stack.sh [config]
```

Do NOT run it with sudo: SocketCAN needs no root, and root-owned ROS2
nodes cannot exchange DDS data with the user-space VR bridge/collector
(FastDDS shared-memory transport is per-user) — commands and states
silently stop flowing. Only `setup_can.sh` needs sudo.

The pieces can also be started individually (same no-sudo rule):

```bash
src/robot_interface/openarm/roh-hand-ros2-adaptor/launch/start.sh [config]              # driver only
src/robot_interface/openarm/roh-hand-ros2-adaptor/launch/start_vr_controller.sh [config] # VR→hand only
```

Set `dry_run: true` in `config/roh_hand.yaml` for hardware-free testing.

## Calibration note

`calibration.closed_rad` (radians at full close, per finger) is currently a
placeholder (1.6 rad ≈ 92°) — measure on hardware before relying on absolute
radian values (plan doc D4). The conversion is linear: `rad = counts/65535 ×
closed_rad`.
