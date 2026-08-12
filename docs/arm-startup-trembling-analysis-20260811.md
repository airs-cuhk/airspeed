# Arm Startup Trembling — Analysis (2026-08-11)

Symptom: during `openarm-control-ros2-adaptor/launch/start.sh`, the arm motors
tremble visibly in the `[2/3] Moving to home position...` phase (gravity
compensation ON), then calm down once the WebSocket control loop starts.

Context: same day the arms were unreachable (`No response from motor
'joint_N'` on both can0/can1). Root cause of *that* was commit `13a9b8d`
(StarChen-Cycler): the Damiao arm buses speak **CAN FD** (1M nominal / 5M
data) — a plain-CAN link receives zero frames (0 RX with healthy TX), which
had been misread as a power/cable fault. `setup_can.sh` now brings can0/can1
up with `fd on dbitrate 5000000`.

## Root causes of the tremble (stacked, wrist-dominant)

1. **Wrist gains are 2–5× the old stack's.**
   `openarm-control-ros2-adaptor/config/robot.yaml`: J5/J7 kp 12, J6 kp 40
   (old airspeed: J5/J7 kp 6, J6 kp 8). The commit trail (`8a85719`,
   `6dec422`, `1371806`) shows this was tuned for the stock arm ("J6 pushed
   to 40 for loaded-wrist tracking"). The arms now carry **ROH dexterous
   hands**, heavier than the stock grippers.

2. **The gravity model doesn't know about the ROH payload.**
   `_move_to_home` (`arm_controller.py:68`) feeds forward torques from
   `urdf/openarm_bimanual_pybullet.urdf`, which models the stock gripper.
   Wrist feed-forward is therefore under-compensated; the PD loop fights the
   residual load with kp 40 / kd 2.7 at J6 — underdamped for the real
   payload, so it rings.

3. **Gravity is recomputed every cycle at 50 Hz.**
   `arm_controller.py:88-89` runs full pinocchio RNE on fresh encoder reads
   each `_move_to_home` iteration (`fps = arm_state_hz = 50`; old stack ran
   30). Position noise becomes torque dither at 50 Hz into a stiff wrist.
   The main control loop deliberately does the opposite — a 5 Hz background
   thread caches gravity ("wasted work on a slowly-varying quantity",
   `arm_controller.py:425-428`). The startup path never got that treatment.

Net: underdamped stiff wrist + wrong payload model + 50 Hz feed-forward
noise = tremble during the home move. The main loop (cached 5 Hz gravity,
slew-clamped commands) is why it settles after startup.

## Candidate fixes (cheapest first)

1. Cache gravity in `_move_to_home` at ~5 Hz (or reuse the main loop's
   background-thread pattern) — removes factor 3, few lines.
2. If still trembling: soften J5–J7 kp or re-tune kd for the ROH payload.
3. Proper fix: update the URDF wrist payload (ROH hand mass/inertia) so the
   feed-forward is actually correct.

## Related records

- CAN FD fix: commit `13a9b8d` (setup_can.sh arm channels `fd on`).
- Gain-tuning history: `git log --oneline -- .../config/robot.yaml`.
- Same-day ROH hand session (CAN silence, DDS cross-user, latch release):
  commits `13a1c28`, `a40b660`; ROH troubleshooting checklist in
  `ROH/wrappers/README.md` §Troubleshooting and
  `ROH/roh_lites_firmware-main/FAQs/FAQs_EN.md` Q2.
