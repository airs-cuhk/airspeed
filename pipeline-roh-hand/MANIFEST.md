# MANIFEST — ROH Hand Canonical Integration Pipeline

**Purpose.** Integrate the ROH dexterous hand (OYMotion OHand / ROH-LiteS001,
CAN can2/can3) into the airspeed canonical data-collection stack so that the
hand speaks the same ROS2 contract as every other device — stamped
`sensor_msgs/JointState` in radians, `ros_header` time domain only — with zero
changes to the data-collection core, and with the partner's safety logic
preserved in full.

**Goal state (done means all of these, each with evidence in `logs/`).**

1. OHand SDK + gesture wrapper vendored under the ROH module (`third_party/`),
   importable without PYTHONPATH tricks.
2. ROH driver node: standalone process owning can2/can3, publishes
   `/roh/<side>/joint_state` (JointState, radians, 30 Hz, currents in
   `effort` as mA), subscribes `/roh/<side>/joint_command`, full safety
   (contact-stop latch, stall protection, send-delta threshold), config-driven
   from `roh_hand.yaml`.
3. VR→hand controller consumes `sensor_msgs/Joy` (axes mapping verified from
   `vr_bridge_server.py` source) and publishes joint commands.
4. `gripper_enabled` flags disable arm gripper motors in hand sessions.
5. Hand-session YAML variant resolves with `ros_header` streams only.
6. Hand stream/CAN failure fails the episode fast, like arm failures.
7. Rewritten tests pin the JointState/`ros_header` contract; pytest green.
8. Read-only (non-control) verification executed on airspeed115 hardware.

**Invariants (never violated).**

- No control commands are ever sent to hardware from this pipeline. The robot
  is unsecured; verification on 115 is read-only / non-invasive only.
- Code flows one direction: local branch `feature/roh-hand-canonical-integration`
  → git bundle → `115:/home/intern/airspeed-canonical`. No tracked-source
  edits on 115. Never run the old 115 stack and the canonical stack
  simultaneously (CAN single-owner rule).
- Topic units are radians at the driver boundary; SDK units (raw counts
  0..65535, mA, mN) never leak onto topics.
- Header-less messages are rejected (`ros_header` only); no receipt-time
  fallbacks are added to the data-collection core.
- One git commit per feature stage; every pipeline step leaves
  `logs/<step>/{step.log,timing.json,STATUS.txt}`.

**Map.**

- Plan & locked decisions D1–D15: `docs/roh-hand-canonical-integration-plan.md`
- Steps: `pipeline-roh-hand/NN_*.py` (run in order; never run N before N−1 PASSes)
- Shared framework: `util_gate.py` (gates/logs/timing/STATUS), `util_paths.py`
- Step artifacts: `pipeline-roh-hand/artifacts/`
- Implementation targets: `src/robot_interface/openarm/` (ROH submodule,
  gripper flags), `src/data_collection_service/` (session YAML, tests)
- Hardware checks run over SSH: host `airspeed115`, repo
  `/home/intern/airspeed-canonical`
