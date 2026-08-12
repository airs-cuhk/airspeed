# Wrist-Twist IK Branch Flip — Pipeline Audit (2026-08-12)

Symptom (operator, hardware): twisting the VR controller wrist makes the
follower arm suddenly jump to a different joint configuration, while the IK
visualization in the web frontend looks continuous and correct.

Scope: arm joint control only (ROH finger control excluded). Read-only
audit — no code changes.

## Pipeline audited

VR bridge → `vr_normalizer.py` → `ik_service._solve_sync` (Pyroki/JAX
optimizer) → `solver_loop.py` → WS `/ws/arm` →
`openarm-control-ros2-adaptor/arm_controller.py` → Damiao MIT commands.

## Alignment check — no order/unit mismatch

- Joint order is consistent end-to-end. Solver uses URDF actuated order
  `[L1..7, R1..7, L_finger1, R_finger1]`; `ik_service.py:89-115` reorders
  explicitly to/from config order `[L1..7, L_grip, R1..7, R_grip]`.
- WS path sends `joint_radians_<side>[:7]` (`solver_loop.py:186-192`);
  control adaptor maps index `i` → `bus.motors[i]` = `joint_1..joint_7`
  (`arm_controller.py` `_apply_joints`). Units are radians throughout the
  arm path. Gross tracking works because this layer is correct.

## Root cause: IK branch flip at the wrist limits

1. **Tight wrist limits** in the solver URDF
   (`openarm-ik-ros2-adaptor/webui-monitor/3d_assets/urdf/openarm_bimanual_copy.urdf`):
   J5 ±90° (±1.5708 rad), J6 ±45° (±0.7854 rad), J7 ±90°. A wrist twist
   beyond what these express forces the optimizer to re-achieve the same
   end-effector pose through a different joint branch (wrist flip: J4/J6
   swing to compensate). Task space continuous, joint space discontinuous.

2. **Frontend cannot show it.** The webui renders the target pose and the
   solved URDF — both keep the end effector on target across the flip.
   "Frontend normal" is compatible with a joint-space flip; only the raw
   joint telemetry (or `ik_service.audit_log`) reveals it.

3. **No joint-space continuity enforcement on the output path.**
   `_broadcast_arm` (`solver_loop.py:213`) sends raw solver output at 50 Hz.
   Continuity relies only on previous-solution warm start
   (`use_home_warm_start: false`, `ik_service.py:148-151`) with rest weight
   15 vs pose weights 80/25 (`config/solver_smooth.yaml`) — not enough to
   hold a branch once the target leaves the reachable wrist envelope. The
   follower's `max_joint_delta_deg: 1.8` slew clamp (≈90°/s at 50 Hz) does
   not prevent the flip; it slews to the new branch, which reads as the
   "sudden change to another control sequence".

4. **Aggravator: home parks J5 exactly ON the limit.**
   `robot_shared.yaml` home is `j5 = ∓90°`; URDF joint5 limit is ±1.570796
   rad. Every session starts at the wrist boundary, so any wrist twist from
   home immediately pushes the solver across the branch edge.

## Ranked causes

1. Wrist branch flip at J5/J7 ±90°, J6 ±45° limits — triggered at once
   because home parks J5 on the limit.
2. (Contributing) No branch-continuity check / joint-space smoothing in the
   IK output path.

## How to confirm on hardware

During a wrist twist, watch solved joint angles (frontend telemetry or the
`AuditLogger` entries): J4/J5/J6 swing tens of degrees in one solve while
`position_error_mm` / `orientation_error_deg` stay small. Small task-space
error + large joint delta = branch flip, not a follower bug.

## Candidate fixes (not applied)

1. Move home J5 off the limit (e.g. ∓60–70°) in `robot_shared.yaml`.
2. Post-solve branch-continuity enforcement in `ik_service._solve_sync`:
   reject solutions whose joint delta from the previous output exceeds a
   threshold, re-solve with a stronger rest cost (or keep previous branch).
3. Widen wrist limits in the URDF only if the real motors mechanically
   allow more travel than ±90°/±45° (verify against motor datasheets first).

## Related records

- Arm startup trembling (same wrist, different mechanism — gains/payload/
  50 Hz RNE dither): `docs/arm-startup-trembling-analysis-20260811.md`.
- Home pose history (j4=50 → 90 → 70 → 110; j5=∓90 since 2026-08-11):
  `src/robot_interface/openarm/robot_shared.yaml` header comment.
