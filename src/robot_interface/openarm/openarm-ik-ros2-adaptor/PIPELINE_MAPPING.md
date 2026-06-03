# OpenArm IK ROS2 Adaptor Pipeline Mapping — Debug Reference

Date: 2026-06-03

Full pipeline: VR ROS2 topics → thread-safe store → VR normalization (rebase + axis map) → JAX IK solve → gripper override → ROS2 JointState + PoseStamped publish + WebSocket arm broadcast + monitoring UI snapshot

---

## HOP 0: ROS2 Subscription → Thread-Safe Data Store

**dimension space**: ROS2 DDS messages → `VRDataStore` (thread-safe Python dataclasses)

```
vr_subscriber.py:207-269 (_run_ros2 daemon thread)
```
```python
    def _run_ros2(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Float32MultiArray, String
        except ImportError as exc:
            logger.error(f"ROS2 import failed in subscriber thread: {exc}")
            self._ros2_available = False
            return

        if not rclpy.ok():
            rclpy.init()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config.ros2.qos.history_depth,
        )

        topics = self.config.ros2.topics
        node_name = self.config.ros2.node_name
        node = Node(node_name)

        def _parse_pose(msg: PoseStamped, side: str) -> None:
            p = msg.pose.position
            o = msg.pose.orientation
            self.data_store.update_pose(
                side=side,
                position=[p.x, p.y, p.z],
                orientation_wxyz=[o.w, o.x, o.y, o.z],
            )

        def _parse_buttons(msg: Float32MultiArray, side: str) -> None:
            self.data_store.update_buttons(side=side, buttons=list(msg.data))

        node.create_subscription(PoseStamped, topics.left_pose, lambda m: _parse_pose(m, "left"), qos_profile)
        node.create_subscription(PoseStamped, topics.right_pose, lambda m: _parse_pose(m, "right"), qos_profile)
        node.create_subscription(PoseStamped, topics.head_pose, lambda m: _parse_pose(m, "head"), qos_profile)
        node.create_subscription(Float32MultiArray, topics.left_buttons, lambda m: _parse_buttons(m, "left"), qos_profile)
        node.create_subscription(Float32MultiArray, topics.right_buttons, lambda m: _parse_buttons(m, "right"), qos_profile)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except Exception as exc:
            logger.error(f"ROS2 executor error: {exc}")
        finally:
            node.destroy_node()
```

**latex**: N/A (structural — ROS2 subscription + thread-safe storage with `threading.Lock`)

**explanation**: A daemon thread subscribes to 5 VR bridge topics (`/vr/head_pose`, `/vr/left_pose`, `/vr/right_pose`, `/vr/left_buttons`, `/vr/right_buttons`) plus raw JSON. Each subscription callback strips the ROS2 wrapper: `PoseStamped` → `[x, y, z]` position + `[w, x, y, z]` quaternion, `Float32MultiArray` → `list[float]` buttons. Data is stored in `VRDataStore` under a `threading.Lock` — last write wins. The `stale_timeout_s` (default 1.0s) gates read access: stale poses return `None`, forcing the solver to idle. If `rclpy` is not importable, `start()` returns `False` and the system degrades gracefully. Source of truth: `config/vr.yaml` `ros2.topics` section.

### what it does

Subscribes to VR bridge ROS2 topics in a daemon thread. Parses `PoseStamped` → position list + WXYZ quaternion, `Float32MultiArray` → float button list. Stores in a thread-safe `VRDataStore` with per-field locking. Stale timeout (1.0s) prevents the solver from acting on dead-reckoned poses when the VR device disconnects.

### output example with emphasis

**Input**: `PoseStamped` on `/vr/right_pose` with `position: {x: 0.20, y: 0.83, z: -0.36}`, `orientation: {x: 0.23, y: 0.16, z: 0.35, w: 0.89}`
**Output** (in `VRDataStore.right_pose`):
```
VRPose(
    position=[0.20, 0.83, -0.36],                ← ROS2 xyz → Python list
    orientation_wxyz=[0.89, 0.23, 0.16, 0.35],   ← ROS2 xyzw → WXYZ (scalar-first)
    timestamp_s=95721.451                         ← time.monotonic(), NOT header.stamp
)
```

Key characteristics:
- Orientation convention converted: ROS2 `xyzw` → solver `wxyz` (scalar-first) in the callback itself
- Timestamp is `time.monotonic()` (receive time), NOT `header.stamp` from the VR bridge
- 6 subscriptions share a single `SingleThreadedExecutor` — all callbacks serialized
- QoS: `BEST_EFFORT, VOLATILE, KEEP_LAST` with configurable depth from `config/vr.yaml`

---

## HOP 1: VR Normalization — SE3 Rebase + Axis Mapping

**dimension space**: VR native coordinates (Y-up, right-handed) → robot base frame (Z-up, REP-103)

```
vr_normalizer.py:133-190 (normalize method)
vr_normalizer.py:236-291 (_produce_targets method)
```
```python
    def normalize(self, data_store: VRDataStore) -> NormalizedTarget:
        now = time.monotonic()

        # Check staleness — revert to WAITING if no VR data for > timeout
        if self.state != CalibrationState.WAITING:
            if (now - self._last_active_s) > self.config.calibration.stale_timeout_s:
                self._reset()
                return NormalizedTarget(state=CalibrationState.WAITING, ...)

        head_pose = data_store.get_head_pose()
        button_states = data_store.get_button_states()

        # Extract trigger analog values (index 0) for gripper control
        left_trigger = _trigger_value(button_states.get("left"))
        right_trigger = _trigger_value(button_states.get("right"))

        # Detect button presses (rising edge) on right controller only
        pin_pressed_now = False
        start_pressed_now = False
        for side in ("right",):
            bs = button_states.get(side)
            if _detect_button(bs, self.config.calibration.pin_button):
                pin_pressed_now = True
            if _detect_button(bs, self.config.calibration.start_button):
                start_pressed_now = True

        pin_rising = pin_pressed_now and not self._prev_pin_pressed
        start_rising = start_pressed_now and not self._prev_start_pressed

        # B-button: pin origin + calibrate anchors → READY
        if pin_rising and head_pose is not None:
            self._pin_and_calibrate(head_pose, data_store, self._home_ee_source)

        # A-button: start IK solving → ACTIVE
        if start_rising and self.state == CalibrationState.READY:
            self.state = CalibrationState.ACTIVE

        # Produce targets only if ACTIVE
        if self.state == CalibrationState.ACTIVE:
            result = self._produce_targets(data_store)
            result.left_trigger = left_trigger
            result.right_trigger = right_trigger
            return result

        return NormalizedTarget(state=self.state, ...)
```

```
    def _produce_targets(self, data_store: VRDataStore) -> NormalizedTarget:
        poses = data_store.get_latest_poses()
        left_target = None
        right_target = None

        for side in ("left", "right"):
            if side not in poses or side not in self._calibration_anchor:
                continue

            rebased = self._rebase_pose(poses[side])
            anchor = self._calibration_anchor[side]

            # Compute delta from anchor (identity when hands haven't moved)
            delta = anchor.inverse() @ rebased
            delta_pos = delta.translation()
            delta_rot = delta.rotation()

            # Apply axis mapping: VR native → robot base frame
            mapped_pos, mapped_rot = _apply_axis_mapping(
                delta_pos.tolist(), delta_rot, self.config
            )
            delta_se3 = jaxlie.SE3.from_rotation_and_translation(
                mapped_rot, jnp.array(mapped_pos),
            )

            # Compose with FK home
            home_ee = self._home_ee.get(side)
            if home_ee is not None:
                target = jaxlie.SE3.from_rotation_and_translation(
                    delta_se3.rotation() @ home_ee.rotation(),
                    home_ee.translation() + delta_se3.translation(),
                )
            else:
                target = delta_se3

            if side == "left":
                left_target = target
            else:
                right_target = target

        return NormalizedTarget(left=left_target, right=right_target, state=self.state)
```

**latex**:
`T_{\text{robot}} = T_{\text{FK home}} \cdot (M_{\text{axis}} \cdot (T_{\text{anchor}}^{-1} \cdot T_{\text{origin}}^{-1} \cdot T_{\text{VR}}) \cdot M_{\text{axis}}^{-1})`

Where:
- `T_{\text{origin}}` = pinned VR origin SE3 (from head pose at B-button press, ground-projected, yaw-only)
- `T_{\text{anchor}}` = calibration anchor SE3 (snapshot of controller at B-button press)
- `T_{\text{VR}}` = current controller pose in VR native frame
- `M_{\text{axis}}` = 3×3 axis mapping matrix from `config/vr.yaml` (VR Y-up → robot Z-up)
- `T_{\text{FK home}}` = FK end-effector pose at home configuration

**explanation**: Two-stage transform. Stage 1 — rebase: pin VR origin from head pose (B-button), project to ground plane (Y=0), extract yaw-only rotation. Compute delta from calibration anchor (snapshot of controller at pin time). When hands are still, delta = identity. Stage 2 — axis mapping: apply 3×3 matrix from `config/vr.yaml` (`native_to_intermediate`) to both position (with `position_scale`) and rotation (via SO3 conjugation). Compose with FK home end-effector pose. The state machine (`WAITING → READY → ACTIVE`) gates this — no targets are produced until the operator presses A-button. Rising-edge detection on B/A buttons prevents repeated triggers. Source of truth: `config/vr.yaml` `axis_mapping` and `calibration` sections.

### what it does

Transforms VR controller poses from native coordinates (Y-up, right-handed) to robot base frame SE3 targets. Two-stage: rebase into pinned VR origin (B-button → READY), then apply axis mapping matrix. Rising-edge button detection prevents repeated triggers. Only produces targets in ACTIVE state (A-button press). Trigger values (channel 0, 0.0–1.0) extracted for gripper control.

### output example with emphasis

**Input**: Left controller at position `[0.45, 1.2, -0.30]` in VR frame, anchor set at home
**Output**:
```
NormalizedTarget(
    left=SE3(                                   ← jaxlie.SE3 in robot base frame
        translation=[0.52, -0.15, 0.33],        ← axis-mapped position (meters)
        rotation=SO3(wxyz=[0.98, 0.0, -0.17, 0.0])  ← axis-mapped + FK-composed
    ),
    right=SE3(...),
    state=<ACTIVE>,                              ← CalibrationState enum
    left_trigger=0.35,                           ← analog trigger value [0, 1]
    right_trigger=0.0,
)
```

Key characteristics:
- Delta from anchor = identity when controller hasn't moved — robot stays at FK home
- Rising-edge detection: `pin_rising = pin_pressed_now and not self._prev_pin_pressed`
- Only right-controller buttons checked for calibration (`for side in ("right",)`)
- Stale timeout: reverts to WAITING if no VR data for `stale_timeout_s` seconds
- Quaternion convention: WXYZ (scalar-first) throughout — matches jaxlie

---

## HOP 2: IK Solve — SE3 Targets → Joint Angles

**dimension space**: `jaxlie.SE3` end-effector targets → `list[float]` joint angles (radians), 9 per arm

```
ik_service.py:122-131 (solve_targets async wrapper)
ik_service.py:133-265 (_solve_sync — synchronous solver)
solver.py:127-164 (PyrokiSolver class)
```
```python
    async def solve_targets(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
    ) -> SolveResult:
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None, self._solve_sync, target_L, target_R
        )

    def _solve_sync(self, target_L, target_R) -> SolveResult:
        t0 = time.perf_counter()

        # Warm-start: home (deterministic) or previous output (responsive)
        if self.config.solver.use_home_warm_start:
            q_input = np.array(self._home_rad, dtype=np.float32)
        else:
            q_input = self.q_current

        q_solver_in = self._home_to_solver_order(q_input)
        q_out = self.solver.solve(target_L, target_R, None, q_solver_in)
        solve_time_ms = (time.perf_counter() - t0) * 1000

        # FK verification
        fk = self.robot.forward_kinematics(q_out)

        # Reorder: solver order → config order
        self.q_current = self._solver_to_config_order(q_out)

        # Split for frontend: [L1..7, L_f1, L_f2, R1..7, R_f1, R_f2]
        sol = q_out.tolist()
        joint_left = sol[:7] + [sol[14], 0.0]
        joint_right = sol[7:14] + [sol[15], 0.0]

        return SolveResult(
            success=True,
            joint_radians_left=joint_left,
            joint_radians_right=joint_right,
            position_error_mm=pos_err,
            orientation_error_deg=ori_err,
            solve_time_ms=solve_time_ms,
            ...
        )
```

```
class PyrokiSolver:
    robot: BaseRobot
    _jit_solve: Callable  # JIT-compiled via jax.jit

    def __init__(self, robot: BaseRobot, solver_config: Any = None):
        self.robot = robot
        self._jit_solve = jax.jit(self._solve_internal)
        self._warmup()  # trigger JIT compilation at startup

    def _solve_internal(self, target_L, target_R, target_Head, q_current):
        costs = self.robot.build_costs(target_L, target_R, target_Head, q_current)
        # ... nonlinear least-squares optimization via jaxls ...
```

**latex**: `q^* = \arg\min_q \sum_i w_i \cdot c_i(q, T_L, T_R)` where costs include position error (weight ~50), orientation error (~10), rest pose (~5), joint limits (~100), self-collision (~10), manipulability. Weights from `config/solver_smooth.yaml`. JIT-compiled via `jax.jit`, warm-started from previous solution or home pose. Runs in `run_in_executor` to avoid blocking the asyncio event loop.

**explanation**: JAX-based nonlinear least-squares optimization using `PyrokiSolver`. The solver is JIT-compiled at startup (`_warmup()`) — first solve is slow (~seconds), subsequent solves are fast (~milliseconds). Joint reordering happens twice: config order `[L1..7, L_grip, R1..7, R_grip]` → solver order `[L1..7, R1..7, L_finger1, R_finger1]` (input), then solver order → config order (output). The frontend receives 9 joints per arm (7 arm + 2 finger, where finger[1] is always 0.0 — filled by HOP 3). Warm-start mode: `use_home_warm_start=True` gives deterministic but slower convergence; `False` is responsive but path-dependent. Solve happens in a thread pool executor — does not block the 50 Hz asyncio loop. Source of truth: `config/solver_smooth.yaml` (cost weights, max_iterations, collision margin) and `config/robot.yaml` (home_position, joint names).

### what it does

Solves bimanual inverse kinematics using JAX JIT-compiled nonlinear optimization. Takes left/right SE3 targets (or None for inactive arms), returns 9 joint angles per arm in radians. Reorders joints between config convention and solver convention. Warm-starts from home (deterministic) or previous solution (responsive). Runs in thread executor to keep 50 Hz loop unblocked.

### output example with emphasis

**Input**: `target_L=SE3(trans=[0.52, -0.15, 0.33], rot=...)`, `target_R=None`
**Output**:
```
SolveResult(
    success=True,
    joint_radians_left=[
        0.508, -0.335, 0.442, 1.277, -0.432, 0.186, 0.052,  ← L1..L7 (rad)
        0.044, 0.0                                            ← L_finger1, L_finger2 (m)
    ],
    joint_radians_right=[
        -0.600, 0.297, -0.348, 1.228, 0.262, -0.153, 0.091,  ← R1..R7 (rad)
        0.044, 0.0                                            ← R_finger1, R_finger2 (m)
    ],
    position_error_mm={"left": 1.2},           ← FK vs target error
    orientation_error_deg={"left": 3.4},
    solve_time_ms=4.7,                         ← JIT-compiled, typical 2-8ms
)
```

Key characteristics:
- Joint reordering: config `[L1..7, L_grip, R1..7, R_grip]` ↔ solver `[L1..7, R1..7, L_f1, R_f1]`
- Finger joints (indices 7, 8) set to `[0.044, 0.0]` as placeholder — overwritten in HOP 3
- Second finger joint (index 8) always 0.0 — redundant with index 7
- Solve failure (exception) returns `success=False` with `failure_reason` string
- Audit log records every solve (max 10,000 entries) with targets, outputs, errors

---

## HOP 3: Gripper Override — Trigger Value → Finger Joint Position

**dimension space**: Joint angles → joint angles (finger joints overridden by analog trigger)

```
solver_loop.py:117-128
```
```python
        # Override finger joints with trigger-based gripper BEFORE snapshot
        # trigger=0 → gripper open → URDF 0.044m, trigger=1 → closed → URDF 0.0m
        if (
            result is not None
            and result.success
            and len(result.joint_radians_left) == 9
            and len(result.joint_radians_right) == 9
        ):
            result.joint_radians_left[7] = 0.044 * (1.0 - left_trigger)
            result.joint_radians_left[8] = 0.044 * (1.0 - left_trigger)
            result.joint_radians_right[7] = 0.044 * (1.0 - right_trigger)
            result.joint_radians_right[8] = 0.044 * (1.0 - right_trigger)
```

**latex**: `q_{\text{finger}} = 0.044 \cdot (1.0 - \tau)` where `\tau \in [0, 1]` is the analog trigger value (channel 0). `\tau=1` (pressed) → `q=0.0` (fully closed). `\tau=0` (released) → `q=0.044` (fully open, URDF joint limit).

**explanation**: The IK solver does not optimize finger joints — they are set directly from VR trigger values. Both finger joints per hand (indices 7 and 8) receive the same value. The constant `0.044` (meters) is the URDF finger joint limit (`openarm_left_finger_joint1` range). The trigger is read from VR button data channel 0 (analog trigger, extracted in HOP 1 via `_trigger_value()`). This happens BEFORE the snapshot is built — the monitoring UI and ROS2 output both see the overridden values. Source of truth: URDF joint limit `openarm_left_finger_joint1` = 0.044m.

### what it does

Overrides the IK solver's finger joint placeholders with VR analog trigger values. Both finger joints per hand (indices 7 and 8) receive `0.044 * (1.0 - trigger)`. Applied before snapshot and ROS2 publish — all downstream consumers see the same gripper state.

### output example with emphasis

**Input**: `left_trigger=0.5`, `right_trigger=0.8`
**Output**:
```
joint_radians_left[7]  = 0.022   ← 0.044 × (1.0 - 0.5), half-closed
joint_radians_left[8]  = 0.022   ← same value (redundant copy)
joint_radians_right[7] = 0.0088  ← 0.044 × (1.0 - 0.8), mostly closed
joint_radians_right[8] = 0.0088  ← same value (redundant copy)
```

Key characteristics:
- Both finger joints per hand always equal — index 8 is redundant
- Override happens AFTER IK solve, BEFORE snapshot/ROS2 publish
- Trigger values come from `vr_normalizer.normalize()` → `NormalizedTarget.left_trigger/right_trigger`
- Only applied when `result.success` and both arms have exactly 9 joints

---

## HOP 4: Joint Flattening, Split, and Quaternion Convention

**dimension space**: 2×9 joint arrays + WXYZ quaternion → left 8 joints + right 8 joints + XYZW quaternion

```
solver_loop.py:146-163
```
```python
        # Publish joint commands to ROS2 + WS arm
        if (
            result is not None
            and result.success
            and len(result.joint_radians_left) == 9
            and len(result.joint_radians_right) == 9
        ):
            joint_commands = result.joint_radians_left + result.joint_radians_right
            if ros2_publisher is not None:
                ros2_publisher.publish(
                    left_joints=result.joint_radians_left[:8],
                    right_joints=result.joint_radians_right[:8],
                    left_target_xyz=result.target_left_position if result.target_left_active else None,
                    left_target_quat_xyzw=_wxyz_to_xyzw(result.target_left_quaternion_wxyz) if result.target_left_active else None,
                    right_target_xyz=result.target_right_position if result.target_right_active else None,
                    right_target_quat_xyzw=_wxyz_to_xyzw(result.target_right_quaternion_wxyz) if result.target_right_active else None,
                )
```

```
solver_loop.py:265-267
```
```python
def _wxyz_to_xyzw(wxyz: list[float]) -> list[float]:
    """Convert quaternion from WXYZ (scalar-first) to XYZW (scalar-last)."""
    return [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]
```

**latex**: `L_{\text{out}} = [L_{1..7}, L_{f1}]` (8 values), `R_{\text{out}} = [R_{1..7}, R_{f1}]` (8 values)
`Q_{\text{xyzw}} = [wxyz_1, wxyz_2, wxyz_3, wxyz_0]` — scalar moves from index 0 to index 3

**explanation**: Each arm's 9-joint array is trimmed to 8: 7 arm joints + 1 finger joint. The second finger joint (index 8, always equal to index 7 from HOP 3) is discarded. Full 18-float array (`joint_commands`) is concatenated for WebSocket broadcast. Target quaternion is converted from WXYZ (jaxlie/solver convention) to XYZW (ROS2 `PoseStamped` convention). Source of truth: `ros2_publisher.py` `publish()` signature — expects 8 joints per arm, and `sensor_msgs/JointState` uses `float64` positions.

### what it does

Trims 9-joint arrays to 8 (drops redundant second finger). Concatenates to 18-float for WebSocket. Converts target quaternion from jaxlie WXYZ to ROS2 XYZW. Passes target pose only if `target_left_active`/`target_right_active` is True.

### output example with emphasis

**Input**: `joint_radians_left=[0.508, -0.335, 0.442, 1.277, -0.432, 0.186, 0.052, 0.022, 0.022]`
**Output**:
```
left_joints  = [0.508, -0.335, 0.442, 1.277, -0.432, 0.186, 0.052, 0.022]  ← [:8], dropped redundant [8]
right_joints = [0.009, 0.003, ...]                                            ← [:8]
joint_commands = left_joints + right_joints                                   ← 18 floats for WS

_wxyz_to_xyzw([0.98, 0.0, -0.17, 0.0]) = [0.0, -0.17, 0.0, 0.98]            ← WXYZ→XYZW
                                                    ↑ scalar moved to end
```

---

## HOP 5: ROS2 JointState + PoseStamped Publish

**dimension space**: Python lists → `sensor_msgs/JointState` + `geometry_msgs/PoseStamped` → DDS

```
ros2_publisher.py:81-108 (publish — called from solver loop)
ros2_publisher.py:150-175 (spin loop — daemon thread)
ros2_publisher.py:182-207 (_publish_joint_state, _publish_pose)
```
```python
    def publish(self, left_joints, right_joints,
                left_target_xyz, left_target_quat_xyzw,
                right_target_xyz, right_target_quat_xyzw) -> None:
        if not self._ros2_available or self._thread is None:
            return
        self._latest = (left_joints, right_joints,
                        left_target_xyz, left_target_quat_xyzw,
                        right_target_xyz, right_target_quat_xyzw)
        self._has_data = True

    # -- in daemon thread --
    while self._thread is not None and rclpy.ok():
        if self._has_data:
            self._has_data = False
            (lj, rj, lxyz, lquat, rxyz, rquat) = self._latest
            now = clock.now().to_msg()

            self._publish_joint_state(self._left_joint_pub, now, 'base_link',
                                      self._left_names, lj)
            self._publish_joint_state(self._right_joint_pub, now, 'base_link',
                                      self._right_names, rj)

            if lxyz is not None:
                self._publish_pose(self._left_pose_pub, now, 'base_link', lxyz, lquat)
            if rxyz is not None:
                self._publish_pose(self._right_pose_pub, now, 'base_link', rxyz, rquat)

        executor.spin_once(timeout_sec=0.02)

    @staticmethod
    def _publish_joint_state(pub, stamp, frame_id, names, values):
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.name = names
        msg.position = [float(v) for v in values]
        pub.publish(msg)
```

**latex**: `msg.\text{position} = [v_1, ..., v_8] \in \mathbb{R}^8` (radians). `msg.\text{name} = [\text{openarm\_left\_joint1}, ..., \text{openarm\_left\_finger\_joint1}]`. Published topics: `/arm/left/joint_commands`, `/arm/right/joint_commands`, `/arm/left/target_pose`, `/arm/right/target_pose`. QoS: `BEST_EFFORT, VOLATILE, KEEP_LAST, depth=1`.

**explanation**: Thread-safe decoupled publish: the solver loop calls `publish()` which stores data under a flag, and a daemon thread picks it up via `spin_once(timeout_sec=0.02)`. This avoids blocking the 50 Hz loop on DDS writes. Each arm publishes 8 joints (7 arm + gripper) in a single `JointState` message with URDF joint names. Target poses (the IK input, not FK output) are published as `PoseStamped` with XYZW quaternion. Joint names are hardcoded in `ws_handler.py:147-156` (8 names per arm). Source of truth: `robot_interface/README.md` JointState convention and URDF joint names from `openarm_description`.

### what it does

Publishes per-arm `JointState` (8 joints, radians) and `PoseStamped` (target pose) to 4 ROS2 topics. Decoupled architecture: solver loop stores data under `_has_data` flag, daemon thread picks it up. Uses BEST_EFFORT QoS with depth=1. Gracefully degrades if `rclpy` is not importable.

### output example with emphasis

**Published topics**:
```
/arm/left/joint_commands   (JointState)     ← 8 joints: L1..L7 + L_finger1
/arm/right/joint_commands  (JointState)     ← 8 joints: R1..R7 + R_finger1
/arm/left/target_pose      (PoseStamped)    ← IK input target, XYZW quaternion
/arm/right/target_pose     (PoseStamped)    ← published only if target_active
```
```
JointState message:
  header:
    stamp: {sec: 1779954107, nanosec: 356294491}   ← ROS2 clock at publish time
    frame_id: base_link
  name: ["openarm_left_joint1", ..., "openarm_left_finger_joint1"]  ← 8 URDF names
  position: [0.508, -0.335, 0.442, 1.277, -0.432, 0.186, 0.052, 0.022]
```

Key characteristics:
- `header.stamp` is ROS2 clock (`clock.now()`), NOT hardware acquisition time
- Target pose only published when `target_left_active`/`target_right_active` is True
- `spin_once(timeout_sec=0.02)` runs at ~50 Hz — matches solver loop cadence
- Joint names from `ws_handler.py:147-156`, not from config

---

## HOP 6: WebSocket Arm Broadcast — Joint Commands → Motor Control

**dimension space**: 18-float joint array + trigger values → JSON WebSocket message

```
solver_loop.py:184-210
```
```python
async def _broadcast_arm(
    app: web.Application,
    joint_commands: list[float],
    left_trigger: float = 0.0,
    right_trigger: float = 0.0,
) -> None:
    arm_websockets: set[web.WebSocketResponse] = app.get("arm_websockets", set())
    if not arm_websockets:
        return
    # Gripper: trigger pressed → close (0°), released → open (-65°)
    left_gripper_deg = -65.0 * (1.0 - left_trigger)
    right_gripper_deg = -65.0 * (1.0 - right_trigger)
    data = json.dumps({
        "type": "arm_commands",
        "joints": joint_commands,
        "left": joint_commands[:9],
        "right": joint_commands[9:],
        "left_gripper_deg": left_gripper_deg,
        "right_gripper_deg": right_gripper_deg,
    })
    for ws in list(arm_websockets):
        if not ws.closed:
            try:
                await ws.send_str(data)
            except Exception:
                pass
```

**latex**: `msg = \{type: \text{"arm\_commands"}, joints: [v_0..v_{17}], left: [v_0..v_8], right: [v_9..v_{17}], left\_gripper\_deg: -65.0 \cdot (1.0 - \tau_L), right\_gripper\_deg: -65.0 \cdot (1.0 - \tau_R)\}`

**explanation**: The full 18-float joint array (9 per arm, including both finger joints) is sent as JSON to all connected arm WebSocket clients at `/ws/arm`. The gripper values are converted to degrees: `0°` (closed) to `-65°` (open). The arm controller (`arm_controller.py`) reads this stream and drives physical motors. This is the CONTROL path — separate from the ROS2 publish path. Source of truth: `arm_controller.py` — expects `left`, `right`, `left_gripper_deg`, `right_gripper_deg` fields.

### what it does

Broadcasts 18-float joint commands (9 per arm, including both finger joints) and gripper values in degrees to all WebSocket clients at `/ws/arm`. The arm controller drives motors from this stream. Gripper angle: `0°` closed to `-65°` open.

### output example with emphasis

**Input**: `joint_commands=[0.508, -0.335, ..., 0.022, -0.600, ..., 0.009]`, `left_trigger=0.5`, `right_trigger=0.8`
**Output**:
```json
{
  "type": "arm_commands",
  "joints": [0.508, -0.335, ..., 0.022, -0.600, ..., 0.009],  ← 18 floats
  "left": [0.508, -0.335, ..., 0.022, 0.022],                 ← 9 floats (L1..L7 + L_f1 + L_f2)
  "right": [-0.600, ..., 0.009, 0.009],                        ← 9 floats (R1..R7 + R_f1 + R_f2)
  "left_gripper_deg": -32.5,                                   ← -65.0 × (1.0 - 0.5)
  "right_gripper_deg": -13.0                                   ← -65.0 × (1.0 - 0.8)
}
```

Key characteristics:
- Sent at 50 Hz (every solver loop iteration)
- `joints` = full 18 floats; `left`/`right` = 9 each (includes redundant finger joint)
- Gripper in degrees for motor controller compatibility
- Connection failures silently caught — dropped WS clients don't crash the loop

---

## HOP 7: Snapshot Broadcast to Monitoring UI

**dimension space**: `SolveResult` + VR status + calibration state → JSON snapshot → WebSocket

```
solver_loop.py:130-144 (build + broadcast)
solver_loop.py:172-181 (_broadcast)
snapshot.py:12-56 (build_snapshot)
```
```python
        snapshot = build_snapshot(
            solve_result=result,
            config=config,
            solve_index=app.get("solve_index", 0),
            solver_status={
                "warmup_complete": ik_service.solver.warmup_complete,
                "warmup_error": ik_service.solver.warmup_error,
            },
            ik_service=ik_service,
            vr_status=vr_status,
            control_source=control_source,
        )

        await _broadcast(app, snapshot)

async def _broadcast(app: web.Application, snapshot: dict[str, Any]) -> None:
    websockets: set[web.WebSocketResponse] = app.get("websockets", set())
    data = json.dumps(snapshot)
    for ws in list(websockets):
        if not ws.closed:
            try:
                await ws.send_str(data)
            except Exception:
                pass
```

**latex**: N/A (structural — JSON serialization of full system state for browser rendering)

**explanation**: A complete system state snapshot (solve result, VR telemetry, calibration state, IK errors, control source, solver health) is serialized to JSON and broadcast to all general WebSocket clients at `/ws`. The browser monitoring UI (`web/app.js`) renders a 3D robot visualization, VR marker overlay, and telemetry panels from this data. This is the MONITORING path — separate from the arm control WebSocket (`/ws/arm`). `_map_vr_poses()` applies the same axis mapping as HOP 1 so the UI shows robot-frame coordinates. Source of truth: `web/app.js` — the browser expects specific JSON fields for rendering.

### what it does

Builds a complete system state snapshot (solve result, VR poses in robot frame, calibration state, errors, control source, solver health) and broadcasts as JSON to all monitoring WebSocket clients at 50 Hz. The browser 3D UI renders robot visualization from this data.

### output example with emphasis

**Snapshot structure**:
```
{
  "timestamp_ns": 1779954107356294491,
  "solve_index": 2847,
  "control_source": "vr",
  "ik": {
    "success": true,
    "joint_radians_left": [...],          ← 9 per arm
    "position_error_mm": {"left": 1.2},
    "solve_time_ms": 4.7,
    "target_left_position": [0.52, -0.15, 0.33],
  },
  "vr": {
    "connected": true,
    "poses": {                            ← axis-mapped to robot base frame
      "left": {"position": [0.52, -0.15, 0.33], "stale": false},
      "right": {...}
    },
    "calibration": {"state": "active", "origin_pinned": true},
  },
  "solver_status": {"warmup_complete": true},
}
```

Key characteristics:
- Sent at 50 Hz to all `/ws` clients — independent of arm control WebSocket
- VR poses in snapshot are axis-mapped (same as HOP 1) so UI shows robot-frame coordinates
- Connection failures silently caught — dropped WS clients don't crash the loop
- `control_source` indicates which path produced the target: `"vr"`, `"drag"`, or `"idle"`

---

## Summary: OpenArm IK ROS2 Adaptor Pipeline

### Data Flow:
```
ROS2 Topics (/vr/*)                          DDS, BEST_EFFORT, ~60 Hz
    |
    v HOP 0: VRSubscriber._run_ros2()        Thread-safe VRDataStore
    |                                         Position: [x,y,z], Orientation: WXYZ
    |
    v HOP 1: VRNormalizer.normalize()         SE3 targets in robot base frame
    |                                         State machine: WAITING→READY→ACTIVE
    |                                         Rebase (origin⁻¹ @ pose) + axis map
    |
    v HOP 2: ik_service.solve_targets()      JAX JIT nonlinear optimization
    |                                         9 joints/arm (radians), ~2-8ms solve
    |
    v HOP 3: Gripper override                Finger joints = 0.044 × (1 - trigger)
    |                                         Indices [7], [8] per arm
    |
    v HOP 4: Flatten + split                 [:8] per arm, drop redundant finger
    |                                         WXYZ → XYZW quaternion conversion
    |
    +--> HOP 5: ROS2 publish                 /arm/left/joint_commands (JointState)
    |                                         /arm/right/joint_commands (JointState)
    |                                         /arm/left/target_pose (PoseStamped)
    |                                         /arm/right/target_pose (PoseStamped)
    |
    +--> HOP 6: WS arm broadcast             /ws/arm → arm_controller → motors
    |                                         18 floats + gripper_deg
    |
    +--> HOP 7: Snapshot broadcast           /ws → browser 3D monitoring UI
                                              50 Hz full system state JSON
```

### Key Source-of-Truth Documents:
1. **`config/vr.yaml`**: Axis mapping matrix (`native_to_intermediate`), calibration buttons (B=pin, A=start), topic names, QoS depth
2. **`config/solver_smooth.yaml`**: IK cost weights, max iterations, collision margin, warm-start mode
3. **`config/robot.yaml`**: Home position (degrees), URDF path, joint names
4. **`robot_interface/README.md`**: JointState + PoseStamped convention (8 joints/arm, radians)
5. **`openarm_ik_solver/solver.py`**: PyrokiSolver — JAX JIT nonlinear least-squares optimization
6. **URDF model** (`openarm_description`): Joint limits (finger = 0.044m), joint names, kinematic chain
