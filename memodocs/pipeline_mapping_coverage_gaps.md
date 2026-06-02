# Pipeline Mapping Coverage Gaps

Date: 2026-06-02

What the 4 PIPELINE_MAPPING.md files do not cover — the code surrounding each data HOP.

---

## teleoperation_interface — `vr_bridge_server.py`

### Not mentioned in any HOP:

**Module initialization (lines 30-63):** `ROOT` path resolution via `LEROBOT_SRC` env var, a `resolve_path()` helper that maps relative paths to the package root. Several import guards (`ROS2_AVAILABLE`, `HAS_AIOHTTP`) gate the entire module — if dependencies are missing, the node silently degrades rather than crashing.

**Logging setup (lines 65-78):** A rotating file handler writes to `logs/vr_bridge.log` with 5 MB rollover, 3 backups. A custom `ThrottledHTTPAccessLogger` suppresses the default aiohttp access log noise.

**`VRBridgeNode` class `__init__` (lines 186-221):** Creates 6 ROS2 publishers on a conditional import path — if `rclpy` is absent, the node is never instantiated and the HTTP server still runs but silently drops pose data. The publishers are: `/vr_raw_data` (String passthrough), `/vr/head_pose`, `/vr/left_pose`, `/vr/right_pose` (PoseStamped), `/vr/left_buttons`, `/vr/right_buttons` (Float32MultiArray). All share `REALTIME_QOS`.

**Rate statistics (lines 247-253):** Every 5 seconds, the node logs the observed publish rate. This is a lightweight in-band telemetry check — if it diverges from the expected ~60 Hz, the VR device or network is the likely culprit.

**`handle_index` (lines 304-309):** Serves the static VR HTML page at `/`. Uses `resolve_path()` to find `static/VRDriverPlaneCamera.html`. Returns 404 if missing.

**`main()` (lines 312-407):** The entry point. Loads `config/config.json` for port/IP, conditionally initializes ROS2 (spins `rclpy` in a daemon thread), starts the aiohttp web app with `handle_pose` at `POST /poseData` and `handle_index` at `GET /`. Sets up signal handlers for SIGINT/SIGTERM. Tears down ROS2 and the event loop on shutdown.

**Signal handling:** Registers `signal.signal(signal.SIGINT, ...)` to gracefully stop both the asyncio loop and the ROS2 spin thread.

---

## sensor_interface — `camera_publisher.py`

### Not mentioned in any HOP:

**Import guards (lines 30-50):** `_HAS_REALSENSE`, `_HAS_CV2`, `_HAS_ROS2` — each protects against missing dependencies. If `cv2` is absent, the stream loop becomes a `time.sleep(0.1)` no-op rather than crashing.

**`_DIRECT_API_STREAMS` (line 55):** A set of stream types (`{"depth", "infra_left", "infra_right"}`) that require direct `pyrealsense2` API access and are explicitly NOT supported. They're warned about during discovery but never enabled.

**QoS profiles (lines 57-68):** Two distinct QoS profiles: `_QOS` for Image topics (BEST_EFFORT, KEEP_LAST depth=1) and `_CAMERA_INFO_QOS` for CameraInfo (TRANSIENT_LOCAL — latched, so late subscribers get the last published intrinsics).

**`_build_camera_info()` (lines 157-171):** Constructs a full `CameraInfo` message with K (intrinsics), P (projection), R (rectification), and D (distortion) matrices. Defaults: fx/fy=615, cx=320, cy=240, plumb_bob with zero distortion. These defaults are placeholders — real calibration values come from `config/camera.yaml`.

**`CameraPublisherNode.__init__` full (lines 181-228):** Much more than just the discovery call. It iterates per-camera stream configs, creates per-stream publishers for both `Image` and `CameraInfo` topics, names topics by suffix convention (`image_raw`, `depth/image_raw`, etc.), and stores per-stream config in a dict (`jpeg_q`, `encoding`, `stype`). Also publishes CameraInfo once at startup.

**`start()` / `stop()` (lines 230-238):** Thread lifecycle management. `start()` spawns a daemon thread running `_stream_loop`. `stop()` sets a flag and joins with a 2-second timeout.

**`_stream_loop()` full (lines 240-284):** The outer loop that iterates `self._streams` per iteration (no per-stream threads). This is important — all cameras share one thread, so a slow camera blocks the others. Includes a `_HAS_CV2` guard, per-stream exception handling with 5-second throttle on error logging, and a `_maybe_log_stats()` call for periodic rate reporting.

**`_maybe_log_stats()`:** Logs per-camera frame counts every 5 seconds for monitoring.

**`main()`:** Argument parsing (`--config`), config loading from YAML, discovery + connection, node creation, spin, and cleanup. Parses the `cameras:` and `intrinsics:` sections of the config file.

**Serial matching logic (lines 122-127):** Not just "connect any camera" — if `serial` in config is not `"auto"`, the discovered camera's serial must match or it's skipped with a warning. This prevents connecting to the wrong physical device.

---

## robot_interface

### `vr_subscriber.py` — Not mentioned:

**`VRPose` and `VRButtonState` dataclasses:** Immutable data containers for pose (position xyz + orientation xyzw + timestamp) and button state (6 floats + timestamp).

**`VRDataStore` class (full, lines 29-136):** The thread-safe store that sits between ROS2 callbacks and the solver loop. Methods beyond the mapped ones: `update_pose()`, `update_buttons()`, `update_raw()`, `get_latest_poses()` (with stale-timeout filtering returning only non-stale poses), `get_head_pose()`, `get_left_buttons()`, `get_right_buttons()`, `is_connected()` (returns False if no message in `stale_timeout_s` seconds), `get_status()` (full snapshot for WebSocket broadcast). The stale timeout is critical — it prevents the IK solver from acting on dead-reckoned poses when the VR device disconnects.

**`VRSubscriber._run_ros2()` (lines 184-230):** The actual daemon thread function. Creates a `rclpy` Node, sets up 5 subscriptions with BEST_EFFORT QoS, and calls `rclpy.spin()`. Each subscription callback parses the ROS2 message and calls the corresponding `data_store.update_*()` method. On `KeyboardInterrupt` or `rclpy` error, it shuts down gracefully. This is the real subscription logic that HOP 0 summarizes.

**Subscription callbacks:** Parse `PoseStamped` → `VRPose`, `Float32MultiArray` → button list, `String` → raw JSON dict. Each strips the ROS2 wrapper and stores only the payload in `VRDataStore`.

**`is_connected` / `ros2_installed` / `is_running` properties:** Status flags consumed by the solver loop to decide whether to fall back to idle/keyboard control.

---

### `solver_loop.py` — Not mentioned:

**The outer `run_solver_loop()` function (lines 24-170, only HOPs 1-4 were mapped):** An `aiohttp` app lifecycle function. Starts with `on_startup`/`on_cleanup` hooks for the IK service. Contains two additional control paths the map didn't cover:

**Priority 2: Legacy target buffer (lines 79-88):** When VR is not active, the solver can accept targets from a drag-based WebSocket UI. The `buffer.read()` pulls left/right targets set by a different interface, parsed via `parse_target()`. This is the "drag" control source.

**Control source state machine:** `"idle"` → `"vr"` (priority 1) → `"drag"` (priority 2). When both sources are silent, the solver isn't called (`result = None`), and the loop still maintains its 50 Hz cadence.

**VR status building (lines 97-115):** Constructs a rich status dict for the WebSocket snapshot broadcast. Calls `_map_vr_poses()` to transform VR-native coordinates to robot base frame (separate from the normalizer — this is for visualization, not IK). Includes calibration state, subscriber health, and ROS2 availability.

**`_broadcast()` (lines 172-182):** The snapshot broadcast path — sends full system state (solve result, VR status, solver health, control source, calibration state) as JSON to all general WebSocket clients. This is separate from `_broadcast_arm()` which only sends joint commands.

**`build_snapshot()` call (line 131):** Assembles a complete state snapshot from solve result, config, VR status, and solver metadata. This is the primary visualization data for the Web UI.

**`_map_vr_poses()` and `_map_calibration()`:** Transform VR poses from native space to robot base frame for visualization purposes (not for IK — the normalizer handles that).

**B-button reset-to-home (lines 70-71):** When the VR normalizer transitions from `waiting` to `ready` (operator presses B-button to pin origin), the IK solver is reset to home pose. This is a critical UX behavior — pinning the VR origin also recenters the robot arms.

**50 Hz cadence sleep (lines 165-169):** `SOLVE_PERIOD_S = 0.02` — the loop computes elapsed time and sleeps the remainder. If a solve takes longer than 20 ms, the sleep is skipped (no negative sleep). This maintains a consistent control rate regardless of solve time variance.

**`_wxyz_to_xyzw()` helper:** Converts quaternion convention from the solver's internal WXYZ to ROS2's XYZW. This is where a coordinate convention bug would live if the mapping were wrong.

---

### `ros2_publisher.py` — Not mentioned:

**`ROS2Publisher` class `__init__` (lines 108-143):** Creates a hidden ROS2 node in a `SingleThreadedExecutor`, sets up 4 publishers (`/arm/left/joint_commands`, `/arm/right/joint_commands`, `/arm/left/target_pose`, `/arm/right/target_pose`) with BEST_EFFORT QoS, and stores joint name lists from config.

**Threaded spin loop (lines 150-175):** Runs in a daemon thread. Uses a `_has_data` flag with `_latest` tuple to decouple the solver loop's `publish()` call from DDS writes. `spin_once(timeout_sec=0.02)` processes pending DDS work non-blockingly. The `_publish_pose()` method constructs `PoseStamped` for target poses — this is the same pattern as `_publish_joint_state` but for pose targets.

**`publish()` method (lines 72-93):** The interface the solver loop calls. Stores the latest joint/pose data under a lock and sets `_has_data = True`. The spin thread picks it up on the next iteration.

**Thread start/stop (lines 95-106):** `start()` spawns the daemon thread. `stop()` sets `_thread = None` and the spin loop exits on the next iteration.

---

### `arm_state_publisher.py` — Not mentioned:

**`ArmStatePublisherNode.__init__` (lines 64-80):** Creates publishers for `/arm/left/joint_state` and `/arm/right/joint_state`, sets up a `create_timer` at the configured `arm_state_hz` (default 20 Hz), and loads joint name lists from config. The joint names default to `left_joint_1` through `left_joint_8` (8 joints: 7 arm + gripper).

**`_callback()` full (lines 82-106):** Beyond the mapped snippet: wraps `_read_arm_state()` in try/except with throttled error logging (5-second throttle), publishes both left and right `JointState` messages, increments a frame counter, and calls `_maybe_log()` for periodic rate reporting.

**`_maybe_log()`:** Logs the actual publish rate every 5 seconds.

**`stop_timers()` (lines 108-112):** Destroys all timers during shutdown to prevent callbacks from firing after the node is destroyed.

**QoS profile (lines 37-47):** BEST_EFFORT, KEEP_LAST depth=1 — same pattern as the command publishers. Dropped state readings are acceptable.

**`main()` (lines 115-138):** Connects to the CAN bus via `OpenArmsFollower` (no calibration, no torque — pure observer), creates the node, spins, and cleans up. The follower connection is the critical step — if the CAN interface is wrong, all state readings are zeros.

**Config-driven joint naming (lines 71-72):** Joint names come from config, not hardcoded. This means the 8-joint convention (7 arm + gripper in one message) is configurable per deployment.

---

## data_collection_service

### `session_config.py` — Not mentioned:

**`StreamConfig`, `FieldRule`, `TimeDomain`, `StreamQoS`, `SessionMeta`, `StorageConfig`, `RecordingControlConfig`:** All frozen dataclasses that form the session schema. `FieldRule` supports `path`, `type` (scalar/sequence), `min`/`max` bounds, and `required` flag.

**`_parse_config()` (the full parser, ~200 lines):** Validates every YAML key against allowed sets (`_ROOT_KEYS`, `_STREAM_KEYS`, `_SESSION_KEYS`). Rejects unknown keys. Parses enums via `TimeDomain.from_string()`. Validates `QoSProfile` fields (reliability, durability, history, depth). Each `FieldRule` is validated for valid path syntax, allowed types, and range consistency. Unknown recording control modes are rejected. This is the ~80 validation checks mentioned in the doc.

**`_UniqueKeyLoader`:** A YAML loader subclass that rejects duplicate mapping keys. This prevents a class of silent bugs where a repeated key in YAML silently overwrites the first value.

**`load_session_config_dict()`:** Alternative entry point that accepts a pre-parsed dict instead of a file path. Used for programmatic config generation/testing.

---

### `registry.py` — Not mentioned:

**`AdapterBinding` and `ConfiguredStreamAdapter`:** The binding links a `(source, message_type)` signature to a `payload_profile` and a `payload_builder` callable. The configured adapter wraps a binding with a stream config and validator.

**`register_streams()` (lines 50-76):** Called during episode creation to pre-register HDF5 groups. Determines whether a stream is image or vector by checking `message_type` for image patterns. For vectors, auto-detects dims from field count unless a sequence field is present (→ `dims=0` for runtime sizing). For images, registers width/height/channels and optionally enables JPEG re-encoding.

**`_resolve_binding()` (lines 127-140):** Looks up by `(source, message_type)` signature. If multiple bindings match, raises `AdapterError` with the ambiguity list. If none match, raises with the list of supported signatures. This is startup-time validation — a misconfigured session YAML fails immediately.

**`_default_bindings()` (lines 157-208):** The registry of all supported message types. Each entry maps a `(source, type)` pair to a payload builder function. Currently defines: `teleop_pose` (PoseStamped → 7 fields: x,y,z + xyzw), `robot_joint_state` (JointState → 8 fields: 7 joints + gripper), `teleop_buttons` (Float32MultiArray → 6 fields), `camera_image` (Image → bytes). This is where new message types are added.

**`AdapterBoundaryValidator` and `AdapterPayloadProfileRegistry`:** The validator checks extracted payloads against named profiles — each profile defines expected fields with types and optional range constraints.

**`extract_header_timestamp()`:** Extracts `header.stamp` from ROS2 messages that have one. Returns `None` for messages without headers (like `Float32MultiArray`).

**`build_boundary_sample()` and `_writer_sample_from_payload()`:** The flattening functions. Recursively walk a validated dict, extracting float leaves in sorted-key order for vectors, or raw bytes for images.

---

### `ros2_collection_node.py` — Not mentioned:

**`RecordingCollectionNode.__init__` full (lines 60-121):** Much more than subscription creation. Creates:
- `RecordingStateMachine` with start/end handlers that call `writer.open_episode()` and `writer.close_episode()`
- `StreamTracker` with configurable expected periods (500 ms default) — tracks per-stream valid/invalid/late counts
- `RecordingControlRouter` that dispatches to one of three control modes (service, manual_ui, device_binding)
- `AdapterRegistry.with_defaults()` and resolves all stream adapters
- The manual operator UI (aiohttp web server on configurable host:port)

**`RecordingStateMachine` interaction:** The start handler creates a new episode directory with the session name and timestamp. The end handler closes the HDF5 file and logs the final path and frame counts.

**`StreamTracker`:** Tracks `valid_count`, `invalid_count`, `late_count` per stream. `record_valid()` checks if the inter-arrival gap exceeds the expected period and increments the late counter. This data is exposed in the operator UI and the status service.

**`RecordingControlRouter`:** Three modes:
- `service`: ROS2 services `/data_collection/record_start`, `/data_collection/record_stop`, `/data_collection/record_status`
- `manual_ui`: A web-based operator dashboard with start/stop buttons and per-stream health
- `device_binding`: Tied to a specific stream's message (e.g., VR button press toggles recording)

**`_create_services()`:** Registers the ROS2 service endpoints.

**`_start_manual_ui()`:** Starts the aiohttp-based operator dashboard at the configured port. Serves per-stream status, recording state, and control buttons.

**`_resolve_message_class()`:** Dynamically imports ROS2 message types from string names (e.g., `"geometry_msgs/PoseStamped"` → `geometry_msgs.msg.PoseStamped`). This is the extensibility mechanism — adding a new message type doesn't require code changes, just a new adapter binding.

**`_qos_profile_from_stream()`:** Translates YAML QoS config (string enums for reliability, durability, history) into `rclpy.qos.QoSProfile` objects.

**Episode lifecycle:** `writer.open_episode()` creates the HDF5 file, registers all groups, and writes root attributes. `writer.close_episode()` finalizes the file. Both are triggered by the `RecordingStateMachine` callbacks.

---

### `airs_hdf5_writer.py` — Not mentioned:

**`AirsHdf5Writer` class (lines 30-157):** The full writer lifecycle:
- `open_episode()`: Creates the HDF5 file, writes root attrs (`description`, `robot_type`, `series_number`, `sample_rate`, `frames`), and pre-creates all registered stream groups with their `type` attr.
- `close_episode()`: Flushes all buffers, writes final frame counts to root attrs, closes the file.
- `register_vector_stream()` / `register_image_stream()`: Creates `_VectorBuffer` / `_ImageBuffer` instances with dims, columns, and encoding params.
- `append_vector()` / `append_image()`: Type-check and route to the correct buffer.

**`_ImageBuffer` class:** Parallel to `_VectorBuffer`. Stores raw bytes (not numpy arrays) for JPEG frames. Optionally re-encodes images to JPEG on append. Has its own batch threshold (`_IMAGE_BATCH = 20`).

**`_flush_batch()` full:** For vectors: stacks buffered arrays into `(N, dims)` via `np.array()`, lazily creates the `data` and `timestamps` datasets on first flush (with `maxshape=(None, dims)` or `maxshape=(None,)` for timestamps), then incrementally `resize()` + writes the new slice. Same pattern for images. This incremental resize approach means the HDF5 file is always valid — a crash mid-collection leaves a readable partial file.

**`_VECTOR_BATCH = 50` and `_IMAGE_BATCH = 20`:** The flush thresholds. Tuned for a balance between memory usage and HDF5 write amplification.

**`AirsHdf5WriterError`:** Custom exception for dimension mismatches and type errors.

---

### `dataset_validator.py` — Not mentioned:

**`_validate_root_attrs()` (lines 71-75):** Checks for required root attributes: `description`, `robot_type`, `series_number`, `sample_rate`, `frames`. Missing attrs are reported as errors.

**`_validate_vector_group()` (lines 78-100):** Checks: `data` dataset exists with dtype `float32`, `timestamps` dataset exists with dtype `uint64`, both have matching length in first dimension, `columns` attr matches the data second dimension, no NaN or Inf values in data.

**`_validate_image_group()` (lines 102-128):** Checks: `data` dataset exists (dtype flexible), `timestamps` dataset exists with dtype `uint64`, matching lengths, first frame starts with JPEG magic bytes `0xFFD8`.

**`ValidationReport` and `ValidationIssue`:** Structured output types. `ValidationReport` has `dataset_path`, `is_valid` (bool), `errors` (list), `warnings` (list). `ValidationIssue` has `path` (HDF5 group path) and `message`.

**`_ROOT_ATTRS`:** The canonical set of required root attributes. Adding a new attribute here makes it mandatory for all future episodes.

---

## Summary of Coverage Gaps

| Module | Major Unmapped Areas |
|--------|---------------------|
| `vr_bridge_server.py` | Module init, logging, `main()`, signal handling, route registration, rate stats |
| `camera_publisher.py` | Thread lifecycle, `_build_camera_info()`, serial matching logic, `main()`, per-stream config, QoS profiles, multiple stream type support |
| `vr_subscriber.py` | VRDataStore (full), `_run_ros2()` daemon, subscription callbacks, stale timeout logic, connection health |
| `solver_loop.py` | Outer loop, drag control path, VR status building, snapshot broadcast, B-button reset, quaternion convention conversion, control source state machine |
| `ros2_publisher.py` | Threaded spin loop, `_publish_pose()`, data decoupling with flags, thread lifecycle |
| `arm_state_publisher.py` | Timer-based loop, CAN follower connection, `main()`, config-driven joint naming |
| `session_config.py` | Full `_parse_config()` (~200 lines, ~80 validations), all dataclass definitions, `_UniqueKeyLoader` |
| `registry.py` | `_default_bindings()`, `_resolve_binding()`, `register_streams()`, payload flattening functions, boundary validator |
| `ros2_collection_node.py` | Episode lifecycle, RecordingStateMachine, StreamTracker, RecordingControlRouter (3 modes), operator UI, ROS2 services |
| `airs_hdf5_writer.py` | Full writer lifecycle, `_ImageBuffer`, `_flush_batch()` (incremental resize), episode open/close, root attr writing |
| `dataset_validator.py` | Per-group validation functions (vector dtype/shape, image magic bytes), `ValidationReport` structure |

The pipeline maps cover the data transformation path (~30% of total code). The remaining ~70% is: lifecycle management (init/start/stop/shutdown), health monitoring (rate stats, connection timeouts, stream tracking), multi-mode control (VR vs drag vs idle, service vs UI vs device_binding), error handling (import guards, exception throttling, partial-file safety), and configuration/validation machinery.
