# Airspeed Data Collection

Domain language for the uniform robot data collection service: ROS2 streams recorded into per-episode HDF5 files, later converted to LeRobot datasets for policy training.

## Language

**Episode**:
One continuous recording, stored as a single `.h5` file named `episode-<task>-T#-<timestamp>.h5` inside the task's output directory.
_Avoid_: recording, session, run

**Task**:
A named unit of data collection created in the operator UI (e.g. `pick-can-and-lemon`); owns an output directory, a `task_meta.json`, and many **Episodes**.
_Avoid_: job, collection

**Stream**:
A named, per-episode time series inside the `.h5` (one ROS2 topic), with its own timestamps and attrs (`columns`, `sample_rate`, `frames`).
_Avoid_: topic, channel, signal

**Collection profile**:
The data-collection setup identity (session YAML `task_id`, e.g. `vr_teleop_with_ik_and_arm_state_button`): which topics, IK mode, and control bindings are active. Config-level, constant across operators and **Tasks**. NOT a robot task.
_Avoid_: task_id (ambiguous), task

**Task completed**:
Per-episode flag: did the robot achieve the task goal? Set by operator action — toggle-stop (A) = True, abort (B during recording) = False. A False episode with clean recording is a usable **failure demo**, kept in the task folder.
_Avoid_: success (overloaded)

**Recording valid**:
Per-episode flag: is the data technically usable for training? True at close; stamped False when the episode is deleted to `.trash/`. Independent of **Task completed** — a failed task can be valid data; a "successful" task with a glitched recording is not.
_Avoid_: success (overloaded)

**Feature name**:
The globally-unique LeRobot dataset name for one dimension of one **Stream** (e.g. `R_joint_state_1`, `head_pose_qw`). Authored once in session YAML `columns:`, flowing verbatim YAML → h5 columns attr → LeRobot feature name. Legacy episodes without semantic columns fall back to `{stream}_{i}`.
_Avoid_: dim_N, stream_i

**Failure demo**:
An episode where **Task completed** = False but **Recording valid** = True — kept in the task folder as legitimate training data (negative example).
_Avoid_: trash, bad take

**Task structure**:
Single-select classification of a **Task**'s temporal composition: `atomic_single` (one motion primitive), `composite_multistage` (sequence of sub-tasks), or `long_horizon` (extended continuous activity). Selected in the UI at task creation.
_Avoid_: task_type (overloaded)

**Deformable objects**:
Yes/no flag on a **Task** indicating whether it manipulates deformable objects (cloth, cables, food). Independent of **Task structure** — any structure bucket can involve deformables.
_Avoid_: folding "deformable" into task_type

**Task index**:
The numeric task ID in the converted LeRobot dataset (`tasks.parquet`), assigned by the converter from distinct **Task prompt** strings. Never recorded into the `.h5`.
_Avoid_: task_id

**Column names**:
Per-dimension labels of a vector **Stream** (e.g. `R_joint_1 … R_gripper` for an 8-DOF arm). Robot-level configuration, constant across all **Tasks**; sourced from session YAML, never typed per task.
_Avoid_: task description, prompt

**Task prompt**:
Free-text description of a **Task** entered in the operator UI at task creation (e.g. "pick up the can with the left hand, then the lemon"), stored in `task_meta.json` and written to every **Episode**'s root attrs. Becomes the model's language instruction at training time.
_Avoid_: column names, description attr

**Pose stream layout**:
The recorded value order of any `*_pose` **Stream**: `[orientation.w, orientation.x, orientation.y, orientation.z, position.x, position.y, position.z]` — deterministic sorted-key flattening, quaternion w-first. Column labels must match this order exactly.
_Avoid_: ROS field order, message order

**Hand session**:
A **Collection profile** variant where ROH dexterous hands replace the OpenArm gripper motors (mutually exclusive with gripper sessions). Arms run 7 joints (`gripper_enabled: false` in robot.yaml / IK config); each hand is its own device publishing stamped `sensor_msgs/JointState` in radians (`/roh/<side>/joint_state`, 6 fingers: thumb, index, middle, ring, pinky, thumb_root; 0 = open) and accepting `/roh/<side>/joint_command`. Finger currents ride in `effort` (mA).
_Avoid_: gripper column, Float32MultiArray streams, ros_receive

**Contact latch**:
ROH driver safety state: a finger current ≥ 250 mA while closing/holding stops the hand and rejects further closing commands until the operator commands a retreat below the recorded stop position (hysteresis) or fully open. Distinct from the VR open latch (right B/A).
_Avoid_: estop, freeze
