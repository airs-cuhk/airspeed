# Data Collection Service — Core

YAML-driven, hardware-agnostic multi-stream data recorder. Subscribes to ROS2 topics,
validates messages against per-stream contracts, and writes AIRS-standard HDF5 episode files.

## Launch

```bash
cd data_collection_service
source /opt/ros/humble/setup.bash
bash run_global_config.sh
```

`global_config.yaml` points to the active session YAML, output directory, and UI port.
Edit it to switch sessions or change the dashboard port.

## Prerequisites

```bash
pip install numpy h5py pyyaml
```

ROS2 Humble required for live recording (mock session mode works without it).

## Quick Start

Collect from live ROS2 topics (with mock data for testing):

```bash
cd data_collection_service
source /opt/ros/humble/setup.bash

# Terminal 1 — mock publishers (simulate hardware)
PYTHONPATH="core:tools" python3 tools/dev_mock_ros2_publishers.py \
  --config config/session_vr_ik_robot_button_control.yaml

# Terminal 2 — collector
DATA_COLLECTION_SERVICE_ROOT=$(pwd) ros2 launch launch/platform_collection.launch.py \
  session_config:=config/session_vr_ik_robot_button_control.yaml
```

Collect from real hardware — just skip the mock publisher:

```bash
cd data_collection_service
source /opt/ros/humble/setup.bash
DATA_COLLECTION_SERVICE_ROOT=$(pwd) ros2 launch launch/platform_collection.launch.py \
  session_config:=config/session_vr_ik_robot_button_control.yaml
```

Pure Python mode (no ROS2, generates synthetic HDF5 file):

```bash
cd data_collection_service
PYTHONPATH="core:tools" python3 tools/dev_mock_session.py \
  --config config/session_vr_ik_robot_button_control.yaml --record
```

Recording dashboard at `http://localhost:8765`.

Validate output:

```bash
PYTHONPATH="core:tools" python3 tools/validate_dataset.py data/episodes/<episode>.h5
```

Run tests:

```bash
PYTHONPATH="core:tests:tools" python3 -m pytest tests/ -q
```

## Architecture

```
Session YAML → SessionConfig → AdapterRegistry.resolve() → ROS2 subscriptions
                                                              |
ROS2 messages → adapter.adapt() → boundary validate → WriterSample
                                                              |
                                        recording gate (is_recording?)
                                                              |
                                        chunked batch buffer → HDF5 flush
```

Seven independently testable modules. Most unit tests run without ROS2.

### config/ — Session YAML Loader

**What it does**: Reads a YAML file from disk and turns it into a frozen Python object
that the rest of the system can rely on without re-validating.

**Why it exists**: Before this module, stream definitions were scattered across Python
files as string literals (`"joint_angles"`, `"/robot_joint_angles"`). Changing a topic
name meant editing code. Now the YAML file is the sole source of truth — change the YAML,
restart the collector, done. No code edit needed.

**Key file**: `session_config.py` (688 lines) — parses every field of the session YAML,
validates QoS enums, checks that `image_encoding` is only set on image streams, rejects
duplicate mapping keys, and produces an immutable `SessionConfig` with `StreamConfig`
entries. If parsing fails, the error message tells you exactly which field is wrong.

### contracts/ — Shared Data Models

**What it does**: Defines the data structures that every other module agrees on. Like a
dictionary that all modules share, so they don't need to know about each other.

**Why it exists**: When the adapter produces a `WriterSample` and the node shell passes
it to the writer, neither side needs to know what the other does internally. They only
need to agree on the shape of `WriterSample`. These shared types are the glue between modules.

**Key types**:
- `WriterSample` — a flattened data point ready for storage. Has exactly three fields: `stream_name`, `timestamp_ns`, and either `values` (tuple of floats) or `image_data` (bytes). The node shell checks which one is set and calls the matching writer method.
- `SourceFamily` — enum of `teleop`, `robot`, `sensor`. Determines which adapter binding to use.
- `StreamStatus` — enum of `healthy`, `degraded`, `stale`, `absent`, `invalid`. Used by the dashboard to show per-stream health.
- `RecordingState` — enum of `IDLE`, `RECORDING`, `SAVING`, `ABORTING`. The state machine's possible states.

### adapters/ — Message Extraction & Flattening

**What it does**: Takes a raw ROS2 message (a `PoseStamped`, a `JointState`, an `Image`,
etc.) and extracts the values declared in the YAML `fields` contract into a flat list of
numbers or bytes that the writer can store.

**Why it exists**: This is the key innovation. In the old system, extracting data from
each message type required handwritten callback code with hardcoded field names:

```python
# Old way — new message type = new callback
def _joint_angles_callback(self, msg):
    data = list(msg.data)  # only works for Float32MultiArray
```

The new adapter is generic — it reads the YAML `fields` list and walks the message
structure to find each declared path. Adding a new stream type with an existing message
type requires zero code. Adding a new message type requires only a small payload builder
function; the flattening code is untouched.

**Key files**:
- `registry.py` — maps `(source, message_type)` pairs to adapter bindings. Each binding knows which payload builder to call and which profile to validate against. The `resolve_session()` method wires up the entire session in one call.
- `common.py` — the generic flattener `_writer_sample_from_payload()`. Recursively walks a validated dict, collects all numeric leaf values in sorted key order, and produces a `WriterSample`. The decision of vector vs. image is made by checking whether the `"data"` key holds bytes — no per-type `if/elif` dispatch.

### schema/ — Payload Profiles & Validation

**What it does**: Defines what a valid message looks like for each payload shape, and
checks every incoming message against those rules before it reaches the writer.

**Why it exists**: ROS2 delivers messages that match the declared type, but it doesn't
check that the *values* are sensible. A `PoseStamped` could have NaN positions or a
quaternion with zero norm. The boundary validator catches these before they land in the
HDF5 file, producing structured errors with the exact field path and reason.

**Key files**:
- `adapter_profiles.py` — defines `AdapterPayloadProfile` entries for each payload shape (teleop_pose, robot_joint_positions, sensor_rgb_image, etc.). Each profile lists which fields must exist, whether they are numeric or binary, and whether they are required.
- `boundary_validator.py` — checks every field in the `AdapterBoundarySample` against its profile: field presence, type match, numeric finiteness (no NaN/Inf), sequence length. Returns a validated sample or raises `AdapterBoundaryValidationError` with a structured error.

### runtime/ — Node Shell & Control

**What it does**: The operational layer. Creates ROS2 subscriptions, runs the ingest loop,
manages recording state, serves the browser dashboard, and tracks per-stream health.

**Why it exists**: Each concern is a separate class, so they can be tested independently
and composed in the node shell. The node shell itself is thin (~264 lines) — it wires
modules together but owns no logic.

**Key files**:
- `ros2_collection_node.py` — the main entry point. Creates subscriptions from the session config, routes incoming messages through the adapter pipeline, and passes `WriterSample` results to the writer. All message-type knowledge lives in the adapter; the node shell just calls `adapter.adapt()` and checks `sample.image_data is not None`.
- `recording_state.py` — a simple state machine with four states: IDLE, RECORDING, SAVING, ABORTING. All three control modes (service, UI, device binding) call the same `start_episode()` / `end_episode()` / `abort_episode()` methods. The state machine owns the truth; everything else reads it.
- `recording_control.py` — routes control commands from three possible sources (ROS2 services, browser buttons, teleop device messages) to the state machine. Each source is gated by `control_mode` in the session YAML — only the active mode's commands are accepted.
- `manual_operator_ui.py` — a stateless HTTP server that pushes recording state and per-stream health to the browser every 200ms via Server-Sent Events. Buttons (Start/Stop/Save/Abort) call back into the control router. The UI reads a snapshot; it owns no state.
- `stream_tracker.py` — counts messages received per stream, tracks valid/invalid ratios, computes observed rate, and classifies each stream as healthy/degraded/stale/absent/invalid. Reset per episode.

### storage/ — AIRS HDF5 Writer

**What it does**: Writes data to HDF5 files in the AIRS standard format, keeping memory
usage constant regardless of how long the episode runs.

**Why it exists**: A naive writer that accumulates all data in memory before writing would
crash on long episodes. This writer uses chunked batches — every 50 vector frames or 20
image frames, it resizes the HDF5 dataset and writes the batch contiguously, then clears
the in-memory buffer. Memory usage stays flat whether the episode is 10 seconds or 10 hours.

**Key file**: `airs_hdf5_writer.py` — creates HDF5 groups and datasets eagerly at episode
open (so the file structure is visible even during recording), appends data in small batches
via incremental `resize()` + write, and finalizes attributes (frame count, sample rate,
column names, encoding) at episode close.

### validation/ — Output Validator

**What it does**: After recording finishes, opens the HDF5 file and checks that it conforms
to the AIRS standard — correct structure, correct dtypes, matching dimensions between data
and timestamps, valid JPEG magic bytes for image streams.

**Why it exists**: Bugs in the recording pipeline (dropped messages, adapter errors, writer
crashes) can produce subtly malformed files that downstream training code chokes on. The
validator catches these before the file leaves the collection machine.

**Key file**: `dataset_validator.py` — checks root attributes, iterates every group,
verifies `type` attr is `vector` or `image`, checks data/timestamp shape consistency,
validates JPEG magic bytes for JPEG-encoded image streams. Returns a structured report
with errors and warnings.

## Session YAML Format

The session YAML is the sole source of truth. Every stream is a 1:1 contract with one ROS2 topic.

```yaml
session_name: "my_session"
recording:
  control_mode: manual_ui       # service | manual_ui | device_binding
  operator_ui_port: 8765
storage:
  output_dir: "data/episodes"
  episode_id_prefix: "ep"

streams:
  - name: "arm_joints"
    source: robot
    topic: "/arm/joint_states"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "position"
        type: sequence
        required: true
```

### Supported Message Types

| Message Type | Output |
|-------------|--------|
| `geometry_msgs/PoseStamped` | Vector: position(xyz) + orientation(xyzw) = 7 dims |
| `sensor_msgs/JointState` | Vector: position + velocity + effort |
| `std_msgs/Float32MultiArray` | Vector: data(N) |
| `sensor_msgs/Image` | Image: raw bytes or JPEG |
| `sensor_msgs/PointCloud2` | Vector: flattened points |
| `sensor_msgs/Imu` | Vector: orientation(4) + angular_vel(3) + linear_accel(3) |

Adding a new message type requires only a payload builder function and a profile entry —
no changes to the flattening pipeline. See [adapter contract guide](../../memodocs/adapter_contract_guide.md).

## Recording Control

Three modes, all driving a single state machine:

```bash
# Manual UI (browser dashboard)
--operator-ui-port 8765

# ROS2 Service (tools/ scripts)
ros2 service call /data_collection/start std_srvs/srv/Trigger {}

# Device Binding (teleop button triggers recording)
# Configure in session YAML: control_mode: device_binding
```

## HDF5 Output (AIRS Standard)

```
ep_20260525_143022.h5
├── .attrs {description, robot_type, series_number, sample_rate, frames}
├── /arm_joints/
│   ├── .attrs: {type: "vector", frames: 566, columns: '["dim_0",...]'}
│   ├── data        (566, D) float32
│   └── timestamps  (566,)  uint64
└── /camera_color/
    ├── .attrs: {type: "image", frames: 566, encoding: "jpeg"}
    ├── data        (566,)  vlen uint8
    └── timestamps  (566,)  uint64
```

## Project Structure

```
data_collection_service/
├── config/          # Session YAML profiles
├── launch/                  # ROS2 launch file
├── tools/                   # Mock publishers, dataset validator
├── tests/                   # 55 tests (unit + integration)
├── README.md
└── core/                    # Python package
    ├── adapters/            # Generic message extraction
    ├── config/              # SessionConfig loader
    ├── contracts/           # WriterSample, enums, models
    ├── runtime/             # ROS2 node, state machine, SSE UI, tracker
    ├── schema/              # Payload profiles + boundary validator
    ├── storage/             # Chunked AIRS HDF5 writer
    └── validation/          # Post-collection HDF5 validator
```

## Performance Ceilings

Single machine, SHM transport:

| Limit | Value |
|-------|-------|
| Messages/sec | 3,000+ tested |
| Streams/session | 10 tested |
| Image size | 10 MB at 30 Hz |
| HDF5 write speed | 452 MB/s |

## Reference

- [Root README](../../README.md) — architecture overview, ROS2 topic contract
- [Robot Interface](../robot_interface/README.md) — JointState + PoseStamped convention
- [Sensor Interface](../sensor_interface/README.md) — Image + CameraInfo convention
- [Teleoperation Interface](../teleoperation_interface/README.md) — PoseStamped + Float32MultiArray convention
