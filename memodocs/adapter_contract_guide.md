# Adapter Contract Guide — Canonical Rules for New Streams

**Date**: 2026-05-20
**Purpose**: Define the abstract rules that every new ROS2 topic and its
corresponding YAML stream entry must follow. This document is the source of
truth for adapter authors. No field name, type, or dimension should be invented
without tracing it to one of these rules.

---

## Core Rule

> Every stream in the session YAML is a 1:1 contract with one ROS2 topic. The
> YAML declares what the topic must provide. The adapter enforces that contract
> at runtime. If the topic does not satisfy the contract, the message is
> rejected with a structured error — never silently accepted.

---

## The Three-Layer Contract

```
Layer 1: Transport (ROS2)     — topic name, message type, QoS
Layer 2: Payload (YAML)       — field paths, types, required/optional
Layer 3: Storage (AIRS)       — group type (vector/image), columns, encoding
```

Each layer is independent. A change at one layer does not force changes at the
others unless the contract boundary itself shifts.

---

## Layer 1 — Transport Contract (ROS2)

### Rule T1: One logical signal per topic

Do not multiplex unrelated signals into one message. If a device produces both
pose and buttons, they go to separate topics.

```
Correct:   /leader/right_pose   (geometry_msgs/PoseStamped)
           /leader/right_buttons (std_msgs/Float32MultiArray)

Wrong:     /leader/right_state   (custom message with pose + buttons)
```

### Rule T2: Use existing ROS2 message types when possible

| ROS2 Message Type | Use for |
|-------------------|---------|
| `geometry_msgs/PoseStamped` | 6-DoF poses (position + orientation) |
| `std_msgs/Float32MultiArray` | 1D numeric sequences (joints, buttons, tactile, IMU) |
| `sensor_msgs/Image` | Camera images, depth maps, 2D arrays |
| `sensor_msgs/PointCloud2` | 3D point clouds (requires special handling) |
| `sensor_msgs/Imu` | IMU with orientation + angular velocity + linear acceleration |

If none of these fit, a new message type requires:
- An `AdapterBinding` in `registry.py`
- A `PayloadProfile` in `adapter_profiles.py`
- A message class entry in `ros2_collection_node.py:_SUPPORTED_MESSAGE_TYPES`

### Rule T3: QoS is explicit

Every stream declares `reliability`, `durability`, `history`, and `depth`.
Do not rely on ROS2 defaults.

```yaml
qos:
  reliability: best_effort   # or reliable
  durability: volatile       # or transient_local
  history: keep_last         # or keep_all
  depth: 10
```

### Rule T4: Time domain is explicit

```yaml
time_domain: ros_header    # use message header.stamp
time_domain: ros_receive   # use arrival time (for headerless messages)
```

`ros_header` requires the message to carry a `header.stamp`. If it does not,
the adapter must reject the message at ingest. `Float32MultiArray` has no
header — it must use `ros_receive`.

---

## Layer 2 — Payload Contract (YAML `fields`)

### Rule P1: Every field path is dot-separated from the message root

```yaml
fields:
  - path: position.x        # maps to msg.pose.position.x
    type: float64
    required: true
```

For nested structures, each leaf is a separate field entry. Do not declare
`position` as a whole — declare `position.x`, `position.y`, `position.z`
individually. This ensures validation can pinpoint which field failed.

### Rule P2: Type vocabulary is closed

| YAML `type` | Python equivalent | Use for |
|-------------|-------------------|---------|
| `float64` | `float` | Continuous values (positions, velocities, forces) |
| `float32` | `float` (stored as float32) | Same, half precision |
| `int32` | `int` | Discrete counts, indices |
| `uint32` | `int` (non-negative) | Dimensions, sizes, counters |
| `uint64` | `int` (non-negative, large) | Timestamps (nanoseconds) |
| `string` | `str` | Frame IDs, encodings, labels |
| `bytes` | `bytes` / `bytearray` | Raw image data, encoded blobs |
| `bool` | `bool` | Flags, binary states |
| `sequence` | `list` / `tuple` / `array` | 1D numeric arrays (joints, buttons) |

Do not invent new types. If a modality does not fit these types, decompose it
further (e.g., a quaternion is 4 × `float64`, not a `quaternion` type).

### Rule P3: `required: false` means the field may be absent

A missing required field causes rejection. A missing optional field is silently
accepted. Use `required: false` for fields that some devices provide and others
do not.

### Rule P4: Field order in YAML does not imply storage order

The YAML `fields` list defines the contract, not the storage layout. The storage
`columns` attr (Layer 3) defines the actual array dimension order. These must be
consistent but are declared separately.

---

## Layer 3 — Storage Contract (AIRS HDF5)

### Rule S1: Every stream maps to exactly one AIRS group type

| Payload shape | AIRS `type` | Storage |
|---------------|-------------|---------|
| 1D numeric sequence (joints, buttons, tactile) | `vector` | `(N, D) float32` |
| Flattened pose (position + orientation) | `vector` | `(N, 7) float32` |
| Multi-dimensional numeric (IMU 9-axis) | `vector` | `(N, 9) float32` |
| Image / 2D array | `image` | `(N,) vlen uint8` |
| Encoded blob (audio, point cloud) | `image` | `(N,) vlen uint8` |

### Rule S2: `columns` names every dimension of a vector

```python
columns = ["px", "py", "pz", "qx", "qy", "qz", "qw"]   # for a 7D pose
columns = ["joint_0", "joint_1", ..., "joint_D"]         # for a D-DOF arm
columns = ["ax", "ay", "az", "gx", "gy", "gz"]           # for a 6-axis IMU
```

`columns` is a JSON string array stored as an HDF5 attribute. The order in
`columns` matches the order of values in the `data` array.

### Rule S3: Image encoding is declared per stream

```yaml
streams:
  front_rgb:
    message_type: sensor_msgs/Image
    image_encoding: jpeg     # decode raw → re-encode as JPEG before storage
```

```yaml
  depth_camera:
    message_type: sensor_msgs/Image
    image_encoding: raw      # store raw bytes as-is
```

If `image_encoding` is omitted for an image stream, it defaults to `raw`.
If `image_encoding` is set on a non-image stream, config loading rejects it.

#### ⚠️ Critical: publish JPEG at the source, not just at storage

Setting `image_encoding: jpeg` tells the **collector's writer** to re-encode
raw RGB to JPEG before writing to HDF5. This reduces the HDF5 file size but
does NOT reduce the ROS2 DDS message size. The raw RGB data still travels over
DDS at full size (~6 MB for 1080p), causing DDS fragmentation.

**Theoretical concern (untested):** FastRTPS documentation states that DDS
messages larger than ~64 KB may be fragmented for UDP transport. On a single
machine, shared memory transport handles large messages without fragmentation
(tested up to 10 MB at 30 Hz with zero degradation). Cross-machine UDP
transport has not been tested.

**Recommendation**: For single-machine deployment, image size is not a concern.
For cross-machine deployment (untested), publishing JPEG at the source is the
prudent default — it reduces message size below the theoretical UDP fragment
threshold.

### Rule S4: Timestamps are always uint64 nanoseconds since Unix epoch

Regardless of the source time domain (`ros_header` or `ros_receive`), the
stored timestamp is always `uint64` nanoseconds since 1970-01-01T00:00:00Z.

---

## The 1:1 Mapping Rule

For every stream in the YAML, there must exist:

1. **One ROS2 topic** publishing the declared `message_type` on the declared
   topic name with the declared QoS.
2. **One adapter binding** registered for the `(source, message_type)` pair,
   with a payload builder that extracts the declared fields.
3. **One payload profile** defining the field rules that the boundary validator
   enforces.
4. **One AIRS group** in the output HDF5 with the declared `type`, `columns`
   (for vectors), and encoding (for images).

If any of these four is missing, the stream is not fully contracted. The
collector will detect the gap at startup (missing adapter binding) or at
ingest (missing payload profile) and fail with an actionable error.

---

## Adapter Boundary — WriterSample Contract

The adapter is the **sole owner of payload shape knowledge**. The node shell
does not inspect payloads, flatten nested dicts, or dispatch on message types.
It receives a `WriterSample` from the adapter and passes it to the writer
verbatim.

### The boundary

```
ROS2 message → adapter.adapt() → WriterSample → node shell (pass-through) → writer
```

`WriterSample` has exactly three fields:

| Field | Type | Set when |
|-------|------|----------|
| `stream_name` | `str` | Always |
| `timestamp_ns` | `int` | Always |
| `values` | `tuple[float, ...]` | Vector stream (PoseStamped, Float32MultiArray, etc.) |
| `image_data` | `bytes` | Image stream (sensor_msgs/Image) |

Exactly one of `values` or `image_data` is set. The node shell checks which one
and calls `writer.append_vector()` or `writer.append_image()` — nothing else.

### Why this matters

Before this contract, the node shell contained:

```python
if stream.message_type == "sensor_msgs/Image":
    self._writer.append_image(stream_name, result.payload["data"], ts_ns)
elif stream.message_type == "geometry_msgs/PoseStamped":
    p, o = result.payload["position"], result.payload["orientation"]
    values = [p["x"], p["y"], p["z"], o["x"], o["y"], o["z"], o["w"]]
    ...
```

This meant adding a new message type required changing the node shell. After the
contract, the node shell is:

```python
sample = adapter.adapt(msg, received_at=now)
if sample.image_data is not None:
    self._writer.append_image(stream_name, sample.image_data, sample.timestamp_ns)
elif sample.values is not None:
    self._writer.append_vector(stream_name, sample.values, sample.timestamp_ns)
```

Adding a new message type touches only the adapter layer — the node shell,
writer, validator, and state machine are unchanged.

### Adapter owns registration too

`ConfiguredStreamAdapter.register_with(writer)` tells the writer what HDF5
datasets to create. The adapter derives dimension and column names from the
YAML `fields` contract:

- **All scalar fields** (`type: float64`, etc.) → `dims = len(fields)`, columns
  are the full field paths (e.g., `position.x`, `orientation.w`)
- **Any sequence field** (`type: sequence`) → `dims = 0`, dimension
  auto-detected from the first frame at runtime
- **Image stream** → registered as image group with encoding from YAML

No hardcoded `dims=7` or `dims=3` in the core.

### Flattening is fully generic

The `_writer_sample_from_payload()` function walks the validated payload dict
and produces a `WriterSample` without knowing the message type:

```python
def _writer_sample_from_payload(stream_name, payload, ts_ns) -> WriterSample:
    data_bytes = payload.get("data")
    if isinstance(data_bytes, (bytes, bytearray)):
        return WriterSample(stream_name=stream_name, timestamp_ns=ts_ns,
                            image_data=bytes(data_bytes))
    values = _collect_scalars(payload)
    return WriterSample(stream_name=stream_name, timestamp_ns=ts_ns,
                        values=tuple(values))
```

`_collect_scalars()` recursively walks nested dicts, lists, and tuples,
collecting all numeric leaf values depth-first in sorted key order. The
decision of image vs. vector is made by checking whether the `"data"` key
holds bytes — no per-message-type `if/elif` dispatch.

This means: a new message type with YAML-declared fields needs **zero
flattening code**. The payload builder still needs to produce the canonical
dict, but once validated, the generic flattener handles the rest.

## Adding a New Modality — Abstract Procedure

Do not guess. Follow these steps in order.

### Step 1: Study the real device output

Before writing any YAML or code, capture real ROS2 messages from the device:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /new_topic --once
ros2 topic type /new_topic
ros2 topic hz /new_topic
```

Record: topic name, message type, actual field names, actual value ranges,
actual publishing rate, presence/absence of header timestamps, frame_id values.

### Step 2: Determine the source family

| Device role | `source` in YAML |
|-------------|-----------------|
| Operator input (controller, VR, joystick) | `teleop` |
| Robot state (joints, end-effector, torque) | `robot` |
| Environment observation (camera, depth, tactile, IMU, audio) | `sensor` |

Do not invent a new source family. If the device does not fit these three,
re-examine whether it is a sensor with an unusual data shape.

### Step 3: Map the payload to existing types

For each field the device produces:
- Does it match an existing `type` in the closed vocabulary (Rule P2)?
- Can it be decomposed into scalar fields (e.g., quaternion → 4 × float64)?
- Can it be stored as `vector` (numeric array) or `image` (byte blob)?

The generic flattener (`_writer_sample_from_payload`) handles the conversion
from validated dict → `WriterSample` automatically. You do not write per-type
flattening code. The YAML `fields` list defines the contract; the payload
builder produces a dict matching it; the flattener walks it generically.

If the answer is "none of the above," the modality needs a new AIRS group type
or a new YAML field type. This is an architecture change — do not proceed
without updating this contract document.

### Step 4: Write the YAML stream entry

```yaml
streams:
  <stream_name>:
    source: <teleop|robot|sensor>
    topic: <ROS2 topic name>
    message_type: <ROS2 message type>
    qos:
      reliability: <best_effort|reliable>
      durability: <volatile|transient_local>
      history: <keep_last|keep_all>
      depth: <queue depth>
    time_domain: <ros_header|ros_receive>
    frame_id: <expected TF frame>       # optional, only for PoseStamped/Image
    image_encoding: <jpeg|raw>          # optional, only for Image
    fields:
      - path: <dot.separated.field>
        type: <closed vocabulary type>
        required: <true|false>
    notes: <operational notes>          # optional
```

### Step 5: Register the adapter binding (only if new message type)

If an existing `(source, message_type)` binding already covers this payload
shape, reuse it. Otherwise add to `registry.py`:

```python
AdapterBinding(
    source="<source>",
    message_type="<message_type>",
    name="<unique_binding_name>",
    payload_profile="<profile_name>",
    payload_builder=<builder_function>,
)
```

### Step 6: Define the payload profile (only if new payload shape)

If an existing `AdapterPayloadProfile` already covers this field structure,
reuse it. Otherwise add to `adapter_profiles.py`:

```python
AdapterPayloadProfile(
    name="<profile_name>",
    source_family="<source>",
    description="<what this profile validates>",
    field_rules=(
        AdapterPayloadFieldRule("<path>", numeric=True/False, required=True/False),
        ...
    ),
)
```

### Step 7: Add message class (only if new ROS2 type)

If the ROS2 message type is not in `_SUPPORTED_MESSAGE_TYPES`, add it to
`ros2_collection_node.py`:

```python
_SUPPORTED_MESSAGE_TYPES["<new_type>"] = <ImportedMessageClass>
```

### Step 8: Verify end-to-end

Both mock tools are generic — they read the YAML fields contract and generate
data from it, no per-type code needed for existing message types:

```bash
# Pure Python record mode (no ROS2 needed)
python3 tools/dev_mock_session.py --config config/session/profile_<device>.yaml --steps 100
python3 tools/validate_dataset.py data/episodes/<episode>.h5

# ROS2 mock publishers (requires ROS2 environment)
source /opt/ros/humble/setup.bash
/usr/bin/python3.10 tools/dev_mock_ros2_publishers.py --config config/session/profile_<device>.yaml --once
```

The mock publisher's `_build_generic_message()` auto-constructs a
`SimpleNamespace` tree from the YAML `fields` — walks each dot-separated path,
creates nested objects, and assigns default values based on the declared type
(`0.0` for numeric, `""` for string, minimal JPEG for bytes, `[0.0, 0.1, 0.2]`
for sequence). For new ROS2 message types, add an import and a branch to the
known-type dispatch above the generic fallback.

---

## Contract Validation Checklist

Before declaring a new stream integrated, verify each item:

| # | Check | Pass? |
|---|-------|-------|
| 1 | Topic name matches YAML `topic` exactly | |
| 2 | Message type matches YAML `message_type` exactly | |
| 3 | QoS in YAML matches publisher QoS | |
| 4 | Header timestamp present if `time_domain: ros_header` | |
| 5 | Frame ID matches YAML `frame_id` if declared | |
| 6 | Every `fields` path resolves in the actual message | |
| 7 | Every `fields` type matches the actual value type | |
| 8 | Adapter binding exists for `(source, message_type)` | |
| 9 | Payload profile exists and all field rules pass | |
| 10 | AIRS group written with correct `type`, `columns`, `encoding` | |
| 11 | Timestamps are `uint64` nanoseconds since epoch | |
| 12 | Validator passes on the output HDF5 | |

---

## Rules for Future Formats

1. **New types must be added to this document first.** The closed vocabulary
   (Rule P2) can only be extended by amending Section "Layer 2 — Payload
   Contract" with the new type name, Python equivalent, and use case.

2. **New AIRS group types must be justified.** `vector` and `image` cover all
   current modalities. A new group type (e.g., `pointcloud`, `audio`) requires:
   - A real device that cannot be represented as vector or image
   - An amendment to this document defining the new type's storage layout
   - A validator update to recognize the new type
   - A reference implementation in `airs_hdf5_writer.py`

3. **Field names come from the device, not from convention.** Do not rename
   `header.frame_id` to `reference_frame` just because it sounds cleaner. The
   YAML `path` must match the actual ROS2 message field structure. If the
   device calls it `frame_id`, the YAML calls it `frame_id`.

4. **One modality, one profile.** Do not create a new profile for a stream that
   differs only in topic name. Profiles are defined by payload shape, not by
   which device produced the payload. Two tactile sensors with the same array
   shape share the same profile.

5. **Abstract before you instantiate.** If you are about to write a third
   nearly-identical adapter binding, stop. The common pattern should be
   abstracted into a parameterized binding or a configurable payload builder.
   Three similar bindings = one binding with parameters.
