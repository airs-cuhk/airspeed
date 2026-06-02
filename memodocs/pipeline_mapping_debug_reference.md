# Data Collection Service — Pipeline Mapping Debug Reference

**Date**: 2026-05-20
**Full pipeline**: ROS2 topic → subscription callback → adapter decode → boundary validate → per-stream buffer → HDF5 flush

---

## HOP 0: Session Config Loading

**dimension space**: YAML on disk → frozen Python dataclasses

```
src/data_collection_service/config/session_config.py:322-340
```
```python
def load_session_config(config_path: str | Path, *, relative_path_root: Path | None = None) -> SessionConfig:
    resolved = Path(config_path).expanduser()
    if relative_path_root is not None and not resolved.is_absolute():
        resolved = (relative_path_root / resolved).resolve()
    else:
        resolved = resolved.resolve()
    if not resolved.exists():
        raise SessionConfigError(f"session config file not found: {resolved}")
    raw = yaml.load(resolved.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    if not isinstance(raw, Mapping):
        raise SessionConfigError("session config must be a YAML mapping at the top level")
    return _parse_config(raw, source_path=str(resolved))
```

**latex**: N/A (structural — YAML parsing with duplicate-key rejection, no mathematical transform)

**explanation**: Loads the session YAML from disk using a custom `_UniqueKeyLoader` that rejects duplicate mapping keys (silent overwrite prevention). Parses into frozen `SessionConfig` dataclass with `StreamConfig` entries, each carrying `FieldRule` tuples defining the per-stream canonical contract. Validation happens at load time — a loaded `SessionConfig` is guaranteed valid. Source of truth: `config/session/profile_mixed.yaml` (the canonical contract file).

### what it does

Receives a filesystem path to a YAML file. Reads and parses the file, rejecting duplicates. Produces an immutable `SessionConfig` object containing all stream definitions, field contracts, QoS settings, storage configuration, and recording control mode. If parsing fails or validation rules are violated, raises `SessionConfigError` with an actionable message. The caller receives a frozen configuration object that requires no further validation.

---

## HOP 1: Adapter Registry Resolution

**dimension space**: `SessionConfig.streams` → `dict[str, ConfiguredStreamAdapter]`

```
src/data_collection_service/adapters/registry.py:113-115
```
```python
def resolve_session(self, config: SessionConfig) -> dict[str, ConfiguredStreamAdapter]:
    return {name: self.resolve(stream) for name, stream in config.streams}
```

**latex**: N/A (structural — lookup-based mapping with uniqueness constraint)

**explanation**: For each stream in the session config, finds the matching `AdapterBinding` by `(source, message_type)` signature. Each binding declares a payload profile name and a payload builder function. The resolved `ConfiguredStreamAdapter` pairs the stream config with its binding and the boundary validator. If multiple bindings match the same signature, resolution fails with an ambiguity error — the config must disambiguate. If no binding matches, resolution fails with a list of supported bindings. Source of truth: the 5 default bindings in `registry.py:_default_bindings()` plus any user-registered bindings.

### what it does

Takes a session config with N streams. For each stream, looks up its `(source, message_type)` pair in the adapter registry. Produces a dictionary mapping stream name to a `ConfiguredStreamAdapter` that knows how to decode that stream's ROS2 messages into canonical payloads and validate them against the declared profile. Fails early if any stream has no matching adapter or has ambiguous matches.

---

## HOP 2: ROS2 Subscription Creation

**dimension space**: YAML stream config → live ROS2 subscriptions with QoS

```
src/data_collection_service/runtime/ros2_collection_node.py:111-119
```
```python
def _create_subscriptions(self) -> None:
    for name, stream in self._config.streams:
        msg_cls = _SUPPORTED_MESSAGE_TYPES.get(stream.message_type)
        if msg_cls is None:
            self.get_logger().warn(f"skipping {name}: unsupported type {stream.message_type}")
            continue
        qos = _qos_profile_from_stream(stream)
        self.create_subscription(msg_cls, stream.topic, self._make_handler(name), qos)
        self.get_logger().info(f"  {name} → {stream.topic}")
```

**latex**: N/A (structural — ROS2 DDS subscription registration)

**explanation**: Iterates the session config's streams and creates one ROS2 subscription per stream. Message class is resolved from `_SUPPORTED_MESSAGE_TYPES` (`PoseStamped`, `Float32MultiArray`, `Image`). QoS profile is translated from the YAML config's normalized enum values to `rclpy.qos.QoSProfile` with matching reliability, durability, history, and depth. If a stream's message type is not in the supported set, the stream is skipped with a warning. Each subscription gets a closure handler that routes to `_handle_message(stream_name, msg)`. Source of truth: `config/session/profile_mixed.yaml` and `_SUPPORTED_MESSAGE_TYPES`.

### what it does

Reads the N stream entries from the loaded session config. For each stream with a supported ROS2 message type, creates a DDS subscription on the declared topic with the declared QoS profile. Binds each subscription to a callback identified by stream name. Streams with unsupported message types are skipped with a logged warning. The ROS2 node begins receiving messages on all subscribed topics immediately.

---

## HOP 3: Ingress Callback → Adapter Adapt → WriterSample

**dimension space**: Raw ROS2 message → `WriterSample` (flattened, validated, writer-ready)

```
src/data_collection_service/runtime/ros2_collection_node.py:139-141
```
```python
now = datetime.now(timezone.utc)
sample = adapter.adapt(msg, received_at=now)
self._stream_tracker.record_valid(stream_name, sample.timestamp_ns)
```

```
src/data_collection_service/adapters/registry.py:73-84
```
```python
def adapt(self, message: Any, *, received_at: datetime) -> WriterSample:
    payload = self.binding.payload_builder(message, self.stream)
    sample = build_boundary_sample(
        self.stream, payload=payload, received_at=received_at,
        source_timestamp=extract_header_timestamp(message),
    )
    try:
        validated = self._validator.validate_sample(sample, profile_name=self.binding.payload_profile)
    except AdapterBoundaryValidationError as exc:
        raise AdapterError(str(exc)) from exc

    ts_ns = int(validated.timestamp.timestamp() * 1e9)
    return _writer_sample_from_payload(
        self.stream.name, validated.payload, ts_ns,
    )
```

The flattening is fully generic — `_writer_sample_from_payload()` walks the
validated payload dict and produces a `WriterSample` without knowing the message
type:

```
src/data_collection_service/adapters/registry.py:138-158
```
```python
def _writer_sample_from_payload(stream_name, payload, ts_ns) -> WriterSample:
    data_bytes = payload.get("data")
    if isinstance(data_bytes, (bytes, bytearray)):
        return WriterSample(stream_name=stream_name, timestamp_ns=ts_ns,
                            image_data=bytes(data_bytes))
    values = _collect_scalars(payload)
    return WriterSample(stream_name=stream_name, timestamp_ns=ts_ns,
                        values=tuple(values))

def _collect_scalars(obj):
    result = []
    if isinstance(obj, dict):
        for _key in sorted(obj):
            result.extend(_collect_scalars(obj[_key]))
    elif isinstance(obj, (list, tuple)):
        for item in obj:
            result.extend(_collect_scalars(item))
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        result.append(float(obj))
    return result
```

**latex**:
- Image: if `payload["data"]` is `bytes` → `WriterSample(image_data=data, timestamp_ns=ts_ns)`
- Vector: otherwise → `v = \bigcup_{leaf\ in\ payload} float(leaf)`, collected depth-first in sorted key order
- `ts_ns = \lfloor t_{validated}.timestamp() \times 10^9 \rfloor`

**explanation**: This is the **adapter boundary**. The adapter extracts fields via a per-type payload builder, validates against the YAML contract, then calls the generic `_writer_sample_from_payload()` which recursively walks the validated dict and collects all scalar values into a flat tuple. The decision of image vs. vector is made by checking whether the `"data"` key holds bytes — no per-message-type dispatch. Adding a new message type requires only a payload builder and a profile; the flattening code is untouched. Source of truth: the stream's `FieldRule` definitions in the session YAML and the binding's `payload_profile` in `adapter_profiles.py`.

### what it does

Receives a raw ROS2 message from a subscription callback. Looks up the pre-resolved adapter. Calls the payload builder to extract canonical fields into a dict, wraps them in an `AdapterBoundarySample`, validates against the YAML contract, converts the timestamp to uint64 nanoseconds, and calls the generic `_writer_sample_from_payload()` which walks the payload dict and produces a `WriterSample` — image if the payload contains byte data, vector otherwise. The node shell does not open the `WriterSample` envelope.

---

## HOP 4: Recording Gate

**dimension space**: `AdapterBoundarySample` → either discarded (if not recording) or passed through

```
src/data_collection_service/runtime/ros2_collection_node.py:133-134
```
```python
if not self._state_machine.is_recording:
    return
```

**latex**: `g(sample) = sample if state == RECORDING else None`

**explanation**: The recording state machine gates all writes. If no episode is active (`is_recording == False`), the validated sample is silently discarded. This is the **single control switch** — all three control modes (service, manual_ui, device_binding) toggle the same `RecordingStateMachine.is_recording` flag. Source of truth: `recording_state.py:RecordingStateMachine`.

### what it does

Checks whether an episode is currently being recorded. If not, discards the validated sample without writing. If yes, passes the sample through to the writer. This is the only gating point in the entire pipeline — there is no sync window, no required-stream check, no stale-sample rejection.

---

## HOP 5: Writer Pass-Through

**dimension space**: `WriterSample` → writer append call (node shell does not inspect)

```
src/data_collection_service/runtime/ros2_collection_node.py:143-147
```
```python
if sample.image_data is not None:
    self._writer.append_image(stream_name, sample.image_data, sample.timestamp_ns)
elif sample.values is not None:
    self._writer.append_vector(stream_name, sample.values, sample.timestamp_ns)
```

**latex**: N/A (structural — the node shell is a pass-through; all flattening happened in HOP 3)

**explanation**: The node shell receives a `WriterSample` from the adapter and routes it to the writer. It does not know about message types, payload shapes, or field names. It checks whether `image_data` or `values` is set and calls the corresponding writer method. This is the entire ingest logic — 4 lines. Source of truth: `WriterSample` contract in `contracts/writer_sample.py`.

### what it does

Receives a WriterSample from the adapter. Checks which field is populated (image_data or values). Calls the writer's append method with the stream name, data, and timestamp. Does not open, inspect, or transform the payload.

---

## HOP 6: Per-Stream Append Buffer

**dimension space**: Single sample → buffered `list[ndarray]` + `list[uint64]`

```
src/data_collection_service/storage/airs_hdf5_writer.py:153-160
```
```python
class _VectorBuffer:
    def append(self, values: object, timestamp_ns: int) -> None:
        arr = np.asarray(values, dtype=np.float32)
        if arr.ndim == 0:
            arr = arr.reshape(1)
        if arr.shape[-1] != self.dims and arr.size == self.dims:
            arr = arr.reshape(1, self.dims)
        self._data.append(arr.ravel().astype(np.float32))
        self._timestamps.append(np.uint64(timestamp_ns))
```

```
src/data_collection_service/storage/airs_hdf5_writer.py:185-191
```
```python
class _ImageBuffer:
    def append(self, raw_data: bytes, timestamp_ns: int) -> None:
        if self._reencode:
            raw_data = _reencode_jpeg(raw_data)
        self._frames.append(raw_data)
        self._timestamps.append(np.uint64(timestamp_ns))
```

**latex**:
- Vector: `D_i = D_{i-1} \cup \{v_i\}, T_i = T_{i-1} \cup \{t_i\}`
- Image: `F_i = F_{i-1} \cup \{b_i\}, T_i = T_{i-1} \cup \{t_i\}` where `b_i` may be JPEG-re-encoded from raw

**explanation**: Each stream has a private in-memory batch buffer. Vector buffers normalize input to `float32` and auto-detect dimension on the first frame (if `dims=0`). Image buffers optionally re-encode to JPEG. When the batch reaches its threshold (50 frames for vector, 20 for image), `_flush_batch()` writes the batch to the HDF5 dataset via incremental `resize()` + contiguous slice write. This keeps memory constant regardless of episode length — only one batch is in memory at a time. The HDF5 datasets were created at `register_*_stream()` time (eager creation), so `_flush_batch()` only resizes and writes, never creates. Source of truth: AIRS standard — `data` dataset shape `(N, dims)` or `(N,)`, `timestamps` dataset shape `(N,)`.

### what it does

Receives one flattened sample. For vectors: coerces to float32, auto-sizes dimension on first frame, appends to a small in-memory batch. For images: optionally JPEG-re-encodes, appends to batch. When the batch fills (50 vector / 20 image frames), calls `_flush_batch()` to resize the HDF5 dataset and write the batch contiguously, then clears the in-memory buffer. This capped-buffer design keeps memory usage constant regardless of episode length.

---

## HOP 7: Episode Close — Flush Remaining + Finalize Attrs

**dimension space**: In-memory batch buffer → HDF5 dataset resize + write + finalize attrs

```
src/data_collection_service/storage/airs_hdf5_writer.py:198-207
```
```python
# _VectorBuffer.flush_remaining:
def flush_remaining(self, file: h5py.File) -> None:
    self._flush_batch()
    self._grp.attrs["frames"] = self._total
    self._grp.attrs["sample_rate"] = float(file.attrs.get("sample_rate", 0.0))
    if "columns" not in self._grp.attrs:
        self._grp.attrs["columns"] = json.dumps(
            [f"dim_{j}" for j in range(self._dims)])
```

```
src/data_collection_service/storage/airs_hdf5_writer.py:240-244
```
```python
# _ImageBuffer.flush_remaining:
def flush_remaining(self, file: h5py.File) -> None:
    self._flush_batch()
    self._grp.attrs["frames"] = self._total
    self._grp.attrs["sample_rate"] = float(file.attrs.get("sample_rate", 0.0))
    self._grp.attrs["camera_name"] = self.name
```

**latex**:
- Vector: `D = \text{stack}([v_0, \ldots, v_{N-1}]) \in \mathbb{R}^{N \times d}, T = [t_0, \ldots, t_{N-1}] \in \mathbb{Z}_{uint64}^{N}`
- Image: `D = [b_0, \ldots, b_{N-1}], T = [t_0, \ldots, t_{N-1}]` where each `b_i` is variable-length `uint8`

**explanation**: At episode close, `_flush_batch()` writes any remaining buffered frames to the HDF5 file (incremental `resize()` + write, matching the hot-path behavior of HOP 6). Then `flush_remaining()` finalizes group attributes: `frames` count, per-group `sample_rate` (matching AIRS standard), and stream-specific metadata — column names for vectors (auto-generated if not set at registration), and `camera_name` for image streams (derived from the YAML stream name). The HDF5 group and datasets were already created at `register_*_stream` time (HOP 3.5), so close only flushes remaining data and writes attrs. Source of truth: AIRS standard `buildin.hdf5` format.

### what it does

Flushes any remaining buffered frames from the in-memory batch to the HDF5 dataset via incremental resize+write. Finalizes per-group attributes: frame count, sample_rate, column names (for vectors), and camera_name (for images, from the YAML stream name). Does not create groups or datasets — those were created eagerly at stream registration time when the episode was opened.

---

## HOP 8: Output Validation

**dimension space**: HDF5 file on disk → `ValidationReport {is_valid, errors[], warnings[]}`

```
src/data_collection_service/validation/dataset_validator.py:48-69
```
```python
def validate_dataset(dataset_path: str | Path) -> ValidationReport:
    path = str(dataset_path)
    report = ValidationReport(dataset_path=path)
    try:
        f = h5py.File(path, "r")
    except Exception as exc:
        report.is_valid = False
        report.errors.append(ValidationIssue("/", f"cannot open file: {exc}"))
        return report
    with f:
        _validate_root_attrs(f, report)
        for group_name in f:
            grp = f[group_name]
            if not isinstance(grp, h5py.Group):
                continue
            if "type" not in grp.attrs:
                report.errors.append(ValidationIssue(f"/{group_name}", "missing 'type' attr"))
                report.is_valid = False
                continue
            gtype = str(grp.attrs["type"])
            if gtype == "vector":
                _validate_vector_group(f, group_name, report)
            elif gtype == "image":
                _validate_image_group(f, group_name, report)
    return report
```

**latex**: N/A (structural validation — checks shape, dtype, attribute presence, JPEG magic bytes)

**explanation**: Post-collection validation verifies that the HDF5 file conforms to the AIRS standard. Root attrs (`description`, `robot_type`, `series_number`, `sample_rate`, `frames`) must be present. Each group must have a `type` attr of `vector` or `image`. Vector groups must have `(N, dims) float32` data and `(N,) uint64` timestamps with matching first dimensions, plus a `columns` attr. Image groups must have `(N,)` data and `(N,) uint64` timestamps with matching lengths; if `encoding=jpeg`, the first frame must start with JPEG magic bytes `0xFFD8`. Source of truth: AIRS standard `buildin.hdf5` format and spec `data-collection-service/.memo/memodocs/spec_data_collection_service.md`.

### what it does

Opens an AIRS-standard HDF5 file. Checks root attributes for presence. Iterates every top-level group, verifying it has a type attribute of "vector" or "image". For vector groups: checks data is 2D float32, timestamps is 1D uint64, and their first dimensions match. For image groups: checks data is 1D, timestamps is 1D uint64, lengths match, and JPEG magic bytes if encoding is "jpeg". Returns a structured report with errors and warnings lists.

---

## Summary: Data Collection Service Pipeline

### Main Pipeline (controls data collection):
```
ROS2 Topic (/leader_pose, /front_rgb, ...)     [raw ROS2 message objects]
    |
    v HOP 0: YAML → SessionConfig               [frozen dataclasses, validated]
    |
    v HOP 1: config → AdapterRegistry.resolve   [dict[name, ConfiguredStreamAdapter]]
    |
    v HOP 2: create_subscription() per stream   [live ROS2 subscriptions with QoS]
    |
    v HOP 3: msg → adapter.adapt()              [WriterSample — flattened, validated, ts_ns]
    |
    v HOP 4: is_recording?                      [gate: discard if IDLE]
    |
    v HOP 5: WriterSample → writer.append_*()   [pass-through, node shell does not inspect]
    |
    v HOP 6: append to chunked batch buffer     [batched in memory, flushed every 20-50 frames]
    |
    v HOP 7: episode close → resize + flush     [contiguous (N,dims) float32 + (N,) uint64]
    |
    +--> /<stream>/data                          [AIRS vector or image group]
    |
    +--> /<stream>/timestamps                    [AIRS timestamps]
    |
    +--> /<stream>/attrs (type, columns, ...)   [AIRS group metadata]
    |
    v HOP 8: validate_dataset()                  [ValidationReport]
```

### Recording Control (side channel):
```
manual_ui button ───┐
CLI service call ───┼──→ RecordingControlRouter.invoke_action("start")
teleop binding ─────┘              │
                                   v
                         RecordingStateMachine.start_episode()
                                   │
                                   v
                         AirsHdf5Writer.open_episode(<id>.h5)
                                   │
                                   v
                         state = RECORDING  (HOP 4 gate opens)
```

### Key Source-of-Truth Documents:
1. **AIRS Standard Spec** (`AIRS_数据采集平台数据接口规范_v1.0.docx`): HDF5 layout — root attrs, per-stream groups, `data[N]` + `timestamps[N]`, `type` attr, `columns` for vector, `width/height/channels/encoding` for image
2. **Reference buildin.hdf5**: Concrete example at `airs-standard-asset/20260506-1005-1010.hdf5.buildin.hdf5` — 24 groups, 48 datasets, 566 frames
3. **Service Spec** (`spec_data_collection_service.md`): Architecture — dumb recorder, no sync, AIRS-native output, single state machine, YAML as canonical contract
4. **Session YAML** (`config/session/profile_mixed.yaml`): Per-stream contracts — field paths, types, required flags, image encoding
5. **Adapter Profiles** (`src/data_collection_service/schema/adapter_profiles.py`): Payload field rules for teleop_pose, teleop_buttons, robot_joint_positions, robot_pose, sensor_rgb_image
6. **Coding Rules** (`.claude/rules/`): File length limits (500 src, 600 test, 400 tool), module separation, one class per module, artifact isolation
