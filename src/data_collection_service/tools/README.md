# Data Collection Service — Tools

All tools in this directory resolve paths from the project root via `__file__`, so they work from any CWD.

---

## Data Generation (Dev / Testing)

### `dev_mock_messages.py`
Shared mock message factories for offline development. Builds synthetic ROS2-message-shaped objects (`PoseStamped`, `JointState`, `Image`, `Float32MultiArray`) without requiring live ROS2 topics, hardware, or `rclpy`. Used by both `dev_mock_session.py` and the test suite.

```bash
# Not run directly — imported as a library
python -c "from tools.dev_mock_messages import make_pose_stamped; print(make_pose_stamped())"
```

### `dev_mock_session.py`
Generates a valid AIRS-standard HDF5 episode file with synthetic mock data. No ROS2 or hardware required. Useful for testing converters, validating the HDF5 schema, and offline development.

```bash
PYTHONPATH="core:tools:$PYTHONPATH" python tools/dev_mock_session.py \
  --config config/session_vr_ik_robot_button_control.yaml \
  --record
```

### `dev_mock_ros2_publishers.py`
Publishes synthetic mock data on real ROS2 topics. The data collection service can subscribe to these topics without real hardware. Useful for integration testing the full ROS2→HDF5 pipeline.

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="core:tools:$PYTHONPATH" python tools/dev_mock_ros2_publishers.py \
  --config config/session_vr_ik_robot_button_control.yaml
```

---

## Validation

### `validate_dataset.py`
CLI wrapper for AIRS HDF5 dataset validation. Checks root attributes, per-group structure, data types, dimension consistency, and JPEG magic bytes. Returns structured pass/fail report.

```bash
python tools/validate_dataset.py data/episodes/episode-20260521T073812317565Z.h5
python tools/validate_dataset.py data/episodes/episode.h5 --json
```

### `validate_conversion.py`
Cross-format integrity checker. Compares converted outputs (JSONL, Parquet, Zarr, LeRobot) against the source HDF5. Verifies row counts, float32 tolerance (1e-6), JPEG byte identity, and timestamp monotonicity.

```bash
python tools/validate_conversion.py              # all formats
python tools/validate_conversion.py --format parquet  # single format
```

---

## Post-Storage Conversion

All converters read AIRS `.h5` files and write standard formats. Each tool runs inline validation automatically after conversion.

| Tool | Input | Output | Time | Best For |
|------|-------|--------|------|----------|
| `convert_h5_to_jsonl.py` | Single `.h5` | `.jsonl` (one JSON per line) | ~0.3s | Debugging, inspection, simple scripts |
| `convert_h5_to_parquet.py` | Single `.h5` | 16 `.parquet` files (Zstd) | ~1s | Analytics, Pandas/DuckDB queries |
| `convert_h5_to_zarr.py` | Single `.h5` | `.zarr` store (Blosc/Zstd) | ~2s | Cloud, multi-GPU, HDF5-like hierarchy |
| `convert_h5_to_lerobot.py` | Single `.h5` | LeRobot v3 (Parquet + H.264 MP4) | ~2.6 min | PyTorch training, HF Hub sharing |
| `convert_folder.py` | Directory of `.h5` | Consolidated `.parquet` per stream | varies | Batch processing, merged datasets |

### Usage

```bash
# Single-stream JSONL (fastest, ~0.3s)
python tools/convert_h5_to_jsonl.py --streams vr_head_pose

# All streams as unified JSONL
python tools/convert_h5_to_jsonl.py

# Parquet (per-stream, ~1s)
python tools/convert_h5_to_parquet.py

# Zarr (preserves hierarchy, ~2s)
python tools/convert_h5_to_zarr.py

# LeRobot v3 (ML-ready, ~2.6 min)
python tools/convert_h5_to_lerobot.py --fps 60 --vcodec h264

# Batch folder consolidation
python tools/convert_folder.py /path/to/episodes/
```

### The Async Stream Challenge

AIRS HDF5 files contain **16 streams at 4 independent frame rates** with per-stream timestamps:

| Rate | Frame Count | Streams |
|------|-------------|---------|
| ~60 Hz | 1,119 | vr_head_pose, vr_left_pose, vr_right_pose, vr_left_buttons, vr_right_buttons |
| ~50 Hz | 820 | ik_left_arm, ik_right_arm, ik_left_gripper, ik_right_gripper |
| ~20 Hz | 358 | arm_left_joints, arm_right_joints, arm_left_gripper, arm_right_gripper |
| ~10 Hz | 179 | camera_head, camera_left_wrist, camera_right_wrist |

Timestamps are `uint64` nanosecond epoch, **NOT frame-aligned** across streams. Each converter handles this differently:
- **JSONL / Parquet / Zarr**: Per-stream files, preserving native timestamps
- **LeRobot v3**: Resamples to canonical VR 60Hz timeline with nearest-timestamp matching

### Format Comparison

| Property | JSONL | Parquet | Zarr | LeRobot v3 |
|----------|-------|---------|------|------------|
| Human-readable | Yes | No | No | No |
| Compression | None | Zstd (5-10×) | Blosc/Zstd (5-10×) | H.264 video + Zstd |
| Random access | No | Yes (row groups) | Yes (chunks) | Yes (Parquet + video) |
| Multi-episode | Per-file | Consolidated | Consolidated | Consolidated |
| ML framework | Any JSON parser | Pandas/DuckDB | NumPy/PyTorch | PyTorch (native) |
| Cloud-native | Yes | Yes | Yes | Yes (HF Hub) |
| Async streams | Per-stream | Per-stream | Per-stream | Resampled to canonical |

### Paths

- **Tools**: `src/data_collection_service/tools/convert_*.py`
- **Artifacts**: `convert/test_artifacts/` (generated outputs, gitignored)
- **Schema**: `convert/test_artifacts/schema.json` (source of truth for stream metadata)

### Test Artifacts

All converters are validated against `convert/test_artifacts/schema.json`. After running, outputs land in `convert/test_artifacts/`:

```
convert/test_artifacts/
├── schema.json              # Machine-readable stream metadata
├── episode.jsonl            # Single-stream JSONL (vr_head_pose)
├── episode_full.jsonl       # All-streams unified JSONL
├── parquet/                 # 16 .parquet files
├── episode.zarr/            # 16 Zarr groups
├── lerobot_dataset/         # LeRobot v3 (meta/ + data/ + videos/)
├── consolidated_parquet/    # Batch output from convert_folder.py
└── validation_report.json   # Cross-format integrity results
```
