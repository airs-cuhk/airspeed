# AIRS HDF5 Post-Storage Conversion Tools

Convert AIRS `.h5` episode files to standard formats for ML training, data analysis, and sharing.

## The Async Stream Challenge

AIRS HDF5 files contain **16 streams at 4 independent frame rates** with per-stream timestamps:

| Rate | Frame Count | Streams |
|------|-------------|---------|
| ~60 Hz | 1,119 | vr_head_pose, vr_left_pose, vr_right_pose, vr_left_buttons, vr_right_buttons |
| ~50 Hz | 820 | ik_left_arm, ik_right_arm, ik_left_gripper, ik_right_gripper |
| ~20 Hz | 358 | arm_left_joints, arm_right_joints, arm_left_gripper, arm_right_gripper |
| ~10 Hz | 179 | camera_head, camera_left_wrist, camera_right_wrist |

Timestamps are `uint64` nanosecond epoch, **NOT frame-aligned** across streams. Each converter handles this differently.

## Tools

| Tool | Input | Output | Best For |
|------|-------|--------|----------|
| `convert_h5_to_jsonl.py` | Single `.h5` | `.jsonl` (one JSON per line) | Debugging, inspection, simple scripts |
| `convert_h5_to_parquet.py` | Single `.h5` | 16 `.parquet` files (Zstd) | Analytics, Pandas/DuckDB queries |
| `convert_h5_to_zarr.py` | Single `.h5` | `.zarr` store (Blosc/Zstd) | Cloud, multi-GPU, HDF5-like hierarchy |
| `convert_h5_to_lerobot.py` | Single `.h5` | LeRobot v3 dataset (Parquet + AV1 MP4) | PyTorch training, HF Hub sharing |
| `convert_folder.py` | Directory of `.h5` | Consolidated `.parquet` per stream | Batch processing, merged datasets |
| `validate_conversion.py` | Any output | `validation_report.json` | Integrity verification |

## Usage

```bash
# Single-stream JSONL (fastest format, 1 second)
python convert/tools/convert_h5_to_jsonl.py --streams vr_head_pose

# All streams as unified JSONL
python convert/tools/convert_h5_to_jsonl.py

# Parquet (per-stream files)
python convert/tools/convert_h5_to_parquet.py

# Zarr (preserves HDF5 hierarchy)
python convert/tools/convert_h5_to_zarr.py

# LeRobot v3 (ML-ready)
python convert/tools/convert_h5_to_lerobot.py --fps 60

# Batch folder consolidation
python convert/tools/convert_folder.py /path/to/episodes/

# Validate all outputs
python convert/tools/validate_conversion.py
```

## Format Comparison

| Property | JSONL | Parquet | Zarr | LeRobot v3 |
|----------|-------|---------|------|------------|
| Human-readable | Yes | No | No | No |
| Compression | None | Zstd (5-10×) | Blosc/Zstd (5-10×) | AV1 video + Zstd |
| Random access | No | Yes (row groups) | Yes (chunks) | Yes (Parquet + video) |
| Multi-episode | Per-file | Consolidated | Consolidated | Consolidated |
| ML framework | Any JSON parser | Pandas/DuckDB | NumPy/PyTorch | PyTorch (native) |
| Cloud-native | Yes | Yes | Yes | Yes (HF Hub) |
| Async streams | Per-stream | Per-stream | Per-stream | Resampled to canonical |

## Test Artifacts

All converters are validated against `test_artifacts/schema.json` which describes the target HDF5 file. After running converters, outputs land in `test_artifacts/`:

```
test_artifacts/
├── schema.json              # Machine-readable stream metadata
├── README.md                # Async stream challenge docs
├── episode.jsonl            # Single-stream JSONL (vr_head_pose)
├── episode_full.jsonl       # All-streams unified JSONL
├── parquet/                 # 16 .parquet files
├── episode.zarr/            # 16 Zarr groups
├── lerobot_dataset/         # LeRobot v3 (meta/ + data/ + videos/)
├── consolidated_parquet/    # Batch output from convert_folder.py
├── test_episodes/           # Test input for batch converter
└── validation_report.json   # Cross-format integrity results
```
