# Test Artifacts — AIRS HDF5 Conversion Target

Source: `src/data_collection_service/data/episodes/episode-20260521T073812317565Z.h5`

## Multi-Rate Async Stream Challenge

This HDF5 file contains **16 streams at 4 different frame rates**, with independent per-stream timestamps:

| Rate | Frame Count | Streams |
|------|-------------|---------|
| ~60 Hz | 1,119 | vr_head_pose, vr_left_pose, vr_right_pose, vr_left_buttons, vr_right_buttons |
| ~50 Hz | 820 | ik_left_arm, ik_right_arm, ik_left_gripper, ik_right_gripper |
| ~20 Hz | 358 | arm_left_joints, arm_right_joints, arm_left_gripper, arm_right_gripper |
| ~10 Hz | 179 | camera_head, camera_left_wrist, camera_right_wrist |

**Key properties**:
- Timestamps are `uint64` nanosecond epoch, NOT frame-aligned across streams
- Each stream has independently monotonic timestamps
- Vector streams: `float32` data of shape `(N, dims)`
- Image streams: variable-length JPEG byte sequences
- Zero NaN, zero Inf, zero corrupt JPEGs

## Why This Matters for Converters

Most robot dataset formats (LeRobot, RLDS) assume a single canonical timeline where all observations are sampled simultaneously. The AIRS format does NOT make this assumption — each sensor publishes at its native rate. Converters must decide how to handle this:

- **JSON Lines**: Output per-stream files, preserving native timestamps
- **Parquet**: One file per stream, native timestamps
- **Zarr**: Preserves HDF5 group hierarchy with native timestamps
- **LeRobot v3**: Must resample to a canonical timeline (VR 60Hz as reference)

## Contents

- `schema.json` — Machine-readable metadata for all 16 streams
- Output files from each converter will be placed here (`.jsonl`, `.parquet`, `.zarr/`, `lerobot_dataset/`)
- `validation_report.json` — Cross-format integrity check results (after all converters run)
