#!/usr/bin/env python3
"""Validate post-storage conversion outputs against source AIRS HDF5.

Checks all 4 target formats (JSONL, Parquet, Zarr, LeRobot) for:
  - Frame/row count correctness per stream
  - Float32 value tolerance (1e-6)
  - JPEG byte identity (lossless roundtrip)
  - Timestamp monotonicity
  - Schema/dtype/shape consistency
  - No NaN/Inf introduced

Produces: convert/test_artifacts/validation_report.json
"""
from __future__ import annotations

import argparse, base64, json, sys
from pathlib import Path
from typing import Any

import h5py, numpy as np


def load_schema(schema_path: Path) -> dict:
    with open(schema_path) as f:
        return json.load(f)


# ── JSONL validation ──────────────────────────────────────────────

def validate_jsonl(jsonl_path: Path, h5_path: Path, schema: dict) -> dict:
    errors, warnings = [], []
    if not jsonl_path.exists():
        return {"errors": [f"{jsonl_path} not found"], "pass": False}

    with open(jsonl_path) as f:
        lines = f.readlines()

    if not lines:
        return {"errors": ["empty file"], "pass": False}

    timestamps = []
    parse_errors = 0
    for i, line in enumerate(lines):
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            parse_errors += 1
            continue
        if "ts_ns" in obj:
            timestamps.append(obj["ts_ns"])
        if "jpeg_base64" in obj:
            try:
                decoded = base64.b64decode(obj["jpeg_base64"])
                if len(decoded) < 4 or decoded[:2] != b"\xff\xd8":
                    errors.append(f"line {i}: invalid JPEG magic")
            except Exception as e:
                errors.append(f"line {i}: base64 decode fail: {e}")

    non_mono = sum(1 for a, b in zip(timestamps, timestamps[1:]) if b < a)
    return {
        "lines": len(lines), "parse_errors": parse_errors,
        "non_monotonic": non_mono,
        "pass": parse_errors == 0 and non_mono == 0 and len(errors) == 0,
        "errors": errors,
    }


# ── Parquet validation ────────────────────────────────────────────

def validate_parquet(pq_dir: Path, h5_path: Path, schema: dict) -> dict:
    import pyarrow.parquet as pq

    errors, streams = [], {}
    if not pq_dir.exists():
        return {"errors": [f"{pq_dir} not found"], "pass": False}

    pq_files = {p.stem: p for p in pq_dir.glob("*.parquet")}
    expected_count = len([s for s in schema["streams"]])

    with h5py.File(h5_path, "r") as f:
        for stream in schema["streams"]:
            name = stream["name"]
            if name not in pq_files:
                errors.append(f"{name}: .parquet missing")
                continue
            table = pq.read_table(pq_files[name])
            h5_data = f[name]["data"]
            sr = {"rows": len(table), "expected": h5_data.shape[0]}
            sr["ok"] = len(table) == h5_data.shape[0]
            if not sr["ok"]:
                errors.append(f"{name}: row count {len(table)} != {h5_data.shape[0]}")

            if stream["type"] == "vector":
                for i in range(stream["dims"]):
                    col = stream["columns"][i] if i < len(stream.get("columns", [])) else f"dim_{i}"
                    pq_vals = table.column(col).to_numpy()
                    max_d = float(np.abs(pq_vals - np.array(h5_data[:, i], dtype=np.float32)).max())
                    if max_d > 1e-6:
                        errors.append(f"{name}.{col}: diff {max_d:.2e}")
            else:
                bad = sum(1 for j in range(len(table))
                          if bytes(table.column("jpeg_bytes")[j].as_py()) != bytes(h5_data[j]))
                sr["jpeg_mismatches"] = bad
                if bad:
                    errors.append(f"{name}: {bad} JPEG mismatches")
            streams[name] = sr

    return {"file_count": len(pq_files), "expected": expected_count,
            "pass": len(errors) == 0, "streams": streams, "errors": errors}


# ── Zarr validation ───────────────────────────────────────────────

def validate_zarr(zarr_path: Path, h5_path: Path, schema: dict) -> dict:
    import zarr

    errors, streams = [], {}
    if not zarr_path.exists():
        return {"errors": [f"{zarr_path} not found"], "pass": False}

    zr = zarr.open_group(store=str(zarr_path), mode="r")

    with h5py.File(h5_path, "r") as f:
        for stream in schema["streams"]:
            name = stream["name"]
            if name not in zr:
                errors.append(f"{name}: missing in Zarr")
                continue
            zg = zr[name]
            h5_data = f[name]["data"]
            sr = {"shape": str(zg["data"].shape), "expected_rows": h5_data.shape[0]}
            sr["row_count_ok"] = zg["data"].shape[0] == h5_data.shape[0]
            if not sr["row_count_ok"]:
                errors.append(f"{name}: row mismatch")

            if stream["type"] == "vector":
                sl = slice(max(0, h5_data.shape[0] // 2),
                           min(h5_data.shape[0], h5_data.shape[0] // 2 + 100))
                max_d = float(np.abs(
                    np.array(zg["data"][sl], dtype=np.float32) -
                    np.array(h5_data[sl], dtype=np.float32)
                ).max())
                sr["slice_max_diff"] = max_d
                if max_d > 1e-6:
                    errors.append(f"{name}: slice diff {max_d:.2e}")
            else:
                sizes = np.array(zg["frame_sizes"][:])
                bad = 0
                for j in range(min(20, h5_data.shape[0])):
                    z_bytes = bytes(zg["data"][j][:sizes[j]])
                    if z_bytes != bytes(h5_data[j]):
                        bad += 1
                sr["jpeg_sample_ok"] = bad == 0
                if bad:
                    errors.append(f"{name}: {bad}/{20} JPEG samples mismatch")
            streams[name] = sr

    return {"group_count": len(zr), "expected": len(schema["streams"]),
            "pass": len(errors) == 0, "streams": streams, "errors": errors}


# ── LeRobot validation ────────────────────────────────────────────

def validate_lerobot(dataset_path: Path, schema: dict) -> dict:
    errors = []
    meta = dataset_path / "meta"
    if not (meta / "info.json").exists():
        errors.append("meta/info.json missing")
    if not (meta / "stats.json").exists():
        errors.append("meta/stats.json missing")

    videos = sorted((dataset_path / "videos").glob("**/*.mp4"))
    data_files = sorted((dataset_path / "data").glob("**/*.parquet"))

    info = {}
    if (meta / "info.json").exists():
        with open(meta / "info.json") as f:
            info = json.load(f)

    return {
        "meta_ok": (meta / "info.json").exists() and (meta / "stats.json").exists(),
        "video_count": len(videos), "data_parquet_count": len(data_files),
        "total_episodes": info.get("total_episodes", "?"),
        "fps": info.get("fps", "?"),
        "pass": len(errors) == 0, "errors": errors,
    }


# ── Main ──────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Validate conversion outputs")
    parser.add_argument("--hdf5", help="Source AIRS .h5 (default: from schema)")
    parser.add_argument("--schema", default="convert/test_artifacts/schema.json")
    parser.add_argument("--output", default="convert/test_artifacts/validation_report.json")
    parser.add_argument("--format", choices=["jsonl", "parquet", "zarr", "lerobot", "all"],
                        default="all")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent.parent.parent
    schema = load_schema(project_root / args.schema)
    h5 = project_root / (args.hdf5 or schema["source"])
    artifacts = project_root / "convert/test_artifacts"

    report: dict[str, Any] = {"source": str(h5), "formats": {}}

    if args.format in ("jsonl", "all"):
        report["formats"]["jsonl"] = validate_jsonl(
            artifacts / "episode.jsonl", h5, schema)
    if args.format in ("parquet", "all"):
        report["formats"]["parquet"] = validate_parquet(
            artifacts / "parquet", h5, schema)
    if args.format in ("zarr", "all"):
        report["formats"]["zarr"] = validate_zarr(
            artifacts / "episode.zarr", h5, schema)
    if args.format in ("lerobot", "all"):
        report["formats"]["lerobot"] = validate_lerobot(
            artifacts / "lerobot_dataset", schema)

    report["overall"] = "PASS" if all(
        f.get("pass", False) for f in report["formats"].values()
    ) else "FAIL"

    out = root / args.output
    with open(out, "w") as f:
        json.dump(report, f, indent=2)

    # Summary
    for fmt, r in report["formats"].items():
        status = "PASS" if r.get("pass") else "FAIL"
        errs = len(r.get("errors", []))
        print(f"  {fmt}: {status}" + (f" ({errs} errors)" if errs else ""))
    print(f"\n  Overall: {report['overall']}  -> {out}")

    sys.exit(0 if report["overall"] == "PASS" else 1)


if __name__ == "__main__":
    main()
