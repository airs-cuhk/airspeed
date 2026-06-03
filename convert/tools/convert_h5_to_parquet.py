#!/usr/bin/env python3
"""Convert AIRS HDF5 episode to Apache Parquet format.

Writes one .parquet file per stream:
  Vector streams: columns = [timestamp_ns: uint64, dim_0..dim_N: float32]
  Image streams: columns = [timestamp_ns: uint64, jpeg_bytes: binary]

Compression: Zstd level 3. Output validated against source HDF5 for row count,
value tolerance (1e-6), and JPEG byte identity.
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import h5py, numpy as np, pyarrow as pa, pyarrow.parquet as pq


def load_schema(schema_path: Path) -> dict:
    with open(schema_path) as f:
        return json.load(f)


def convert_to_parquet(h5_path: Path, schema: dict, output_dir: Path) -> dict:
    """Convert all streams to Parquet. Returns {stream_name: row_count}."""
    result = {}
    with h5py.File(h5_path, "r") as f:
        for stream in schema["streams"]:
            name = stream["name"]
            if name not in f:
                print(f"  SKIP {name}: not in HDF5")
                continue
            grp = f[name]
            gtype = stream["type"]
            data = grp["data"]
            ts = grp["timestamps"]
            n = data.shape[0]

            if gtype == "vector":
                dims = stream["dims"]
                columns = stream.get("columns", [f"dim_{i}" for i in range(dims)])
                fields = [pa.field("timestamp_ns", pa.uint64())] + [
                    pa.field(c, pa.float32()) for c in columns
                ]
                arrays = [pa.array(np.array(ts, dtype=np.uint64))] + [
                    pa.array(np.array(data[:, i], dtype=np.float32)) for i in range(dims)
                ]
            else:
                fields = [
                    pa.field("timestamp_ns", pa.uint64()),
                    pa.field("jpeg_bytes", pa.binary()),
                ]
                arrays = [
                    pa.array(np.array(ts, dtype=np.uint64)),
                    pa.array([bytes(b) for b in data], type=pa.binary()),
                ]

            table = pa.Table.from_arrays(arrays, schema=pa.schema(fields))
            out = output_dir / f"{name}.parquet"
            pq.write_table(table, out, compression="zstd", compression_level=3)
            result[name] = n
            size_kb = out.stat().st_size / 1024
            print(f"  {name}: {n} rows -> {out} ({size_kb:.0f} KB)")
    return result


def validate_parquet(output_dir: Path, h5_path: Path, schema: dict) -> dict:
    """Validate Parquet output against source HDF5. Returns report dict."""
    report = {"errors": [], "warnings": [], "streams": {}}

    with h5py.File(h5_path, "r") as f:
        for stream in schema["streams"]:
            name = stream["name"]
            pq_file = output_dir / f"{name}.parquet"
            if not pq_file.exists():
                report["errors"].append(f"{name}: .parquet not found")
                continue

            table = pq.read_table(pq_file)
            h5_data = f[name]["data"]
            h5_ts = f[name]["timestamps"]
            row_ok = len(table) == h5_data.shape[0]
            sr = {"rows_match": row_ok, "hdf5_rows": h5_data.shape[0], "parquet_rows": len(table)}
            if not row_ok:
                report["errors"].append(f"{name}: row count mismatch")

            # Check timestamps
            pq_ts = table.column("timestamp_ns").to_numpy()
            ts_match = np.array_equal(pq_ts, np.array(h5_ts, dtype=np.uint64))
            sr["timestamps_match"] = ts_match
            if not ts_match:
                n_diff = int(np.sum(pq_ts != np.array(h5_ts, dtype=np.uint64)))
                report["errors"].append(f"{name}: {n_diff} timestamp mismatches")

            if stream["type"] == "vector":
                # Check float32 values
                for i in range(stream["dims"]):
                    col_name = stream["columns"][i] if i < len(stream.get("columns", [])) else f"dim_{i}"
                    pq_vals = table.column(col_name).to_numpy()
                    h5_vals = np.array(h5_data[:, i], dtype=np.float32)
                    max_diff = float(np.abs(pq_vals - h5_vals).max())
                    if max_diff > 1e-6:
                        report["errors"].append(f"{name}.{col_name}: max diff {max_diff:.2e} > 1e-6")
                sr["max_float_diff"] = max_diff
            else:
                # Check JPEG bytes identity
                bad = 0
                for i in range(len(table)):
                    if bytes(table.column("jpeg_bytes")[i].as_py()) != bytes(h5_data[i]):
                        bad += 1
                sr["jpeg_mismatches"] = bad
                if bad:
                    report["errors"].append(f"{name}: {bad} JPEG byte mismatches")

            report["streams"][name] = sr

    report["valid"] = len(report["errors"]) == 0
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert AIRS HDF5 to Apache Parquet")
    parser.add_argument("--hdf5", help="Path to AIRS .h5 (default: from schema.json)")
    parser.add_argument("--schema", default="convert/test_artifacts/schema.json")
    parser.add_argument("--output-dir", default="convert/test_artifacts/parquet")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    root = Path.cwd()
    schema = load_schema(root / args.schema)
    h5 = root / (args.hdf5 or schema["source"])
    out = root / args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    if args.validate_only:
        report = validate_parquet(out, h5, schema)
        print(json.dumps(report, indent=2))
        sys.exit(0 if report["valid"] else 1)

    counts = convert_to_parquet(h5, schema, out)
    print(f"\n  Total: {len(counts)} streams, {sum(counts.values())} total rows")

    # Validate
    report = validate_parquet(out, h5, schema)
    status = "PASS" if report["valid"] else "FAIL"
    print(f"  Validation: {status}")
    if report["errors"]:
        for e in report["errors"]:
            print(f"    ERROR: {e}")
    if report["warnings"]:
        for w in report["warnings"]:
            print(f"    WARN: {w}")

    # Print per-stream row counts for verification
    expected = {s["name"]: s["frame_count"] for s in schema["streams"]}
    print(f"\n  Row count verification:")
    for name, n in sorted(counts.items()):
        exp = expected.get(name, "?")
        ok = "✓" if n == exp else "✗"
        print(f"    {ok} {name}: {n} (expected {exp})")


if __name__ == "__main__":
    main()
