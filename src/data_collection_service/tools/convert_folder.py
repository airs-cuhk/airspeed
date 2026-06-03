#!/usr/bin/env python3
"""Convert a folder of AIRS HDF5 episodes into consolidated Parquet or Zarr.

Adds episode_id column (Parquet) or episode_index array (Zarr) to distinguish
source episodes. Handles variable-length streams across episodes.
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import h5py, numpy as np, pyarrow as pa, pyarrow.parquet as pq


def load_schema(schema_path: Path) -> dict:
    with open(schema_path) as f:
        return json.load(f)


def folder_to_parquet(h5_dir: Path, schema: dict, output_dir: Path) -> dict:
    """Consolidate multiple .h5 episodes into Parquet. Returns counts dict."""
    h5_files = sorted(h5_dir.glob("*.h5"))
    if not h5_files:
        raise FileNotFoundError(f"no .h5 files in {h5_dir}")

    # First pass: discover all stream names and types from first file
    with h5py.File(h5_files[0], "r") as f0:
        stream_info = {name: {"type": str(g.attrs["type"]),
                              "dims": g["data"].shape[1] if g["data"].ndim > 1 else 0}
                       for name, g in f0.items()}

    # Accumulate per-stream
    accum: dict[str, dict] = {name: {"ts": [], "data": [], "ep": []}
                               for name in stream_info}

    for ep_id, h5_path in enumerate(h5_files):
        with h5py.File(h5_path, "r") as f:
            for name in f.keys():
                if name not in accum:
                    continue
                grp = f[name]
                n = grp["data"].shape[0]
                accum[name]["ts"].append(np.array(grp["timestamps"][:], dtype=np.uint64))
                accum[name]["data"].append(grp["data"][:])
                accum[name]["ep"].append(np.full(n, ep_id, dtype=np.int32))
        print(f"  ep_{ep_id}: {h5_path.name}")

    # Write consolidated Parquet
    output_dir.mkdir(parents=True, exist_ok=True)
    counts = {}
    for name, si in stream_info.items():
        ts_all = np.concatenate(accum[name]["ts"])
        ep_all = np.concatenate(accum[name]["ep"])
        data_all = np.concatenate(accum[name]["data"])

        if si["type"] == "vector":
            dims = si["dims"]
            columns = schema_stream_columns(schema, name, dims)
            fields = [pa.field("timestamp_ns", pa.uint64()),
                      pa.field("episode_id", pa.int32())]
            arrays = [pa.array(ts_all), pa.array(ep_all)]
            for i in range(dims):
                fields.append(pa.field(columns[i] if i < len(columns) else f"dim_{i}", pa.float32()))
                arrays.append(pa.array(data_all[:, i].astype(np.float32)))
        else:
            fields = [pa.field("timestamp_ns", pa.uint64()),
                      pa.field("episode_id", pa.int32()),
                      pa.field("jpeg_bytes", pa.binary())]
            arrays = [pa.array(ts_all), pa.array(ep_all),
                      pa.array([bytes(b) for b in data_all], type=pa.binary())]

        table = pa.Table.from_arrays(arrays, schema=pa.schema(fields))
        out = output_dir / f"{name}.parquet"
        pq.write_table(table, out, compression="zstd", compression_level=3)
        counts[name] = len(table)
        print(f"  {name}: {len(table)} rows -> {out}")

    return counts


def schema_stream_columns(schema: dict, name: str, dims: int) -> list[str]:
    for s in schema["streams"]:
        if s["name"] == name:
            cols = s.get("columns", [])
            if len(cols) == dims:
                return cols
    return [f"dim_{i}" for i in range(dims)]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert folder of AIRS HDF5 episodes to consolidated Parquet")
    parser.add_argument("input_dir", nargs="?", default="convert/test_artifacts/test_episodes",
                        help="Directory containing .h5 episode files")
    parser.add_argument("--schema", default="convert/test_artifacts/schema.json")
    parser.add_argument("--output", default="convert/test_artifacts/consolidated_parquet")
    parser.add_argument("--format", choices=["parquet"], default="parquet")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent.parent.parent
    schema = load_schema(project_root / args.schema)
    h5_dir = project_root / args.input_dir
    out = project_root / args.output

    h5_files = sorted(h5_dir.glob("*.h5"))
    print(f"Input: {len(h5_files)} episodes in {h5_dir}")

    if args.format == "parquet":
        counts = folder_to_parquet(h5_dir, schema, out)
        total = sum(counts.values())
        print(f"\n  Total: {len(counts)} streams, {total} total rows, {len(h5_files)} episodes")

        # Verify
        for name, n in sorted(counts.items()):
            print(f"    {name}: {n} rows")

    print(f"  Output: {out}")


if __name__ == "__main__":
    main()
