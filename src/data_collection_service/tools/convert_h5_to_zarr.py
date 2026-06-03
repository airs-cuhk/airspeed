#!/usr/bin/env python3
"""Convert AIRS HDF5 episode to Zarr format, preserving group hierarchy.

Vector streams: (N, dims) float32 arrays chunked (1000, dims)
Image streams: (N,) object arrays with VLenUTF8 codec for JPEG bytes
All group attrs (type, columns, encoding) copied from HDF5.
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import h5py, numpy as np, zarr
from zarr.codecs import BloscCodec

COMPRESSOR = BloscCodec(cname="zstd", clevel=3, shuffle="shuffle")


def load_schema(schema_path: Path) -> dict:
    with open(schema_path) as f:
        return json.load(f)


def convert_to_zarr(h5_path: Path, schema: dict, output_path: Path) -> None:
    root = zarr.open_group(store=str(output_path), mode="w")

    with h5py.File(h5_path, "r") as f:
        # Copy root attrs
        for k, v in f.attrs.items():
            root.attrs[k] = v.item() if hasattr(v, "item") else str(v)

        for stream in schema["streams"]:
            name = stream["name"]
            grp = f[name]
            gtype = stream["type"]
            data = grp["data"]
            ts = grp["timestamps"]

            zg = root.create_group(name)
            zg.attrs["type"] = gtype

            # Copy group-level attrs
            for k, v in grp.attrs.items():
                if k == "type":
                    continue
                val = v.item() if hasattr(v, "item") else v
                zg.attrs[k] = str(val) if not isinstance(val, (int, float, bool)) else val

            n = data.shape[0]

            if gtype == "vector":
                dims = stream["dims"]
                z_data = zg.create_array(
                    "data", shape=(n, dims), chunks=(min(1000, n), dims),
                    dtype="float32", compressors=[COMPRESSOR],
                )
            else:
                # Image streams: store JPEG bytes as padded uint8 array + sizes array
                raw_bytes = [bytes(b) for b in data[:]]
                max_len = max(len(b) for b in raw_bytes)
                padded = np.zeros((n, max_len), dtype=np.uint8)
                sizes = np.zeros(n, dtype=np.uint32)
                for i, b in enumerate(raw_bytes):
                    padded[i, :len(b)] = np.frombuffer(b, dtype=np.uint8)
                    sizes[i] = len(b)

                z_data = zg.create_array(
                    "data", shape=(n, max_len),
                    chunks=(min(100, n), max(max_len // 4, 1)),
                    dtype="uint8", compressors=[COMPRESSOR],
                )
                z_sizes = zg.create_array(
                    "frame_sizes", shape=(n,),
                    chunks=(min(1000, n),), dtype="uint32",
                )
                z_sizes[:] = sizes

            z_ts = zg.create_array(
                "timestamps", shape=(n,), chunks=(min(1000, n),),
                dtype="uint64", compressors=[COMPRESSOR],
            )

            if gtype == "vector":
                z_data[:] = data[:]
            else:
                z_data[:] = padded
            z_ts[:] = ts[:]
            print(f"  {name}: ({n},) -> zarr group")


def validate_zarr(zarr_path: Path, h5_path: Path, schema: dict) -> dict:
    report = {"errors": [], "warnings": [], "streams": {}}
    zr = zarr.open_group(store=str(zarr_path), mode="r")

    with h5py.File(h5_path, "r") as f:
        for stream in schema["streams"]:
            name = stream["name"]
            if name not in zr:
                report["errors"].append(f"{name}: missing in Zarr")
                continue

            zg = zr[name]
            h5_data = f[name]["data"]
            h5_ts = f[name]["timestamps"]
            sr = {}

            # Check shapes (image data is padded, so shape differs)
            if stream["type"] == "vector":
                shape_ok = zg["data"].shape == h5_data.shape
                sr["shape_match"] = shape_ok
                if not shape_ok:
                    report["errors"].append(f"{name}/data: shape mismatch {zg['data'].shape} != {h5_data.shape}")
            else:
                shape_ok = zg["data"].shape[0] == h5_data.shape[0]
                sr["shape_match"] = shape_ok

            ts_ok = zg["timestamps"].shape == h5_ts.shape
            sr["ts_shape_match"] = ts_ok
            if not ts_ok:
                report["errors"].append(f"{name}/timestamps: shape mismatch")

            # Check dtype
            sr["dtype_match"] = str(zg["data"].dtype) == str(h5_data.dtype)

            # Check attrs
            if zg.attrs.get("type") != stream["type"]:
                report["errors"].append(f"{name}: type attr mismatch")
            sr["type_attr_ok"] = zg.attrs.get("type") == stream["type"]

            # Check data slice
            n = h5_data.shape[0]
            sl = slice(max(0, n // 2), min(n, n // 2 + 100))
            z_slice = zg["data"][sl]
            h5_slice = h5_data[sl]
            if stream["type"] == "vector":
                max_diff = float(np.abs(
                    np.array(z_slice, dtype=np.float32) - np.array(h5_slice, dtype=np.float32)
                ).max())
                sr["slice_max_diff"] = max_diff
                if max_diff > 1e-6:
                    report["errors"].append(f"{name}: slice diff {max_diff:.2e}")
            else:
                # Image: compare unpadded bytes
                z_sizes = zg["frame_sizes"][sl]
                mismatches = 0
                for j, idx in enumerate(range(sl.start, sl.stop)):
                    z_bytes = bytes(zg["data"][idx][:z_sizes[j]])
                    h5_bytes = bytes(h5_data[idx])
                    if z_bytes != h5_bytes:
                        mismatches += 1
                sr["slice_jpeg_mismatches"] = mismatches
                if mismatches:
                    report["errors"].append(f"{name}: {mismatches} JPEG mismatches in slice")

            report["streams"][name] = sr

    report["valid"] = len(report["errors"]) == 0
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert AIRS HDF5 to Zarr")
    parser.add_argument("--hdf5", help="Path to AIRS .h5 (default: from schema.json)")
    parser.add_argument("--schema", default="convert/test_artifacts/schema.json")
    parser.add_argument("--output", default="convert/test_artifacts/episode.zarr")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent.parent.parent
    schema = load_schema(project_root / args.schema)
    h5 = project_root / (args.hdf5 or schema["source"])
    out = project_root / args.output

    if args.validate_only:
        report = validate_zarr(out, h5, schema)
        print(json.dumps(report, indent=2))
        sys.exit(0 if report["valid"] else 1)

    convert_to_zarr(h5, schema, out)
    print(f"\n  Zarr store: {out}")

    # Validate
    report = validate_zarr(out, h5, schema)
    status = "PASS" if report["valid"] else "FAIL"
    groups_ok = sum(1 for s in report["streams"].values()
                    if s.get("shape_match") and s.get("type_attr_ok"))
    print(f"  Validation: {status}  ({groups_ok}/{len(report['streams'])} groups OK)")
    if report["errors"]:
        for e in report["errors"]:
            print(f"    ERROR: {e}")


if __name__ == "__main__":
    main()
