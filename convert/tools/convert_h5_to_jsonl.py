#!/usr/bin/env python3
"""Convert AIRS HDF5 episode to JSON Lines format.

Two modes:
  --streams <name>  : output only that stream's frames (one line per frame)
  (no --streams)    : output all streams, using fastest (VR@60Hz) as canonical
                      timeline with nearest-timestamp matching for slower streams

Output: one JSON object per line. Vector values as float arrays, JPEG images as base64.
"""
from __future__ import annotations

import argparse, base64, json, sys, textwrap
from pathlib import Path

import h5py, numpy as np


def load_schema(schema_path: Path) -> dict:
    with open(schema_path) as f:
        return json.load(f)


def hdf5_path(schema: dict, project_root: Path) -> Path:
    p = project_root / schema["source"]
    if not p.exists():
        raise FileNotFoundError(f"HDF5 source not found: {p}")
    return p


def _nearest_idx(timestamps: np.ndarray, query_ts: np.ndarray) -> np.ndarray:
    """For each query timestamp, find the index of the nearest source timestamp."""
    idxs = np.searchsorted(timestamps, query_ts, side="left")
    idxs = np.clip(idxs, 0, len(timestamps) - 1)
    # Compare left vs right neighbour
    left = timestamps[idxs]
    idxs_prev = np.clip(idxs - 1, 0, len(timestamps) - 1)
    right = timestamps[idxs_prev]
    # Choose the closer one
    dist_left = np.abs(left.astype(np.int64) - query_ts.astype(np.int64))
    dist_right = np.abs(right.astype(np.int64) - query_ts.astype(np.int64))
    use_prev = dist_right < dist_left
    idxs[use_prev] = idxs_prev[use_prev]
    return idxs


def convert_single_stream(h5_path: Path, stream_name: str, output_path: Path) -> int:
    """Convert one stream to JSONL. Returns number of frames written."""
    with h5py.File(h5_path, "r") as f:
        if stream_name not in f:
            raise KeyError(f"stream {stream_name!r} not found in HDF5")
        grp = f[stream_name]
        gtype = str(grp.attrs["type"])
        data = grp["data"]
        ts = grp["timestamps"]
        n = data.shape[0]

        with open(output_path, "w") as out:
            for i in range(n):
                frame: dict = {"stream": stream_name, "ts_ns": int(ts[i]), "type": gtype}
                if gtype == "vector":
                    frame["values"] = [float(v) for v in data[i]]
                else:
                    frame["jpeg_base64"] = base64.b64encode(bytes(data[i])).decode()
                out.write(json.dumps(frame) + "\n")
    return n


def convert_all_streams(h5_path: Path, schema: dict, output_path: Path) -> dict:
    """Convert all streams to a unified JSONL using VR 60Hz as canonical timeline.
    Returns dict of stream_name -> frame_count written."""
    with h5py.File(h5_path, "r") as f:
        # Find canonical stream (most frames)
        streams = {s["name"]: s for s in schema["streams"]}
        canonical_name = max(streams, key=lambda n: streams[n]["frame_count"])
        canonical = f[canonical_name]
        canonical_ts = canonical["timestamps"][:]
        n_canonical = len(canonical_ts)

        # Pre-load all stream timestamps for nearest-neighbor lookup
        stream_data = {}
        for name in sorted(f.keys()):
            grp = f[name]
            stream_data[name] = {
                "type": str(grp.attrs["type"]),
                "data": grp["data"],
                "ts": grp["timestamps"][:],
            }

        written = {}
        with open(output_path, "w") as out:
            for i in range(n_canonical):
                frame: dict = {"frame_idx": i, "ts_ns": int(canonical_ts[i])}
                for name, sd in stream_data.items():
                    idx = _nearest_idx(sd["ts"], np.array([canonical_ts[i]]))[0]
                    ts_actual = int(sd["ts"][idx])
                    ts_delta_ns = int(canonical_ts[i]) - ts_actual
                    if sd["type"] == "vector":
                        frame[name] = {
                            "ts_ns": ts_actual,
                            "ts_delta_ns": ts_delta_ns,
                            "values": [float(v) for v in sd["data"][idx]],
                        }
                    else:
                        frame[name] = {
                            "ts_ns": ts_actual,
                            "ts_delta_ns": ts_delta_ns,
                            "jpeg_base64": base64.b64encode(bytes(sd["data"][idx])).decode(),
                        }
                out.write(json.dumps(frame) + "\n")
                written[name] = written.get(name, 0) + (1 if i < len(sd["ts"]) else 0)
    return {canonical_name: n_canonical}


def validate_output(jsonl_path: Path, h5_path: Path, stream_name: str | None = None) -> dict:
    """Validate JSONL output against source HDF5. Returns report dict."""
    report = {"file": str(jsonl_path), "errors": [], "warnings": []}

    with open(jsonl_path) as f:
        lines = f.readlines()

    if not lines:
        report["errors"].append("empty file")
        return report

    report["line_count"] = len(lines)
    timestamps = []

    for i, line in enumerate(lines):
        try:
            obj = json.loads(line)
        except json.JSONDecodeError as e:
            report["errors"].append(f"line {i}: invalid JSON: {e}")
            continue

        if "ts_ns" not in obj:
            report["warnings"].append(f"line {i}: missing ts_ns")
        else:
            timestamps.append(obj["ts_ns"])

        # Verify JPEG base64 roundtrip if present
        if "jpeg_base64" in obj:
            try:
                decoded = base64.b64decode(obj["jpeg_base64"])
                if len(decoded) < 2:
                    report["warnings"].append(f"line {i}: JPEG too short ({len(decoded)} bytes)")
                elif decoded[:2] != b"\xff\xd8":
                    report["errors"].append(f"line {i}: JPEG missing SOI marker")
            except Exception as e:
                report["errors"].append(f"line {i}: base64 decode failed: {e}")

    # Check timestamp monotonicity
    if len(timestamps) > 1:
        non_mono = sum(1 for a, b in zip(timestamps, timestamps[1:]) if b < a)
        if non_mono:
            report["errors"].append(f"{non_mono} non-monotonic timestamps")
        else:
            report["timestamps_monotonic"] = True

    report["valid"] = len(report["errors"]) == 0
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert AIRS HDF5 episode to JSON Lines",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              %(prog)s --streams vr_head_pose
              %(prog)s --streams vr_head_pose,vr_left_pose
              %(prog)s                       # all streams, unified timeline
              %(prog)s --validate            # validate existing output
        """),
    )
    parser.add_argument("--hdf5", help="Path to AIRS .h5 file (default: from schema.json)")
    parser.add_argument("--schema", default="convert/test_artifacts/schema.json",
                        help="Path to schema.json")
    parser.add_argument("--output-dir", default="convert/test_artifacts",
                        help="Output directory for .jsonl files")
    parser.add_argument("--streams", help="Comma-separated stream names (single-stream mode)")
    parser.add_argument("--validate", action="store_true", help="Validate existing output")
    args = parser.parse_args()

    project_root = Path.cwd()
    schema = load_schema(project_root / args.schema)
    h5 = project_root / (args.hdf5 or schema["source"])
    out_dir = project_root / args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.validate:
        jsonl = out_dir / "episode.jsonl"
        if not jsonl.exists():
            print(f"Nothing to validate: {jsonl} not found")
            sys.exit(1)
        report = validate_output(jsonl, h5)
        print(json.dumps(report, indent=2))
        sys.exit(0 if report.get("valid") else 1)

    if args.streams:
        # Single-stream mode
        for name in args.streams.split(","):
            name = name.strip()
            out = out_dir / f"episode_{name}.jsonl"
            n = convert_single_stream(h5, name, out)
            print(f"  {name}: {n} frames -> {out} ({out.stat().st_size / 1024:.0f} KB)")

            # Auto-validate
            report = validate_output(out, h5, name)
            status = "PASS" if report.get("valid") else "FAIL"
            print(f"  validate: {status}  lines={report.get('line_count')}  "
                  f"mono={report.get('timestamps_monotonic', 'N/A')}")

        # Also write single-stream default
        first = args.streams.split(",")[0].strip()
        out = out_dir / "episode.jsonl"
        n = convert_single_stream(h5, first, out)
        print(f"  default ({first}): {n} frames -> {out}")
    else:
        # All-streams unified mode
        out = out_dir / "episode_full.jsonl"
        counts = convert_all_streams(h5, schema, out)
        canonical_name = max(counts, key=counts.get)
        print(f"  unified: {counts[canonical_name]} frames (canonical: {canonical_name})")
        print(f"  -> {out} ({out.stat().st_size / 1024:.0f} KB)")

        # Basic validation
        report = validate_output(out, h5)
        status = "PASS" if report.get("valid") else "FAIL"
        print(f"  validate: {status}  lines={report.get('line_count')}  "
              f"mono={report.get('timestamps_monotonic', 'N/A')}")


if __name__ == "__main__":
    main()
