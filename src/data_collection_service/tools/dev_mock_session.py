#!/usr/bin/env python3
"""Generate a valid AIRS-standard HDF5 session with synthetic mock data.

from __future__ import annotations

Record mode::

    python3 tools/dev_mock_session.py \\
        --config config/session/session_vr_ik_robot_button_control.yaml \\
        --output /tmp/test-session.h5 \\
        --steps 100

Stream-only mode (no HDF5, runs until Ctrl+C)::

    python3 tools/dev_mock_session.py \\
        --config config/session/session_vr_ik_robot_button_control.yaml \\
        --stream-only \\
        --rate-hz 10
"""


import argparse
from datetime import datetime, timedelta, timezone
import os
from pathlib import Path
import sys
import time as _time

ROOT = Path(os.environ.get("DATA_COLLECTION_SERVICE_ROOT", Path.cwd()))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load_session_config
from core.adapters import AdapterRegistry
from core.storage import AirsHdf5Writer
from core.validation import validate_dataset, format_validation_report
from dev_mock_messages import make_message_for_stream

UTC = timezone.utc


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Generate valid AIRS HDF5 with synthetic data.")
    p.add_argument("--config", required=True, help="Path to session YAML")
    p.add_argument("--stream-only", action="store_true", help="Run indefinitely without writing HDF5")
    p.add_argument("--output", default=None, help="Output HDF5 path")
    p.add_argument("--steps", type=int, default=10, help="Number of steps (record mode)")
    p.add_argument("--rate-hz", type=float, default=None, help="Override step rate in Hz")
    p.add_argument("--quiet", action="store_true", help="Suppress progress output")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config_path = Path(args.config).expanduser().resolve()
    if not config_path.exists():
        print(f"error: config file not found: {config_path}", file=sys.stderr)
        return 1

    try:
        config = load_session_config(config_path)
    except Exception as exc:
        print(f"error: failed to load config: {exc}", file=sys.stderr)
        return 1

    try:
        registry = AdapterRegistry.with_defaults()
        adapters = registry.resolve_session(config)
    except Exception as exc:
        print(f"error: adapter resolution failed: {exc}", file=sys.stderr)
        return 1

    rate_hz = args.rate_hz or 10.0
    interval_s = 1.0 / rate_hz

    if args.stream_only:
        return _run_stream_only(config, adapters, rate_hz, interval_s, args.quiet)
    return _run_record(config, adapters, rate_hz, interval_s, args)


def _run_stream_only(config, adapters, rate_hz, interval_s, quiet):
    print(f"config: {config.session.name}")
    print(f"streams: {len(adapters)}")
    print(f"rate: {rate_hz:.1f} Hz  (interval: {interval_s*1000:.1f} ms)")
    print(f"mode: stream-only — no HDF5 output, Ctrl+C to stop\n")

    step_idx = 0
    t0 = _time.perf_counter()
    base_ts = datetime.now(UTC)
    try:
        while True:
            step_ts = base_ts + timedelta(seconds=step_idx * interval_s)
            ts_str = step_ts.strftime("%H:%M:%S") + f".{step_ts.microsecond // 1000:03d}"
            print(f"[{ts_str}] step={step_idx}")
            for name, stream in config.streams:
                msg = make_message_for_stream(name, stream, timestamp=step_ts)
                print(f"  {stream.source}/{name}: {stream.message_type}")
            step_idx += 1
            target = step_idx * interval_s
            actual = _time.perf_counter() - t0
            if target - actual > 0:
                _time.sleep(target - actual)
    except KeyboardInterrupt:
        pass
    elapsed = _time.perf_counter() - t0
    print(f"\nstopped after {step_idx} steps in {elapsed:.1f}s ({step_idx/elapsed:.1f} Hz)")
    return 0


def _run_record(config, adapters, rate_hz, interval_s, args):
    output_path = Path(args.output) if args.output else ROOT / "data" / "episodes"
    output_path = output_path.resolve()
    output_dir = output_path if output_path.is_dir() else output_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    num_steps = max(args.steps, 1)
    started_at = datetime.now(UTC)

    robot_type = "_".join(sorted({d.role for _, d in config.session.devices})) if config.session.devices else "unknown"
    writer = AirsHdf5Writer(
        output_dir,
        description=config.session.name,
        robot_type=robot_type,
        series_number=config.session.operator_id,
    )
    episode_id = started_at.strftime("episode-%Y%m%dT%H%M%S%fZ")
    writer.open_episode(episode_id)

    for adapter in adapters.values():
        adapter.register_with(writer)

    base_ts = started_at + timedelta(seconds=1)
    for step_idx in range(num_steps):
        step_ts = base_ts + timedelta(seconds=step_idx * interval_s)
        for name, stream in config.streams:
            msg = make_message_for_stream(name, stream, timestamp=step_ts)
            adapter = adapters[name]
            try:
                result = adapter.adapt(msg, received_at=step_ts)
                if result.image_data is not None:
                    writer.append_image(name, result.image_data, result.timestamp_ns)
                elif result.values is not None:
                    writer.append_vector(name, result.values, result.timestamp_ns)
            except Exception as exc:
                if not args.quiet:
                    print(f"  {name}: ingest error: {exc}")
        if not args.quiet and (step_idx + 1) % max(1, num_steps // 10) == 0:
            print(f"  step {step_idx + 1}/{num_steps}")

    writer.close_episode(sample_rate=rate_hz, success=True, termination_reason="completed")
    if not args.quiet:
        print(f"output: {output_dir / episode_id}.h5")
        print(f"steps: {num_steps}")

    report = validate_dataset(output_dir / f"{episode_id}.h5")
    if not args.quiet:
        print(format_validation_report(report))
    return 0 if report.is_valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
