#!/usr/bin/env python3
"""CLI wrapper for AIRS HDF5 dataset validation."""
import argparse, json, os, sys
from pathlib import Path
ROOT = Path(os.environ.get("DATA_COLLECTION_SERVICE_ROOT", Path.cwd()))
SRC = ROOT / "src"
if str(SRC) not in sys.path: sys.path.insert(0, str(SRC))
from core.validation import validate_dataset, format_validation_report

def main(argv=None):
    p = argparse.ArgumentParser(description="Validate an AIRS-standard HDF5 dataset.")
    p.add_argument("dataset_path", help="Path to HDF5 file")
    p.add_argument("--json", action="store_true", help="JSON output")
    args = p.parse_args(argv)
    report = validate_dataset(args.dataset_path)
    if args.json:
        print(json.dumps(report.to_dict(), indent=2, sort_keys=True))
    else:
        print(format_validation_report(report))
    return 0 if report.is_valid else 1

if __name__ == "__main__": raise SystemExit(main())
