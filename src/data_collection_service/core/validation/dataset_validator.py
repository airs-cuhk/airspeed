"""Validate an AIRS-standard HDF5 dataset."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import h5py
import numpy as np


@dataclass(frozen=True)
class ValidationIssue:
    path: str
    message: str


@dataclass
class ValidationReport:
    dataset_path: str
    is_valid: bool = True
    errors: list[ValidationIssue] = field(default_factory=list)
    warnings: list[ValidationIssue] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "dataset_path": self.dataset_path,
            "is_valid": self.is_valid,
            "errors": [{"path": e.path, "message": e.message} for e in self.errors],
            "warnings": [{"path": w.path, "message": w.message} for w in self.warnings],
        }


_ROOT_ATTRS = frozenset({"description", "robot_type", "series_number", "sample_rate", "frames"})


def validate_dataset(dataset_path: str | Path) -> ValidationReport:
    path = str(dataset_path)
    report = ValidationReport(dataset_path=path)

    try:
        f = h5py.File(path, "r")
    except Exception as exc:
        report.is_valid = False
        report.errors.append(ValidationIssue("/", f"cannot open file: {exc}"))
        return report

    with f:
        _validate_root_attrs(f, report)
        for group_name in f:
            grp = f[group_name]
            if not isinstance(grp, h5py.Group):
                continue
            if "type" not in grp.attrs:
                report.errors.append(ValidationIssue(f"/{group_name}", "missing 'type' attr"))
                report.is_valid = False
                continue
            gtype = str(grp.attrs["type"])
            if gtype == "vector":
                _validate_vector_group(f, group_name, report)
            elif gtype == "image":
                _validate_image_group(f, group_name, report)
            else:
                report.errors.append(ValidationIssue(f"/{group_name}", f"unknown type: {gtype!r}"))
                report.is_valid = False

    return report


def _validate_root_attrs(f: h5py.File, report: ValidationReport) -> None:
    for attr in _ROOT_ATTRS:
        if attr not in f.attrs:
            report.errors.append(ValidationIssue("/", f"missing root attr: {attr}"))
            report.is_valid = False


def _validate_vector_group(f: h5py.File, name: str, report: ValidationReport) -> None:
    grp = f[name]
    for ds_name in ("data", "timestamps"):
        if ds_name not in grp:
            report.errors.append(ValidationIssue(f"/{name}", f"missing dataset: {ds_name}"))
            report.is_valid = False
            return
    data, ts = grp["data"], grp["timestamps"]
    if data.ndim != 2:
        report.errors.append(ValidationIssue(f"/{name}/data", f"ndim must be 2, got {data.ndim}"))
        report.is_valid = False
    if data.dtype != np.float32:
        report.errors.append(ValidationIssue(f"/{name}/data", f"dtype must be float32, got {data.dtype}"))
        report.is_valid = False
    if ts.ndim != 1:
        report.errors.append(ValidationIssue(f"/{name}/timestamps", f"ndim must be 1, got {ts.ndim}"))
        report.is_valid = False
    if ts.dtype != np.uint64:
        report.errors.append(ValidationIssue(f"/{name}/timestamps", f"dtype must be uint64, got {ts.dtype}"))
        report.is_valid = False
    if data.shape[0] != ts.shape[0]:
        report.errors.append(ValidationIssue(
            f"/{name}", f"data[0]={data.shape[0]} != timestamps[0]={ts.shape[0]}",
        ))
        report.is_valid = False
    if "columns" not in grp.attrs:
        report.warnings.append(ValidationIssue(f"/{name}", "missing columns attr"))


def _validate_image_group(f: h5py.File, name: str, report: ValidationReport) -> None:
    grp = f[name]
    for ds_name in ("data", "timestamps"):
        if ds_name not in grp:
            report.errors.append(ValidationIssue(f"/{name}", f"missing dataset: {ds_name}"))
            report.is_valid = False
            return
    data, ts = grp["data"], grp["timestamps"]
    if data.ndim != 1:
        report.errors.append(ValidationIssue(f"/{name}/data", f"ndim must be 1, got {data.ndim}"))
        report.is_valid = False
    if ts.ndim != 1:
        report.errors.append(ValidationIssue(f"/{name}/timestamps", f"ndim must be 1, got {ts.ndim}"))
        report.is_valid = False
    if data.shape[0] != ts.shape[0]:
        report.errors.append(ValidationIssue(
            f"/{name}", f"data[0]={data.shape[0]} != timestamps[0]={ts.shape[0]}",
        ))
        report.is_valid = False
    encoding = str(grp.attrs.get("encoding", ""))
    if encoding == "jpeg" and data.shape[0] > 0:
        first = data[0]
        if len(first) < 2 or bytes(first[:2]) != b"\xff\xd8":
            report.errors.append(ValidationIssue(
                f"/{name}/data[0]", "JPEG magic bytes missing (expected 0xFFD8)",
            ))
            report.is_valid = False
    for attr in ("width", "height", "channels"):
        if attr not in grp.attrs:
            report.warnings.append(ValidationIssue(f"/{name}", f"missing attr: {attr}"))


def format_validation_report(report: ValidationReport) -> str:
    lines = [
        f"Dataset: {report.dataset_path}",
        f"Status: {'valid' if report.is_valid else 'invalid'}",
        f"Errors: {len(report.errors)}",
        f"Warnings: {len(report.warnings)}",
    ]
    for e in report.errors:
        lines.append(f"[ERROR] {e.path}: {e.message}")
    for w in report.warnings:
        lines.append(f"[WARN] {w.path}: {w.message}")
    return "\n".join(lines)


__all__ = ["ValidationIssue", "ValidationReport", "format_validation_report", "validate_dataset"]
