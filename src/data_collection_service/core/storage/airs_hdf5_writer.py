"""AIRS-standard HDF5 writer: one file per episode, flat per-stream groups.

Uses chunked append-backed datasets: frames are buffered in small batches
and flushed periodically, keeping memory constant regardless of episode
length while amortizing HDF5 resize overhead.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
from typing import Any

import h5py
import numpy as np


class AirsHdf5WriterError(ValueError):
    """Raised when AIRS HDF5 writing cannot proceed safely."""


# Flush every N frames to amortize HDF5 resize cost
_VECTOR_BATCH = 50
_IMAGE_BATCH = 20


class AirsHdf5Writer:
    """Write one AIRS-standard HDF5 file per episode.

    Layout::

        <episode_id>.h5
        ├── / (root attrs: description, robot_type, series_number,
        │        sample_rate, frames, success, termination_reason)
        ├── <stream_name>/
        │     ├── attrs: type, columns (vector) / width,height,channels,encoding (image)
        │     ├── data:       (N, D) float32  or  (N,) vlen uint8
        │     └── timestamps: (N,) uint64
        └── ...
    """

    def __init__(
        self,
        output_dir: str | Path,
        *,
        description: str = "",
        robot_type: str = "",
        series_number: str = "",
    ) -> None:
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._description = description
        self._robot_type = robot_type
        self._series_number = series_number
        self._file: h5py.File | None = None
        self._streams: dict[str, _StreamBuffer] = {}

    # -- episode lifecycle --

    def open_episode(self, episode_id: str) -> None:
        if self._file is not None:
            raise AirsHdf5WriterError("an episode is already open; close it first")
        path = self._output_dir / f"{episode_id}.h5"
        self._file = h5py.File(path, "w")
        self._streams = {}

    def close_episode(self, *, sample_rate: float = 0.0,
                      success: bool | None = None,
                      termination_reason: str = "") -> str:
        if self._file is None:
            raise AirsHdf5WriterError("no episode is open")
        total_frames = 0
        for buf in self._streams.values():
            buf.flush_remaining(self._file)
            total_frames = max(total_frames, buf.frame_count)
        self._file.attrs["description"] = self._description
        self._file.attrs["robot_type"] = self._robot_type
        self._file.attrs["series_number"] = self._series_number
        self._file.attrs["sample_rate"] = float(sample_rate)
        self._file.attrs["frames"] = total_frames
        if success is not None:
            self._file.attrs["success"] = bool(success)
        if termination_reason:
            self._file.attrs["termination_reason"] = termination_reason
        path = self._file.filename
        self._file.close()
        self._file = None
        return path

    @staticmethod
    def move_to_trash(episode_path: str | Path) -> str:
        """Move an episode file to .trash/ under its parent directory.

        Returns the new path in .trash/, or the original path if the move failed.
        """
        src = Path(episode_path)
        if not src.exists():
            return str(src)
        trash_dir = src.parent / ".trash"
        trash_dir.mkdir(parents=True, exist_ok=True)
        dst = trash_dir / src.name
        shutil.move(str(src), str(dst))
        return str(dst)

    # -- per-stream registration (creates HDF5 group + datasets immediately) --

    def register_vector_stream(
        self, name: str, dims: int = 0, *, columns: tuple[str, ...] = (),
    ) -> None:
        self._require_open()
        if name in self._streams:
            raise AirsHdf5WriterError(f"stream {name!r} already registered")
        grp = self._file.create_group(name)
        grp.attrs["type"] = "vector"
        if columns:
            grp.attrs["columns"] = json.dumps(list(columns))
        if dims > 0:
            grp.create_dataset(
                "data", shape=(0, dims), maxshape=(None, dims),
                dtype=np.float32, chunks=(_VECTOR_BATCH, dims),
            )
        grp.create_dataset(
            "timestamps", shape=(0,), maxshape=(None,),
            dtype=np.uint64, chunks=(_VECTOR_BATCH,),
        )
        self._streams[name] = _VectorBuffer(name, grp, dims)

    def register_image_stream(
        self, name: str, *, width: int, height: int, channels: int,
        encoding: str = "jpeg", reencode_to_jpeg: bool = False,
    ) -> None:
        self._require_open()
        if name in self._streams:
            raise AirsHdf5WriterError(f"stream {name!r} already registered")
        grp = self._file.create_group(name)
        grp.attrs["type"] = "image"
        grp.attrs["width"] = width
        grp.attrs["height"] = height
        grp.attrs["channels"] = channels
        grp.attrs["encoding"] = "jpeg" if reencode_to_jpeg else encoding
        dt = h5py.vlen_dtype(np.dtype("uint8"))
        grp.create_dataset(
            "data", shape=(0,), maxshape=(None,),
            dtype=dt, chunks=(_IMAGE_BATCH,),
        )
        grp.create_dataset(
            "timestamps", shape=(0,), maxshape=(None,),
            dtype=np.uint64, chunks=(_IMAGE_BATCH,),
        )
        self._streams[name] = _ImageBuffer(name, grp, reencode_to_jpeg)

    # -- append (hot path — buffers in memory, flushes in batches) --

    def append_vector(self, name: str, values: object, timestamp_ns: int) -> None:
        buf = self._streams.get(name)
        if not isinstance(buf, _VectorBuffer):
            raise AirsHdf5WriterError(f"{name!r} is not a registered vector stream")
        buf.append(values, timestamp_ns)

    def append_image(self, name: str, raw_data: bytes, timestamp_ns: int) -> None:
        buf = self._streams.get(name)
        if not isinstance(buf, _ImageBuffer):
            raise AirsHdf5WriterError(f"{name!r} is not a registered image stream")
        buf.append(raw_data, timestamp_ns)

    # -- helpers --

    def _require_open(self) -> None:
        if self._file is None:
            raise AirsHdf5WriterError("no episode is open")


# ---------------------------------------------------------------------------
# Chunked append-backed buffers
#
# Design: we don't append one row at a time (HDF5 resize() per frame is slow).
# Instead we buffer in memory and flush every _VECTOR_BATCH (50) / _IMAGE_BATCH
# (20) frames. Each flush resizes the dataset once and writes the entire batch
# contiguously. Memory is capped at one batch — a 10-hour episode uses the same
# memory as a 10-second one.
# ---------------------------------------------------------------------------


class _VectorBuffer:
    def __init__(self, name: str, grp: h5py.Group, dims: int) -> None:
        self.name = name
        self._grp = grp
        self._dims = dims
        self._data_buf: list[np.ndarray] = []
        self._ts_buf: list[np.uint64] = []
        self._total = 0

    @property
    def frame_count(self) -> int:
        return self._total + len(self._data_buf)

    def append(self, values: object, timestamp_ns: int) -> None:
        arr = np.asarray(values, dtype=np.float32).ravel()
        if self._dims == 0:
            self._dims = arr.size
        elif arr.size != self._dims:
            raise AirsHdf5WriterError(
                f"{self.name}: dimension mismatch — expected {self._dims}, got {arr.size}"
            )
        self._data_buf.append(arr.astype(np.float32))
        self._ts_buf.append(np.uint64(timestamp_ns))
        if len(self._data_buf) >= _VECTOR_BATCH:
            self._flush_batch()

    def _flush_batch(self) -> None:
        if not self._data_buf:
            return
        n = len(self._data_buf)
        data_arr = np.array(self._data_buf, dtype=np.float32)
        ts_arr = np.array(self._ts_buf, dtype=np.uint64)

        # Lazy dataset creation for variable-dimension streams
        if "data" not in self._grp:
            self._grp.create_dataset(
                "data", shape=(0, self._dims), maxshape=(None, self._dims),
                dtype=np.float32, chunks=(_VECTOR_BATCH, self._dims),
            )

        dset = self._grp["data"]
        tset = self._grp["timestamps"]
        new_size = self._total + n
        dset.resize((new_size, self._dims))
        tset.resize((new_size,))
        dset[self._total:new_size] = data_arr
        tset[self._total:new_size] = ts_arr

        self._total = new_size
        self._data_buf.clear()
        self._ts_buf.clear()

    def flush_remaining(self, file: h5py.File) -> None:
        self._flush_batch()
        self._grp.attrs["frames"] = self._total
        self._grp.attrs["sample_rate"] = float(file.attrs.get("sample_rate", 0.0))
        if "columns" not in self._grp.attrs:
            self._grp.attrs["columns"] = json.dumps(
                [f"dim_{j}" for j in range(self._dims)]
            )


class _ImageBuffer:
    def __init__(self, name: str, grp: h5py.Group, reencode_to_jpeg: bool) -> None:
        self.name = name
        self._grp = grp
        self._reencode = reencode_to_jpeg
        self._frame_buf: list[bytes] = []
        self._ts_buf: list[np.uint64] = []
        self._total = 0

    @property
    def frame_count(self) -> int:
        return self._total + len(self._frame_buf)

    def append(self, raw_data: bytes, timestamp_ns: int) -> None:
        if self._reencode:
            raw_data = _reencode_jpeg(raw_data)
        self._frame_buf.append(raw_data)
        self._ts_buf.append(np.uint64(timestamp_ns))
        if len(self._frame_buf) >= _IMAGE_BATCH:
            self._flush_batch()

    def _flush_batch(self) -> None:
        if not self._frame_buf:
            return
        n = len(self._frame_buf)
        dset = self._grp["data"]
        tset = self._grp["timestamps"]
        new_size = self._total + n
        dset.resize((new_size,))
        tset.resize((new_size,))
        for i, frame in enumerate(self._frame_buf):
            dset[self._total + i] = np.frombuffer(frame, dtype=np.uint8)
        tset[self._total:new_size] = np.array(self._ts_buf, dtype=np.uint64)

        self._total = new_size
        self._frame_buf.clear()
        self._ts_buf.clear()

    def flush_remaining(self, file: h5py.File) -> None:
        self._flush_batch()
        self._grp.attrs["frames"] = self._total
        self._grp.attrs["sample_rate"] = float(file.attrs.get("sample_rate", 0.0))
        self._grp.attrs["camera_name"] = self.name


def _reencode_jpeg(raw: bytes) -> bytes:
    try:
        import cv2
    except ImportError:
        return raw
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return raw
    _, enc = cv2.imencode(".jpg", img)
    return enc.tobytes()


__all__ = ["AirsHdf5Writer", "AirsHdf5WriterError"]
