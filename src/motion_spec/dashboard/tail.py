# SPDX-License-Identifier: MPL-2.0
"""Follow a frame log the runtime is still writing."""

from __future__ import annotations

from pathlib import Path

from google.protobuf.message import DecodeError

from motion_spec.runs.verify import RecordError
from motion_spec.telemetry import frame_log_pb


class FrameLogTail:
    """The run's lossless history, read forward as the runtime appends to it.

    The header record carries the decode contract, so the tail stays closed until that record
    is complete. After that every poll returns whatever whole records arrived since the last
    one; a half-written trailing record leaves the offset put and is picked up next time.
    """

    def __init__(self, log_path: Path | str):
        self.path = Path(log_path)
        self.contract: frame_log_pb.LogContract | None = None
        self._fh = None
        self._offset = 0

    def open(self) -> frame_log_pb.LogContract | None:
        """The log's contract once its header record is complete, else None."""
        if self.contract is not None:
            return self.contract
        if not self.path.exists():
            return None
        try:
            self.contract = frame_log_pb.read_contract(self.path)
        except (RecordError, DecodeError):
            return None  # header still being written
        self._fh = frame_log_pb.open_log(self.path)
        return self.contract

    def poll(self, stride: int = 1) -> list[dict]:
        """Frames appended since the previous poll, in order, keeping every nth record."""
        contract = self.open()
        if contract is None:
            return []
        records, self._offset = frame_log_pb.stream_records(
            self._fh, contract, self._offset, stride
        )
        return records

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
