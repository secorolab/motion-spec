# SPDX-License-Identifier: MPL-2.0
"""Follow a frame log the runtime is still writing."""

from __future__ import annotations

from pathlib import Path

from google.protobuf.message import DecodeError

from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.archive import ArchiveError


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

    def open(self) -> bool:
        """True once the log's header record is complete and its contract has been read."""
        if self.contract is not None:
            return True
        if not self.path.exists():
            return False
        try:
            self.contract = frame_log_pb.read_contract(self.path)
        except (ArchiveError, DecodeError):
            return False  # header still being written
        self._fh = self.path.open("rb")
        self._offset = 0
        return True

    def poll(self) -> list[dict]:
        """Frames appended since the previous poll, in order."""
        if not self.open():
            return []
        records, self._offset = frame_log_pb.stream_records(self._fh, self.contract, self._offset)
        return records

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
