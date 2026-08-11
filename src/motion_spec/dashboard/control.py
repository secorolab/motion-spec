# SPDX-License-Identifier: MPL-2.0
"""Write the generated loop's sim-control block.

48 bytes, little-endian, one 8-byte word per field, so each word is individually atomic and
no seqlock is needed: set the field, then bump seq. The runtime copies ack_seq back once it
has applied that seq. Simulated platforms only -- on a real one the block is never created.
"""

from __future__ import annotations

import mmap
import struct

from motion_spec.dashboard.frames import shm_path

SIZE = 48
VERSION = 1
_VERSION_OFF, _SEQ_OFF, _PAUSE_OFF, _SPEED_OFF, _STOP_OFF, _ACK_OFF = range(0, SIZE, 8)
SPEED_MIN, SPEED_MAX = 0.1, 10.0


def ctrl_shm_name(schema_hash: str) -> str:
    return f"/motion_spec_ctrl_{schema_hash[:16]}"


class ControlChannel:
    """Pause / speed / stop, written to the control block if the run created one."""

    def __init__(self, schema_hash: str | None = None, *, name: str | None = None):
        if name is None and schema_hash is None:
            raise ValueError("ControlChannel needs a schema hash or an explicit name")
        self.name = name or ctrl_shm_name(schema_hash)
        self._mm: mmap.mmap | None = None

    @property
    def available(self) -> bool:
        """False when the block is absent -- a real platform, or a run without sim control."""
        if self._mm is not None:
            return True
        path = shm_path(self.name)
        if not path.exists() or path.stat().st_size < SIZE:
            return False
        with path.open("r+b") as fh:
            self._mm = mmap.mmap(fh.fileno(), SIZE)
        return True

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def _read(self, offset: int, fmt: str):
        if not self.available:
            return None
        return struct.unpack_from(fmt, self._mm, offset)[0]

    def _write(self, offset: int, fmt: str, value) -> int | None:
        """Set one field, then publish it by bumping seq; returns the seq the runtime must ack."""
        if not self.available:
            return None
        struct.pack_into(fmt, self._mm, offset, value)
        seq = struct.unpack_from("<Q", self._mm, _SEQ_OFF)[0] + 1
        struct.pack_into("<Q", self._mm, _SEQ_OFF, seq)
        return seq

    def set_pause(self, paused: bool) -> int | None:
        return self._write(_PAUSE_OFF, "<q", 1 if paused else 0)

    def set_speed(self, speed: float) -> int | None:
        return self._write(_SPEED_OFF, "<d", min(max(float(speed), SPEED_MIN), SPEED_MAX))

    def request_stop(self) -> int | None:
        return self._write(_STOP_OFF, "<q", 1)

    @property
    def version(self) -> int | None:
        return self._read(_VERSION_OFF, "<Q")

    @property
    def seq(self) -> int | None:
        return self._read(_SEQ_OFF, "<Q")

    @property
    def applied(self) -> int | None:
        """The last seq the runtime acknowledged; equal to `seq` once the command took effect."""
        return self._read(_ACK_OFF, "<Q")

    @property
    def paused(self) -> bool | None:
        value = self._read(_PAUSE_OFF, "<q")
        return None if value is None else bool(value)

    @property
    def speed(self) -> float | None:
        value = self._read(_SPEED_OFF, "<d")
        return None if value is None else (value or 1.0)
