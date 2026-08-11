# SPDX-License-Identifier: MPL-2.0
"""Read the offscreen camera block the simulated runtime publishes.

48-byte header then a width*height*3 RGB8 payload, behind the same seqlock the frame
publisher uses: an odd seq means the writer is mid-frame, so the reader retries.
"""

from __future__ import annotations

import mmap
import struct
from pathlib import Path

from motion_spec.dashboard.frames import shm_path

HEADER = struct.Struct("<Qqqqdq")  # seq, width, height, channels, t, frame_index
HEADER_SIZE = HEADER.size
CHANNELS = 3


def cam_shm_name(schema_hash: str, sensor_id: str) -> str:
    return f"/motion_spec_cam_{schema_hash[:16]}_{sensor_id}"


class CameraReader:
    """Latest rendered frame for one declared camera sensor."""

    def __init__(self, schema_hash: str | None = None, sensor_id: str = "", *, name: str = ""):
        if not name and schema_hash is None:
            raise ValueError("CameraReader needs a schema hash or an explicit name")
        self.name = name or cam_shm_name(schema_hash, sensor_id)
        self.sensor_id = sensor_id
        self._mm: mmap.mmap | None = None

    def open(self) -> bool:
        """False while the block is absent -- a real platform, or the render not started yet."""
        if self._mm is not None:
            return True
        path: Path = shm_path(self.name)
        if not path.exists() or path.stat().st_size <= HEADER_SIZE:
            return False
        with path.open("rb") as fh:
            self._mm = mmap.mmap(fh.fileno(), 0, prot=mmap.PROT_READ)
        return True

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def latest_rgb(self, retries: int = 8) -> tuple[bytes, int, int, float] | None:
        """(rgb8 payload, width, height, sim time), or None while the writer keeps tearing it."""
        if not self.open():
            return None
        mm = self._mm
        for _ in range(retries):
            seq, width, height, channels, t, _index = HEADER.unpack_from(mm, 0)
            if seq == 0 or seq & 1:  # nothing rendered yet / mid-write
                continue
            size = width * height * channels
            if channels != CHANNELS or size <= 0 or HEADER_SIZE + size > len(mm):
                return None
            payload = mm[HEADER_SIZE : HEADER_SIZE + size]
            if struct.unpack_from("<Q", mm, 0)[0] != seq:
                continue
            return payload, width, height, t
        return None
