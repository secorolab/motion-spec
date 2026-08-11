# SPDX-License-Identifier: MPL-2.0
"""In-memory state of one run, fed from the frame log and the shm block."""

from __future__ import annotations

import threading

TERMINAL_STATUS = frozenset({"COMPLETED", "FAILED", "INTERRUPTED"})


class RunStore:
    """One run's frames.

    The tail feed is authoritative and lossless; the shm feed only supplies `latest`, which may
    sit a few ticks ahead of it. Passive by design -- whoever owns the poll loops (the app)
    calls in, so the store itself is testable without threads.
    """

    def __init__(self, run_id: str, contract=None):
        self.run_id = run_id
        self.contract = contract
        self.frames: list[dict] = []
        self.latest: dict | None = None
        self.status: str | None = None
        self._lock = threading.Lock()

    def add_frames(self, frames) -> None:
        frames = list(frames)
        if not frames:
            return
        with self._lock:
            self.frames.extend(frames)
            self._offer(frames[-1])

    def set_latest(self, frame: dict | None) -> None:
        if frame is None:
            return
        with self._lock:
            self._offer(frame)

    def _offer(self, frame: dict) -> None:
        if self.latest is None or frame["step"] >= self.latest["step"]:
            self.latest = frame

    def snapshot(self) -> list[dict]:
        with self._lock:
            return list(self.frames)

    @property
    def completed(self) -> bool:
        if self.status in TERMINAL_STATUS:
            return True
        if self.latest is None or self.contract is None:
            return False
        return self.latest["fsm_state"] == self.contract.header.end_state

    def series(self, kind: str, slot, field: str | None = None, max_points: int = 2000):
        """(t, value) pairs for one slot field, strided down to at most `max_points`."""
        points = [
            (frame["t"], float(value))
            for frame in self.snapshot()
            if (value := _slot_value(frame, kind, slot, field)) is not None
        ]
        stride = max(1, len(points) // max_points) if max_points else 1
        return points[::stride]


def _slot_value(frame: dict, kind: str, slot, field: str | None):
    if kind == "quantities":
        return frame["quantities"].get(slot)
    entries = frame.get(kind) or []
    if not isinstance(slot, int) or not 0 <= slot < len(entries) or entries[slot] is None:
        return None
    return entries[slot].get(field)
