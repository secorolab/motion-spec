# SPDX-License-Identifier: MPL-2.0
"""A simulated run's control block: pause, step and pace its loop from outside the program.

48 bytes, little-endian, one 8-byte word per field, so each word is individually atomic and
no seqlock is needed: set the field, then bump seq. The runtime copies ack_seq back once it
has applied that seq. Simulated platforms only -- on a real one the block is never created.
Stopping a run is not the block's: `motion-spec cancel` asks through rec.
"""

from __future__ import annotations

import mmap
import os
import struct
import time
from pathlib import Path

from motion_spec.runs.provenance import run_generation
from motion_spec.telemetry.frame_log_pb import HEADER_FILE, ctrl_shm_name, read_contract, shm_path

SIZE = 48
VERSION = 2
(VERSION_OFF, SEQ_OFF, PAUSE_OFF, SPEED_OFF, ACK_OFF, STEPS_OFF) = range(0, SIZE, 8)
SPEED_MIN, SPEED_MAX = 0.1, 10.0
ACTIONS = ("pause", "resume", "step", "speed", "status")


class ControlChannel:
    """Pause / step / speed, written to the control block if the run created one."""

    def __init__(self, name: str):
        self.name = name
        self._mm: mmap.mmap | None = None

    def open(self) -> mmap.mmap | None:
        """The mapped block; None when it is absent -- a real platform, or a run without sim
        control -- or was laid out by a program of another block version."""
        if self._mm is not None:
            return self._mm
        path = shm_path(self.name)
        if not path.exists() or path.stat().st_size < SIZE:
            return None
        with path.open("r+b") as fh:
            block = mmap.mmap(fh.fileno(), SIZE)
        if struct.unpack_from("<Q", block, VERSION_OFF)[0] != VERSION:
            block.close()
            return None
        self._mm = block
        return block

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def read(self, offset: int, fmt: str):
        block = self.open()
        if block is None:
            return None
        return struct.unpack_from(fmt, block, offset)[0]

    def write(self, offset: int, fmt: str, value) -> int | None:
        """Set one field, then publish it by bumping seq; returns the seq the runtime must ack."""
        block = self.open()
        if block is None:
            return None
        struct.pack_into(fmt, block, offset, value)
        seq = struct.unpack_from("<Q", block, SEQ_OFF)[0] + 1
        struct.pack_into("<Q", block, SEQ_OFF, seq)
        return seq

    def set_pause(self, paused: bool) -> int | None:
        return self.write(PAUSE_OFF, "<q", 1 if paused else 0)

    def set_speed(self, speed: float) -> int | None:
        return self.write(SPEED_OFF, "<d", min(max(float(speed), SPEED_MIN), SPEED_MAX))

    def request_steps(self, ticks: int = 1) -> int | None:
        """Owe a paused run this many more ticks; it counts them down one per poll."""
        pending = self.read(STEPS_OFF, "<q")
        if pending is None:
            return None
        return self.write(STEPS_OFF, "<q", max(0, pending) + max(1, int(ticks)))

    def command(self, action: str, value: float | None = None) -> None:
        """Write one of `ACTIONS`, as `control_run` takes it, to the block.

        Raises:
            ValueError: `speed` has no VALUE.
        """
        if action == "pause":
            self.set_pause(True)
        elif action == "resume":
            self.set_pause(False)
        elif action == "step":
            self.set_pause(True)
            self.request_steps(int(value or 1))
        elif action == "speed":
            if value is None:
                raise ValueError(f"'speed' needs a factor, {SPEED_MIN} to {SPEED_MAX}")
            self.set_speed(value)
        else:
            # re-publish the current state to ask the loop for an ack
            self.set_pause(bool(self.paused))

    @property
    def seq(self) -> int | None:
        return self.read(SEQ_OFF, "<Q")

    @property
    def applied(self) -> int | None:
        """The last seq the runtime acknowledged; equal to `seq` once the command took effect."""
        return self.read(ACK_OFF, "<Q")

    @property
    def paused(self) -> bool | None:
        value = self.read(PAUSE_OFF, "<q")
        return None if value is None else bool(value)

    @property
    def speed(self) -> float | None:
        value = self.read(SPEED_OFF, "<d")
        return None if value is None else (value or 1.0)


def run_channel(run_dir: Path) -> ControlChannel:
    """The control block of the run in RUN_DIR, named as the runner named it for the program.

    Raises:
        ValueError: the run's record names no generation, so its block has no name.
    """
    run_dir = Path(run_dir)
    generation = run_generation(run_dir)
    if generation is None:
        raise ValueError(f"{run_dir}: the run's document names no generation")
    header = read_contract(generation / "generated" / "contract" / HEADER_FILE).header
    name = os.environ.get("MOTION_SPEC_CTRL_SHM_NAME") or ctrl_shm_name(
        header.schema_hash, run_dir.name
    )
    return ControlChannel(name=name)


def acknowledged(channel: ControlChannel, timeout_s: float = 0.4) -> bool:
    """Whether the loop reads the block: it copies each seq back once applied."""
    if channel.open() is None:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (channel.applied or 0) >= (channel.seq or 0):
            return True
        time.sleep(0.01)
    return False


def control_run(run_dir: Path, action: str, value: float | None = None) -> dict:
    """Pause, resume, step VALUE ticks (1 when None), or pace at VALUE times real time the
    simulated run in RUN_DIR; `status` only asks the loop to acknowledge. Returns the block as
    the loop left it: whether there is one, whether the loop answers, and its fields.

    Raises:
        ValueError: ACTION is none of `ACTIONS`, or `speed` has no VALUE.
    """
    if action not in ACTIONS:
        raise ValueError(f"unknown control action '{action}' -- use one of {', '.join(ACTIONS)}")
    channel = run_channel(run_dir)
    try:
        channel.command(action, value)
        return {
            "available": channel.open() is not None,
            "alive": acknowledged(channel),
            "paused": channel.paused,
            "speed": channel.speed,
            "seq": channel.seq,
            "applied": channel.applied,
            "speed_range": [SPEED_MIN, SPEED_MAX],
        }
    finally:
        channel.close()
