# SPDX-License-Identifier: MPL-2.0
"""The control block a simulated run polls: written only at its own layout version."""

from __future__ import annotations

import struct
from pathlib import Path

from motion_spec.runs.control import SIZE, SPEED_MAX, VERSION, ControlChannel


def test_the_block_is_written_only_at_its_own_layout_version(tmp_path: Path) -> None:
    block = tmp_path / "ctrl"
    # A program of another layout version: its words sit at other offsets.
    block.write_bytes(struct.pack("<Q", VERSION - 1) + bytes(SIZE))
    assert not ControlChannel(name=str(block)).available

    block.write_bytes(struct.pack("<Q", VERSION) + bytes(SIZE - 8))
    channel = ControlChannel(name=str(block))
    channel.set_pause(True)
    channel.request_steps(3)
    channel.set_speed(2 * SPEED_MAX)

    assert (channel.paused, channel.speed, channel.seq) == (True, SPEED_MAX, 3)
    assert struct.unpack_from("<q", block.read_bytes(), 40)[0] == 3
