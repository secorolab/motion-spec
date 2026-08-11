# SPDX-License-Identifier: MPL-2.0
"""The sim-control writer against the frozen 48-byte block layout."""

from __future__ import annotations

import struct

from motion_spec.dashboard.control import SIZE, ControlChannel, ctrl_shm_name

# The contract, restated independently of the implementation: version, seq, pause, speed,
# stop, ack_seq -- one 8-byte word each, little-endian.
BLOCK = struct.Struct("<QQqdqQ")


def _block(tmp_path, version=1):
    path = tmp_path / "ctrl_block"
    path.write_bytes(BLOCK.pack(version, 0, 0, 0.0, 0, 0))
    return path


def _read(path):
    return BLOCK.unpack(path.read_bytes())


def test_the_block_is_forty_eight_bytes_in_the_declared_order(tmp_path):
    assert SIZE == 48 and BLOCK.size == 48
    path = _block(tmp_path)
    channel = ControlChannel(name=str(path))
    assert channel.available and channel.version == 1

    channel.set_pause(True)
    version, seq, pause, speed, stop, ack = _read(path)
    assert (version, pause, stop, ack) == (1, 1, 0, 0)
    assert seq == 1  # the field is set first, then published by bumping seq
    assert speed == 0.0
    channel.close()


def test_each_command_bumps_seq_once(tmp_path):
    path = _block(tmp_path)
    channel = ControlChannel(name=str(path))

    assert channel.set_pause(True) == 1
    assert channel.set_speed(2.0) == 2
    assert channel.request_stop() == 3
    assert channel.seq == 3

    _version, _seq, pause, speed, stop, _ack = _read(path)
    assert (pause, speed, stop) == (1, 2.0, 1)
    assert channel.paused is True
    channel.close()


def test_speed_is_clamped_to_the_contract_range(tmp_path):
    path = _block(tmp_path)
    channel = ControlChannel(name=str(path))

    channel.set_speed(50.0)
    assert channel.speed == 10.0
    channel.set_speed(0.001)
    assert channel.speed == 0.1
    channel.close()


def test_an_unset_speed_reads_as_realtime(tmp_path):
    channel = ControlChannel(name=str(_block(tmp_path)))
    assert channel.speed == 1.0  # a zeroed word means "no override", not "stopped"
    channel.close()


def test_applied_tracks_the_runtimes_ack(tmp_path):
    path = _block(tmp_path)
    channel = ControlChannel(name=str(path))
    channel.set_pause(True)
    assert channel.applied == 0 and channel.seq == 1  # not yet acknowledged

    version, seq, pause, speed, stop, _ack = _read(path)
    path.write_bytes(BLOCK.pack(version, seq, pause, speed, stop, seq))  # the C++ side acks
    assert channel.applied == channel.seq
    channel.close()


def test_a_run_without_a_control_block_is_simply_unavailable(tmp_path):
    """What plan 1a not having landed -- or a real platform -- looks like from here."""
    channel = ControlChannel(name=str(tmp_path / "absent"))
    assert channel.available is False
    assert channel.set_pause(True) is None
    assert channel.set_speed(2.0) is None
    assert channel.request_stop() is None
    assert channel.applied is None and channel.seq is None


def test_the_control_name_follows_the_frame_shm_suffix_rule():
    assert ctrl_shm_name("0123456789abcdef0123") == "/motion_spec_ctrl_0123456789abcdef"
