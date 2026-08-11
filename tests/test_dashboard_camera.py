# SPDX-License-Identifier: MPL-2.0
"""The camera-block reader against the frozen header + RGB8 payload layout."""

from __future__ import annotations

import struct

from motion_spec.dashboard.camera import HEADER_SIZE, CameraReader, cam_shm_name

# The contract, restated independently: seq, width, height, channels, t, frame_index.
HEADER = struct.Struct("<Qqqqdq")


def _block(tmp_path, *, seq=2, width=4, height=3, channels=3, t=1.5, index=7, payload=None):
    path = tmp_path / "cam_block"
    if payload is None:
        payload = bytes(range(width * height * channels))
    path.write_bytes(HEADER.pack(seq, width, height, channels, t, index) + payload)
    return path


def test_the_header_is_forty_eight_bytes_then_the_payload(tmp_path):
    assert HEADER_SIZE == 48 and HEADER.size == 48
    path = _block(tmp_path)
    frame = CameraReader(name=str(path)).latest_rgb()

    assert frame is not None
    rgb, width, height, t = frame
    assert (width, height, t) == (4, 3, 1.5)
    assert len(rgb) == 4 * 3 * 3
    assert rgb == bytes(range(36))


def test_a_reader_never_returns_a_torn_image(tmp_path):
    assert CameraReader(name=str(_block(tmp_path, seq=3))).latest_rgb() is None  # odd: mid-write
    assert CameraReader(name=str(_block(tmp_path, seq=0))).latest_rgb() is None  # never rendered


def test_a_block_that_is_not_rgb8_is_refused(tmp_path):
    path = _block(tmp_path, channels=4)
    assert CameraReader(name=str(path)).latest_rgb() is None


def test_a_short_payload_is_refused_rather_than_padded(tmp_path):
    path = _block(tmp_path, width=8, height=8, payload=b"\x01\x02\x03")
    assert CameraReader(name=str(path)).latest_rgb() is None


def test_a_missing_block_reads_as_no_image(tmp_path):
    reader = CameraReader(name=str(tmp_path / "absent"))
    assert reader.open() is False
    assert reader.latest_rgb() is None


def test_the_camera_name_carries_the_hash_and_the_sensor(tmp_path):
    assert (
        cam_shm_name("0123456789abcdef0123", "wrist") == "/motion_spec_cam_0123456789abcdef_wrist"
    )
