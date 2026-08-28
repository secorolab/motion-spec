# SPDX-License-Identifier: MPL-2.0
"""The real-platform camera: what a ROS image message becomes, and what the MJPEG loop sends."""

from __future__ import annotations

import io
import json
from types import SimpleNamespace

import pytest
from PIL import Image

from motion_spec.dashboard import catalog, ros_camera


def _message(encoding, pixels, width=2, height=1, step=None):
    """A sensor_msgs/Image as far as the conversion reads one."""
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=width * 3 if step is None else step,
        data=bytes(pixels),
    )


class _Camera:
    """A stub source: one frame, republished under a new seq only when told to."""

    def __init__(self, advancing: bool):
        self.advancing = advancing
        self.seq = 1

    def latest(self):
        seq = self.seq
        if self.advancing:
            self.seq += 1
        # A different colour per seq, so two encoded parts cannot be equal by accident.
        return (seq, 4, 2, bytes([seq * 20, 0, 0]) * 8)


def _polls(count: int):
    """A running() that lets the stream poll `count` times and then stops it."""
    left = iter(range(count))
    return lambda: next(left, None) is not None


def test_bgr8_arrives_as_rgb8():
    """No cv_bridge: the channel swap is the whole conversion, and it is its own inverse."""
    assert ros_camera.bgr_to_rgb(bytes([1, 2, 3, 4, 5, 6])) == bytes([3, 2, 1, 6, 5, 4])
    assert ros_camera.rgb_frame(_message("bgr8", [1, 2, 3, 4, 5, 6])) == (
        2,
        1,
        bytes([3, 2, 1, 6, 5, 4]),
    )
    assert ros_camera.rgb_frame(_message("rgb8", [1, 2, 3, 4, 5, 6])) == (
        2,
        1,
        bytes([1, 2, 3, 4, 5, 6]),
    )


def test_row_padding_is_dropped_and_a_foreign_encoding_refused():
    padded = _message("rgb8", [1, 2, 3, 9, 9, 4, 5, 6, 9, 9], width=1, height=2, step=5)
    assert ros_camera.rgb_frame(padded) == (1, 2, bytes([1, 2, 3, 4, 5, 6]))

    with pytest.raises(ValueError, match="mono8"):
        ros_camera.rgb_frame(_message("mono8", [1, 2]))


def test_every_new_frame_is_one_multipart_jpeg():
    parts = list(ros_camera.mjpeg_stream(_Camera(advancing=True), _polls(3), fps=1000))
    assert len(parts) >= 2

    payloads = []
    for part in parts:
        head, payload = part.split(b"\r\n\r\n", 1)
        assert head.startswith(b"--frame\r\nContent-Type: image/jpeg")
        assert payload.endswith(b"\r\n")
        payload = payload[:-2]
        assert f"Content-Length: {len(payload)}".encode() in head
        payloads.append(payload)
    assert payloads[0] != payloads[1]  # a part per frame, not the same frame twice
    assert Image.open(io.BytesIO(payloads[0])).size == (4, 2)


def test_a_seq_that_does_not_move_is_sent_once():
    """The loop polls at 20 fps; a camera publishing slower must not re-send what it already sent."""
    parts = list(ros_camera.mjpeg_stream(_Camera(advancing=False), _polls(5), fps=1000))
    assert len(parts) == 1


def test_a_source_with_no_frame_yet_sends_nothing():
    empty = SimpleNamespace(latest=lambda: None)
    assert list(ros_camera.mjpeg_stream(empty, _polls(3), fps=1000)) == []


def _contract(tmp_path, camera: dict):
    """A generation carrying one camera in its contract, as the generator writes it."""
    contract = tmp_path / "generated" / "contract"
    contract.mkdir(parents=True)
    (contract / "frame_layout.json").write_text(json.dumps({"cameras": [camera]}))
    return tmp_path


def test_a_cameras_provider_reaches_the_page_that_reads_it(tmp_path):
    """The topic is the model's answer to where the images come from, so the page is told it."""
    generation = _contract(
        tmp_path,
        {
            "id": "wrist",
            "width": 640,
            "height": 480,
            "topic": "/wrist/color",
            "message": "sensor_msgs/msg/Image",
        },
    )
    (camera,) = catalog.generation_cameras(generation)
    assert camera["topic"] == "/wrist/color"
    assert camera["message"] == "sensor_msgs/msg/Image"


def test_a_camera_with_no_provider_offers_the_page_no_topic(tmp_path):
    """Nothing states where it is published, so nothing is passed on -- and the page, finding no
    topic, shows no live pane rather than guessing one from the camera's name."""
    generation = _contract(tmp_path, {"id": "wrist", "width": 640, "height": 480})
    (camera,) = catalog.generation_cameras(generation)
    assert camera["topic"] is None
    assert camera["message"] is None
