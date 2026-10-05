# SPDX-License-Identifier: MPL-2.0
"""Live camera for a real platform: the newest frame off a ROS image topic, as MJPEG.

A simulated run records mp4 and replays it; hardware has no recording, so the pane shows what
the camera publishes right now. Latest-frame slot only, like the shm reader: a dropped frame is
a frame the viewer never needed. A dashboard without a ROS environment still serves every other
page, and says so in the pane instead.
"""

from __future__ import annotations

import io
import threading
import time

from motion_spec.runs.ros_video import RosImageSubscription, rgb_rows

DEFAULT_TOPIC = "/camera/color/image_raw"
NODE_NAME = "motion_spec_dashboard_camera"
BOUNDARY = "frame"


def bgr_to_rgb(data: bytes) -> bytes:
    """Swap the channel order of a packed 3-bytes-per-pixel image, without cv_bridge or numpy."""
    swapped = bytearray(data)
    swapped[0::3], swapped[2::3] = data[2::3], data[0::3]
    return bytes(swapped)


def rgb_frame(msg) -> tuple[int, int, bytes]:
    """(width, height, rgb8 pixels) of one sensor_msgs/Image; bgr8 is swapped on the way through."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported encoding: {msg.encoding}")
    data = rgb_rows(msg)
    return msg.width, msg.height, bgr_to_rgb(data) if msg.encoding == "bgr8" else data


class RosCameraSource(RosImageSubscription):
    """One rclpy node spinning in a background thread, holding the newest frame it received."""

    def __init__(self, topic: str = DEFAULT_TOPIC):
        super().__init__(topic or DEFAULT_TOPIC, NODE_NAME)
        self._frame: tuple[int, int, int, bytes] | None = None  # seq, width, height, rgb8
        self._seq = 0
        self._lock = threading.Lock()
        self._thread.start()

    def alive(self) -> bool:
        """Whether the node is up: false once rclpy is missing, the topic failed, or we closed."""
        self._ready.wait(timeout=2.0)
        return self._thread.is_alive() and not self._stop.is_set() and self.error is None

    def latest(self) -> tuple[int, int, int, bytes] | None:
        with self._lock:
            return self._frame

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _absorb(self, msg) -> None:
        try:
            width, height, rgb = rgb_frame(msg)
        except ValueError as exc:
            self.error = str(exc)
            return self._stop.set()
        with self._lock:
            self._seq += 1
            self._frame = (self._seq, width, height, rgb)


_SOURCE: RosCameraSource | None = None
_SOURCE_LOCK = threading.Lock()


def source_for(topic: str) -> RosCameraSource:
    """The process's camera source, restarted when the pane asks for a different topic."""
    global _SOURCE
    topic = topic or DEFAULT_TOPIC
    with _SOURCE_LOCK:
        if _SOURCE is not None and (_SOURCE.topic != topic or _SOURCE.error is not None):
            _SOURCE.close()
            _SOURCE = None
        if _SOURCE is None:
            _SOURCE = RosCameraSource(topic)
        return _SOURCE


def jpeg(frame: tuple[int, int, int, bytes], quality: int = 80) -> bytes:
    _seq, width, height, rgb = frame
    from PIL import Image

    buffer = io.BytesIO()
    Image.frombytes("RGB", (width, height), rgb).save(buffer, "JPEG", quality=quality)
    return buffer.getvalue()


def part(payload: bytes) -> bytes:
    return (
        (
            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(payload)}\r\n\r\n"
        ).encode()
        + payload
        + b"\r\n"
    )


def mjpeg_stream(source, running, *, fps: float = 20.0, quality: int = 80):
    """Yield one multipart part per new frame, at most `fps` a second.

    A generator rather than a write loop, so the pacing and the skip are testable without a
    socket: the caller writes what this yields and stops when the client is gone.
    """
    period = 1.0 / fps
    seen = None
    while running():
        frame = source.latest()
        if frame is not None and frame[0] != seen:
            seen = frame[0]
            yield part(jpeg(frame, quality))
        time.sleep(period)
