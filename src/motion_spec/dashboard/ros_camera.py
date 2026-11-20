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
STREAM_FPS = 20.0
JPEG_QUALITY = 80


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
        return self._thread.is_alive() and not self.stop.is_set() and self.error is None

    def latest(self) -> tuple[int, int, int, bytes] | None:
        with self._lock:
            return self._frame

    def close(self) -> None:
        self.stop.set()
        self._thread.join(timeout=2.0)

    def absorb(self, msg) -> None:
        try:
            if msg.encoding not in ("rgb8", "bgr8"):
                raise ValueError(f"unsupported encoding: {msg.encoding}")
            rgb = rgb_rows(msg)
            if msg.encoding == "bgr8":
                # Swapped by slicing, without cv_bridge or numpy.
                swapped = bytearray(rgb)
                swapped[0::3], swapped[2::3] = rgb[2::3], rgb[0::3]
                rgb = bytes(swapped)
            width, height = msg.width, msg.height
        except ValueError as exc:
            self.error = str(exc)
            self.stop.set()
            return
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


def mjpeg_stream(source, running):
    """Yield one multipart part per new frame, at most `STREAM_FPS` a second.

    A generator rather than a write loop, so the pacing and the skip are testable without a
    socket: the caller writes what this yields and stops when the client is gone.
    """
    period = 1.0 / STREAM_FPS
    seen = None
    while running():
        frame = source.latest()
        if frame is not None and frame[0] != seen:
            seen = frame[0]
            _seq, width, height, rgb = frame
            from PIL import Image

            buffer = io.BytesIO()
            Image.frombytes("RGB", (width, height), rgb).save(buffer, "JPEG", quality=JPEG_QUALITY)
            payload = buffer.getvalue()
            yield (
                (
                    f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(payload)}\r\n\r\n"
                ).encode()
                + payload
                + b"\r\n"
            )
        time.sleep(period)
