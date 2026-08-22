# SPDX-License-Identifier: MPL-2.0
"""Live camera for a real platform: the newest frame off a ROS image topic, as MJPEG.

A simulated run records mp4 and replays it; hardware has no recording, so the pane shows what
the camera publishes right now. Latest-frame slot only, like the shm reader: a dropped frame is
a frame the viewer never needed. rclpy is imported inside the spin thread -- a dashboard without
a ROS environment must still serve every other page, and say so in the pane instead.
"""

from __future__ import annotations

import io
import threading
import time

DEFAULT_TOPIC = "/camera/color/image_raw"
NODE_NAME = "motion_spec_dashboard_camera"
BOUNDARY = "frame"


def bgr_to_rgb(data: bytes) -> bytes:
    """Swap the channel order of a packed 3-bytes-per-pixel image, without cv_bridge or numpy."""
    swapped = bytearray(data)
    swapped[0::3], swapped[2::3] = data[2::3], data[0::3]
    return bytes(swapped)


def _packed(msg) -> bytes:
    """The message's pixels with any row padding dropped, so step never has to be carried on."""
    data = bytes(msg.data)
    stride = msg.width * 3
    if not msg.step or msg.step == stride:
        return data
    return b"".join(data[row * msg.step : row * msg.step + stride] for row in range(msg.height))


def rgb_frame(msg) -> tuple[int, int, bytes]:
    """(width, height, rgb8 pixels) of one sensor_msgs/Image; bgr8 is swapped on the way through."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported encoding: {msg.encoding}")
    data = _packed(msg)
    return msg.width, msg.height, bgr_to_rgb(data) if msg.encoding == "bgr8" else data


class RosCameraSource:
    """One rclpy node spinning in a background thread, holding the newest frame it received."""

    def __init__(self, topic: str = DEFAULT_TOPIC):
        self.topic = topic or DEFAULT_TOPIC
        self.error: str | None = None
        self._frame: tuple[int, int, int, bytes] | None = None  # seq, width, height, rgb8
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._spin, name=NODE_NAME, daemon=True)
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

    def _spin(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except ImportError as exc:
            self.error = f"no ROS env: {exc}"
            return self._ready.set()
        # Our own context, so the dashboard never disturbs an rclpy anyone else in this process
        # initialised.
        context = rclpy.Context()
        node = executor = None
        try:
            rclpy.init(context=context)
            node = rclpy.create_node(NODE_NAME, context=context)
            # Sensor QoS: best-effort still matches a reliable publisher, the reverse does not.
            node.create_subscription(Image, self.topic, self._absorb, qos_profile_sensor_data)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._ready.set()
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:  # a topic that cannot be subscribed is a pane message, not a crash
            self.error = str(exc)
        finally:
            self._ready.set()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)


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
