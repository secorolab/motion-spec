# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Record real ROS RGB image topics as MP4 files."""

from __future__ import annotations

import os
import queue
import subprocess
import threading
import time
from dataclasses import dataclass
from io import BufferedWriter
from pathlib import Path


@dataclass(frozen=True)
class CameraRecording:
    """One declared RGB camera selected for a real-world recording."""

    id: str
    topic: str
    rate_hz: float


def real_camera_recordings(cameras: list, requested: list[str] | None) -> list[CameraRecording]:
    """The REQUESTED cameras among CAMERAS, a real platform's as the model declares them, that
    name a ROS image topic to record."""
    by_id = {camera.id: camera for camera in cameras if camera.topic}
    return [
        CameraRecording(camera_id, by_id[camera_id].topic, float(by_id[camera_id].rate_hz or 30.0))
        for camera_id in requested or ()
        if camera_id in by_id
    ]


def rgb_rows(message) -> bytes:
    """The image as tightly packed rows, the only layout rawvideo accepts."""
    row_size = message.width * 3
    data = bytes(message.data)
    if not message.step or message.step == row_size:
        return data
    return b"".join(
        data[row * message.step : row * message.step + row_size] for row in range(message.height)
    )


class RosImageSubscription:
    """A ROS image topic spun on a thread of its own, each message handed to `absorb`.

    rclpy is imported inside the thread, so a process without a ROS environment still runs and
    reports the missing environment as its error.
    """

    def __init__(self, topic: str, node_name: str):
        self.topic = topic
        self.node_name = node_name
        self.error: str | None = None
        self.stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self.spin, name=node_name, daemon=True)

    def spin(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except ImportError as exc:
            self.error = f"no ROS environment: {exc}"
            return self._ready.set()
        # own context: leaves any other rclpy in this process alone
        context = Context()
        node = executor = None
        try:
            rclpy.init(context=context)
            node = rclpy.create_node(self.node_name, context=context)
            # best-effort matches reliable publishers too
            node.create_subscription(Image, self.topic, self.absorb, qos_profile_sensor_data)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._ready.set()
            while not self.stop.is_set():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:  # noqa: BLE001 - ROS middleware exceptions are implementation-defined.
            self.error = str(exc)
        finally:
            self._ready.set()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)

    def absorb(self, message) -> None:
        """Take one image the topic delivered, on the spinning thread."""
        raise NotImplementedError


class RosImageRecorder(RosImageSubscription):
    """Write one RGB ROS image stream to MP4 without delaying the control process.

    Frames are placed in constant-rate slots by capture time, repeating the last
    frame across slots nothing arrived for, so the file runs at real-time length
    however unevenly the topic delivers.
    """

    def __init__(self, recording: CameraRecording, output: Path, queue_depth: int = 8):
        super().__init__(recording.topic, f"motion_spec_video_{recording.id}")
        self.recording = recording
        self.output = output
        self.dropped = 0
        self._frames: queue.Queue = queue.Queue(maxsize=queue_depth)
        self._ffmpeg: subprocess.Popen[bytes] | None = None
        self._writer = threading.Thread(
            target=self.write, name=f"motion-spec-video-{recording.id}-encode"
        )

    def start(self) -> None:
        """Start the ROS subscription before the controller begins."""
        self._writer.start()
        self._thread.start()
        self._ready.wait(timeout=2.0)

    def close(self) -> None:
        """Stop the subscription and finalize the MP4 if it received frames."""
        self.stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            self.error = self.error or "ROS image recorder did not stop"
        try:
            self._frames.put(None, timeout=5.0)
        except queue.Full:
            pass
        self._writer.join(timeout=15.0)
        if self._writer.is_alive():
            self.error = self.error or "ffmpeg did not finish"
        if self.dropped:
            self.error = self.error or f"dropped {self.dropped} frames (encoder fell behind)"

    def absorb(self, message) -> None:
        try:
            self._frames.put_nowait((time.time(), message))
        except queue.Full:
            self.dropped += 1

    def write(self) -> None:
        origin: float | None = None
        last_slot = 0
        last_frame = b""
        encoder: BufferedWriter | None = None
        try:
            while (item := self._frames.get()) is not None:
                receipt, message = item
                problem = image_problem(message)
                if problem:
                    self.error = problem
                    continue
                encoder = encoder or self.start_encoder(message)
                header_stamp = message.header.stamp
                stamp = (
                    header_stamp.sec + header_stamp.nanosec * 1e-9
                    if header_stamp.sec or header_stamp.nanosec
                    else receipt
                )
                if origin is None:
                    origin = stamp
                slot = round((stamp - origin) * self.recording.rate_hz)
                # a duplicate or late frame; the first frame always takes slot 0
                if slot <= last_slot and last_frame:
                    continue
                frame = rgb_rows(message)
                if not self.encode(encoder, frame, last_frame, slot - last_slot - 1):
                    return
                last_slot = slot
                last_frame = frame
        finally:
            self.finish_encoding(encoder)

    def encode(self, encoder: BufferedWriter, frame: bytes, last_frame: bytes, gap: int) -> bool:
        """Write LAST_FRAME into the GAP empty slots, then FRAME; False once ffmpeg has stopped."""
        try:
            encoder.writelines([last_frame] * max(gap, 0))
            encoder.write(frame)
            encoder.flush()
        except (BrokenPipeError, OSError) as exc:
            self.error = f"ffmpeg stopped: {exc}"
            return False
        return True

    def start_encoder(self, message) -> BufferedWriter:
        """Start ffmpeg encoding frames the size and pixel order of MESSAGE; the pipe into it."""
        self.output.parent.mkdir(parents=True, exist_ok=True)
        frames_in, frames_out = os.pipe()
        encoder = os.fdopen(frames_out, "wb")
        try:
            self._ffmpeg = subprocess.Popen(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-f",
                    "rawvideo",
                    "-pixel_format",
                    "rgb24" if message.encoding == "rgb8" else "bgr24",
                    "-video_size",
                    f"{message.width}x{message.height}",
                    "-framerate",
                    str(self.recording.rate_hz),
                    "-i",
                    "pipe:0",
                    "-an",
                    "-c:v",
                    "libx264",
                    "-pix_fmt",
                    "yuv420p",
                    str(self.output),
                ],
                stdin=frames_in,
                # Ctrl-C is for the controller, not ffmpeg
                start_new_session=True,
            )
        except BaseException:
            encoder.close()
            raise
        finally:
            # ffmpeg holds its own copy
            os.close(frames_in)
        return encoder

    def finish_encoding(self, encoder: BufferedWriter | None) -> None:
        """Close the pipe into ffmpeg and wait for it to finish the file."""
        if encoder is not None:
            encoder.close()
        if self._ffmpeg is None:
            return
        try:
            self._ffmpeg.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._ffmpeg.terminate()
            self._ffmpeg.wait()


def image_problem(message) -> str | None:
    """Why MESSAGE is no RGB image the recorder can encode, or None when it is one."""
    if message.encoding not in ("rgb8", "bgr8"):
        return f"unsupported image encoding {message.encoding!r}"
    if message.width <= 0 or message.height <= 0 or message.step < message.width * 3:
        return "invalid RGB image dimensions"
    return None
