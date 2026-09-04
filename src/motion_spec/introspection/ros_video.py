# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Record real ROS RGB image topics as MP4 files."""

from __future__ import annotations

import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CameraRecording:
    """One declared RGB camera selected for a real-world recording."""

    id: str
    topic: str
    rate_hz: float


def real_camera_recordings(schema: dict, requested: list[str] | None) -> list[CameraRecording]:
    """Resolve requested real-camera names through the generated contract."""
    if not requested or schema.get("platform", {}).get("simulated"):
        return []
    cameras = {
        camera.get("id"): camera
        for camera in schema.get("cameras") or []
        if camera.get("id") and camera.get("topic")
    }
    recordings = []
    for camera_id in requested:
        camera = cameras.get(camera_id)
        if camera is None:
            continue
        recordings.append(
            CameraRecording(camera_id, camera["topic"], float(camera.get("rate_hz") or 30.0))
        )
    return recordings


class RosImageRecorder:
    """Write one RGB ROS image stream to MP4 without delaying the control process."""

    def __init__(self, recording: CameraRecording, output: Path):
        self.recording = recording
        self.output = output
        self.error: str | None = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._ffmpeg: subprocess.Popen[bytes] | None = None
        self._thread = threading.Thread(target=self._spin, name=f"motion-spec-video-{recording.id}")

    def start(self) -> None:
        """Start the ROS subscription before the controller begins."""
        self._thread.start()
        self._ready.wait(timeout=2.0)

    def close(self) -> None:
        """Stop the subscription and finalize the MP4 if it received frames."""
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            self.error = self.error or "ROS image recorder did not stop"

    def _absorb(self, message) -> None:
        if message.encoding not in ("rgb8", "bgr8"):
            self.error = f"unsupported image encoding {message.encoding!r}"
            return
        if self._ffmpeg is None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
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
                stdin=subprocess.PIPE,
                # Its own session: a terminal interrupt is for the controller, not for ffmpeg.
                start_new_session=True,
            )
        if message.width <= 0 or message.height <= 0 or message.step < message.width * 3:
            self.error = "invalid RGB image dimensions"
            return
        data = bytes(message.data)
        row_size = message.width * 3
        if message.step != row_size:
            data = b"".join(
                data[row * message.step : row * message.step + row_size]
                for row in range(message.height)
            )
        try:
            assert self._ffmpeg.stdin is not None
            self._ffmpeg.stdin.write(data)
            self._ffmpeg.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            self.error = f"ffmpeg stopped: {exc}"

    def _spin(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except ImportError as exc:
            self.error = f"no ROS image recorder: {exc}"
            return self._ready.set()
        context = rclpy.Context()
        node = executor = None
        try:
            rclpy.init(context=context)
            node = rclpy.create_node(f"motion_spec_video_{self.recording.id}", context=context)
            node.create_subscription(
                Image, self.recording.topic, self._absorb, qos_profile_sensor_data
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._ready.set()
            while not self._stop.is_set():
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
            if self._ffmpeg is not None:
                if self._ffmpeg.stdin is not None:
                    self._ffmpeg.stdin.close()
                try:
                    self._ffmpeg.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self._ffmpeg.terminate()
                    self._ffmpeg.wait()
