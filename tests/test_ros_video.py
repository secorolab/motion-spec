import io
from pathlib import Path
from types import SimpleNamespace

from motion_spec.introspection.ros_video import (
    CameraRecording,
    RosImageRecorder,
    frame_slot,
    image_stamp,
    rgb_rows,
)


def _image(stamp: float, fill: int, width: int = 2, height: int = 2, step: int | None = None):
    step = width * 3 if step is None else step
    row = bytes([fill] * (width * 3)) + bytes(step - width * 3)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(stamp), nanosec=int((stamp % 1) * 1e9))
        ),
        encoding="rgb8",
        width=width,
        height=height,
        step=step,
        data=row * height,
    )


class _Stdin(io.BytesIO):
    """Keeps what was written after the writer closes the pipe."""

    def close(self):
        self.written = self.getvalue()
        super().close()


class _FakeFfmpeg:
    def __init__(self):
        self.stdin = _Stdin()

    def wait(self, timeout=None):
        return 0


def test_frame_slot_rounds_to_the_nearest_constant_rate_slot() -> None:
    assert frame_slot(10.0, 10.0, 30.0) == 0
    assert frame_slot(10.034, 10.0, 30.0) == 1
    assert frame_slot(10.5, 10.0, 30.0) == 15


def test_image_stamp_prefers_the_driver_stamp() -> None:
    assert image_stamp(_image(12.5, 0), 99.0) == 12.5
    unstamped = _image(0.0, 0)
    assert image_stamp(unstamped, 99.0) == 99.0


def test_rgb_rows_strips_row_padding() -> None:
    padded = _image(0.0, 7, width=2, height=2, step=8)
    assert rgb_rows(padded) == bytes([7] * 12)


def test_writer_repeats_the_last_frame_across_missed_slots(tmp_path: Path) -> None:
    recorder = RosImageRecorder(CameraRecording("rk", "/rk/color", 10.0), tmp_path / "rk.mp4")
    fake = _FakeFfmpeg()
    recorder._open_ffmpeg = lambda message: setattr(recorder, "_ffmpeg", fake)  # type: ignore[method-assign]

    recorder._writer.start()
    # slot 0, slot 1, a duplicate in slot 1, then slot 4: two fill frames precede it.
    for stamp, fill in ((100.0, 1), (100.1, 2), (100.12, 3), (100.4, 4)):
        recorder._absorb(_image(stamp, fill))
    recorder._frames.put(None)
    recorder._writer.join(timeout=5.0)

    written = fake.stdin.written
    frames = [written[i : i + 12] for i in range(0, len(written), 12)]
    assert [frame[0] for frame in frames] == [1, 2, 2, 2, 4]
    assert recorder.error is None


def test_absorb_counts_frames_the_encoder_could_not_take(tmp_path: Path) -> None:
    recorder = RosImageRecorder(
        CameraRecording("rk", "/rk/color", 10.0), tmp_path / "rk.mp4", queue_depth=1
    )
    recorder._absorb(_image(1.0, 1))
    recorder._absorb(_image(1.1, 2))
    assert recorder.dropped == 1
