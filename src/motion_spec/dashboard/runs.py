# SPDX-License-Identifier: MPL-2.0
"""Discover generations and their runs on disk."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from rec import State, Verdict
from rec.stores.file_store import FileStore

from motion_spec.dashboard.frames import FrameLayout
from motion_spec.dashboard.roots import HEADER_REL
from motion_spec.runs.provenance import PROVENANCE_DIR, rec_run
from motion_spec.telemetry import frame_log_pb
from motion_spec.telemetry.frame_log_pb import read_health

LOG_REL = Path("logs") / "frame_log.pb"
ENDED = {State.COMPLETE, State.CANCELED}
LIVE_WINDOW_S = 10.0

LIFECYCLE_CACHE: dict[Path, tuple[tuple, tuple[State, Verdict]]] = {}


def lifecycle(run_dir: Path) -> tuple[State, Verdict]:
    try:
        stat = FileStore(run_dir / PROVENANCE_DIR).path(run_dir.name).stat()
    except OSError:
        return State.NEW, Verdict.UNAVAILABLE
    key = (stat.st_mtime_ns, stat.st_size)
    cached = LIFECYCLE_CACHE.get(run_dir)
    if cached is not None and cached[0] == key:
        return cached[1]
    lifecycle = rec_run(run_dir).lifecycle()
    LIFECYCLE_CACHE[run_dir] = (key, lifecycle)
    return lifecycle


@dataclass
class RunInfo:
    """One `<generation>/runs/<run-id>` directory."""

    dir: Path

    @property
    def run_id(self) -> str:
        return self.dir.name

    @property
    def log_path(self) -> Path:
        return frame_log_pb.log_path(self.dir / LOG_REL)

    @property
    def state(self) -> State:
        """Where the run is, as rec recorded it; `new` while nothing is recorded."""
        return lifecycle(self.dir)[0]

    @property
    def verdict(self) -> Verdict:
        """How the run turned out; `unavailable` until it is complete."""
        return lifecycle(self.dir)[1]

    @property
    def health(self) -> dict | None:
        return read_health(self.log_path)

    def is_live(self) -> bool:
        """Not finished, and the frame log grew within the last `LIVE_WINDOW_S` seconds.

        A run killed outright never records an end, so the log's age decides. The window is
        wide because buffered writes make mtime lag. A caller polling the shm block should
        prefer its advancing seq.
        """
        if self.state in ENDED or not self.log_path.exists():
            return False
        return time.time() - self.log_path.stat().st_mtime <= LIVE_WINDOW_S


@dataclass
class GenerationInfo:
    """One generation bundle, at `<root>/<name>/<timestamp>`; NAME is the model's stem or `--name`."""

    dir: Path
    root: Path | None = None

    @property
    def model(self) -> str:
        if self.root is None:
            return self.dir.parent.name
        parts = self.dir.relative_to(self.root).parts
        return parts[0] if len(parts) > 1 else self.dir.parent.name

    @property
    def built_at(self) -> float:
        """When this bundle was last written, which is what "latest" means to someone reading."""
        return self.dir.stat().st_mtime

    @property
    def layout(self) -> FrameLayout:
        return FrameLayout(frame_log_pb.read_contract(self.dir / HEADER_REL))

    @property
    def runs(self) -> list[RunInfo]:
        runs_dir = self.dir / "runs"
        if not runs_dir.is_dir():
            return []
        return [RunInfo(d) for d in sorted(runs_dir.iterdir(), reverse=True) if d.is_dir()]


class GenerationCatalog:
    """Every generation under the output root, newest first."""

    def __init__(self, root):
        self.root = Path(root)

    def generations(self) -> list[GenerationInfo]:
        # `latest` links to a bundle already found
        found = {
            layout.parent.parent.parent
            for depth in ("*/*", "*/*/*")
            for layout in self.root.glob(f"{depth}/{HEADER_REL}")
            if not layout.parent.parent.parent.is_symlink()
        }
        generations = [GenerationInfo(d, self.root) for d in found]
        newest: dict[str, float] = {}
        for generation in generations:
            built = generation.built_at
            newest[generation.model] = max(newest.get(generation.model, built), built)
        return sorted(generations, key=lambda gen: (newest[gen.model], gen.built_at), reverse=True)
