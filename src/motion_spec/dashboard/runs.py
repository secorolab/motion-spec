# SPDX-License-Identifier: MPL-2.0
"""Discover generations and their runs on disk."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

from motion_spec.dashboard.frames import FrameLayout
from motion_spec.introspection.provenance import rec_run_lifecycle_from_file
from motion_spec.introspection.replay import read_health

LAYOUT_REL = Path("generated") / "contract" / "frame_layout.json"
LOG_REL = Path("logs") / "frame_log.pb"
# REC's run-state vocabulary (see provenance._REC_RUN_STATUS); the rest -- QUEUED, RUNNING --
# mean the run may still produce frames.
TERMINAL_STATUS = frozenset({"COMPLETED", "FAILED", "INTERRUPTED", "CANCELLED"})


@dataclass
class RunInfo:
    """One `<generation>/runs/<run-id>` directory."""

    dir: Path

    @property
    def run_id(self) -> str:
        return self.dir.name

    @property
    def log_path(self) -> Path:
        return self.dir / LOG_REL

    @property
    def manifest(self) -> dict | None:
        path = self.dir / "manifest.json"
        return json.loads(path.read_text()) if path.exists() else None

    @property
    def status(self) -> str | None:
        """QUEUED / RUNNING / COMPLETED / FAILED / INTERRUPTED / CANCELLED, as REC recorded it."""
        return rec_run_lifecycle_from_file(self.dir / "rec.ld.json").get("status")

    @property
    def health(self) -> dict | None:
        return read_health(self.log_path)

    def is_live(self, within_s: float = 10.0) -> bool:
        """Not finished, and the frame log grew within the last `within_s` seconds.

        Asking the status alone is not enough: a run killed outright never records a terminal
        state, so a stale log is what actually distinguishes it from one still ticking. The log
        is written through buffered stdio, so mtime lags the tick by an unpredictable margin --
        hence the wide window. A caller already polling the shm block has the sharper signal in
        its advancing seq and should prefer it.
        """
        if self.status in TERMINAL_STATUS or not self.log_path.exists():
            return False
        return time.time() - self.log_path.stat().st_mtime <= within_s


@dataclass
class GenerationInfo:
    """One generation bundle, at `<root>/<model>/<timestamp>` or under a named output dir.

    `motion-spec gen -o <dir>` writes `<dir>/<model>/<timestamp>` and a `<dir>/latest` link,
    so the folder someone named is the first segment under the root either way.
    """

    dir: Path
    root: Path | None = None

    @property
    def model(self) -> str:
        if self.root is None:
            return self.dir.parent.name
        parts = self.dir.relative_to(self.root).parts
        return parts[0] if len(parts) > 1 else self.dir.parent.name

    @property
    def timestamp(self) -> str:
        return self.dir.name

    @property
    def built_at(self) -> float:
        """When this bundle was last written, which is what "latest" means to someone reading."""
        return self.dir.stat().st_mtime

    @property
    def built(self) -> bool:
        return (self.dir / "build" / "main").exists()

    @property
    def layout(self) -> FrameLayout:
        return FrameLayout.load(self.dir / LAYOUT_REL)

    @property
    def runs(self) -> list[RunInfo]:
        runs_dir = self.dir / "runs"
        if not runs_dir.is_dir():
            return []
        return [RunInfo(d) for d in sorted(runs_dir.iterdir(), reverse=True) if d.is_dir()]


class GenerationCatalog:
    """Every generation under a set of output roots, newest first."""

    def __init__(self, roots):
        self.roots = [Path(root) for root in roots]

    def generations(self) -> list[GenerationInfo]:
        # a bundle sits two or three levels down, and `latest` is a link to one already found
        found = {
            layout.parent.parent.parent: root
            for root in self.roots
            if root.is_dir()
            for depth in ("*/*", "*/*/*")
            for layout in root.glob(f"{depth}/{LAYOUT_REL}")
            if not any(part.is_symlink() for part in (layout.parent.parent.parent,))
        }
        generations = [GenerationInfo(d, root) for d, root in found.items()]
        # a model is as recent as its newest generation, so the list leads with what was last built
        newest: dict[str, float] = {}
        for generation in generations:
            built = generation.built_at
            newest[generation.model] = max(newest.get(generation.model, built), built)
        return sorted(
            generations,
            key=lambda gen: (newest[gen.model], gen.built_at),
            reverse=True,
        )

    def runs(self) -> list[tuple[GenerationInfo, RunInfo]]:
        return [(gen, run) for gen in self.generations() for run in gen.runs]
