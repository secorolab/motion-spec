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
        """STARTED / COMPLETED / FAILED / INTERRUPTED, as REC recorded it."""
        return rec_run_lifecycle_from_file(self.dir / "rec.ld.json").get("status")

    @property
    def health(self) -> dict | None:
        return read_health(self.log_path)

    def is_live(self, within_s: float = 3.0) -> bool:
        """Started, and the frame log grew within the last `within_s` seconds."""
        if self.status != "STARTED" or not self.log_path.exists():
            return False
        return time.time() - self.log_path.stat().st_mtime <= within_s


@dataclass
class GenerationInfo:
    """One `<root>/<model>/<timestamp>` generation bundle."""

    dir: Path

    @property
    def model(self) -> str:
        return self.dir.parent.name

    @property
    def timestamp(self) -> str:
        return self.dir.name

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
        found = {
            layout.parent.parent.parent
            for root in self.roots
            if root.is_dir()
            for layout in root.glob(str(Path("*") / "*" / LAYOUT_REL))
        }
        return sorted(
            (GenerationInfo(d) for d in found),
            key=lambda gen: (gen.model, gen.timestamp),
            reverse=True,
        )

    def runs(self) -> list[tuple[GenerationInfo, RunInfo]]:
        return [(gen, run) for gen in self.generations() for run in gen.runs]
