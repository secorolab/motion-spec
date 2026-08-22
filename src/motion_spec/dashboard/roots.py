# SPDX-License-Identifier: MPL-2.0

"""Where the dashboard reads: the two roots, and how a request names a path inside them.

The roots are rebound while the server runs -- someone points the page at another output
directory -- so every other module reaches them through this one rather than importing
their values, which would freeze whichever root was current at import time.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path

GENERATION_DIR_ENV = "MOTION_SPEC_GEN"


# Roots the dashboard browses, replaced at startup by `serve` and by /api/roots.
GENERATIONS = Path(os.environ.get(GENERATION_DIR_ENV, "").strip() or Path.cwd())


WORKSPACE = GENERATIONS.parent


FRONTEND = Path(__file__).with_name("frontend")


IGNORED = {
    "build",
    ".git",
    ".venv",
    "generations",
    "install",
    "log",
    "__pycache__",
    "test",
    "tests",
}


# The DSL's own file types. A model's other files -- robot.toml, a yaml beside it -- are
# listed by where they sit instead (see authored_sources), so this stays free of suffixes
# like .toml that mean something else everywhere else in the workspace.
AUTHORED = (".robmot", ".fsm", ".scenex", ".scene", ".ktree", ".bdd", ".bddx")


def current_roots() -> dict:
    """Return the currently browsed source and generation roots."""
    return {"sources": str(WORKSPACE), "logs": str(GENERATIONS)}


def set_root(kind: str, value: str) -> dict:
    """Change one dashboard browse root after validating its directory."""
    global GENERATIONS, WORKSPACE
    path = Path(value).expanduser().resolve()
    if not path.is_dir():
        raise ValueError("root must be an existing directory")
    if kind == "logs":
        GENERATIONS = path
    elif kind == "sources":
        WORKSPACE = path
    else:
        raise ValueError("unknown root")
    directory_size.cache_clear()
    storage_info.cache_clear()
    return current_roots()


def pick_root(kind: str) -> dict:
    """Open the host folder chooser and use its selected directory."""
    initial = current_roots().get(kind)
    if initial is None:
        raise ValueError("unknown root")
    result = subprocess.run(
        ["zenity", "--file-selection", "--directory", "--filename", f"{initial}/"],
        check=False,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if result.returncode:
        raise ValueError("folder selection cancelled")
    return set_root(kind, result.stdout.strip())


def trash(path: Path) -> None:
    """Move a path to the desktop trash. Nothing the dashboard removes should be unrecoverable."""
    result = subprocess.run(
        ["gio", "trash", "--", str(path)], check=False, capture_output=True, text=True
    )
    if result.returncode:
        raise ValueError(f"could not move {path.name} to trash: {result.stderr.strip()}")


def relative_path(root: Path, value: str) -> Path:
    """Resolve a request path inside root, rejecting traversal and missing paths."""
    path = (root / value).resolve()
    if root.resolve() not in (path, *path.parents) or not path.exists():
        raise ValueError("unknown path")
    return path


def expected_path(root: Path, value: str) -> Path:
    """Resolve a request path inside root that need not exist yet, e.g. a run just named."""
    path = (root / value).resolve()
    if root.resolve() not in (path, *path.parents):
        raise ValueError("unknown path")
    return path


def json_file(path: Path) -> dict:
    """One generated JSON document, parsed once per version of the file.

    A contract is written once and read on every list, every poll and every page. Keying the
    cache on the file's mtime and size means a regenerated file is read again and an unchanged
    one is not -- these documents run to hundreds of kilobytes.
    """
    if not path.exists():
        return {}
    stamp = path.stat()
    return _parsed_json(path, stamp.st_mtime_ns, stamp.st_size)


@lru_cache(maxsize=64)
def _parsed_json(path: Path, _mtime_ns: int, _size: int) -> dict:
    return json.loads(path.read_text())


@lru_cache(maxsize=256)
def directory_size(path: Path) -> int:
    """Byte size of one generation bundle, cached for the dashboard session."""
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


@lru_cache(maxsize=1)
def storage_info() -> dict:
    """Generation and frame-log storage totals for the dashboard sidebar."""
    files = [path for path in GENERATIONS.rglob("*") if path.is_file()]
    return {
        "generations_bytes": sum(path.stat().st_size for path in files),
        "logs_bytes": sum(path.stat().st_size for path in files if path.name == "frame_log.pb"),
    }


def stamp_iso(name: str) -> str | None:
    """A run or generation stamp as ISO 8601 UTC, for the browser to show in local time."""
    match = re.fullmatch(r"(\d{8}T\d{6}\d{6})Z", name)
    if not match:
        return None
    return (
        datetime.strptime(match.group(1), "%Y%m%dT%H%M%S%f")
        .replace(tzinfo=timezone.utc)
        .isoformat()
    )


LAYOUT_REL = "generated/contract/frame_layout.json"


_TRACED: dict[str, str] = {}


def trace(line: str) -> None:
    """Print a state line the first time it changes, so a run leaves a readable trail."""
    key = line.split(":")[0]
    if _TRACED.get(key) != line:
        _TRACED[key] = line
        print(f"[trace] {time.strftime('%H:%M:%S')} {line}", file=sys.stderr, flush=True)


RUN_LOG = "dashboard-run.log"
