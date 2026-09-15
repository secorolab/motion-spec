# SPDX-License-Identifier: MPL-2.0

"""Where the dashboard reads: the two roots, and how a request names a path inside them.

The roots are rebound while the server runs -- someone points the page at another output
directory -- so every other module reaches them through this one rather than importing
their values, which would freeze whichever root was current at import time.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path

# Re-exported: the dashboard's callers ask roots for it, and it is the same removal every
# other part of motion-spec uses.
from motion_spec.utils import trash as trash

GENERATION_DIR_ENV = "MOTION_SPEC_GEN"


# Roots the dashboard browses, replaced at startup by `serve` and by /api/roots.
def _default_generations() -> Path:
    """The root the CLI writes to, or the working directory: importing must not fail."""
    from motion_spec.setup import generations_root

    try:
        return generations_root()
    except RuntimeError:
        return Path.cwd()


GENERATIONS = _default_generations()


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
    # What `motion-spec setup` checks out under src/: four third-party repositories, none of
    # them anybody's authored model, and between them more files than the workspace itself.
    "thirdparty",
}


# The DSL's own file types, plus .toml for the config a model names in its exec-context. By
# extension alone: "anything beside a model" swept in a whole repository from one .robmot at
# the top of a workspace.
AUTHORED = (".robmot", ".fsm", ".scenex", ".scene", ".ktree", ".bdd", ".bddx", ".toml")


# .toml belongs to the wider world too, so the tooling files that spell it are named here.
NOT_AUTHORED = {"METADATA.toml", "netlify.toml", "pixi.toml", "pyproject.toml", "theme.toml"}


def current_roots() -> dict:
    """Return the currently browsed source and generation roots, and what is serving them."""
    from importlib import metadata

    try:
        version = metadata.version("motion_spec")
    except metadata.PackageNotFoundError:
        version = None
    return {"sources": str(WORKSPACE), "logs": str(GENERATIONS), "version": version}


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


def choose(initial: str, *, directory: bool = True) -> str:
    """The path the host chooser returns, starting at INITIAL."""
    result = subprocess.run(
        [
            "zenity",
            "--file-selection",
            *(["--directory"] if directory else []),
            "--filename",
            initial,
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if result.returncode:
        raise ValueError("selection cancelled")
    return result.stdout.strip()


def pick_root(kind: str) -> dict:
    """Open the host folder chooser and use its selected directory."""
    initial = current_roots().get(kind)
    if initial is None:
        raise ValueError("unknown root")
    return set_root(kind, choose(f"{initial}/"))


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

    Keyed on mtime and size, so a regenerated file is read again and an unchanged one is not.
    These run to hundreds of kilobytes and are read on every list, poll and page.
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
        # Either name: a run still being written, and an archived one that has been packed.
        "logs_bytes": sum(
            path.stat().st_size
            for path in files
            if path.name in ("frame_log.pb", "frame_log.pb.zst")
        ),
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
