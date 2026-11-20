# SPDX-License-Identifier: MPL-2.0

"""Where the dashboard reads: the two roots, and how a request names a path inside them.

The roots are rebound while the server runs, so other modules read them through this one
rather than importing their values.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
from functools import lru_cache
from importlib import metadata
from pathlib import Path

from motion_spec.runs.provenance import FRAME_LOG_PACKED, artifact_size
from motion_spec.setup import ENVIRONMENT_FILES, generations_root
from motion_spec.telemetry.frame_log_pb import HEADER_FILE


def pidfile(port: int | str, role: str = "dashboard") -> Path:
    """Where the dashboard on PORT, or the lab it started, records its pid; "*" globs them."""
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR") or tempfile.gettempdir())
    return runtime / f"motion-spec-{role}-{port}.pid"


# importing must not fail
try:
    GENERATIONS = generations_root()
except RuntimeError:
    GENERATIONS = Path.cwd()


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
    "thirdparty",
}


AUTHORED = (".robmot", ".fsm", ".scenex", ".scene", ".ktree", ".bdd", ".bddx", ".toml")


NOT_AUTHORED = {"METADATA.toml", "netlify.toml", "pixi.toml", "pyproject.toml", "theme.toml"}

AUTHORED_NAMES = set(ENVIRONMENT_FILES)


def package_version() -> str | None:
    """The installed motion-spec's version; None when it is not installed as a package."""
    try:
        return metadata.version("motion_spec")
    except metadata.PackageNotFoundError:
        return None


def current_roots() -> dict:
    """Return the currently browsed source and generation roots, and what is serving them."""
    return {"sources": str(WORKSPACE), "logs": str(GENERATIONS), "version": package_version()}


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
    path = expected_path(root, value)
    if not path.exists():
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
    """
    if not path.exists():
        return {}
    stamp = path.stat()
    return parsed_json(path, stamp.st_mtime_ns, stamp.st_size)


@lru_cache(maxsize=64)
def parsed_json(path: Path, _mtime_ns: int, _size: int) -> dict:
    return json.loads(path.read_text())


directory_size = lru_cache(maxsize=256)(artifact_size)


@lru_cache(maxsize=1)
def storage_info() -> dict:
    """Generation and frame-log storage totals for the dashboard sidebar."""
    files = [path for path in GENERATIONS.rglob("*") if path.is_file()]
    return {
        "generations_bytes": sum(path.stat().st_size for path in files),
        # a log being written, or a packed archived one
        "logs_bytes": sum(
            path.stat().st_size
            for path in files
            if path.name in ("frame_log.pb", FRAME_LOG_PACKED)
        ),
    }


HEADER_REL = f"generated/contract/{HEADER_FILE}"
STATE_REL = ".dashboard"


_TRACED: dict[str, str] = {}


def trace(line: str) -> None:
    """Print a state line the first time it changes, so a run leaves a readable trail."""
    key = line.split(":")[0]
    if _TRACED.get(key) != line:
        _TRACED[key] = line
        print(f"[trace] {time.strftime('%H:%M:%S')} {line}", file=sys.stderr, flush=True)


RUN_LOG = "dashboard-run.log"
