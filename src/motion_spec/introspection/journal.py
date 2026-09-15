# SPDX-License-Identifier: MPL-2.0
"""Append-only journal of what was asked of a workspace, for auditable development studies.

JSON Lines: each entry stands alone, so an interrupted write damages one line. Writing it must
never interfere with the work, so failures are swallowed.
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

from motion_spec.setup import WORKSPACE_VARIABLE

JOURNAL_VARIABLE = "MOTION_SPEC_JOURNAL"
JOURNAL_FILE = Path(".motion-spec") / "journal.jsonl"


def journal_path() -> Path | None:
    """Where this invocation is journalled, or None outside a workspace."""
    override = os.environ.get(JOURNAL_VARIABLE, "").strip()
    if override:
        return Path(override).expanduser()
    named = os.environ.get(WORKSPACE_VARIABLE, "").strip()
    return Path(named).expanduser() / JOURNAL_FILE if named else None


def record(command: str) -> None:
    """Append this process's invocation of `command` to the journal."""
    try:
        path = journal_path()
        if path is None:
            return
        from motion_spec.formats import FORMATS

        path.parent.mkdir(parents=True, exist_ok=True)
        entry = {
            "v": FORMATS["journal"].current,
            "ts": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "command": command,
            "argv": sys.argv[1:],
            "cwd": str(Path.cwd()),
        }
        with path.open("a") as sink:
            sink.write(json.dumps(entry) + "\n")
    except OSError:
        pass


def entries(path: Path | None = None) -> list[dict]:
    """The journal, oldest first, skipping any line that is not a readable entry."""
    path = path or journal_path()
    if path is None or not path.is_file():
        return []
    from motion_spec.formats import FORMATS

    known = FORMATS["journal"]
    read = []
    for line in path.read_text(errors="replace").splitlines():
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        # An entry from a newer motion-spec is left where it is rather than half-read.
        if isinstance(entry, dict) and entry.get("v", known.oldest) <= known.current:
            read.append(entry)
    return read
