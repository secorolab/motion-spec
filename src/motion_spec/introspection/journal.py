# SPDX-License-Identifier: MPL-2.0
"""Append-only journal of diagnostic invocations, for auditable development studies.

Every inspection command (replay, dashboard, ir, diff) appends one line: when, from where,
what was asked. The journal is evidence about *how* a failure was diagnosed — by the record
or by other means — so writing it must never interfere with the diagnosis itself: failures
to write are swallowed.
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path


def journal_path() -> Path:
    override = os.environ.get("MOTION_SPEC_JOURNAL")
    if override:
        return Path(override)
    return Path.home() / ".motion_spec" / "journal.jsonl"


def record(command: str) -> None:
    """Append this process's invocation of `command` to the journal."""
    try:
        path = journal_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        entry = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "command": command,
            "argv": sys.argv[1:],
            "cwd": str(Path.cwd()),
        }
        with path.open("a") as sink:
            sink.write(json.dumps(entry) + "\n")
    except OSError:
        pass
