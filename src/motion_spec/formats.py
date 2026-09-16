# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""What this motion-spec writes, and which versions of it this motion-spec can still read.

One table, so a file's version can be reported against a range rather than against a single
number: "version 3, this reads 1 to 2" says what to do; "unsupported" does not.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Format:
    """One generated file, the version written now and the oldest still readable."""

    file: str
    current: int
    oldest: int
    changes: dict[int, str]

    @property
    def supported(self) -> str:
        return f"{self.oldest}" if self.oldest == self.current else f"{self.oldest}-{self.current}"


FORMATS = {
    "config": Format(
        "motion-spec.config.toml",
        current=1,
        oldest=1,
        changes={1: "workspace and setup settings"},
    ),
    "journal": Format(
        ".motion-spec/journal.jsonl",
        current=1,
        oldest=1,
        changes={1: "one entry per command: ts, command, argv, cwd"},
    ),
    "marker": Format(
        "install marker",
        current=2,
        oldest=1,
        # A v1 marker predates `setup` adopting checkouts, so everything it recorded was cloned.
        changes={
            1: "the installed ref alone",
            2: "the ref, and where the source came from: cloned, adopted or pip",
        },
    ),
}


def check(name: str, version: int, path) -> str | None:
    """Raise when VERSION cannot be read, or return what to warn about when it is not current."""
    known = FORMATS[name]
    if version > known.current:
        raise ValueError(
            f"{path}: {known.file} version {version}, and this motion-spec reads "
            f"{known.supported}. It was written by a newer motion-spec."
        )
    if version < known.oldest:
        raise ValueError(
            f"{path}: {known.file} version {version}, and this motion-spec reads "
            f"{known.supported}. Regenerate it."
        )
    if version < known.current:
        return (
            f"{path}: {known.file} version {version}, now at {known.current} "
            f"({known.changes.get(known.current, 'changed')}). It is read as it stands."
        )
    return None
