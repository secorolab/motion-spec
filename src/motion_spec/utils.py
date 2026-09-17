# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""What more than one part of motion-spec needs: removal, and running a tool visibly."""

from __future__ import annotations

import errno
import json
import os
import pty
import re
import shlex
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

STATE_DIRECTORY = ".motion-spec"
LOG_DIRECTORY = "logs"

# Every colour the toolchain writes, as SGR parameters. One table: the CLI paints its own lines
# from it, and a tool motion-spec runs is handed it, because that tool is another process and the
# CLI cannot reach into a line it has already printed.
SGR = "\x1b["  # what a terminal reads as: the rest of this is a colour, until RESET
RESET = f"{SGR}0m"
STAMP_COLOUR = "90"
LEVEL_COLOURS = {
    "info": "",
    "step": "1;34",
    "warn": "1;33",
    "error": "1;31",
    "done": "1;32",
}
LEVEL_WIDTH = 6


def paint(text: str, colour: str) -> str:
    """TEXT in COLOUR, or unchanged when the colour is the terminal's own."""
    return f"{SGR}{colour}m{text}{RESET}" if colour else text


# Painted and padded here, so a tool is handed the finished word rather than the means to
# build it: padding counts an escape the reader cannot see, and one place should get that right.
LEVEL_LABELS = {
    level: paint(f"{level:<{LEVEL_WIDTH}}", colour) for level, colour in LEVEL_COLOURS.items()
}

LOG_FORMAT_VARIABLE = "MOTION_SPEC_LOG_FORMAT"
LOG_FORMAT = f"{paint('%(asctime)s', STAMP_COLOUR)}  %(levelname)s %(message)s"
LOG_DATEFMT_VARIABLE = "MOTION_SPEC_LOG_DATEFMT"
LOG_DATEFMT = "%H:%M:%S"
LOG_LEVELS_VARIABLE = "MOTION_SPEC_LOG_LEVELS"


def tool_environment(env: dict[str, str] | None = None) -> dict[str, str]:
    """ENV with the format and the level words motion-spec's own tools write their lines in."""
    return {
        **(env if env is not None else os.environ),
        LOG_FORMAT_VARIABLE: LOG_FORMAT,
        LOG_DATEFMT_VARIABLE: LOG_DATEFMT,
        LOG_LEVELS_VARIABLE: json.dumps(LEVEL_LABELS),
    }


def command_log(root: Path, command: str) -> Path:
    """The file one invocation of COMMAND tees its tools' output into."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
    return root / STATE_DIRECTORY / LOG_DIRECTORY / f"{stamp}Z-{command}.log"


def generation_log(generation: Path) -> Path:
    """One console per generation: everything done to it, in the order it happened.

    A file per command would split what the operator watched as a single stream -- generating
    and building are one sitting, and the build's first line answers the generation's last.
    """
    return generation / LOG_DIRECTORY / "console.log"


class _Mirror:
    """Everything written to a stream, written to a file as well."""

    def __init__(self, stream, sink):
        self._stream = stream
        self._sink = sink
        self._readable = _for_the_file()

    def write(self, text: str) -> int:
        self._sink.write(self._readable(text.encode("utf-8", "replace")))
        self._sink.flush()
        return self._stream.write(text)

    def __getattr__(self, name):
        return getattr(self._stream, name)


# Two spaces before a line motion-spec did not write itself, so a tool's output reads as
# belonging under the stamped line that started it rather than as a line of its own.
INDENT = "  "


# A line that already carries the stamp is motion-spec speaking through one of its own tools,
# and belongs in the same column as the rest of what motion-spec says.
STAMPED = re.compile(rb"^(\x1b\[[0-9;]*m)?\d\d:\d\d:\d\d[\s\x1b]")


def _indenter():
    """Indent every line of a byte stream arriving in arbitrary chunks.

    A carriage return redraws the line it is already on -- a progress bar -- so only a newline
    starts one worth indenting.
    """
    pad = INDENT.encode()
    fresh = True

    def prefix(line: bytes) -> bytes:
        return b"" if not line or STAMPED.match(line) else pad

    def indent(chunk: bytes) -> bytes:
        nonlocal fresh
        lines = chunk.split(b"\n")
        out = [(prefix(lines[0]) if fresh else b"") + lines[0]]
        out += [prefix(line) + line for line in lines[1:]]
        fresh = chunk.endswith(b"\n")
        return b"\n".join(out)

    return indent


ANSI = re.compile(rb"\x1b\[[0-9;?]*[a-zA-Z]")


def _for_the_file():
    """Strip colour and progress-bar redraws from a copy bound for the log, not the screen."""
    pending = bytearray()
    pad = INDENT.encode()

    def clean(line: bytes) -> bytes:
        # rstrip first: under a pty every line ends CRLF, which redraws nothing.
        text = ANSI.sub(b"", line.rstrip(b"\r").rpartition(b"\r")[2])
        # A redrawn line lost its indent along with everything before the last return.
        if text and line.startswith(pad) and not text.startswith(pad):
            text = pad + text
        return text + b"\n"

    def readable(chunk: bytes = b"", *, last: bool = False) -> bytes:
        pending.extend(chunk)
        out = bytearray()
        while True:
            end = pending.find(b"\n")
            if end < 0:
                break
            out += clean(bytes(pending[:end]))
            del pending[: end + 1]
        if last and pending:
            out += clean(bytes(pending))
            pending.clear()
        return bytes(out)

    return readable


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%H:%M:%S")


def _outcome(returncode: int, started: float) -> str:
    """How a tool ended. The signal is named, since SIGKILL is what identifies an OOM kill."""
    elapsed = time.monotonic() - started
    if returncode < 0:
        name = next(
            (member.name for member in signal.Signals if member.value == -returncode),
            f"signal {-returncode}",
        )
        return f"# killed by {name} after {elapsed:.1f}s"
    return f"# exit {returncode} after {elapsed:.1f}s"


def log_header(log: Path | None, command: str, facts: dict[str, object]) -> None:
    """Open LOG with what produced it: after a crash the file is all that is left."""
    if log is None:
        return
    log.parent.mkdir(parents=True, exist_ok=True)
    width = max((len(key) for key in facts), default=0)
    lines = [
        f"# motion-spec {command}",
        f"# started    {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
        f"# argv       {shlex.join(sys.argv)}",
        *(f"# {key:<{width}} {value}" for key, value in facts.items()),
        "",
    ]
    with log.open("ab") as sink:
        sink.write("\n".join(lines).encode())


def usable_cores() -> int:
    """Cores this process may actually run on: a cgroup or a taskset narrows what the host has."""
    if hasattr(os, "sched_getaffinity"):
        return len(os.sched_getaffinity(0)) or 1
    return os.cpu_count() or 1


def total_memory() -> int | None:
    """Bytes of RAM in this machine, or None where the system will not say."""
    try:
        return os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE")
    except (ValueError, OSError):  # not a POSIX machine, or the names are unknown
        return None


def machine_facts() -> dict[str, object]:
    """What about this machine decides whether a build finishes or takes it down."""
    memory = total_memory()
    return {
        "host": f"{os.uname().nodename} {os.uname().sysname} {os.uname().release}",
        "cpus": f"{usable_cores()} of {os.cpu_count()} usable",
        "memory": "unknown" if memory is None else f"{memory / 1024**3:.1f} GiB",
    }


def show_warning(message, category, filename, lineno, file=None, line=None) -> None:
    """A Python warning, indented like every other line motion-spec did not write."""
    import warnings

    text = warnings.formatwarning(message, category, filename, lineno, line)
    stream = file or sys.stderr
    stream.write("".join(f"{INDENT}{part}\n" for part in text.rstrip().splitlines()))


@contextmanager
def mirrored_stderr(log: Path | None):
    """Keep what motion-spec itself says in LOG, beside what its tools said.

    The warnings a model raises and the lines the CLI prints are this process's, so `tee` never
    sees them; without this the log holds the tools' half of a session the operator watched whole.
    """
    if log is None:
        yield
        return
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("ab") as sink:
        previous = sys.stderr
        sys.stderr = _Mirror(previous, sink)
        try:
            yield
        finally:
            sys.stderr = previous


def tee(
    command: list,
    *,
    log: Path | None,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    check: bool = True,
) -> int:
    """Run COMMAND, showing its output as it happens and keeping a copy in LOG.

    Under a pty: a pipe would make git and cmake switch to their non-interactive output.
    """
    argv = [str(part) for part in command]
    if log is None:
        return subprocess.run(argv, cwd=cwd, env=env, check=check).returncode

    log.parent.mkdir(parents=True, exist_ok=True)
    controller, worker = pty.openpty()
    indent = _indenter()
    readable = _for_the_file()
    started = time.monotonic()
    try:
        with log.open("ab") as sink:
            sink.write(f"\n[{_stamp()}] $ {shlex.join(argv)}\n".encode())
            sink.flush()
            process = subprocess.Popen(
                argv,
                cwd=str(cwd) if cwd else None,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=worker,
                stderr=worker,
                close_fds=True,
            )
            # Closed here, or the read below never sees EOF.
            os.close(worker)
            worker = -1
            while True:
                try:
                    chunk = os.read(controller, 65536)
                except OSError as error:
                    if error.errno == errno.EIO:  # the child closed the other end
                        break
                    raise
                if not chunk:
                    break
                chunk = indent(chunk)
                sys.stderr.buffer.write(chunk)
                sys.stderr.buffer.flush()
                sink.write(readable(chunk))
                # Two handles append here; unflushed they interleave by buffer, not by time.
                sink.flush()
            returncode = process.wait()
            sink.write(readable(last=True))
            sink.write(f"[{_stamp()}] {_outcome(returncode, started)}\n".encode())
            sink.flush()
    finally:
        os.close(controller)
        if worker >= 0:
            os.close(worker)
    if check and returncode:
        raise subprocess.CalledProcessError(returncode, argv)
    return returncode


def trash(path: Path) -> None:
    """Move PATH to the desktop trash."""
    try:
        result = subprocess.run(
            ["gio", "trash", "--", str(path)], check=False, capture_output=True, text=True
        )
    except FileNotFoundError as error:
        # Deleting it instead is the one outcome this module exists to prevent.
        raise ValueError(f"cannot remove {path.name}: gio is not installed") from error
    if result.returncode:
        raise ValueError(f"could not move {path.name} to trash: {result.stderr.strip()}")


def trash_if_present(path: Path) -> bool:
    """Move PATH to the trash when it is there at all, saying whether anything moved."""
    if not (path.exists() or path.is_symlink()):
        return False
    trash(path)
    return True


def tree_size(path: Path) -> int:
    """Bytes under PATH, or its own size when it is a file. A broken symlink counts as nothing."""
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def human_bytes(count: int) -> str:
    """A size to put in front of someone before they answer a question about deleting it."""
    size = float(count)
    for unit in ("B", "KiB", "MiB"):
        if size < 1024:
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} GiB"
