# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""What more than one part of motion-spec needs: removal, and running a tool visibly."""

from __future__ import annotations

import errno
import os
import pty
import re
import shlex
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from datetime import UTC, datetime
from pathlib import Path

STATE_DIRECTORY = ".motion-spec"
LOG_DIRECTORY = "logs"

SGR = "\x1b["
RESET = f"{SGR}0m"
STAMP_COLOUR = "90"
LEVEL_COLOURS = {"info": "", "step": "1;34", "warn": "1;33", "error": "1;31", "done": "1;32"}
LEVEL_WIDTH = 6


LEVEL_LABELS = {
    level: f"{SGR}{colour}m{level:<{LEVEL_WIDTH}}{RESET}" if colour else f"{level:<{LEVEL_WIDTH}}"
    for level, colour in LEVEL_COLOURS.items()
}

LOG_DATEFMT = "%H:%M:%S"


def command_log(root: Path, command: str) -> Path:
    """The file one invocation of COMMAND tees its tools' output into."""
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%f")
    return root / STATE_DIRECTORY / LOG_DIRECTORY / f"{stamp}Z-{command}.log"


class Mirror:
    """Everything written to a stream, written to a file as well."""

    def __init__(self, stream, sink):
        self._stream = stream
        self._sink = sink
        self._readable = for_the_file()

    def write(self, text: str) -> int:
        self._sink.write(self._readable(text.encode("utf-8", "replace")))
        self._sink.flush()
        return self._stream.write(text)

    def __getattr__(self, name):
        return getattr(self._stream, name)


INDENT = "  "


STAMPED = re.compile(rb"^(\x1b\[[0-9;]*m)?\d\d:\d\d:\d\d[\s\x1b]")


def indenter():
    """Indent every line of a byte stream arriving in arbitrary chunks.

    A carriage return redraws the line it is already on -- a progress bar -- so only a newline
    starts one worth indenting.
    """
    pad = INDENT.encode()
    fresh = True

    def indent(chunk: bytes) -> bytes:
        nonlocal fresh
        indented = b"\n".join(
            pad + line if (fresh or number) and line and not STAMPED.match(line) else line
            for number, line in enumerate(chunk.split(b"\n"))
        )
        fresh = chunk.endswith(b"\n")
        return indented

    return indent


ANSI = re.compile(rb"\x1b\[[0-9;?]*[a-zA-Z]")


def for_the_file():
    """Strip colour and progress-bar redraws from a copy bound for the log, not the screen."""
    pending = bytearray()
    pad = INDENT.encode()

    def readable(chunk: bytes = b"", *, last: bool = False) -> bytes:
        pending.extend(chunk)
        *lines, rest = bytes(pending).split(b"\n")
        pending[:] = b"" if last else rest
        if last and rest:
            lines.append(rest)
        out = bytearray()
        for line in lines:
            # under a pty every line ends CRLF
            text = ANSI.sub(b"", line.rstrip(b"\r").rpartition(b"\r")[2])
            if text and line.startswith(pad) and not text.startswith(pad):
                text = pad + text
            out += text + b"\n"
        return bytes(out)

    return readable


def log_header(log: Path | None, command: str, facts: dict[str, object]) -> None:
    """Open LOG with what produced it: after a crash the file is all that is left."""
    if log is None:
        return
    log.parent.mkdir(parents=True, exist_ok=True)
    width = max((len(key) for key in facts), default=0)
    lines = [
        f"# motion-spec {command}",
        f"# started    {datetime.now(UTC).isoformat(timespec='seconds')}",
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
    except (ValueError, OSError):
        return None


def machine_facts() -> dict[str, object]:
    """What about this machine decides whether a build finishes or takes it down."""
    memory = total_memory()
    host = os.uname()
    return {
        "host": f"{host.nodename} {host.sysname} {host.release}",
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
        sys.stderr = Mirror(previous, sink)
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
    indent = indenter()
    readable = for_the_file()
    started = time.monotonic()
    try:
        with log.open("ab") as sink:
            sink.write(
                f"\n[{datetime.now(UTC).strftime('%H:%M:%S')}] $ {shlex.join(argv)}\n".encode()
            )
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
            # else the read never sees EOF
            os.close(worker)
            worker = -1
            while True:
                try:
                    chunk = os.read(controller, 65536)
                except OSError as error:
                    if error.errno == errno.EIO:  # child closed its end
                        break
                    raise
                if not chunk:
                    break
                chunk = indent(chunk)
                sys.stderr.buffer.write(chunk)
                sys.stderr.buffer.flush()
                sink.write(readable(chunk))
                # two writers: flush to keep time order
                sink.flush()
            returncode = process.wait()
            sink.write(readable(last=True))
            stamp = datetime.now(UTC).strftime("%H:%M:%S")
            elapsed = time.monotonic() - started
            # named: SIGKILL marks an OOM kill
            if returncode < 0:
                name = next(
                    (member.name for member in signal.Signals if member.value == -returncode),
                    f"signal {-returncode}",
                )
                outcome = f"# killed by {name} after {elapsed:.1f}s"
            else:
                outcome = f"# exit {returncode} after {elapsed:.1f}s"
            sink.write(f"[{stamp}] {outcome}\n".encode())
            sink.flush()
    finally:
        os.close(controller)
        if worker >= 0:
            os.close(worker)
    if check and returncode:
        raise subprocess.CalledProcessError(returncode, argv)
    return returncode


def trash(path: Path) -> Path | None:
    """Move PATH to the desktop trash; where it landed, when the home trash records it."""
    original = path.absolute()
    try:
        result = subprocess.run(
            ["gio", "trash", "--", str(path)], check=False, capture_output=True, text=True
        )
    except FileNotFoundError as error:
        raise ValueError(f"cannot remove {path.name}: gio is not installed") from error
    if result.returncode:
        raise ValueError(f"could not move {path.name} to trash: {result.stderr.strip()}")
    from configparser import ConfigParser, Error
    from urllib.parse import unquote

    home = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share")) / "Trash"
    records = []
    for info in (home / "info").glob("*.trashinfo"):
        entry = ConfigParser(interpolation=None)
        try:
            entry.read_string(info.read_text(encoding="utf-8", errors="replace"))
        except Error:
            continue
        if unquote(entry.get("Trash Info", "Path", fallback="")) == str(original):
            records.append(info)
    if not records:
        return None
    return home / "files" / max(records, key=lambda info: info.stat().st_mtime).stem
