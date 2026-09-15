# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""What more than one part of motion-spec needs: removal, and running a tool visibly."""

from __future__ import annotations

import errno
import os
import pty
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

STATE_DIRECTORY = ".motion-spec"
LOG_DIRECTORY = "logs"


def command_log(root: Path, command: str) -> Path:
    """The file one invocation of COMMAND tees its tools' output into."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
    return root / STATE_DIRECTORY / LOG_DIRECTORY / f"{stamp}Z-{command}.log"


def generation_log(generation: Path, command: str) -> Path:
    """Where a command about one generation tees its tools' output."""
    return generation / LOG_DIRECTORY / f"{command}.log"


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
    try:
        with log.open("ab") as sink:
            sink.write(f"$ {' '.join(argv)}\n".encode())
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
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                sink.write(chunk)
            returncode = process.wait()
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
