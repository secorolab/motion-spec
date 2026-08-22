# SPDX-License-Identifier: MPL-2.0

"""The model files the dashboard browses, reads, writes and hands to another program."""

from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

from motion_spec.dashboard import roots
from motion_spec.dashboard.roots import AUTHORED, IGNORED, relative_path


def browsable(path: Path) -> bool:
    """Whether the sources tree shows this one file.

    Two rules, both read off the tree itself: a DSL file anywhere, and anything sitting in a
    directory a `.robmot` lives in -- that is what makes `robot.toml` a model's config here
    and a package's build file everywhere else.
    """
    if path.suffix in AUTHORED:
        return True
    return path.is_file() and any(path.parent.glob("*.robmot"))


def authored_sources() -> list[str]:
    """Every source file the tree lists, relative to the sources root.

    Pruned rather than filtered: rglob descends into `build`, `install` and `.git` and yields
    every file in them before anything can reject it, which is a hundred thousand paths to
    walk for the hundred this returns. os.walk lets the ignored directories be dropped before
    they are entered.
    """
    listed = []
    for folder, folders, files in os.walk(roots.WORKSPACE):
        folders[:] = [name for name in folders if name not in IGNORED and not name.startswith(".")]
        here = Path(folder)
        # A directory a model lives in holds that model's other files -- its robot.toml, a
        # yaml beside it -- whatever they are named.
        model_dir = any(name.endswith(".robmot") for name in files)
        listed += [
            str((here / name).relative_to(roots.WORKSPACE))
            for name in files
            if (model_dir or Path(name).suffix in AUTHORED) and not name.startswith(".")
        ]
    return listed


def source_path(value: str) -> Path:
    """Resolve a source file inside the sources root.

    Only the working tree: a generation's vendored copies are a record of what was built, and
    reading them here would show a file the tree cannot point at and that the working copy may
    already have moved past.
    """
    path = relative_path(roots.WORKSPACE, value)
    if not path.is_file() or not browsable(path):
        raise ValueError(f"not an authored model file: {value}")
    return path


def save_source(value: str, text: str) -> dict:
    """Write an authored file back, so a small change can be made where it is read.

    Only what the tree already lists and only under the sources root -- `source_path` decides
    that, the same as reading does. Written through a neighbouring temporary file so a failed
    write cannot leave a half-saved model behind.
    """
    path = source_path(value)
    pending = path.with_name(f".{path.name}.saving")
    try:
        pending.write_text(text)
        os.replace(pending, path)
    finally:
        pending.unlink(missing_ok=True)
    return {"saved": True, "bytes": len(text.encode()), "absolute": str(path)}


def read_source(value: str) -> dict:
    path = source_path(value)
    return {
        "path": value,
        # The page shows where the file is; it should not have to join a root onto a name.
        "absolute": str(path),
        "text": path.read_text(),
        "editors": list(editors()),
        "terminal": (terminal() or (None,))[0],
    }


# GUI editors take the file directly; terminal ones need a terminal emulator to live in.
GUI_EDITORS = {"code": ["code", "--goto"], "zed": ["zed"], "kate": ["kate"], "gedit": ["gedit"]}


TERMINAL_EDITORS = ("nvim", "vim", "hx", "emacs", "nano", "micro")


# How each terminal takes "then run this command"; -e is the x-terminal-emulator convention.
TERMINAL_ARGS = {
    "xdg-terminal-exec": [],
    "kitty": [],
    "foot": [],
    "ghostty": ["-e"],
    "alacritty": ["-e"],
    "wezterm": ["start", "--"],
    "gnome-terminal": ["--"],
    "konsole": ["-e"],
    "xterm": ["-e"],
}


def terminal() -> tuple[str, list[str]] | None:
    """The desktop's terminal and the argv that runs a command in a new window.

    $TERMINAL, then the freedesktop and Debian pointers at the user's chosen terminal, and
    only then a known one off PATH -- picking a favourite here would override their default.
    """
    override = os.environ.get("TERMINAL")
    for name in (override, "xdg-terminal-exec", "x-terminal-emulator", *TERMINAL_ARGS):
        path = shutil.which(name) if name else None
        if path:
            real = Path(os.path.realpath(path)).name
            return real, [path, *TERMINAL_ARGS.get(real, ["-e"])]
    return None


def editors() -> dict[str, list[str]]:
    """Every editor that can open a source file here, as name -> argv prefix.

    MS_DASHBOARD_EDITOR is always offered; terminal editors only when a terminal exists to
    host them, since the server has none of its own.
    """
    host = terminal()
    found = {}
    if host:
        found.update({name: [*host[1], name] for name in TERMINAL_EDITORS if shutil.which(name)})
    found.update({name: argv for name, argv in GUI_EDITORS.items() if shutil.which(name)})
    override = os.environ.get("MS_DASHBOARD_EDITOR")
    if override:
        argv = shlex.split(override)
        found[Path(argv[0]).name] = argv
    return found or {"xdg-open": ["xdg-open"]}


def open_terminal(value: str) -> dict:
    """Open a terminal in the folder that holds one authored DSL file."""
    host = terminal()
    if host is None:
        raise ValueError("no terminal emulator found")
    name, argv = host
    folder = source_path(value).parent
    shell = os.environ.get("SHELL", "/bin/sh")
    command = f"cd {shlex.quote(str(folder))} && exec {shlex.quote(shell)}"
    subprocess.Popen([*argv, "sh", "-c", command], start_new_session=True)
    return {"opened": str(folder), "terminal": name}


def open_source(value: str, name: str | None = None) -> dict:
    """Open one authored DSL file in the chosen editor."""
    path = source_path(value)
    available = editors()
    name = name or next(iter(available))
    if name not in available:
        raise ValueError(f"unknown editor: {name}")
    subprocess.Popen([*available[name], str(path)], start_new_session=True)
    return {"opened": value, "editor": name}


def _key(name: str | None) -> str:
    """One spelling for a name authored with dashes and generated with underscores."""
    return (name or "").replace("-", "_")


def authored_lines(text: str) -> dict:
    """(motion, constraint) -> (line number, expression, authored motion name) per source line.

    Display only: what a constraint is and what it drives comes off the log header, this says
    where the reader can go and read it.
    """
    lines = {}
    motion = section = None
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip().rstrip(",")
        match = re.match(r"guarded-motion\s+\(ns=[^)]+\)\s+([\w-]+)", line)
        if match:
            motion = match.group(1)
        block = re.match(r"(while|until|when)\b.*\{$", line)
        if block:
            section = block.group(1)
        elif line == "}":
            section = None
        if section not in ("while", "until") or ":" not in line:
            continue
        name, _, expression = line.partition(":")
        name, expression = name.strip(), expression.strip()
        if re.fullmatch(r"[\w-]+", name) and expression:
            lines[(_key(motion), _key(name))] = (number, expression, motion)
    return lines
