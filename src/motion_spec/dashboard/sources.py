# SPDX-License-Identifier: MPL-2.0

"""The model files the dashboard browses, reads, writes and hands to another program."""

from __future__ import annotations

import difflib
import os
import re
import shlex
import shutil
import subprocess
from functools import lru_cache
from pathlib import Path

from motion_spec.dashboard import roots
from motion_spec.dashboard.roots import (
    AUTHORED,
    AUTHORED_NAMES,
    IGNORED,
    NOT_AUTHORED,
    relative_path,
)


def browsable(path: Path) -> bool:
    """Whether the sources tree shows this one file: a DSL file, by its extension, anywhere.

    The environment file is named, not matched.
    """
    return path.name in AUTHORED_NAMES or (
        path.suffix in AUTHORED and path.name not in NOT_AUTHORED
    )


def authored_sources() -> list[str]:
    """Every source file the tree lists, relative to the sources root.

    Ignored directories are pruned before they are entered.
    """
    listed = []
    for folder, folders, files in os.walk(roots.WORKSPACE):
        folders[:] = [name for name in folders if name not in IGNORED and not name.startswith(".")]
        here = Path(folder)
        listed += [
            str((here / name).relative_to(roots.WORKSPACE))
            for name in files
            if browsable(Path(name)) and not name.startswith(".")
        ]
    return listed


def source_path(value: str) -> Path:
    """Resolve a source file inside the sources root, never a generation's vendored copy."""
    path = relative_path(roots.WORKSPACE, value)
    if not path.is_file() or not browsable(path):
        raise ValueError(f"not an authored model file: {value}")
    return path


def save_source(value: str, text: str) -> dict:
    """Write an authored file back, so a small change can be made where it is read.

    Written through a neighbouring temporary file, so a failed write leaves no half-saved
    model. The syntax check reports on what was written; it never blocks the save.
    """
    path = source_path(value)
    pending = path.with_name(f".{path.name}.saving")
    try:
        pending.write_text(text)
        os.replace(pending, path)
    finally:
        pending.unlink(missing_ok=True)
    return {
        "saved": True,
        "bytes": len(text.encode()),
        "absolute": str(path),
        "check": check_syntax(path),
    }


def check_syntax(path: Path) -> dict | None:
    """Whether textX can still parse this file, and where it gave up if not.

    None when no grammar is registered for it or textX is not installed.
    """
    try:
        from textx import metamodel_for_file
        from textx.exceptions import TextXError, TextXRegistrationError
    except ImportError:
        return None
    try:
        metamodel = metamodel_for_file(str(path))
    except TextXRegistrationError:
        return None
    try:
        # by path: imports resolve against the file's directory
        metamodel.model_from_file(str(path))
    except TextXError as problem:
        return {
            "ok": False,
            "line": problem.line or None,
            "column": problem.col or None,
            "message": re.sub(rf"^{re.escape(str(path))}:\d+:\d+:\s*", "", str(problem)).strip(),
        }
    except RecursionError:
        return {"ok": False, "message": "too deeply nested to parse"}
    except (OSError, ValueError, KeyError, AttributeError, TypeError) as problem:
        return {"ok": False, "message": f"{type(problem).__name__}: {problem}"}
    return {"ok": True, "message": "syntax OK"}


def read_source(value: str) -> dict:
    path = source_path(value)
    host = terminal()
    return {
        "path": value,
        "absolute": str(path),
        "text": path.read_text(),
        "git_head": git_baseline(path, git_root(path)),
        "editors": list(editors()),
        "terminal": host[0] if host else None,
    }


def aligned_rows(was: list[str], now: list[str]) -> dict:
    """Two texts aligned line for line, as the diff page's rows plus whether anything differs."""
    rows = []
    for kind, left_from, left_to, right_from, right_to in difflib.SequenceMatcher(
        None, was, now, autojunk=False
    ).get_opcodes():
        for index in range(max(left_to - left_from, right_to - right_from)):
            here, there = left_from + index, right_from + index
            rows.append(
                {
                    "kind": kind,
                    "left": {"n": here + 1, "text": was[here]} if here < left_to else None,
                    "right": {"n": there + 1, "text": now[there]} if there < right_to else None,
                }
            )
    return {"same": all(row["kind"] == "equal" for row in rows), "rows": rows}


def git_root(path: Path) -> Path | None:
    """The git repository this file lives in, or None when there is not one."""
    try:
        repo = subprocess.run(
            ["git", "-C", str(path.parent), "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None
    return Path(repo.stdout.strip()) if repo.returncode == 0 else None


def git_baseline(path: Path, root: Path | None) -> str | None:
    """The file's content at HEAD of ROOT, its `git_root`, so the editor can mark what changed
    since the last commit.

    None when the tree is not a git repository, git is not installed, or the file has no
    commit yet.
    """
    if root is None:
        return None
    try:
        shown = subprocess.run(
            ["git", "-C", str(root), "show", f"HEAD:{path.relative_to(root).as_posix()}"],
            capture_output=True,
            text=True,
            check=False,
        )
    except (OSError, ValueError):
        return None
    return shown.stdout if shown.returncode == 0 else None


def git_diff(value: str) -> dict:
    """This file at git HEAD beside the file as it is now, in the rows the diff page draws.

    The same shape source_drift returns.
    """
    path = source_path(value)
    root = git_root(path)
    committed = git_baseline(path, root)
    working = {"name": path.name, "workspace": value, "workspace_path": str(path)}
    if root is None or committed is None:
        return {
            **working,
            "archived": None,
            "same": False,
            "rows": [],
            "error": "not a committed file in a git repository",
        }
    return {
        **working,
        "archived": f"HEAD:{path.relative_to(root).as_posix()}",
        **aligned_rows(committed.splitlines(), path.read_text().splitlines()),
    }


def git_checkout(value: str) -> dict:
    """Discard this file's uncommitted changes, restoring it from git HEAD."""
    path = source_path(value)
    root = git_root(path)
    if root is None:
        raise ValueError("not inside a git repository")
    result = subprocess.run(
        ["git", "-C", str(root), "checkout", "HEAD", "--", path.relative_to(root).as_posix()],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise ValueError(result.stderr.strip() or "git checkout failed")
    return {"text": path.read_text()}


GUI_EDITORS = {"code": ["code", "--goto"], "zed": ["zed"], "kate": ["kate"], "gedit": ["gedit"]}


TERMINAL_EDITORS = ("nvim", "vim", "hx", "emacs", "nano", "micro")


# -e is the x-terminal-emulator convention
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


@lru_cache(maxsize=1)
def terminal() -> tuple[str, list[str]] | None:
    """The desktop's terminal and the argv that runs a command in a new window.

    $TERMINAL, then the freedesktop and Debian pointers at the user's chosen terminal, and
    only then a known one off PATH.
    """
    override = os.environ.get("TERMINAL")
    for name in (override, "xdg-terminal-exec", "x-terminal-emulator", *TERMINAL_ARGS):
        path = shutil.which(name) if name else None
        if path:
            real = Path(os.path.realpath(path)).name
            return real, [path, *TERMINAL_ARGS.get(real, ["-e"])]
    return None


@lru_cache(maxsize=1)
def editors() -> dict[str, list[str]]:
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


@lru_cache(maxsize=32)
def parsed_motion_spec(path: str, version: int):
    """The motion-spec-dsl model of the source at PATH as of VERSION (its mtime); None where it no
    longer parses, so a page shows no source lines rather than failing."""
    from motion_spec_dsl.langs import motion_spec_metamodel
    from textx.exceptions import TextXError

    try:
        return motion_spec_metamodel().model_from_file(path)
    except TextXError:
        return None


def authored_lines(path: Path) -> dict:
    """(motion, constraint) -> (line number, expression, authored motion name) for every named
    `while`/`until` constraint of the source at PATH.

    Display only: what a constraint is comes off the log header.
    """
    from motion_spec_dsl.classes.constraints import flatten_constraint_items
    from textx import get_children_of_type, get_location

    model = parsed_motion_spec(str(path), path.stat().st_mtime_ns)
    if model is None:
        return {}
    text = path.read_text().splitlines()
    lines = {}
    for motion in get_children_of_type("GuardedMotion", model):
        for section in (motion.while_, motion.until):
            if section is None:
                continue
            for item in flatten_constraint_items(section.constraints):
                if not item.name:
                    continue
                number = get_location(item)["line"]
                expression = text[number - 1].strip().rstrip(",").partition(":")[2].strip()
                # authored with dashes, generated with underscores
                key = (motion.name.replace("-", "_"), item.name.replace("-", "_"))
                lines[key] = (number, expression, motion.name)
    return lines


def declaration_lines(path: Path) -> dict:
    """authored name -> the line that declares it in the source at PATH. A name reused in two
    contexts points at where it was first written rather than the last."""
    from textx import get_children, get_location

    model = parsed_motion_spec(str(path), path.stat().st_mtime_ns)
    if model is None:
        return {}
    lines: dict[str, int] = {}
    for item in get_children(lambda node: isinstance(getattr(node, "name", None), str), model):
        if item.name:
            number = get_location(item)["line"]
            lines[item.name] = min(lines.get(item.name, number), number)
    return lines
