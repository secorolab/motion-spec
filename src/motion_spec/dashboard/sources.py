# SPDX-License-Identifier: MPL-2.0

"""The model files the dashboard browses, reads, writes and hands to another program."""

from __future__ import annotations

import difflib
import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

from motion_spec.dashboard import roots
from motion_spec.dashboard.roots import AUTHORED, IGNORED, NOT_AUTHORED, relative_path


def browsable(path: Path) -> bool:
    """Whether the sources tree shows this one file: a DSL file, by its extension, anywhere.

    One rule, not "a DSL file, or anything beside one": a directory only has to hold a single
    `.robmot` for that second rule to sweep in everything else living there, which at the top
    of a workspace is the whole repository.
    """
    return path.suffix in AUTHORED and path.name not in NOT_AUTHORED


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
        listed += [
            str((here / name).relative_to(roots.WORKSPACE))
            for name in files
            if browsable(Path(name)) and not name.startswith(".")
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
    write cannot leave a half-saved model behind. The save always happens: the check that
    follows reports on what was written rather than standing between the reader and their file.
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

    textX in this process rather than `textx check` in another: the same load either way, but
    the failure arrives as an exception carrying its own line and column instead of a message
    to scrape back out of a subprocess's stderr.

    None when nothing here can check this kind of file -- no grammar registered for it, or no
    textX installed at all, both of which are silence rather than a pass it did not earn.
    """
    try:
        from textx import metamodel_for_file
        from textx.exceptions import TextXError, TextXRegistrationError
    except ImportError:
        return None  # the optional DSL feature is not installed
    try:
        metamodel = metamodel_for_file(str(path))
    except TextXRegistrationError:
        return None  # no grammar for this extension: nothing to be wrong about
    try:
        # By path, not by the text just written: a model's imports resolve against its own
        # directory, so the file has to be read from where its neighbours are.
        metamodel.model_from_file(str(path))
    except TextXError as problem:
        # line/col are 0 or None when the fault is the whole file rather than a place in it
        # (an import that does not resolve, say) -- then there is no line worth pointing at.
        return {
            "ok": False,
            "line": getattr(problem, "line", None) or None,
            "column": getattr(problem, "col", None) or None,
            "message": _tidy(problem, path),
        }
    except RecursionError:
        return {"ok": False, "message": "too deeply nested to parse"}
    except (OSError, ValueError, KeyError, AttributeError, TypeError) as problem:
        # A model can fail to load for reasons that are not syntax -- a missing import, a
        # reference to something absent. Still worth saying; just not placeable on a line.
        return {"ok": False, "message": f"{type(problem).__name__}: {problem}"}
    return {"ok": True, "message": "syntax OK"}


def _tidy(problem: Exception, path: Path) -> str:
    """The parser's complaint without the absolute path and position it already reports."""
    return re.sub(rf"^{re.escape(str(path))}:\d+:\d+:\s*", "", str(problem)).strip()


def read_source(value: str) -> dict:
    path = source_path(value)
    return {
        "path": value,
        # The page shows where the file is; it should not have to join a root onto a name.
        "absolute": str(path),
        "text": path.read_text(),
        "git_head": git_baseline(path),
        "editors": list(editors()),
        "terminal": (terminal() or (None,))[0],
    }


def aligned_rows(was: list[str], now: list[str]) -> dict:
    """Two texts aligned line for line, as the diff page's rows plus whether anything differs.

    Lives here rather than beside either caller: a generation's archived copy against the
    working tree, and git HEAD against the working tree, are the same comparison drawn the
    same way.
    """
    rows, changed = [], False
    for kind, left_from, left_to, right_from, right_to in difflib.SequenceMatcher(
        None, was, now, autojunk=False
    ).get_opcodes():
        left = list(range(left_from, left_to))
        right = list(range(right_from, right_to))
        changed = changed or kind != "equal"
        # A replaced block pairs line for line, and the shorter side runs out into blanks
        # rather than shifting everything below it out of step with the other column.
        for index in range(max(len(left), len(right))):
            here = left[index] if index < len(left) else None
            there = right[index] if index < len(right) else None
            rows.append(
                {
                    "kind": kind,
                    "left": None if here is None else {"n": here + 1, "text": was[here]},
                    "right": None if there is None else {"n": there + 1, "text": now[there]},
                }
            )
    return {"same": not changed, "rows": rows}


def _git_root(path: Path) -> Path | None:
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


def git_baseline(path: Path) -> str | None:
    """The file's content at HEAD, so the editor can mark what changed since the last commit.

    None when the tree is not a git repository, git is not installed, or the file has no
    commit yet -- the reader sees plain text with no diff marks rather than every line marked
    as new.
    """
    root = _git_root(path)
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

    The same shape source_drift returns, so one diff page renders both: what a generation was
    built from vs the working tree, and what was committed vs the working tree.
    """
    path = source_path(value)
    root = _git_root(path)
    committed = git_baseline(path)
    if root is None or committed is None:
        return {
            "name": path.name,
            "workspace": value,
            "workspace_path": str(path),
            "archived": None,
            "same": False,
            "rows": [],
            "error": "not a committed file in a git repository",
        }
    return {
        "name": path.name,
        "workspace": value,
        "workspace_path": str(path),
        "archived": f"HEAD:{path.relative_to(root).as_posix()}",
        **aligned_rows(committed.splitlines(), path.read_text().splitlines()),
    }


def git_checkout(value: str) -> dict:
    """Discard this file's uncommitted changes, restoring it from git HEAD.

    Named by its one path, the same as save and delete -- never a bare `git checkout .` that
    could discard something the reader never looked at here.
    """
    path = source_path(value)
    root = _git_root(path)
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


def declaration_lines(text: str) -> dict:
    """authored name -> the line that declares it, for `<kind> <name> = ...` and `<kind> <name> {`.

    `authored_lines` covers constraints, which are named inside `while`/`until`; a spec quantity
    is declared instead, so its line is found by the shape of a declaration. First one wins: a
    name reused in two contexts points at where it was first written rather than the last.
    """
    lines: dict[str, int] = {}
    for number, line in enumerate(text.splitlines(), 1):
        match = re.match(r"\s*[\w-]+\s+([A-Za-z][\w-]*)\s*[={]", line)
        if match:
            lines.setdefault(match.group(1), number)
    return lines
