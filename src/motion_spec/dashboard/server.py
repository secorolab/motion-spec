# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Serve recorded motion-spec generations and runs to the dashboard frontend."""

from __future__ import annotations

import argparse
import atexit
import dataclasses
import json
import os
import re
import secrets
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import threading
import tomllib
from datetime import datetime, timezone
from functools import lru_cache
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from collections import deque
from pathlib import Path, PurePosixPath
from urllib.parse import parse_qs, urlparse

import rdflib
from rdflib import Dataset

from motion_spec.dashboard.control import SPEED_MAX, SPEED_MIN, ControlChannel
from motion_spec.dashboard.frames import (
    FrameLayout,
    ShmFrameReader,
    SignalFields,
    shm_path,
    slot_signals,
)
from motion_spec.dashboard.graph import GraphService
from motion_spec.dashboard.runs import GenerationCatalog, GenerationInfo, RunInfo
from motion_spec.dashboard.store import RunStore
from motion_spec.dashboard.tail import FrameLogTail
from google.protobuf.message import DecodeError

from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.lifecycle_events import socket_path
from motion_spec.introspection.replay import read_health, resolve_archive, validate_header

GENERATION_DIR_ENV = "MOTION_SPEC_GEN"
# Roots the dashboard browses, replaced at startup by `serve` and by /api/roots.
GENERATIONS = Path(os.environ.get(GENERATION_DIR_ENV, "").strip() or Path.cwd())
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
}
AUTHORED = (".robmot", ".fsm", ".scenex")
# How REC says a run is over; anything else (QUEUED, RUNNING) means it still has work to do.
RUN_ENDED = {"COMPLETED", "FAILED", "INTERRUPTED", "CANCELLED"}
LIFECYCLE = None


def roots() -> dict:
    """Return the currently browsed source and generation roots."""
    return {"sources": str(WORKSPACE), "logs": str(GENERATIONS)}


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
    return roots()


def pick_root(kind: str) -> dict:
    """Open the host folder chooser and use its selected directory."""
    initial = roots().get(kind)
    if initial is None:
        raise ValueError("unknown root")
    result = subprocess.run(
        ["zenity", "--file-selection", "--directory", "--filename", f"{initial}/"],
        check=False,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if result.returncode:
        raise ValueError("folder selection cancelled")
    return set_root(kind, result.stdout.strip())


def trash(path: Path) -> None:
    """Move a path to the desktop trash. Nothing the dashboard removes should be unrecoverable."""
    result = subprocess.run(
        ["gio", "trash", "--", str(path)], check=False, capture_output=True, text=True
    )
    if result.returncode:
        raise ValueError(f"could not move {path.name} to trash: {result.stderr.strip()}")


def relative_path(root: Path, value: str) -> Path:
    """Resolve a request path inside root, rejecting traversal and missing paths."""
    path = (root / value).resolve()
    if root.resolve() not in (path, *path.parents) or not path.exists():
        raise ValueError("unknown path")
    return path


def expected_path(root: Path, value: str) -> Path:
    """Resolve a request path inside root that need not exist yet, e.g. a run just named."""
    path = (root / value).resolve()
    if root.resolve() not in (path, *path.parents):
        raise ValueError("unknown path")
    return path


def source_path(value: str) -> Path:
    """Resolve an authored DSL file inside the sources root."""
    path = relative_path(WORKSPACE, value)
    if path.suffix not in AUTHORED or not path.is_file():
        raise ValueError(f"not an authored model file: {value}")
    return path


def read_source(value: str) -> dict:
    path = source_path(value)
    return {
        "path": value,
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


def json_file(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


@lru_cache(maxsize=256)
def directory_size(path: Path) -> int:
    """Byte size of one generation bundle, cached for the dashboard session."""
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


@lru_cache(maxsize=1)
def storage_info() -> dict:
    """Generation and frame-log storage totals for the dashboard sidebar."""
    files = [path for path in GENERATIONS.rglob("*") if path.is_file()]
    return {
        "generations_bytes": sum(path.stat().st_size for path in files),
        "logs_bytes": sum(path.stat().st_size for path in files if path.name == "frame_log.pb"),
    }


def generation_info(path: Path) -> dict:
    layout = json_file(path / "generated/contract/frame_layout.json")
    source = next((item for item in (path / "generated/source").glob("*.robmot")), None)
    return {
        "path": str(path.relative_to(GENERATIONS)),
        "name": GenerationInfo(path, GENERATIONS).model,
        "created": stamp_iso(GenerationInfo(path).timestamp),
        "variant": (path.parent.name if path.parent.parent != GENERATIONS else None),
        "built_at": datetime.fromtimestamp(GenerationInfo(path).built_at, timezone.utc).isoformat(),
        "source": source.name if source else None,
        "backend": layout.get("platform", {}).get("backend"),
        "platform": layout.get("platform", {}).get("name"),
        "simulated": layout.get("platform", {}).get("simulated"),
        "schema_hash": layout.get("schema_hash"),
        "size_bytes": directory_size(path),
        "pools": layout.get("pools", {}),
        "runs": len(GenerationInfo(path).runs),
        "has_fsm": any((path / "generated/model").glob("*_fsm.svg")),
        "cameras": generation_cameras(path),
    }


def generation_details(path: Path) -> dict:
    """Add authored motion metadata without slowing the generation sidebar."""
    details = generation_info(path)
    source_files = sorted((path / "generated/source").glob("*"))
    robmot = next((source for source in source_files if source.suffix == ".robmot"), None)
    fsm = next((source for source in source_files if source.suffix == ".fsm"), None)
    description = re.search(r'description:\s*"([^"]+)"', fsm.read_text()) if fsm else None
    # A generation with no run yet has no log to read, so the count comes off the source.
    constraints = authored_lines(robmot.read_text()) if robmot else {}
    details["authored_constraints"] = len(constraints)
    details["motions"] = len({motion for motion, _name in constraints})
    details["folder"] = str(path)
    details["spec_name"] = Path(details["source"] or path.name).stem
    details["description"] = description.group(1) if description else None
    details["source_files"] = [
        {"name": source.name, "path": str(source)} for source in source_files if source.is_file()
    ]
    details["generated_files"] = [
        {"name": str(source.relative_to(path / "generated")), "path": str(source)}
        for source in sorted((path / "generated").rglob("*"))
        if source.is_file() and "source" not in source.relative_to(path / "generated").parts
    ]
    generated = path / "generated"
    details["rdf_graphs"] = sorted(
        str(source.relative_to(generated)) for source in generated.rglob("*.ld.json")
    )
    return details


def rdf_name(term: object) -> str:
    """Return the compact RDF name used in the dashboard."""
    return str(term).rsplit("/", 1)[-1].rsplit("#", 1)[-1]


def provenance_graph(path: Path, selected: list[str]) -> dict:
    """Return every RDF term and triple for the browser WebGL renderer."""
    graph = Dataset()
    generated = (path / "generated").resolve()
    for name in selected:
        source = (generated / name).resolve()
        if generated not in source.parents or source.suffix != ".json" or not source.is_file():
            raise ValueError("unknown RDF graph")
        graph.parse(source, format="json-ld")
    triples = [
        (subject, predicate, obj)
        for subject, predicate, obj, _context in graph.quads((None, None, None, None))
    ]
    terms = sorted(
        {term for subject, _predicate, obj in triples for term in (subject, obj)}, key=str
    )
    node_ids = {term: str(index) for index, term in enumerate(terms)}
    return {
        "nodes": [
            {"id": node_ids[term], "label": rdf_name(term), "value": str(term)} for term in terms
        ],
        "links": [
            {
                "source": node_ids[subject],
                "target": node_ids[obj],
                "label": rdf_name(predicate),
                "value": str(predicate),
            }
            for subject, predicate, obj in triples
        ],
    }


def stamp_iso(name: str) -> str | None:
    """A run or generation stamp as ISO 8601 UTC, for the browser to show in local time."""
    match = re.fullmatch(r"(\d{8}T\d{6}\d{6})Z", name)
    if not match:
        return None
    return (
        datetime.strptime(match.group(1), "%Y%m%dT%H%M%S%f")
        .replace(tzinfo=timezone.utc)
        .isoformat()
    )


def run_info(path: Path) -> dict:
    log = path / "logs/frame_log.pb"
    health = read_health(log) or {}
    run_id = RunInfo(path).run_id
    # A run that has only just started has no readable log yet; it still belongs in the list.
    try:
        period_ns = resolve_archive(path)[3].header.nominal_period_ns
    except (ArchiveError, DecodeError, OSError):
        period_ns = 0
    match = re.fullmatch(r"run-(\d{8}T\d{6}\d{6}Z)", run_id)
    return {
        "path": str(path.relative_to(GENERATIONS)),
        "id": run_id,
        "started": stamp_iso(match.group(1)) if match else None,
        "complete": health.get("complete") or RunInfo(path).status == "COMPLETED",
        "status": RunInfo(path).status,
        "written_frames": health.get("written_frames"),
        "duration_s": health.get("written_frames", 0) * period_ns / 1e9,
        "dropped_frames": health.get("dropped_frames"),
    }


def downsample(values: list[float], target: int = 1600) -> list[float]:
    step = max(1, len(values) // target)
    return values[::step]


# How the dashboard labels the gain roles a controller slot carries; others keep their role name.
GAIN_LABELS = {
    "proportional_gain": "Kp",
    "integral_gain": "Ki",
    "derivative_gain": "Kd",
    "decay_rate": "decay",
}


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


def run_source_text(run_dir: Path, manifest: dict | None) -> str:
    """The authored `.robmot` this run was generated from, vendored in the run or beside it."""
    vendored = [
        run_dir / rel
        for rel in (manifest or {}).get("files", {}).get("sources", ())
        if rel.endswith(".robmot")
    ]
    generated = sorted((run_dir.parent.parent / "generated/source").glob("*.robmot"))
    source = next((path for path in (*vendored, *generated) if path.is_file()), None)
    return source.read_text() if source else ""


def _slot_ids(slots: list, field: str) -> list[str]:
    """The ids these slots name in one role, in order, without repeats or blanks."""
    return list(dict.fromkeys(value for slot in slots if (value := getattr(slot, field))))


def _by_constraint(slots) -> dict:
    """Slots grouped by the constraint they serve -- the axis controllers of one share it.

    An aggregate monitor watches several constraints and names none of them singly, so it
    groups under its own id and stays one row.
    """
    groups: dict[str, list] = {}
    for slot in slots:
        groups.setdefault(slot.constraint_iri or slot.id, []).append(slot)
    return groups


def authored_key(slot, motion, name: str) -> tuple[str, str]:
    """The (motion, constraint) a slot was authored as, from the IRI the header carries.

    A constraint IRI reads `<model>/<motion>/<phase>/<constraint>`, so it says which motion the
    slot serves. A name alone does not: one authored in several motions -- an elbow held
    everywhere -- would otherwise take the first motion's line for all of them.
    """
    # A generated conjunction has no motion of its own in its IRI, but what it watches does:
    # it is the motion's `until`, so it belongs in that motion, not in a block beside it.
    for iri in (slot.constraint_iri, *(member.iri for member in slot.watched)):
        segments = PurePosixPath(urlparse(iri or "").path).parts
        if len(segments) >= 3 and segments[-2] in ("while", "until", "when"):
            return (_key(segments[-3]), _key(name))
    return (_key(motion.id), _key(name))


def _constraint_row(motion, kind: str, group: list, constants: dict, authored: dict) -> dict:
    spelled = {_key(name): name for _, _, name in authored.values()}
    """One constraint's row: its identity and signals from the header, its line from the source."""
    first = group[0]
    name = first.constraint_id or rdf_name(first.constraint_iri) or first.id
    key = authored_key(first, motion, name)
    line, expression, authored_motion = authored.get(key, (None, None, None))
    # The header names the pair the evaluator compares; measured/setpoint only where it does not.
    operands = list(dict.fromkeys(value for slot in group for value in slot.operand_ids))
    compared = operands or [*_slot_ids(group, "measured_id"), *_slot_ids(group, "setpoint_id")]
    evaluator = next(iter(_slot_ids(group, "evaluator_id")), None)
    return {
        # As the source spells it, so a generated row lands in the motion's own block
        "motion": authored_motion or spelled.get(key[0]) or key[0] or motion.id,
        "handler": motion.id,
        "line": line,
        "name": name,
        "expression": expression,
        "kind": kind,
        # The closure the header names, or the slot computing the error where it names none.
        "evaluator": evaluator or first.iri or first.id or None,
        "between": compared,
        "tracking": [value for value in compared if value not in constants],
        # An aggregate monitor's scalar says only that every member holds; each member says why,
        # and they are not one plot: a velocity and a distance share no axis.
        "members": list(
            {
                member.id: {
                    "id": member.id,
                    "iri": member.iri,
                    "error": member.error_id,
                    "tolerance": constants.get(member.tolerance_id),
                }
                for slot in group
                for member in slot.watched
                if member.error_id
            }.values()
        ),
        "error": _slot_ids(group, "error_id"),
        "control": _slot_ids(group, "output_id"),
        "monitors": [slot.id for slot in group] if kind == "monitored" else [],
        "setpoints": [
            {"label": value, "value": constants[value]}
            for value in _slot_ids(group, "setpoint_id")
            if value in constants
        ],
        "gains": {
            GAIN_LABELS.get(gain.role, gain.role): gain.value
            for slot in group
            for gain in slot.gains
        },
        "tolerance": next(
            (constants[slot.tolerance_id] for slot in group if slot.tolerance_id in constants), None
        ),
    }


def source_constraints(run_dir: Path, manifest: dict | None, contract) -> list[dict]:
    """Return one row per constraint this run recorded, read off the log's own header.

    The header says which constraint every controller and monitor slot serves, which quantity
    ids carry its error, output, measurement and setpoint, which constant holds its tolerance
    and which gains the controller runs -- so the join is on identity the run wrote down rather
    than on names recovered from the source, and a run plots wherever it is stored.

    The row shape is a contract with the frontend: motion, handler, line, name, expression,
    kind, evaluator, between, tracking, error, control, monitors, setpoints, gains, tolerance.
    """
    authored = authored_lines(run_source_text(run_dir, manifest))
    constants = {
        key: row.value
        for row in contract.header.constants
        for key in (row.id, row.source_id)
        if key
    }
    return [
        _constraint_row(motion, kind, group, constants, authored)
        for motion in contract.header.motions
        for kind, slots in (("controlled", motion.controllers), ("monitored", motion.monitors))
        for group in _by_constraint(slots).values()
    ]


SLOT_ROLES = (
    ("error_id", "error"),
    ("output_id", "output"),
    ("measured_id", "measured"),
    ("setpoint_id", "setpoint"),
    ("tolerance_id", "tolerance"),
)


def signal_index(contract) -> dict:
    """signal id -> what it is: the motion and constraint it belongs to, and its role there.

    A picker offering `constraint_0.error` beside `eacc_ctrl_home_x` asks the reader to know
    the schema. The header says which constraint each slot serves, so say that instead.
    """
    index: dict[str, dict] = {}
    for motion in contract.header.motions:
        # a controller writes a constraint slot; a monitor writes a monitor slot
        for pool, slots, keys in (
            (
                "constraints",
                motion.controllers,
                ("error", "output", "measured", "setpoint", "satisfied"),
            ),
            ("monitors", motion.monitors, ("value", "satisfied")),
        ):
            for slot in slots:
                owner = slot.constraint_id or rdf_name(slot.constraint_iri) or slot.id
                fields = contract.fields.get(pool) or ()
                field = fields[slot.number]["id"] if slot.number < len(fields) else None
                named = {role for attribute, role in SLOT_ROLES if getattr(slot, attribute, "")}
                for key in keys:
                    where = {"motion": motion.id, "constraint": owner, "role": key, "slot": slot.id}
                    # The pooled slot mirrors a signal the header already names: offering both
                    # is the same series twice, under a name nobody can read.
                    if field and key not in named:
                        index[f"{field}.{key}"] = where
                    if pool == "monitors":
                        index[f"{slot.id}.{key}"] = where
                for attribute, role in SLOT_ROLES:
                    signal = getattr(slot, attribute, "")
                    if signal:
                        index.setdefault(
                            signal,
                            {
                                "motion": motion.id,
                                "constraint": owner,
                                "role": role,
                                "slot": slot.id,
                            },
                        )
                for member in slot.watched:
                    if member.error_id:
                        index.setdefault(
                            member.error_id,
                            {
                                "motion": motion.id,
                                "constraint": member.id,
                                "role": "error",
                                "slot": slot.id,
                            },
                        )
    return index


def replay_data(run_dir: Path) -> dict:
    """Return replay metadata without decoding the complete frame log."""
    pending = False
    try:
        run_dir, log, manifest, contract = resolve_archive(run_dir)
    except ArchiveError:
        # A run named but not yet writing: the generation carries the same header record the
        # runtime will put at the front of the log, and that record is itself a zero-frame
        # log -- so the page is built from it and follows the real log when it begins.
        record = run_dir.parent.parent / "generated/contract/frame_log_header.pb"
        if not record.is_file():
            raise
        log, manifest, contract = record, None, frame_log_pb.read_contract(record)
        pending = True
    constraints = source_constraints(run_dir, manifest, contract)
    health = read_health(log) or {}
    frame_count = health.get("written_frames", 0)
    signals = ["timing.compute_ms", "timing.period_ms"]
    mirrored = {
        f"{contract.fields['constraints'][slot.number]['id']}.{role}"
        for motion in contract.header.motions
        for slot in motion.controllers
        if slot.number < len(contract.fields["constraints"])
        for attribute, role in SLOT_ROLES
        if getattr(slot, attribute, "")
    }
    signals.extend(
        name
        for field in contract.fields["constraints"]
        for key in ("error", "output", "measured", "setpoint", "satisfied")
        if (name := f"{field['id']}.{key}") not in mirrored
    )
    signals.extend(
        f"{slot.id}.{key}"
        for motion in contract.header.motions
        for slot in motion.monitors
        for key in ("value", "satisfied")
    )
    signals.extend(slot_signals(contract))
    indices = {motion.id: motion.index for motion in contract.header.motions}
    spelling: dict[str, str] = {}  # gate id -> the motion name the source uses
    windows = log_events(log, contract)["windows"]
    logged = set(signals) | {field["id"] for field in contract.fields["quantities"]}
    parts = {}
    for name in logged:
        prefix, _, _ = name.partition(".")
        parts.setdefault(prefix, []).append(name)
    for constraint in constraints:
        constraint["monitors"] = [
            f"{name}.{key}" for name in constraint["monitors"] for key in ("value", "satisfied")
        ]
        for key in ("tracking", "control", "monitors", "error"):
            constraint[key] = [
                signal
                for name in constraint[key]
                for signal in ([name] if name in logged else sorted(parts.get(name, ())))
            ]
        handler = constraint.pop("handler")
        spelling.setdefault(handler, constraint["motion"])
        constraint["window"] = windows.get(indices.get(handler))
    signals.extend(
        signal
        for constraint in constraints
        for signal in (*constraint["tracking"], *constraint["control"], *constraint["monitors"])
    )
    # A run copied out of its generation has none to go back to; the log says everything else.
    generation = run_dir.parent.parent
    return {
        "generation": (
            str(generation.relative_to(GENERATIONS))
            if GENERATIONS.resolve() in generation.resolve().parents
            else None
        ),
        "frames": frame_count,
        "duration": frame_count * contract.header.nominal_period_ns / 1e9,
        "states": [state.id for state in contract.header.fsm_states],
        "events": log_events(log, contract)["events"],
        "signals": list(dict.fromkeys(signals)),
        "signal_index": {
            name: {**where, "motion": spelling.get(where["motion"], where["motion"])}
            for name, where in signal_index(contract).items()
        },
        "constraints": constraints,
        # The live poll names the motion by its gate; the panel is headed by the authored name.
        "motion_names": spelling,
        # A `when` guard's monitor runs while the PREDECESSOR motion is active: the contract's
        # owner, not the block it was authored in, says whose window carries its data.
        "monitor_owners": {
            slot.id: spelling.get(motion.id, motion.id)
            for motion in contract.header.motions
            for slot in motion.monitors
        },
        "videos": run_videos(run_dir),
        "header": validate_header(log, contract),
        "health": health,
        "pending": pending,
    }


_EVENTS: dict[tuple[str, int], list] = {}


def log_events(log: Path, contract) -> list:
    """Every FSM state entry and fired event in one log, scanned once per log revision.

    A frame carries both: fsm_state is where the machine is, last_event is what fired to put
    it there, so a transition shows up as an event marker and a state marker on the same frame.
    Keyed by size, so a live log that grew is rescanned and a finished one never is.
    """
    key = (str(log), log.stat().st_size)
    if key not in _EVENTS:
        states = [state.id for state in contract.header.fsm_states]
        fired = [event.id for event in contract.header.fsm_events]
        events, windows, index = [], {}, 0
        state_was = event_was = None
        with log.open("rb") as fh:
            frame_log_pb._read_delimited(fh)
            while data := frame_log_pb._read_delimited(fh, partial_ok=True):
                record = contract.record_cls()
                record.ParseFromString(data)
                if record.WhichOneof("record") != "frame":
                    continue
                frame = record.frame
                if frame.last_event != event_was:
                    event_was = frame.last_event
                    if 0 <= event_was < len(fired):
                        events.append({"frame": index, "kind": "event", "label": fired[event_was]})
                if frame.fsm_state != state_was:
                    state_was = frame.fsm_state
                    label = states[state_was] if 0 <= state_was < len(states) else str(state_was)
                    events.append({"frame": index, "kind": "state", "label": label})
                window = windows.setdefault(frame.active_motion, [index, index])
                window[1] = index
                index += 1
        if len(_EVENTS) > 32:
            _EVENTS.pop(next(iter(_EVENTS)))
        _EVENTS[key] = {"events": events, "windows": windows}
    return _EVENTS[key]


def signal_reader(contract):
    """Map a signal name onto a raw frame, the way the plots read one.

    The plot history and the live increments read the same log; sharing the lookup means a
    signal cannot mean one thing while the run writes and another once it is finished.
    """
    constraint_slots = {
        field["id"]: index for index, field in enumerate(contract.fields["constraints"])
    }
    slots = slot_signals(contract)
    monitor_slots = {
        slot.id: (contract.fields["monitors"][slot.number], motion.index)
        for motion in contract.header.motions
        for slot in motion.monitors
        if slot.number < len(contract.fields["monitors"])
    }
    quantities = {field["id"]: field for field in contract.fields["quantities"]}

    def value(frame, name: str):
        if name == "timing.compute_ms":
            return frame.compute_ns / 1e6
        if name == "timing.period_ms":
            return frame.period_ns / 1e6
        if name in quantities:
            field = quantities[name]
            gate = contract.gate.get("quantities")
            if gate is not None and field["index"] not in gate.get(frame.active_motion, ()):
                return None
            return float(getattr(frame, field["name"]))
        if name in slots:
            kind, field, attribute = slots[name]
            gate = contract.gate.get(kind)
            if gate is not None and field["index"] not in gate.get(frame.active_motion, ()):
                return None
            return float(getattr(getattr(frame, field["name"]), attribute))
        prefix, _, key = name.rpartition(".")
        if prefix in constraint_slots:
            field = contract.fields["constraints"][constraint_slots[prefix]]
            return getattr(getattr(frame, field["name"]), key)
        if prefix in monitor_slots:
            field, owner = monitor_slots[prefix]
            if frame.active_motion != owner:
                return None
            return getattr(getattr(frame, field["name"]), key)
        raise ValueError(f"unknown signal: {name}")

    return value


def plot_data(run_dir: Path, names: list[str], window: tuple | None = None) -> dict:
    """Stream and downsample requested fields without shaping complete frames.

    Sampling follows the window asked for, so a motion that lasted a handful of frames is
    drawn from those frames rather than missed between two samples of the whole run.
    """
    _, log, _manifest, contract = resolve_archive(run_dir)
    value = signal_reader(contract)
    health = read_health(log) or {}
    first, last = window or (0, max(0, health.get("written_frames", 0) - 1))
    step = max(1, (last - first + 1) // 1600)
    series = {name: [] for name in names}
    index = 0
    with log.open("rb") as fh:
        frame_log_pb._read_delimited(fh)
        while data := frame_log_pb._read_delimited(fh, partial_ok=True):
            record = contract.record_cls()
            record.ParseFromString(data)
            if record.WhichOneof("record") != "frame":
                continue
            frame = record.frame
            if first <= index <= last and (index - first) % step == 0:
                for name in names:
                    series[name].append(value(frame, name))
            index += 1
    return {
        "signals": series,
        "events": log_events(log, contract)["events"],
        "sample_step": step,
        "first_frame": first,
    }


JUPYTER = {}


def lab_settings() -> Path:
    """A settings directory of our own, so the framed lab is dark without touching ~/.jupyter."""
    directory = Path(tempfile.gettempdir()) / "motion-spec-lab-settings"
    themes = directory / "@jupyterlab" / "apputils-extension"
    themes.mkdir(parents=True, exist_ok=True)
    (themes / "themes.jupyterlab-settings").write_text(
        json.dumps({"theme": "JupyterLab Dark", "adaptive-theme": False}, indent=1)
    )
    return directory


def jupyter_server() -> dict:
    """The embedded JupyterLab, started on first use and framed by this dashboard only."""
    if JUPYTER.get("process") and JUPYTER["process"].poll() is None:
        return {"url": JUPYTER["url"], "root": str(WORKSPACE)}
    if shutil.which("jupyter") is None:
        raise ValueError(
            "JupyterLab is not installed. Install it with: pip install 'motion-spec[replay]'"
        )
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    token = secrets.token_urlsafe(24)
    settings = lab_settings()
    framing = json.dumps(
        {
            "headers": {
                "Content-Security-Policy": "frame-ancestors 'self' http://127.0.0.1:8080 http://localhost:8080"
            }
        }
    )
    JUPYTER["process"] = subprocess.Popen(
        [
            "jupyter",
            "lab",
            "--no-browser",
            f"--port={port}",
            f"--ServerApp.root_dir={WORKSPACE}",
            f"--IdentityProvider.token={token}",
            f"--LabServerApp.user_settings_dir={settings}",
            f"--ServerApp.tornado_settings={framing}",
            "--ServerApp.disable_check_xsrf=True",
            "--ServerApp.open_browser=False",
        ],
        cwd=WORKSPACE,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    JUPYTER["url"] = f"http://127.0.0.1:{port}/lab?token={token}"
    JUPYTER["base"] = f"http://127.0.0.1:{port}"
    JUPYTER["token"] = token
    return {"url": JUPYTER["url"], "root": str(WORKSPACE)}


def stop_jupyter() -> None:
    """Take the embedded lab down with the dashboard that started it."""
    process = JUPYTER.get("process")
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(5)
    except subprocess.TimeoutExpired:
        process.kill()


RUNNING: dict[str, dict] = {}


def generation_cameras(generation_dir: Path) -> list[dict]:
    """The cameras this generation declares, which are the ones that publish frames to record."""
    scene = (
        json_file(generation_dir / "generated/model/ir.json")
        .get("composition", {})
        .get("scene", {})
    )
    return [
        {"id": camera.get("id"), "width": camera.get("width"), "height": camera.get("height")}
        for camera in scene.get("cameras") or []
        if camera.get("id")
    ]


LAYOUT_REL = "generated/contract/frame_layout.json"


def is_simulated(generation_dir: Path) -> bool:
    """Whether this generation targets a simulator, which is what makes a display a choice."""
    return bool(json_file(generation_dir / LAYOUT_REL).get("platform", {}).get("simulated"))


def run_videos(run_dir: Path) -> list[str]:
    """The cameras this run recorded, named by their video beside the log."""
    return sorted(path.stem for path in (run_dir / "logs").glob("*.mp4"))


def video_file(run_dir: Path, camera: str) -> Path:
    path = (run_dir / "logs" / f"{camera}.mp4").resolve()
    if path.parent != (run_dir / "logs").resolve() or not path.is_file():
        raise ValueError(f"no recording for camera: {camera}")
    return path


def run_ended(run_dir: Path) -> bool:
    """Whether this run is written and marked: archived, and REC says how it ended."""
    if not (run_dir / "manifest.json").exists():
        # No manifest, no archive -- and no reason to pay the REC parse on every live poll.
        trace(f"run_ended {run_dir.name}: archived=False")
        return False
    status = RunInfo(run_dir).status
    trace(f"run_ended {run_dir.name}: archived=True status={status}")
    return status in RUN_ENDED


_TRACED: dict[str, str] = {}


def trace(line: str) -> None:
    """Print a state line the first time it changes, so a run leaves a readable trail."""
    key = line.split(":")[0]
    if _TRACED.get(key) != line:
        _TRACED[key] = line
        print(f"[trace] {time.strftime('%H:%M:%S')} {line}", file=sys.stderr, flush=True)


def run_status(generation_dir: Path) -> dict:
    """Whether this generation has a run in progress, and where its output is going.

    A run is over when its archive is written, not when the process that started it exits:
    the CLI still verifies and reports for a while after, which is no longer this run.
    """
    started = RUNNING.get(str(generation_dir))
    process = started["process"] if started else None
    busy = process is not None and process.poll() is None
    # The dashboard names the run when it starts it, so the directory to watch is known
    # before anything exists on disk.
    run_dir = generation_dir / "runs" / started["run_id"] if started else None
    run = run_dir if run_dir is not None and run_dir.is_dir() else None
    live = busy and not (run is not None and run_ended(run))
    trace(
        f"run_status {generation_dir.name}: busy={busy} running={live} run={run.name if run else None}"
    )
    return {
        "running": live,
        "busy": busy,
        "pid": process.pid if busy else None,
        "run": str(run.relative_to(GENERATIONS)) if live and run else None,
        "exit_code": None if busy or process is None else process.returncode,
        "log": str(generation_dir / RUN_LOG) if process is not None else None,
        # What the run says about its own frame log, once it has written a manifest to say it in:
        # false means it kept none on purpose, and the page shows its console instead of waiting.
        "recorded": run_recorded(run_dir),
    }


def run_recorded(run_dir: Path | None) -> bool | None:
    """Whether the run wrote a frame log, per its manifest. None until the manifest exists."""
    manifest = run_dir / "manifest.json" if run_dir else None
    if manifest is None or not manifest.is_file():
        return None
    try:
        return json.loads(manifest.read_text()).get("recorded", True)
    except (OSError, ValueError):
        return None


RUN_LOG = "dashboard-run.log"


def run_arguments(options: dict, simulated: bool) -> list[str]:
    """The run flags a browser's choices amount to. Only a simulator has a display or a pace.

    A window paces itself, but a headless loop runs as fast as the machine allows, which is
    nothing to watch live plots of: realtime asks the runtime for `--rtf 1`, which the CLI
    forwards to the executable untouched. Fast keeps the uncapped loop.
    """
    argv = (
        ["--headless"] + (["--rtf", "1"] if options.get("realtime", True) else [])
        if simulated and options.get("headless")
        else []
    )
    # Whether to record is a choice on any platform: no log means no replay and no live plots.
    if options.get("log", True) is False:
        argv.append("--no-log")
    # A run started from the page arms and waits for play: the transport is right there, and a
    # simulation that takes off on its own is already past what the operator wanted to watch.
    if simulated:
        argv.append("--start-paused")
    return argv


def start_run(generation_dir: Path, options: dict) -> dict:
    """Run a generation again, with the options `motion-spec run` takes for one.

    The CLI owns what a run is -- the run directory, the archive, the recovery afterwards --
    so start it rather than reimplementing it here, and let it say what it made.
    """
    if not (generation_dir / "generated/model/ir.json").exists():
        raise ValueError("not a generation")
    if run_status(generation_dir)["busy"]:
        raise ValueError("this generation is already running")
    from motion_spec.generation.pipeline import new_id

    # Name the run here rather than letting the CLI pick: the browser can then open the run
    # page at once and wait for the log, instead of polling for a directory to appear.
    run_id = new_id("run")
    argv = ["motion-spec", "run", str(generation_dir), "--cwd", str(WORKSPACE), "--run-id", run_id]
    # What a browser can meaningfully choose: the rest the dashboard already knows or the CLI
    # decides. A run always verifies what it archived; a recording nobody checked is not
    # worth the disk it sits on.
    # Only a simulator has a display to drop or a frame to record, whatever the browser posted.
    simulated = is_simulated(generation_dir)
    argv += run_arguments(options, simulated)
    # The runtime records: it holds the rendered frame, so it writes the video itself rather
    # than a reader sampling the live block it publishes for viewing.
    declared = {camera["id"] for camera in generation_cameras(generation_dir)}
    recording = [
        camera
        for camera in (options.get("cameras") or () if simulated else ())
        if camera in declared or camera == "default"
    ]
    for camera in recording:
        argv += ["--record", camera]
    sink = (generation_dir / RUN_LOG).open("wb")
    process = subprocess.Popen(
        argv, cwd=WORKSPACE, stdout=sink, stderr=sink, start_new_session=True
    )
    RUNNING[str(generation_dir)] = {"process": process, "run_id": run_id}
    return {
        **run_status(generation_dir),
        "command": argv,
        "recording": recording,
        "run": str((generation_dir / "runs" / run_id).relative_to(GENERATIONS)),
    }


ROBOT_TOML_REL = "generated/source/robot.toml"
# What a device says to wait for a connection, where its table says nothing.
CONNECT_TIMEOUT_MS = 1000


def _endpoints(table: dict, name: str):
    """Every endpoint one table tree names, as what it is rather than what it is called.

    A table with a host is reached over the network on every port it names; one whose port is a
    device path is reached over serial. Everything else -- topics, poses, tunings -- names
    nothing to connect to.
    """
    port = table.get("port")
    if name and isinstance(table.get("ip"), str):
        yield {
            "name": name,
            "kind": "network",
            "host": table["ip"],
            "ports": {
                key: value
                for key, value in table.items()
                if key.startswith("port") and isinstance(value, int) and not isinstance(value, bool)
            },
            "timeout_ms": table.get("connection_timeout_ms") or CONNECT_TIMEOUT_MS,
        }
    elif name and isinstance(port, str):
        # a bare name is one the driver itself resolves under /dev/
        yield {"name": name, "kind": "serial", "device": port if "/" in port else f"/dev/{port}"}
    for key, value in table.items():
        if isinstance(value, dict):
            yield from _endpoints(value, f"{name}.{key}" if name else key)


def device_endpoints(toml_path: Path) -> list[dict]:
    """What the config connects to: derived from the data, never from section names."""
    with toml_path.open("rb") as fh:
        return list(_endpoints(tomllib.load(fh), ""))


def _tcp_probe(host: str, port: int, timeout_s: float) -> dict:
    """Whether something answers on one port, right now."""
    try:
        socket.create_connection((host, port), timeout_s).close()
    except OSError as exc:
        return {"port": port, "ok": False, "detail": str(exc)}
    return {"port": port, "ok": True, "detail": None}


def _serial_probe(device: str) -> dict:
    """Whether one serial device is there and this user may open it."""
    if not Path(device).exists():
        return {"ok": False, "detail": "no such device"}
    if not os.access(device, os.R_OK | os.W_OK):
        return {"ok": False, "detail": "device is not readable and writable by this user"}
    return {"ok": True, "detail": None}


def probe_devices(generation_dir: Path) -> dict:
    """Try each endpoint once, now: TCP connect for network, stat for serial.

    The wire only: this says a port answers and a device node is openable, never that the
    protocol behind it agrees. The config's credentials stay in the config.
    """
    toml_path = generation_dir / ROBOT_TOML_REL
    if not toml_path.is_file():
        return {"devices": [], "config": None}
    devices = []
    for endpoint in device_endpoints(toml_path):
        if endpoint["kind"] == "network":
            timeout_s = endpoint["timeout_ms"] / 1000
            probes = [
                _tcp_probe(endpoint["host"], port, timeout_s) for port in endpoint["ports"].values()
            ]
            # A host named with no port to knock on is listed, and answers for nothing.
            devices.append(
                {
                    **endpoint,
                    "ports": probes,
                    "ok": all(p["ok"] for p in probes) if probes else None,
                }
            )
        else:
            devices.append({**endpoint, **_serial_probe(endpoint["device"])})
    return {"devices": devices, "config": str(toml_path)}


HEALTH: dict = {"checks": None, "stamp": None, "thread": None}


def health_report(refresh: bool = False) -> dict:
    """The CLI's health checks, run once and remembered; refresh reruns them.

    Some checks configure CMake projects, so they take seconds: run them on a thread the page
    polls rather than holding a request open, and keep the answer for the server's lifetime.
    """
    thread = HEALTH["thread"]
    running = thread is not None and thread.is_alive()
    if not running and (refresh or HEALTH["checks"] is None):
        from motion_spec.health import check_health

        def collect() -> None:
            checks = check_health(("all",), ("mujoco", "robif2b"))
            HEALTH["checks"] = [dataclasses.asdict(check) for check in checks]
            HEALTH["stamp"] = time.time()

        HEALTH["thread"] = threading.Thread(target=collect, daemon=True)
        HEALTH["thread"].start()
        running = True
    return {"running": running, "checks": HEALTH["checks"], "stamp": HEALTH["stamp"]}


CONSOLE_CHUNK = 256 * 1024


def console_slice(log: Path, offset: int) -> dict:
    """One poll of a text log: the bytes from offset, and where to ask from next.

    Raw text, ANSI escapes included: the page renders the colors and drops the rest.
    """
    if not log.is_file():
        return {"text": "", "offset": 0, "size": 0}
    size = log.stat().st_size
    if offset > size:
        offset = 0  # the log was replaced; start over
    with log.open("rb") as fh:
        fh.seek(offset)
        data = fh.read(CONSOLE_CHUNK)
    return {
        "text": data.decode("utf-8", errors="replace"),
        "offset": offset + len(data),
        "size": size,
    }


def console_log_for(path: Path) -> Path:
    """Which terminal log a dashboard path means: a run's console, or a generation's runner log."""
    if (path / "generated/model/ir.json").is_file():
        return path / RUN_LOG
    return path / "logs" / "console.log"


GENERATING: dict[str, dict] = {}
# under GENERATIONS; the catalog only lists generation dirs, but keep the logs out of the way
GENERATE_LOGS = ".dashboard"


def start_generate(source: Path, run: bool = True) -> dict:
    """Generate a .robmot from its page -- and run it, unless asked only to make it.

    Either way the terminal words are the record. Generate-only still builds, so the
    generation's own page can run it later; `gen` prints the directory it made on stdout,
    which is what hands it to `build`.
    """
    from motion_spec.generation.pipeline import new_id

    job = new_id("gen")
    log_dir = GENERATIONS / GENERATE_LOGS
    log_dir.mkdir(exist_ok=True)
    log = log_dir / f"{job}.log"
    if run:
        argv = ["motion-spec", "run", str(source), "-o", str(GENERATIONS), "--cwd", str(WORKSPACE)]
    else:
        # gen narrates on stdout and ends with the bare generation path; the capture eats
        # both. Announce the path into the log ourselves -- generate_status reads it from
        # there -- and let build's output carry the rest. (No tee back into the log: a
        # /dev/fd reopen starts at offset 0 and overwrites what the others wrote.)
        chain = (
            f"g=$(motion-spec gen code {shlex.quote(str(source))}"
            f" -o {shlex.quote(str(GENERATIONS))} | tail -n 1)"
            f' && printf "generation: %s\\n" "$g" && exec motion-spec build "$g"'
        )
        argv = ["sh", "-c", chain]
    sink = log.open("wb")
    process = subprocess.Popen(
        argv, cwd=WORKSPACE, stdout=sink, stderr=sink, start_new_session=True
    )
    GENERATING[job] = {"process": process, "log": log, "generation": None}
    return {"job": job, **generate_status(job)}


def generate_status(job: str) -> dict:
    """How far a page-started generation has got, and which generation it made."""
    started = GENERATING.get(job)
    if started is None:
        raise ValueError("unknown job")
    process = started["process"]
    busy = process.poll() is None
    if started["generation"] is None:
        # the CLI names the generation before generating: "generation: <abs path>" on stderr
        for line in started["log"].read_text(errors="replace").splitlines():
            if line.startswith("generation: "):
                made = Path(line.removeprefix("generation: ").strip())
                if GENERATIONS.resolve() in made.resolve().parents:
                    started["generation"] = str(made.resolve().relative_to(GENERATIONS.resolve()))
                break
    return {
        "busy": busy,
        "pid": process.pid if busy else None,
        "exit_code": None if busy else process.returncode,
        "generation": started["generation"],
    }


def generate_console(job: str, offset: int) -> dict:
    """The terminal output of a page-started generation, followed by byte offset."""
    started = GENERATING.get(job)
    if started is None:
        raise ValueError("unknown job")
    return console_slice(started["log"], offset)


_LIVE: dict[str, dict] = {}
# A log nobody has appended to for this long is one nobody is writing any more.
LIVE_IDLE_S = 3.0
# The most points one poll returns per signal; a poll's worth of ticks is decimated onto this.
LIVE_PLOT_POINTS = 300
# How often the sampler copies the runtime's latest-frame block.
LIVE_SAMPLE_HZ = 200
# ~20 s of run at that rate: how far back a poll can still reach.
LIVE_RING = 4000
# A session nobody has polled for this long has no page behind it any more.
LIVE_SESSION_S = 5.0


class ShmSampler(threading.Thread):
    """Copy the runtime's latest-frame block into a ring the live polls serve from.

    The block holds one frame, so a state that lasts under ~5 ms can pass between two samples;
    the frame log keeps every tick and the archive stays what a reader goes back to. This feeds
    the live view only, where re-reading the log once per poll is what made the page lag.
    """

    def __init__(self, reader: ShmFrameReader, fields: SignalFields, states: list, fired: list):
        super().__init__(daemon=True)
        self.reader, self.fields = reader, fields
        self.states, self.fired = states, fired
        self.lock = threading.Lock()
        self.ring = deque(maxlen=LIVE_RING)
        self.taken = 0  # samples ever appended; a poll cursor counts in these
        self.events: list = []
        self.frame, self.t, self.motion = 0, 0.0, None
        self.reading = False  # the block has published at least one frame
        self.advanced = 0.0  # monotonic clock at the last step advance
        self.touched = time.monotonic()
        self.stopped = threading.Event()
        self.seen = None  # step of the last frame read, advance or not
        self._state = self._event = None

    def touch(self) -> None:
        self.touched = time.monotonic()

    def moving(self) -> bool:
        """Whether the run stepped recently -- what `writing` means with no log to watch."""
        return time.monotonic() - self.advanced < LIVE_IDLE_S

    def since(self, cursor: int) -> tuple:
        """The samples appended since `cursor`, and the cursor that follows them."""
        with self.lock:
            ring, taken = list(self.ring), self.taken
        return ring[max(0, cursor - (taken - len(ring))) :], taken

    def close(self) -> None:
        self.stopped.set()

    def run(self) -> None:
        period = 1.0 / LIVE_SAMPLE_HZ
        checks = 0
        while not self.stopped.wait(period):
            if time.monotonic() - self.touched > LIVE_SESSION_S:
                break
            raw = self.reader.latest_raw()
            if raw is None:
                # The block appears when the runtime starts and goes when the run unlinks it;
                # a stat every sample would be its own load, so ask about once a second.
                checks += 1
                gone = self.reading and checks % LIVE_SAMPLE_HZ == 0
                if gone and not shm_path(self.reader.name).exists():
                    break
                continue
            self.absorb(raw)
        self.reader.close()

    def absorb(self, raw: bytes) -> None:
        """One sample, dropped unless the run has stepped since the last one."""
        self.reading = True
        core = self.fields.core(raw)
        step = core["step"]
        if step == self.seen:
            return
        # The block's standing frame is not an advance: a run armed at the play button may have
        # published one already, and armed means no events yet.
        first, self.seen = self.seen is None, step
        if first:
            return
        with self.lock:
            self.ring.append((self.taken, step, core["t"], core["active_motion"], raw))
            self.taken += 1
            self.frame, self.t, self.motion = step, core["t"], core["active_motion"]
            self.advanced = time.monotonic()
            if core["last_event"] != self._event:
                self._event = core["last_event"]
                if 0 <= self._event < len(self.fired):
                    self.events.append(
                        {"frame": step, "kind": "event", "label": self.fired[self._event]}
                    )
            if core["fsm_state"] != self._state:
                self._state = core["fsm_state"]
                label = (
                    self.states[self._state]
                    if 0 <= self._state < len(self.states)
                    else str(self._state)
                )
                self.events.append({"frame": step, "kind": "state", "label": label})


def _live_sampler(run_dir: Path, contract) -> ShmSampler | None:
    """This run's frame-block sampler, started, or None when the generation names no layout."""
    try:
        layout = FrameLayout.for_generation(run_dir.parent.parent)
        fields = SignalFields(layout, contract)
    except (OSError, ValueError, KeyError) as error:
        trace(f"live sampler {run_dir.name}: no frame layout ({error})")
        return None
    sampler = ShmSampler(
        ShmFrameReader(os.environ.get("MOTION_SPEC_SHM_NAME") or layout.shm_name, layout, contract),
        fields,
        [state.id for state in contract.header.fsm_states],
        [event.id for event in contract.header.fsm_events],
    )
    sampler.start()
    return sampler


def _follow_log(session: dict, log: Path, contract, period: float) -> None:
    """Where the run is, read off the log itself -- for a build that publishes no frame block.

    The archive is protobuf, so this parses; it is the fallback, never the live path.
    """
    if session["tail"] is None:
        session["tail"] = FrameLogTail(log)
    states = [state.id for state in contract.header.fsm_states]
    fired = [event.id for event in contract.header.fsm_events]
    for frame in session["tail"].poll(max(1, round(0.05 / period))):
        index = frame["step"]
        session["frame"] = index
        session["t"] = frame["t"]
        session["motion"] = frame["active_motion"]
        if frame["last_event"] != session["event"]:
            session["event"] = frame["last_event"]
            if 0 <= session["event"] < len(fired):
                session["events"].append(
                    {"frame": index, "kind": "event", "label": fired[session["event"]]}
                )
        if frame["fsm_state"] != session["state"]:
            session["state"] = frame["fsm_state"]
            label = (
                states[session["state"]]
                if 0 <= session["state"] < len(states)
                else str(session["state"])
            )
            session["events"].append({"frame": index, "kind": "state", "label": label})


def _close_live(session: dict) -> None:
    """A session that ends drops its sampler and whatever log handle it fell back to."""
    if session.get("sampler") is not None:
        session["sampler"].close()
    if session.get("tail") is not None:
        session["tail"].close()
    if session.get("channel") is not None:
        session["channel"].close()


def live_state(run_dir: Path, signals=()) -> dict:
    """How far a run being written has got, and the states and events it has passed.

    Live is the runtime's shared-memory frame, sampled at LIVE_SAMPLE_HZ into a ring: `signals`
    is served out of that ring as the increment since this page's last poll, values and events
    and states alike. Only a build that publishes no block falls back to following the log.
    """
    session = _LIVE.get(str(run_dir))
    if session is None:
        # Resolving parses the full header contract -- once per session, never per poll.
        _, log, _manifest, contract = resolve_archive(run_dir)
        for stale in list(_LIVE.values()):
            _close_live(stale)
        _LIVE.clear()
        session = _LIVE[str(run_dir)] = {
            "log": str(log),
            "contract": contract,
            "sampler": _live_sampler(run_dir, contract),
            "cursor": None,
            "tail": None,
            "events": [],
            "frame": 0,
            "size": -1,
            "state": None,
            "event": None,
            "motion": None,
        }
    log, contract = Path(session["log"]), session["contract"]
    sampler = session["sampler"]
    if sampler is not None and not sampler.is_alive():
        # A page whose tab was backgrounded stops polling; the sampler gives up and this asks
        # again. What it missed meanwhile is the archive's, not the live view's.
        sampler = session["sampler"] = _live_sampler(run_dir, contract)
        session["cursor"] = None
    if sampler is not None:
        sampler.touch()
    live = sampler is not None and sampler.reading
    period = (contract.header.nominal_period_ns or 1_000_000) / 1e9
    if not live:
        _follow_log(session, log, contract, period)
    names = tuple(signals)
    plot = None
    if names and live:
        # Only a cursor being created skips to the ring's end: history is the page's /api/plot
        # backfill. The cursor counts samples, not names, so a signal set that changes mid-run
        # -- a motion entering opens new charts -- keeps every sample since the last poll.
        if session["cursor"] is None:
            session["cursor"] = sampler.taken
        new, session["cursor"] = sampler.since(session["cursor"])
        stride = max(1, -(-len(new) // LIVE_PLOT_POINTS))
        sampled = new[::stride]
        rows = [
            sampler.fields.extract(raw, motion, names) for _index, _step, _t, motion, raw in sampled
        ]
        plot = {
            "frames": [step for _index, step, _t, _motion, _raw in sampled],
            "series": {name: [row[at] for row in rows] for at, name in enumerate(names)},
        }
    frame = sampler.frame if live else session["frame"]
    t = sampler.t if live else session.get("t")
    motion = sampler.motion if live else session["motion"]
    events = list(sampler.events) if live else session["events"]
    motions = contract.header.motions
    # A loop that answers is live even while paused. Only a run with no control block to ask
    # -- real hardware, or a build older than it -- has to be judged by its log growing.
    control = _control_status(session, run_dir, sampler.moving() if live else False)
    stat = log.stat()
    grew = stat.st_size > session["size"] >= 0
    session["size"] = stat.st_size
    writing = control["alive"] or (sampler.moving() if live else grew)
    if not control["available"] and not live:
        writing = writing or time.time() - stat.st_mtime < LIVE_IDLE_S
    return {
        "writing": writing,
        # Finished means archived and marked: the run row can only say how it ended once REC
        # has recorded that.
        "archived": run_ended(run_dir),
        "frames": frame + 1,
        "duration": t or (frame + 1) * period,
        "events": events,
        "control": control,
        "active_motion": (
            motions[motion].id if motion is not None and 0 <= motion < len(motions) else None
        ),
        **({"plot": plot} if plot is not None else {}),
    }


def _control_status(session: dict, run_dir: Path, moving: bool) -> dict:
    """The control block read, not pinged: a status poll must never write and wait.

    A moving sampler is proof of life for free; only an idle run gets the publish-and-ack
    ping, at most once every couple of seconds, so an armed run's polls stay cheap and a
    dead runtime is still found out.
    """
    channel = session.get("channel")
    if channel is None:
        generation_dir = run_dir.parents[1]
        channel = session["channel"] = ControlChannel(
            json_file(generation_dir / LAYOUT_REL).get("schema_hash")
        )
    now = time.monotonic()
    if moving:
        alive = True
        session["control_alive"] = (True, now)
    else:
        cached = session.get("control_alive")
        if cached is not None and now - cached[1] < 2.0:
            alive = cached[0]
        else:
            if channel.available:
                channel.set_pause(bool(channel.paused))
                alive = acknowledged(channel)
            else:
                alive = False
            session["control_alive"] = (alive, now)
    return {
        "available": channel.available,
        "alive": alive,
        "paused": channel.paused,
        "speed": channel.speed,
        "seq": channel.seq,
        "applied": channel.applied,
        "speed_range": [SPEED_MIN, SPEED_MAX],
    }


def run_control(path: Path, options: dict) -> dict:
    """Pause, step, cancel, or slow a simulation through the block its loop polls.

    Takes a run or its generation. `alive` is the loop's own ack, not a guess about who
    started it; only a simulated run creates the block.
    """
    generation_dir = path if (path / LAYOUT_REL).exists() else path.parent.parent
    channel = ControlChannel(json_file(generation_dir / LAYOUT_REL).get("schema_hash"))
    action = options.get("action")
    if action == "pause":
        channel.set_pause(True)
    elif action == "resume":
        channel.set_pause(False)
    elif action == "step":
        channel.set_pause(True)
        channel.request_steps(int(options.get("ticks") or 1))
    elif action == "speed":
        channel.set_speed(float(options.get("speed")))
    elif action == "cancel":
        channel.request_stop()
    elif action is None:
        # re-publish what the block says, to ask the loop for an ack
        channel.set_pause(bool(channel.paused))
    else:
        raise ValueError(f"unknown control action: {action}")
    return {
        "available": channel.available,
        "alive": acknowledged(channel),
        "paused": channel.paused,
        "speed": channel.speed,
        "seq": channel.seq,
        "applied": channel.applied,
        "speed_range": [SPEED_MIN, SPEED_MAX],
    }


def acknowledged(channel: ControlChannel, timeout_s: float = 0.4) -> bool:
    """Whether the loop reads the block: it copies each seq back once applied."""
    if not channel.available:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (channel.applied or 0) >= (channel.seq or 0):
            return True
        time.sleep(0.01)
    return False


def run_notebook(run_dir: Path) -> dict:
    """Seed a notebook beside the run that loads its signals, and open it in JupyterLab."""
    jupyter_server()
    notebook = run_dir / "analysis.ipynb"
    if not notebook.exists():
        notebook.write_text(json.dumps(NOTEBOOK_TEMPLATE(run_dir), indent=1))
    relative = notebook.relative_to(WORKSPACE)
    return {"url": f"{JUPYTER['base']}/lab/tree/{relative}?token={JUPYTER['token']}"}


def NOTEBOOK_TEMPLATE(run_dir: Path) -> dict:
    """One notebook whose first cells load this run and draw one of its signals."""
    cells = [
        [
            "from pathlib import Path\n",
            "\n",
            "from motion_spec.dashboard.server import plot_data, replay_data\n",
            "\n",
            f"run = Path({str(run_dir)!r})\n",
            "meta = replay_data(run)\n",
            "meta['frames'], meta['duration'], len(meta['signals'])",
        ],
        ["# every signal this run can answer for\n", "meta['signals'][:20]"],
        [
            "import matplotlib.pyplot as plt\n",
            "\n",
            "names = meta['constraints'][0]['tracking'] or meta['signals'][:1]\n",
            "data = plot_data(run, names)\n",
            "step = data['sample_step']\n",
            "\n",
            "figure, axes = plt.subplots(figsize=(7, 3))\n",
            "for name, series in data['signals'].items():\n",
            "    axes.plot([point * step for point in range(len(series))], series, label=name, lw=1)\n",
            "axes.set_xlabel('frame')\n",
            "axes.legend(fontsize=7)\n",
            "figure.tight_layout()\n",
            "figure.savefig('plot.pdf')  # vector, for LaTeX",
        ],
    ]
    return {
        "nbformat": 4,
        "nbformat_minor": 5,
        "metadata": {"kernelspec": {"name": "python3", "display_name": "Python 3"}},
        "cells": [
            {
                "cell_type": "code",
                "execution_count": None,
                "metadata": {},
                "outputs": [],
                "source": source,
            }
            for source in cells
        ],
    }


GRAPH_SAMPLE_S = 0.1  # the graph wants the shape of a run, not its every tick
_GRAPHS: dict[tuple[str, int, bool], GraphService] = {}


def run_model_manifest(run_dir: Path) -> Path | None:
    """The model graph this run names, wherever the manifest says it lives."""
    manifest = json_file(run_dir / "manifest.json").get("files", {})
    named = manifest.get("model")
    if not named:
        return None
    path = (run_dir / named).resolve()
    return path if path.is_file() else None


def run_graph(run_dir: Path, *, frames: bool) -> GraphService:
    """One run's queryable dataset: its model, plus what the recording says happened.

    Reading the frames is what fills `urn:runtime` and `urn:live`, and on a long run that
    costs tens of seconds, so a query that asks only about the model does not pay for it.
    Kept per log revision: a finished run is read once, a growing one is read again.
    """
    _, log, _manifest, contract = resolve_archive(run_dir)
    key = (str(log), log.stat().st_size, frames)
    if key not in _GRAPHS:
        store = RunStore(run_dir.name, contract)
        tail = FrameLogTail(log)
        if frames and tail.open():
            # shaping every frame to project a tenth of them is the waste, not the reading
            period = (contract.header.nominal_period_ns or 1_000_000) / 1e9
            stride = max(1, round(GRAPH_SAMPLE_S / period))
            while records := tail.poll(stride):
                store.add_frames(records)
            tail.close()
        _GRAPHS.clear()
        _GRAPHS[key] = GraphService(
            run_dir.parent.parent, store, manifest=run_model_manifest(run_dir)
        )
    return _GRAPHS[key]


QUERIES_REL = "queries.json"


def saved_queries(run_dir: Path) -> list:
    """The queries kept beside this run."""
    stored = json_file(run_dir / QUERIES_REL)
    return stored.get("queries", []) if isinstance(stored, dict) else []


def save_queries(run_dir: Path, queries: list) -> dict:
    """Keep a run's queries with the run, so they outlive the browser that wrote them."""
    if not (run_dir / "logs" / "frame_log.pb").exists():
        raise ValueError("queries belong to a run")
    texts = [str(query) for query in queries][:200]
    (run_dir / QUERIES_REL).write_text(json.dumps({"queries": texts}, indent=1))
    return {"saved": len(texts)}


def run_query(run_dir: Path, sparql: str) -> dict:
    """Answer one SPARQL query against a run, or say why it could not be answered."""
    # only a query that reaches for the recording pays for reading it
    recorded = any(word in sparql for word in ("urn:runtime", "urn:live", "sosa", "GRAPH ?"))
    service = run_graph(run_dir, frames=recorded)
    started = time.perf_counter()
    try:
        headers, rows = _query_rows(service, sparql)
    except Exception as exc:
        detail = f"{type(exc).__name__}: {exc}"
        if not len(service.model):
            # no prefixes, no model triples: the generation this run names is not there
            detail += (
                ". This run's model graph is unavailable -- the generation it names is missing, "
                "so only its recorded observations can be queried."
            )
        raise ValueError(detail) from exc
    prefixes = service.namespaces()
    return {
        "model_triples": len(service.model),
        "headers": headers or ["result"],
        "rows": [[curie(term, prefixes) for term in row] for row in rows[:500]],
        "count": len(rows),
        "truncated": len(rows) > 500,
        "elapsed_ms": round((time.perf_counter() - started) * 1000, 1),
        "namespaces": prefixes,
    }


def _query_rows(service: GraphService, sparql: str) -> tuple[list[str], list]:
    """Answer any query shape as headers and rows: ASK says so, a graph comes back as text."""
    service.sync()
    result = service.dataset.query(sparql)
    if result.type == "ASK":
        return ["answer"], [(result.askAnswer,)]
    if result.type in ("CONSTRUCT", "DESCRIBE"):
        return ["triples"], [
            (line,) for line in result.serialize(format="turtle").decode().splitlines() if line
        ]
    return [str(var) for var in (result.vars or [])], [tuple(row) for row in result]


def curie(term, prefixes: dict) -> str | None:
    """A term as its shortest bound prefix form, so a table stays readable."""
    if term is None:
        return None
    text = str(term)
    if not isinstance(term, rdflib.URIRef):
        return text
    prefix, namespace = max(
        ((prefix, ns) for prefix, ns in prefixes.items() if text.startswith(ns)),
        key=lambda item: len(item[1]),
        default=(None, None),
    )
    return f"{prefix}:{text[len(namespace) :]}" if prefix else text


class LifecycleListener:
    """Bridge REC lifecycle datagrams into one dashboard revision stream."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._revision = 0
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        socket_path().unlink(missing_ok=True)
        self._socket.bind(str(socket_path()))
        threading.Thread(target=self._listen, daemon=True).start()

    def wait(self, revision: int, timeout: float) -> int:
        """Wait for a new REC transition or a stream keepalive timeout."""
        with self._condition:
            self._condition.wait_for(lambda: self._revision != revision, timeout=timeout)
            return self._revision

    def _listen(self) -> None:
        while True:
            self._socket.recv(4096)
            with self._condition:
                self._revision += 1
                self._condition.notify_all()


class DashboardHandler(SimpleHTTPRequestHandler):
    """Expose only data endpoints; all dashboard layout lives in static browser assets."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=FRONTEND, **kwargs)

    def end_headers(self) -> None:
        """Always serve the local dashboard shell and scripts fresh."""
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def send_video(self, path: Path) -> None:
        """Serve one recording, honouring Range so the player can seek."""
        size = path.stat().st_size
        start, end = 0, size - 1
        asked = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range", "") or "")
        if asked and (asked.group(1) or asked.group(2)):
            if asked.group(1):
                start = min(int(asked.group(1)), size - 1)
                end = int(asked.group(2)) if asked.group(2) else end
            else:
                start = max(0, size - int(asked.group(2)))
            end = min(end, size - 1)
        partial = asked is not None
        self.send_response(HTTPStatus.PARTIAL_CONTENT if partial else HTTPStatus.OK)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end - start + 1))
        if partial:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        with path.open("rb") as fh:
            fh.seek(start)
            remaining = end - start + 1
            while remaining > 0 and (chunk := fh.read(min(1 << 16, remaining))):
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def send_json(self, data: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream_events(self) -> None:
        """Keep one browser stream open for REC lifecycle changes."""
        assert LIFECYCLE is not None
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        revision = LIFECYCLE.wait(-1, 0)
        try:
            while True:
                changed = LIFECYCLE.wait(revision, 30)
                if changed != revision:
                    revision = changed
                    self.wfile.write(b"event: lifecycle\ndata: changed\n\n")
                else:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if not parsed.path.startswith("/api/"):
            return super().do_GET()
        if parsed.path == "/api/events":
            return self.stream_events()
        try:
            query = parse_qs(parsed.query)
            value = query.get("path", [""])[0]
            if parsed.path == "/api/roots":
                return self.send_json(roots())
            if parsed.path == "/api/pick-root":
                return self.send_json(pick_root(query.get("kind", [""])[0]))
            if parsed.path == "/api/generations":
                return self.send_json(
                    [
                        generation_info(generation.dir)
                        for generation in GenerationCatalog([GENERATIONS]).generations()
                    ]
                )
            if parsed.path == "/api/storage":
                return self.send_json(storage_info())
            if parsed.path == "/api/generation":
                return self.send_json(generation_details(relative_path(GENERATIONS, value)))
            if parsed.path == "/api/generation-graph":
                return self.send_json(
                    provenance_graph(relative_path(GENERATIONS, value), query.get("graph", []))
                )
            if parsed.path == "/api/runs":
                generation = relative_path(GENERATIONS, value)
                runs = sorted(path for path in generation.glob("runs/*") if path.is_dir())
                return self.send_json([run_info(path) for path in reversed(runs)])
            if parsed.path == "/api/run":
                return self.send_json(run_status(relative_path(GENERATIONS, value)))
            if parsed.path == "/api/devices":
                return self.send_json(probe_devices(relative_path(GENERATIONS, value)))
            if parsed.path == "/api/health":
                return self.send_json(health_report(bool(query.get("refresh", [""])[0])))
            if parsed.path == "/api/generate":
                return self.send_json(generate_status(query.get("job", [""])[0]))
            if parsed.path == "/api/console":
                offset = int(query.get("offset", ["0"])[0])
                job = query.get("job", [""])[0]
                if job:
                    return self.send_json(generate_console(job, offset))
                # expected_path, not relative_path: a run directory is named before it exists
                target = expected_path(GENERATIONS, value)
                return self.send_json(console_slice(console_log_for(target), offset))
            if parsed.path == "/api/queries":
                return self.send_json(saved_queries(expected_path(GENERATIONS, value)))
            if parsed.path == "/api/video":
                run = relative_path(GENERATIONS, value)
                return self.send_video(video_file(run, query.get("camera", [""])[0]))
            if parsed.path == "/api/replay":
                return self.send_json(replay_data(expected_path(GENERATIONS, value)))
            if parsed.path == "/api/plot":
                bounds = query.get("window", [])
                return self.send_json(
                    plot_data(
                        relative_path(GENERATIONS, value),
                        query.get("signal", []),
                        (int(bounds[0]), int(bounds[1])) if len(bounds) == 2 else None,
                    )
                )
            if parsed.path == "/api/sources":
                files = [
                    str(path.relative_to(WORKSPACE))
                    for path in WORKSPACE.rglob("*")
                    if path.suffix in AUTHORED
                    and not IGNORED.intersection(path.relative_to(WORKSPACE).parts)
                    and not any(part.startswith(".") for part in path.relative_to(WORKSPACE).parts)
                ]
                return self.send_json(sorted(files))
            if parsed.path == "/api/jupyter":
                return self.send_json(jupyter_server())
            if parsed.path == "/api/source":
                return self.send_json(read_source(value))
            self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
        except (OSError, ValueError, KeyError, IndexError, json.JSONDecodeError) as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def do_POST(self) -> None:
        """Update dashboard roots, open a source, or delete selected runs and generations."""
        if not self._same_origin():
            return self.send_json({"error": "cross-origin request"}, HTTPStatus.FORBIDDEN)
        try:
            length = int(self.headers["Content-Length"])
            body = json.loads(self.rfile.read(length))
            if self.path == "/api/roots":
                return self.send_json(set_root(body["kind"], body["path"]))
            if self.path == "/api/open":
                return self.send_json(open_source(body["path"], body.get("editor")))
            if self.path == "/api/terminal":
                return self.send_json(open_terminal(body["path"]))
            if self.path == "/api/run":
                return self.send_json(
                    start_run(relative_path(GENERATIONS, body["path"]), body.get("options") or {})
                )
            if self.path == "/api/generate":
                # source_path takes any authored file; only a .robmot is a whole run to make
                if not body["path"].endswith(".robmot"):
                    raise ValueError("only a .robmot generates")
                return self.send_json(
                    start_generate(source_path(body["path"]), run=body.get("run", True))
                )
            if self.path == "/api/live":
                return self.send_json(
                    live_state(relative_path(GENERATIONS, body["path"]), body.get("signals") or ())
                )
            if self.path == "/api/control":
                return self.send_json(
                    run_control(relative_path(GENERATIONS, body["path"]), body.get("options") or {})
                )
            if self.path == "/api/queries":
                return self.send_json(
                    save_queries(expected_path(GENERATIONS, body["path"]), body["queries"])
                )
            if self.path == "/api/sparql":
                return self.send_json(
                    run_query(relative_path(GENERATIONS, body["path"]), body["query"])
                )
            if self.path == "/api/notebook":
                return self.send_json(run_notebook(relative_path(GENERATIONS, body["path"])))
            if self.path != "/api/delete":
                return self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
            selected = body["paths"]
            targets = [relative_path(GENERATIONS, value) for value in selected]
            if not targets or any(not self._deletable(path) for path in targets):
                raise ValueError("only generation bundles and run archives can be deleted")
            targets = [
                target
                for target in targets
                if not any(target in other.parents for other in targets)
            ]
            for target in sorted(targets, key=lambda item: len(item.parts), reverse=True):
                trash(target)
            # a model folder emptied of its generations is no longer a model folder
            folders = 0
            for folder in {target.parent for target in targets}:
                if GENERATIONS in folder.parents and folder.is_dir() and not any(folder.iterdir()):
                    trash(folder)
                    folders += 1
            directory_size.cache_clear()
            storage_info.cache_clear()
            self.send_json({"deleted": len(targets), "folders": folders})
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def _same_origin(self) -> bool:
        """Only the dashboard's own page may POST: these endpoints delete and spawn."""
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            return False
        origin = self.headers.get("Origin")
        return origin is None or urlparse(origin).hostname in ("127.0.0.1", "localhost")

    @staticmethod
    def _deletable(path: Path) -> bool:
        return (path / "generated/model/ir.json").exists() or (
            path.parent.name == "runs" and (path / "logs/frame_log.pb").exists()
        )


def serve(port: int = 8080, logs: Path | None = None, sources: Path | None = None) -> None:
    """Serve the dashboard for one pair of roots until interrupted."""
    global GENERATIONS, LIFECYCLE, WORKSPACE
    if logs is not None:
        GENERATIONS = Path(logs).expanduser().resolve()
        WORKSPACE = GENERATIONS.parent
    if sources is not None:
        WORKSPACE = Path(sources).expanduser().resolve()
    LIFECYCLE = LifecycleListener()
    atexit.register(stop_jupyter)
    for name in (signal.SIGTERM, signal.SIGINT):
        signal.signal(name, lambda *_: sys.exit(0))
    server = ThreadingHTTPServer(("127.0.0.1", port), DashboardHandler)
    print(f"motion-spec dashboard: http://127.0.0.1:{port}")
    print(f"  runs from {GENERATIONS}\n  sources from {WORKSPACE}")
    server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--logs", type=Path, help=f"generation root (default ${GENERATION_DIR_ENV})"
    )
    parser.add_argument(
        "--sources", type=Path, help="model sources root (default: the logs root's parent)"
    )
    args = parser.parse_args()
    serve(args.port, args.logs, args.sources)


if __name__ == "__main__":
    main()
