# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Serve recorded motion-spec generations and runs to the dashboard frontend."""

from __future__ import annotations

import argparse
import atexit
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
from datetime import datetime, timezone
from functools import lru_cache
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rdflib
from rdflib import Dataset

from motion_spec.dashboard.graph import GraphService
from motion_spec.dashboard.runs import GenerationCatalog, GenerationInfo, RunInfo
from motion_spec.dashboard.store import RunStore
from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.lifecycle_events import socket_path
from motion_spec.introspection.replay import (
    read_health,
    resolve_archive,
    validate_header,
)

WORKSPACE = Path("/home/batsy/work/ms")
GENERATIONS = WORKSPACE / "generations"
FRONTEND = Path(__file__).with_name("frontend")
IGNORED = {"build", ".git", ".venv", "generations", "install", "log", "__pycache__", "test", "tests"}
AUTHORED = (".robmot", ".fsm", ".scenex")
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
        check=False, capture_output=True, text=True, timeout=300,
    )
    if result.returncode:
        raise ValueError("folder selection cancelled")
    return set_root(kind, result.stdout.strip())


def relative_path(root: Path, value: str) -> Path:
    """Resolve a request path inside root, rejecting traversal and missing paths."""
    path = (root / value).resolve()
    if root.resolve() not in (path, *path.parents) or not path.exists():
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
    "xdg-terminal-exec": [], "kitty": [], "foot": [], "ghostty": ["-e"], "alacritty": ["-e"],
    "wezterm": ["start", "--"], "gnome-terminal": ["--"], "konsole": ["-e"], "xterm": ["-e"],
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
        found.update({
            name: [*host[1], name] for name in TERMINAL_EDITORS if shutil.which(name)
        })
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
        "name": GenerationInfo(path).model,
        "created": GenerationInfo(path).timestamp,
        "source": source.name if source else None,
        "backend": layout.get("platform", {}).get("backend"),
        "platform": layout.get("platform", {}).get("name"),
        "simulated": layout.get("platform", {}).get("simulated"),
        "schema_hash": layout.get("schema_hash"),
        "size_bytes": directory_size(path),
        "pools": layout.get("pools", {}),
        "runs": len(GenerationInfo(path).runs),
        "has_fsm": any((path / "generated/model").glob("*_fsm.svg")),
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
    details["source_files"] = [{"name": source.name, "path": str(source)} for source in source_files if source.is_file()]
    details["generated_files"] = [
        {"name": str(source.relative_to(path / "generated")), "path": str(source)}
        for source in sorted((path / "generated").rglob("*"))
        if source.is_file() and "source" not in source.relative_to(path / "generated").parts
    ]
    generated = path / "generated"
    details["rdf_graphs"] = sorted(str(source.relative_to(generated)) for source in generated.rglob("*.ld.json"))
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
    triples = [(subject, predicate, obj) for subject, predicate, obj, _context in graph.quads((None, None, None, None))]
    terms = sorted({term for subject, _predicate, obj in triples for term in (subject, obj)}, key=str)
    node_ids = {term: str(index) for index, term in enumerate(terms)}
    return {
        "nodes": [{"id": node_ids[term], "label": rdf_name(term), "value": str(term)} for term in terms],
        "links": [
            {"source": node_ids[subject], "target": node_ids[obj], "label": rdf_name(predicate), "value": str(predicate)}
            for subject, predicate, obj in triples
        ],
    }


def run_info(path: Path) -> dict:
    log = path / "logs/frame_log.pb"
    health = read_health(log) or {}
    run_id = RunInfo(path).run_id
    _, _, _, contract = resolve_archive(path)
    match = re.fullmatch(r"run-(\d{8}T\d{6}\d{6}Z)", run_id)
    return {
        "path": str(path.relative_to(GENERATIONS)),
        "id": run_id,
        "started": datetime.strptime(match.group(1), "%Y%m%dT%H%M%S%fZ").replace(tzinfo=timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC") if match else "—",
        "complete": health.get("complete") or RunInfo(path).status == "COMPLETED",
        "status": RunInfo(path).status,
        "written_frames": health.get("written_frames"),
        "duration_s": health.get("written_frames", 0) * contract.header.nominal_period_ns / 1e9,
        "dropped_frames": health.get("dropped_frames"),
    }


def downsample(values: list[float], target: int = 1600) -> list[float]:
    step = max(1, len(values) // target)
    return values[::step]


# How a spatial slot's proto fields read as authored components.
SLOT_PARTS = {
    "poses": {"position.x": "px", "position.y": "py", "position.z": "pz",
              "orientation.x": "qx", "orientation.y": "qy", "orientation.z": "qz",
              "orientation.w": "qw"},
    "twists": {"linear.x": "lx", "linear.y": "ly", "linear.z": "lz",
               "angular.x": "ax", "angular.y": "ay", "angular.z": "az"},
    "wrenches": {"force.x": "fx", "force.y": "fy", "force.z": "fz",
                 "torque.x": "tx", "torque.y": "ty", "torque.z": "tz"},
}


def slot_signals(contract) -> dict:
    """Every spatial slot component as signal name -> (kind, field, component)."""
    return {
        f"{field['id']}.{part}": (kind, field, attribute)
        for kind, parts in SLOT_PARTS.items()
        for field in contract.fields.get(kind, ())
        for part, attribute in parts.items()
        if hasattr(contract.record_cls().frame, field["name"])
    }


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


def _constraint_row(motion, kind: str, group: list, constants: dict, authored: dict) -> dict:
    """One constraint's row: its identity and signals from the header, its line from the source."""
    first = group[0]
    name = first.constraint_id or rdf_name(first.constraint_iri) or first.id
    key = (_key(motion.id).removeprefix("motion_"), _key(name))
    line, expression, authored_motion = authored.get(key) or next(
        (value for (_motion, authored_name), value in authored.items() if authored_name == key[1]),
        (None, None, None),
    )
    # The header names the pair the evaluator compares; measured/setpoint only where it does not.
    operands = list(dict.fromkeys(value for slot in group for value in slot.operand_ids))
    compared = operands or [*_slot_ids(group, "measured_id"), *_slot_ids(group, "setpoint_id")]
    evaluator = next(iter(_slot_ids(group, "evaluator_id")), None)
    return {
        "motion": authored_motion or motion.id,
        "handler": motion.id,
        "line": line,
        "name": name,
        "expression": expression,
        "kind": kind,
        # The closure the header names, or the slot computing the error where it names none.
        "evaluator": evaluator or first.iri or first.id or None,
        "between": compared,
        "tracking": [value for value in compared if value not in constants],
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


def replay_data(run_dir: Path) -> dict:
    """Return replay metadata without decoding the complete frame log."""
    run_dir, log, manifest, contract = resolve_archive(run_dir)
    constraints = source_constraints(run_dir, manifest, contract)
    health = read_health(log) or {}
    frame_count = health.get("written_frames", 0)
    signals = ["timing.compute_ms", "timing.period_ms"]
    signals.extend(
        f"{field['id']}.{key}"
        for field in contract.fields["constraints"]
        for key in ("error", "output", "measured", "setpoint", "satisfied")
    )
    signals.extend(
        f"{slot.id}.{key}"
        for motion in contract.header.motions for slot in motion.monitors
        for key in ("value", "satisfied")
    )
    signals.extend(slot_signals(contract))
    indices = {motion.id: motion.index for motion in contract.header.motions}
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
                signal for name in constraint[key]
                for signal in ([name] if name in logged else sorted(parts.get(name, ())))
            ]
        constraint["window"] = windows.get(indices.get(constraint.pop("handler")))
    signals.extend(
        signal for constraint in constraints
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
        "constraints": constraints,
        "header": validate_header(log, contract),
        "health": health,
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


def plot_data(run_dir: Path, names: list[str], window: tuple | None = None) -> dict:
    """Stream and downsample requested fields without shaping complete frames.

    Sampling follows the window asked for, so a motion that lasted a handful of frames is
    drawn from those frames rather than missed between two samples of the whole run.
    """
    _, log, _manifest, contract = resolve_archive(run_dir)
    constraint_slots = {field["id"]: index for index, field in enumerate(contract.fields["constraints"])}
    slots = slot_signals(contract)
    monitor_slots = {
        slot.id: (contract.fields["monitors"][slot.number], motion.index)
        for motion in contract.header.motions for slot in motion.monitors
        if slot.number < len(contract.fields["monitors"])
    }
    quantities = {field["id"]: field for field in contract.fields["quantities"]}
    health = read_health(log) or {}
    first, last = window or (0, max(0, health.get("written_frames", 0) - 1))
    step = max(1, (last - first + 1) // 1600)
    series = {name: [] for name in names}

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
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    token = secrets.token_urlsafe(24)
    settings = lab_settings()
    framing = json.dumps({"headers": {
        "Content-Security-Policy": "frame-ancestors 'self' http://127.0.0.1:8080 http://localhost:8080",
    }})
    JUPYTER["process"] = subprocess.Popen(
        [
            "jupyter", "lab", "--no-browser", f"--port={port}",
            f"--ServerApp.root_dir={WORKSPACE}", f"--IdentityProvider.token={token}",
            f"--LabServerApp.user_settings_dir={settings}",
            f"--ServerApp.tornado_settings={framing}",
            "--ServerApp.disable_check_xsrf=True", "--ServerApp.open_browser=False",
        ],
        cwd=WORKSPACE, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
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
        [
            "# every signal this run can answer for\n",
            "meta['signals'][:20]",
        ],
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
        "nbformat": 4, "nbformat_minor": 5,
        "metadata": {"kernelspec": {"name": "python3", "display_name": "Python 3"}},
        "cells": [
            {"cell_type": "code", "execution_count": None, "metadata": {}, "outputs": [], "source": source}
            for source in cells
        ],
    }


GRAPH_SAMPLE_S = 0.1  # the graph wants the shape of a run, not its every tick
_GRAPHS: dict[tuple[str, int, bool], GraphService] = {}


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
        _GRAPHS[key] = GraphService(run_dir.parent.parent, store)
    return _GRAPHS[key]


def run_query(run_dir: Path, sparql: str) -> dict:
    """Answer one SPARQL query against a run, or say why it could not be answered."""
    # only a query that reaches for the recording pays for reading it
    recorded = any(word in sparql for word in ("urn:runtime", "urn:live", "sosa", "GRAPH ?"))
    service = run_graph(run_dir, frames=recorded)
    started = time.perf_counter()
    try:
        headers, rows = service.query(sparql)
    except Exception as exc:
        raise ValueError(f"{type(exc).__name__}: {exc}") from exc
    prefixes = service.namespaces()
    return {
        "headers": headers or ["result"],
        "rows": [[curie(term, prefixes) for term in row] for row in rows[:500]],
        "count": len(rows),
        "truncated": len(rows) > 500,
        "elapsed_ms": round((time.perf_counter() - started) * 1000, 1),
        "namespaces": prefixes,
    }


def curie(term, prefixes: dict) -> str | None:
    """A term as its shortest bound prefix form, so a table stays readable."""
    if term is None:
        return None
    text = str(term)
    if not isinstance(term, rdflib.URIRef):
        return text
    prefix, namespace = max(
        ((prefix, ns) for prefix, ns in prefixes.items() if text.startswith(ns)),
        key=lambda item: len(item[1]), default=(None, None),
    )
    return f"{prefix}:{text[len(namespace):]}" if prefix else text


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
                    [generation_info(generation.dir) for generation in GenerationCatalog([GENERATIONS]).generations()]
                )
            if parsed.path == "/api/storage":
                return self.send_json(storage_info())
            if parsed.path == "/api/generation":
                return self.send_json(generation_details(relative_path(GENERATIONS, value)))
            if parsed.path == "/api/generation-graph":
                return self.send_json(
                    provenance_graph(
                        relative_path(GENERATIONS, value),
                        query.get("graph", []),
                    )
                )
            if parsed.path == "/api/runs":
                generation = relative_path(GENERATIONS, value)
                runs = sorted(path for path in generation.glob("runs/*") if path.is_dir())
                return self.send_json([run_info(path) for path in reversed(runs)])
            if parsed.path == "/api/replay":
                return self.send_json(replay_data(relative_path(GENERATIONS, value)))
            if parsed.path == "/api/plot":
                bounds = query.get("window", [])
                return self.send_json(plot_data(
                    relative_path(GENERATIONS, value), query.get("signal", []),
                    (int(bounds[0]), int(bounds[1])) if len(bounds) == 2 else None,
                ))
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
                target for target in targets
                if not any(target in other.parents for other in targets)
            ]
            for target in sorted(targets, key=lambda item: len(item.parts), reverse=True):
                shutil.rmtree(target)
            directory_size.cache_clear()
            storage_info.cache_clear()
            self.send_json({"deleted": len(targets)})
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


def main() -> None:
    global LIFECYCLE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    LIFECYCLE = LifecycleListener()
    atexit.register(stop_jupyter)
    for name in (signal.SIGTERM, signal.SIGINT):
        signal.signal(name, lambda *_: sys.exit(0))
    server = ThreadingHTTPServer(("127.0.0.1", args.port), DashboardHandler)
    print(f"motion-spec dashboard: http://127.0.0.1:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
