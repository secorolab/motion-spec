# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Serve recorded motion-spec generations and runs to the dashboard frontend."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import socket
import subprocess
import threading
from datetime import datetime, timezone
from functools import lru_cache
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from rdflib import Dataset

from motion_spec.dashboard.runs import GenerationCatalog, GenerationInfo, RunInfo
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
    constraints = source_constraints(path)
    source_files = sorted((path / "generated/source").glob("*"))
    fsm = next((source for source in source_files if source.suffix == ".fsm"), None)
    description = re.search(r'description:\s*"([^"]+)"', fsm.read_text()) if fsm else None
    details["authored_constraints"] = len(constraints)
    details["motions"] = len({constraint["motion"] for constraint in constraints})
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


def source_constraints(generation_dir: Path) -> list[dict]:
    """Return the motion constraints as their authored `.robmot` source lines."""
    source = next((path for path in (generation_dir / "generated/source").glob("*.robmot")), None)
    if source is None:
        return []
    controllers = json_file(generation_dir / "generated/model/ir.json").get("communication", {}).get(
        "introspection", {}
    ).get("controllers", [])
    controller_signals = {
        controller["id"]: [controller[key] for key in ("error_signal", "output_signal") if controller.get(key)]
        for controller in controllers
    }
    bindings = {}
    for name, target in re.findall(
        r"(?:pid|impedance|feed-forward)\s+([\w-]+)\s*\{\s*(?:constraint:\s*)?<([^>]+)>",
        source.read_text(),
    ):
        bindings.setdefault(target, []).extend(
            signal
            for controller, signals in controller_signals.items()
            if controller.startswith(name.replace("-", "_"))
            for signal in signals
        )
    constraints = []
    motion = None
    for number, line in enumerate(source.read_text().splitlines(), 1):
        text = line.strip().rstrip(",")
        match = re.match(r"guarded-motion\s+\(ns=[^)]+\)\s+([\w-]+)", text)
        if match:
            motion = match.group(1)
        if ":" not in text or not any(word in text for word in ("keeping ", "moving ", "progress ")):
            continue
        name, expression = text.split(":", 1)
        signals = list(dict.fromkeys(bindings.get(f"{motion}.{name}", [])))
        constraints.append(
            {
                "motion": motion,
                "line": number,
                "name": name,
                "expression": expression.strip(),
                "signals": signals,
            }
        )
    return constraints


def replay_data(run_dir: Path) -> dict:
    """Return replay metadata without decoding the complete frame log."""
    _, log, _manifest, contract = resolve_archive(run_dir)
    constraints = source_constraints(run_dir.parent.parent)
    health = read_health(log) or {}
    frame_count = health.get("written_frames", 0)
    signals = ["timing.compute_ms", "timing.period_ms"]
    signals.extend(
        f"{field['id']}.{key}"
        for field in contract.fields["constraints"]
        for key in ("error", "output", "measured", "setpoint", "satisfied")
    )
    signals.extend(
        f"{field['id']}.{key}"
        for field in contract.fields["monitors"]
        for key in ("value", "satisfied")
    )
    signals.extend(signal for constraint in constraints for signal in constraint["signals"])
    return {
        "generation": str(run_dir.parent.parent.relative_to(GENERATIONS)),
        "frames": frame_count,
        "duration": frame_count * contract.header.nominal_period_ns / 1e9,
        "states": [state.id for state in contract.header.fsm_states],
        "events": [],
        "signals": list(dict.fromkeys(signals)),
        "constraints": constraints,
        "header": validate_header(log, contract),
        "health": health,
    }


def plot_data(run_dir: Path, names: list[str]) -> dict:
    """Stream and downsample requested fields without shaping complete frames."""
    _, log, _manifest, contract = resolve_archive(run_dir)
    constraint_slots = {field["id"]: index for index, field in enumerate(contract.fields["constraints"])}
    monitor_slots = {field["id"]: index for index, field in enumerate(contract.fields["monitors"])}
    quantities = {field["id"]: field for field in contract.fields["quantities"]}
    health = read_health(log) or {}
    step = max(1, health.get("written_frames", 0) // 1600)
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
        prefix, key = name.rsplit(".", 1)
        if prefix in constraint_slots:
            field = contract.fields["constraints"][constraint_slots[prefix]]
            return getattr(getattr(frame, field["name"]), key)
        if prefix in monitor_slots:
            field = contract.fields["monitors"][monitor_slots[prefix]]
            return getattr(getattr(frame, field["name"]), key)
        raise ValueError(f"unknown signal: {name}")

    events, previous = [], None
    states = [state.id for state in contract.header.fsm_states]
    index = 0
    with log.open("rb") as fh:
        frame_log_pb._read_delimited(fh)
        while data := frame_log_pb._read_delimited(fh, partial_ok=True):
            record = contract.record_cls()
            record.ParseFromString(data)
            if record.WhichOneof("record") != "frame":
                continue
            frame = record.frame
            if frame.fsm_state != previous:
                state = frame.fsm_state
                events.append({"frame": index, "state": states[state] if 0 <= state < len(states) else str(state)})
                previous = state
            if index % step == 0:
                for name in names:
                    series[name].append(value(frame, name))
            index += 1
    return {
        "signals": series,
        "events": events,
        "sample_step": step,
    }


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
                return self.send_json(plot_data(relative_path(GENERATIONS, value), query.get("signal", [])))
            if parsed.path == "/api/sources":
                files = [
                    str(path.relative_to(WORKSPACE))
                    for path in WORKSPACE.rglob("*.robmot")
                    if not IGNORED.intersection(path.relative_to(WORKSPACE).parts)
                    and not any(part.startswith(".") for part in path.relative_to(WORKSPACE).parts)
                ]
                return self.send_json(sorted(files))
            self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
        except (OSError, ValueError, KeyError, IndexError, json.JSONDecodeError) as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def do_POST(self) -> None:
        """Update dashboard roots or delete selected run archives and generations."""
        try:
            length = int(self.headers["Content-Length"])
            body = json.loads(self.rfile.read(length))
            if self.path == "/api/roots":
                return self.send_json(set_root(body["kind"], body["path"]))
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
    server = ThreadingHTTPServer(("127.0.0.1", args.port), DashboardHandler)
    print(f"motion-spec dashboard: http://127.0.0.1:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
