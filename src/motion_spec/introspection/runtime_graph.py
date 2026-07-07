# SPDX-License-Identifier: MPL-2.0
"""Recover runtime RDF from archive-local run data."""

from __future__ import annotations

import json
import os
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import load_manifest, sha256_file
from motion_spec.introspection.artifacts import MSPROV, prov_uri


def _archive_location(rel: str) -> rdflib.URIRef:
    """atLocation for an archived file, relative to runtime/runtime.ttl (portable, no
    machine path). Resolves correctly when the graph is parsed from its own file base."""
    return rdflib.URIRef(os.path.relpath(rel, "runtime"))

PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
BDD = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
AGN = rdflib.Namespace("https://secorolab.github.io/metamodels/agent#")
OBS = rdflib.Namespace("https://secorolab.github.io/metamodels/observation#")
RT = rdflib.Namespace("https://secorolab.github.io/metamodels/runtime#")
MSRUN = rdflib.Namespace("https://secorolab.github.io/motion-spec/runtime/")
RUNTIME_RDF_CONTRACT_VERSION = 1
KIND_STATE = 0
KIND_EVENT = 1
KIND_CONSTRAINT_SAT = 2
KIND_CONSTRAINT_UNSAT = 3
KIND_MONITOR = 4
TRIGGER_TYPES = {
    KIND_STATE: "StateOccurrence",
    KIND_EVENT: "EventOccurrence",
    KIND_CONSTRAINT_SAT: "ConstraintSatisfiedOccurrence",
    KIND_CONSTRAINT_UNSAT: "ConstraintUnsatisfiedOccurrence",
    KIND_MONITOR: "MonitorOccurrence",
}


def _node(identifier: str) -> rdflib.URIRef:
    safe = identifier.replace(":", "/")
    return MSRUN[safe]


def _uri(value: str | None) -> rdflib.URIRef | None:
    return rdflib.URIRef(value) if value else None


def _literal(g: rdflib.Graph, subject: rdflib.URIRef, predicate: rdflib.URIRef, value) -> None:
    if value is not None:
        g.add((subject, predicate, rdflib.Literal(value)))


def _state_maps(schema: dict) -> tuple[dict[int, dict], dict[int, dict]]:
    states = {
        state.get("index", idx): state
        for idx, state in enumerate(schema.get("fsm", {}).get("states", []))
    }
    events = {
        event.get("index", idx): event
        for idx, event in enumerate(schema.get("fsm", {}).get("events", []))
    }
    return states, events


def _state_meta(schema: dict, states: dict[int, dict], state_idx: int) -> tuple[dict | None, dict]:
    state = states.get(state_idx)
    if not state:
        return None, {}
    return state, schema.get("by_state", {}).get(state.get("id"), {})


def _slot_uri(slot: dict, *keys: str) -> rdflib.URIRef | None:
    for key in keys:
        uri = _uri(slot.get(key))
        if uri is not None:
            return uri
    return None


def _add_signal_sample(
    g: rdflib.Graph,
    run_id: str,
    frame_node: rdflib.URIRef,
    frame: dict,
    slot_idx: int,
    role: str,
    signal_uri: str | None,
    value,
    *,
    controller_uri: rdflib.URIRef | None = None,
    constraint_uri: rdflib.URIRef | None = None,
    monitor_uri: rdflib.URIRef | None = None,
) -> None:
    signal = _uri(signal_uri)
    if signal is None:
        return
    sample = _node(f"signal:{run_id}:{frame['step']}:{slot_idx}:{role}")
    g.add((sample, rdflib.RDF.type, MSRUN.SignalSample))
    g.add((sample, MSRUN.atFrame, frame_node))
    g.add((sample, MSRUN.signal, signal))
    g.add((sample, MSRUN.role, rdflib.Literal(role)))
    _literal(g, sample, rdflib.RDF.value, value)
    if controller_uri is not None:
        g.add((sample, MSRUN.controller, controller_uri))
    if constraint_uri is not None:
        g.add((sample, MSRUN.constraint, constraint_uri))
    if monitor_uri is not None:
        g.add((sample, MSRUN.monitor, monitor_uri))


def _project_frame_samples(
    g: rdflib.Graph,
    run_id: str,
    schema: dict,
    frame: dict,
    frame_node: rdflib.URIRef,
    *,
    include_slots: bool,
) -> None:
    states, _events = _state_maps(schema)
    state, meta = _state_meta(schema, states, frame.get("fsm_state", -1))
    if state and state.get("uri"):
        g.add((frame_node, MSRUN.activeState, rdflib.URIRef(state["uri"])))
    for key, value in frame.get("timing", {}).items():
        _literal(g, frame_node, MSRUN[key], value)
    if not include_slots:
        return

    controllers = meta.get("controllers") or meta.get("constraints") or []
    for idx, sample in enumerate(frame.get("constraints", [])):
        if not sample.get("active") or idx >= len(controllers):
            continue
        slot = controllers[idx]
        controller_uri = _slot_uri(slot, "uri", "controller_uri")
        constraint_uri = _slot_uri(slot, "constraint_uri")
        if controller_uri is None:
            continue
        node = _node(f"controller-sample:{run_id}:{frame['step']}:{idx}")
        g.add((node, rdflib.RDF.type, MSRUN.ControllerSample))
        g.add((node, MSRUN.atFrame, frame_node))
        g.add((node, MSRUN.controller, controller_uri))
        g.add((node, MSRUN.slotIndex, rdflib.Literal(idx)))
        g.add((node, MSRUN.active, rdflib.Literal(True)))
        if constraint_uri is not None:
            g.add((node, MSRUN.constraint, constraint_uri))
        for src, pred in (
            ("satisfied", MSRUN.satisfied),
            ("sat_t", MSRUN.satSince),
            ("error", MSRUN.error),
            ("output", MSRUN.output),
            ("measured", MSRUN.measured),
            ("setpoint", MSRUN.setpoint),
        ):
            value = bool(sample[src]) if src == "satisfied" and src in sample else sample.get(src)
            _literal(g, node, pred, value)
        _add_signal_sample(
            g,
            run_id,
            frame_node,
            frame,
            idx,
            "error",
            slot.get("error_signal_uri") or slot.get("error_uri"),
            sample.get("error"),
            controller_uri=controller_uri,
            constraint_uri=constraint_uri,
        )
        _add_signal_sample(
            g,
            run_id,
            frame_node,
            frame,
            idx,
            "output",
            slot.get("output_signal_uri") or slot.get("output_uri"),
            sample.get("output"),
            controller_uri=controller_uri,
            constraint_uri=constraint_uri,
        )

    monitors = meta.get("monitors") or []
    for idx, sample in enumerate(frame.get("monitors", [])):
        if not sample.get("active") or idx >= len(monitors):
            continue
        slot = monitors[idx]
        monitor_uri = _slot_uri(slot, "uri", "monitor_uri")
        if monitor_uri is None:
            continue
        node = _node(f"monitor-sample:{run_id}:{frame['step']}:{idx}")
        g.add((node, rdflib.RDF.type, MSRUN.MonitorSample))
        g.add((node, MSRUN.atFrame, frame_node))
        g.add((node, MSRUN.monitor, monitor_uri))
        g.add((node, MSRUN.slotIndex, rdflib.Literal(idx)))
        g.add((node, MSRUN.active, rdflib.Literal(True)))
        if slot.get("event_uri"):
            g.add((node, MSRUN.event, rdflib.URIRef(slot["event_uri"])))
        for src, pred in (
            ("satisfied", MSRUN.satisfied),
            ("sat_t", MSRUN.satSince),
            ("value", MSRUN.value),
        ):
            value = bool(sample[src]) if src == "satisfied" and src in sample else sample.get(src)
            _literal(g, node, pred, value)
        _add_signal_sample(
            g,
            run_id,
            frame_node,
            frame,
            idx,
            "value",
            slot.get("error_signal_uri") or slot.get("error_uri"),
            sample.get("value"),
            monitor_uri=monitor_uri,
        )


def _project_trigger_occurrences(g: rdflib.Graph, run_id: str, schema: dict, frames: list[dict]) -> None:
    states, events = _state_maps(schema)
    seen = set()
    for frame in frames:
        frame_node = _node(f"frame:{run_id}:{frame['step']}")
        for trigger in frame.get("triggers", []):
            key = (
                trigger.get("kind"),
                trigger.get("idx"),
                trigger.get("fsm_state"),
                trigger.get("t"),
                trigger.get("wall_ns"),
            )
            if key in seen:
                continue
            seen.add(key)
            kind = trigger.get("kind")
            idx = trigger.get("idx")
            if kind == KIND_EVENT and events.get(idx, {}).get("id") == "E_STEP":
                continue
            typename = TRIGGER_TYPES.get(kind, "Occurrence")
            occ = _node(f"occurrence:{run_id}:{typename}:{trigger.get('wall_ns', 0)}:{idx}")
            g.add((occ, rdflib.RDF.type, MSRUN[typename]))
            g.add((occ, MSRUN.atFrame, frame_node))
            _literal(g, occ, MSRUN.t, trigger.get("t"))
            _literal(g, occ, MSRUN.wall_ns, trigger.get("wall_ns"))
            _literal(g, occ, MSRUN.slotIndex, idx)
            state, meta = _state_meta(schema, states, trigger.get("fsm_state", -1))
            if state and state.get("uri"):
                g.add((occ, MSRUN.fsmState, rdflib.URIRef(state["uri"])))
            if kind == KIND_STATE and idx in states and states[idx].get("uri"):
                g.add((occ, MSRUN.state, rdflib.URIRef(states[idx]["uri"])))
            elif kind == KIND_EVENT and idx in events and events[idx].get("uri"):
                g.add((occ, MSRUN.event, rdflib.URIRef(events[idx]["uri"])))
            elif kind in (KIND_CONSTRAINT_SAT, KIND_CONSTRAINT_UNSAT):
                controllers = meta.get("controllers") or meta.get("constraints") or []
                if isinstance(idx, int) and 0 <= idx < len(controllers):
                    controller_uri = _slot_uri(controllers[idx], "uri", "controller_uri")
                    constraint_uri = _slot_uri(controllers[idx], "constraint_uri")
                    if controller_uri is not None:
                        g.add((occ, MSRUN.controller, controller_uri))
                    if constraint_uri is not None:
                        g.add((occ, MSRUN.constraint, constraint_uri))
            elif kind == KIND_MONITOR:
                monitors = meta.get("monitors") or []
                if isinstance(idx, int) and 0 <= idx < len(monitors):
                    monitor_uri = _slot_uri(monitors[idx], "uri", "monitor_uri")
                    if monitor_uri is not None:
                        g.add((occ, MSRUN.monitor, monitor_uri))
                    if monitors[idx].get("event_uri"):
                        g.add((occ, MSRUN.event, rdflib.URIRef(monitors[idx]["event_uri"])))


def _sample_steps(frames: list[dict], schema: dict) -> set:
    # ponytail: keep dense numeric curves in frame_log.bin; add graph samples at semantic changes.
    if not frames:
        return set()
    _states, events = _state_maps(schema)
    steps = {frames[0].get("step"), frames[-1].get("step")}
    last_state = object()
    last_constraints = None
    last_monitors = None
    seen_triggers = set()
    for frame in frames:
        step = frame.get("step")
        state = frame.get("fsm_state")
        constraints = tuple((bool(c.get("active")), bool(c.get("satisfied"))) for c in frame.get("constraints", []))
        monitors = tuple((bool(m.get("active")), bool(m.get("satisfied"))) for m in frame.get("monitors", []))
        if state != last_state or constraints != last_constraints or monitors != last_monitors:
            steps.add(step)
        last_state = state
        last_constraints = constraints
        last_monitors = monitors
        for trigger in frame.get("triggers", []):
            if trigger.get("kind") == KIND_EVENT and events.get(trigger.get("idx"), {}).get("id") == "E_STEP":
                continue
            key = (
                trigger.get("kind"),
                trigger.get("idx"),
                trigger.get("fsm_state"),
                trigger.get("t"),
                trigger.get("wall_ns"),
            )
            if key not in seen_triggers:
                seen_triggers.add(key)
                steps.add(step)
    return steps


def _add_rec_timing(g: rdflib.Graph, run_dir: Path, manifest: dict, activity: rdflib.URIRef) -> None:
    rec_rel = manifest.get("files", {}).get("rec", "rec.jsonld")
    rec_path = run_dir / rec_rel
    if not rec_path.exists():
        return
    rec = json.loads(rec_path.read_text())
    for key, pred in (("startedAtTime", PROV.startedAtTime), ("endedAtTime", PROV.endedAtTime)):
        if rec.get(key):
            g.add((activity, pred, rdflib.Literal(rec[key], datatype=rdflib.XSD.dateTime)))


def project_runtime(run_dir: Path | str, frames: list[dict], *, frame_count: int | None = None) -> rdflib.Graph:
    run_dir, manifest = load_manifest(run_dir)
    schema = json.loads((run_dir / manifest["files"]["schema"]).read_text())
    g = rdflib.Graph()
    for prefix, ns in {
        "prov": PROV,
        "bdd": BDD,
        "agn": AGN,
        "obs": OBS,
        "rt": RT,
        "msrun": MSRUN,
        "msprov": rdflib.Namespace(MSPROV),
    }.items():
        g.bind(prefix, ns)

    run = _node(f"run:{manifest['run_id']}")
    # Agents and the execution activity are shared provenance concepts: emit the same
    # canonical msprov IRIs the codegen graph uses so the runtime, codegen and rec graphs
    # join on one node per concept (rather than three parallel ones).
    rp = schema.get("runtime_provenance", {})
    activity = rdflib.URIRef(prov_uri(rp.get("activity_id", "activity:controller_execution")))
    producer = rdflib.URIRef(prov_uri(rp.get("producer_agent_id", "agent:controller_process")))
    runtime = rdflib.URIRef(prov_uri(rp.get("runtime_agent_id", "agent:runtime")))
    frame_log = _node("entity:frame_log")
    schema_entity = _node("entity:schema_json")
    layout_entity = _node("entity:frame_layout_json")
    model_entity = _node("entity:model_jsonld")
    provenance_entity = _node("entity:provenance_jsonld")

    g.add((run, rdflib.RDF.type, PROV.Entity))
    g.add((run, rdflib.RDF.type, RT.Runtime))
    g.add((activity, rdflib.RDF.type, PROV.Activity))
    g.add((activity, rdflib.RDF.type, BDD.SimulatedExecution))
    g.add((activity, PROV.wasAssociatedWith, producer))
    g.add((activity, PROV.used, schema_entity))
    g.add((activity, PROV.used, layout_entity))
    g.add((activity, PROV.used, model_entity))
    g.add((activity, PROV.used, provenance_entity))
    g.add((producer, rdflib.RDF.type, PROV.SoftwareAgent))
    g.add((producer, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((producer, PROV.actedOnBehalfOf, runtime))
    g.add((runtime, rdflib.RDF.type, PROV.SoftwareAgent))
    if str(schema.get("runtime_provenance", {}).get("runtime_agent_id", "")).endswith(":mujoco"):
        g.add((runtime, rdflib.RDF.type, RT.MuJoCoRuntime))
    g.add((frame_log, rdflib.RDF.type, PROV.Entity))
    g.add((frame_log, PROV.wasGeneratedBy, activity))
    g.add(
        (
            frame_log,
            PROV.atLocation,
            _archive_location(manifest["files"]["frame_log"]),
        )
    )
    for entity, key in (
        (schema_entity, "schema"),
        (layout_entity, "frame_layout"),
        (model_entity, "model"),
        (provenance_entity, "provenance"),
    ):
        rel = manifest.get("files", {}).get(key)
        if rel and (run_dir / rel).exists():
            g.add((entity, rdflib.RDF.type, PROV.Entity))
            g.add((entity, PROV.atLocation, _archive_location(rel)))
    g.add((run, MSRUN.contractVersion, rdflib.Literal(RUNTIME_RDF_CONTRACT_VERSION)))
    g.add((run, MSRUN.runId, rdflib.Literal(manifest["run_id"])))
    g.add((run, MSRUN.frameCount, rdflib.Literal(len(frames) if frame_count is None else frame_count)))
    g.add((run, PROV.wasGeneratedBy, activity))
    _add_rec_timing(g, run_dir, manifest, activity)

    sample_steps = _sample_steps(frames, schema)
    for frame in frames:
        frame_node = _node(f"frame:{manifest['run_id']}:{frame['step']}")
        g.add((frame_node, rdflib.RDF.type, MSRUN.Frame))
        g.add((frame_node, MSRUN.step, rdflib.Literal(frame["step"])))
        g.add((frame_node, MSRUN.t, rdflib.Literal(frame["t"])))
        _project_frame_samples(
            g,
            manifest["run_id"],
            schema,
            frame,
            frame_node,
            include_slots=frame.get("step") in sample_steps,
        )
        g.add((run, MSRUN.frame, frame_node))
    _project_trigger_occurrences(g, manifest["run_id"], schema, frames)
    return g


def write_runtime_ttl(run_dir: Path | str, frames: list[dict], *, frame_count: int | None = None) -> Path:
    run_dir = Path(run_dir)
    graph = project_runtime(run_dir, frames, frame_count=frame_count)
    manifest_path = run_dir / "manifest.json"
    runtime_rel = "runtime/runtime.ttl"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        runtime_rel = manifest.get("files", {}).get("runtime_ttl") or runtime_rel
    out = run_dir / runtime_rel
    out.parent.mkdir(parents=True, exist_ok=True)
    graph.serialize(out, format="turtle")
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest.setdefault("artifacts", {})[runtime_rel] = {
            "role": "runtime",
            "sha256": sha256_file(out),
        }
        manifest.setdefault("files", {})["runtime_ttl"] = runtime_rel
        _record_runtime_ttl_with_rec(run_dir, manifest, out)
        rec_path = run_dir / manifest.get("files", {}).get("rec", "rec.jsonld")
        if rec_path.exists():
            manifest.setdefault("artifacts", {})["rec.jsonld"] = {
                "role": "rec",
                "sha256": sha256_file(rec_path),
            }
        manifest_path.write_text(json.dumps(manifest, indent=4) + "\n")
    return out


def _record_runtime_ttl_with_rec(run_dir: Path, manifest: dict, runtime_ttl: Path) -> None:
    try:
        from motion_spec.introspection.archive import _ensure_local_rec_importable

        _ensure_local_rec_importable()
        from rec import Run
        from rec.observers import FileObserver
    except ImportError as exc:
        raise RuntimeError("REC is required to update runtime.ttl provenance") from exc

    rec_path = run_dir / manifest.get("files", {}).get("rec", "rec.jsonld")
    if not rec_path.exists():
        return
    observer = FileObserver(rec_path, run_id=manifest.get("run_id"))
    run = Run(observers=[observer], run_id=manifest.get("run_id"))
    run.add_agent(
        prov_uri("agent:replay_process"),
        ["prov:SoftwareAgent", "obs:ObservationProvider"],
        role="runtime_ttl_recovery",
    )
    run.add_activity(
        prov_uri("activity:runtime_ttl_recovery"),
        ["prov:Activity"],
        role="runtime_ttl_recovery",
        wasAssociatedWith=prov_uri("agent:replay_process"),
    )
    run.add_artefact(
        str(runtime_ttl.resolve()),
        gen_activity=prov_uri("activity:runtime_ttl_recovery"),
        archivePath=str(runtime_ttl.relative_to(run_dir)),
        role="runtime_ttl",
        sha256=sha256_file(runtime_ttl),
        size_bytes=runtime_ttl.stat().st_size,
    )
    observer.close()
