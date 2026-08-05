# SPDX-License-Identifier: MPL-2.0
"""Recover runtime RDF from archive-local run data."""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import load_manifest, sha256_file
from motion_spec_dsl.rdf_parser.vocab import CSTR_HDL
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.provenance import MSPROV, prov_uri, rec_run_lifecycle, rec_types


def _dt_literal(wall_ns) -> rdflib.Literal | None:
    """Absolute wall time (epoch nanoseconds) as an xsd:dateTime (rdflib canonicalizes to +00:00,
    matching motion-spec.ld.json / rec.ld.json / provenance.ld.json)."""
    if wall_ns is None:
        return None
    sec, ns = divmod(int(wall_ns), 1_000_000_000)
    dt = datetime.fromtimestamp(sec, timezone.utc).replace(microsecond=ns // 1000)
    return rdflib.Literal(dt)


def _condition_map(run_dir: Path, manifest: dict) -> dict[str, rdflib.URIRef]:
    """Map monitor IRI -> its constraint-condition IRI, read from the co-archived model graph
    (the compiled-from-.robmot jsonld). The condition node already carries the operator (@type),
    measured quantity, and setpoint/threshold, so occurrences reference it by URI rather than
    copying those values in — the model graph stays the single source for the spec."""
    mapping: dict[str, rdflib.URIRef] = {}
    for rel in manifest.get("files", {}).get("model_imports") or []:
        path = run_dir / rel
        if not path.exists() or path.suffix != ".json":
            continue
        try:
            mg = rdflib.Graph().parse(path, format="json-ld")
        except Exception:
            continue
        for s, p, o in mg:
            if isinstance(o, rdflib.URIRef) and p == CSTR_HDL["constraint"]:
                mapping[str(s)] = o
    return mapping


PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
BDD = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
AGN = rdflib.Namespace("https://secorolab.github.io/metamodels/agent#")
OBS = rdflib.Namespace("https://secorolab.github.io/metamodels/observation#")
EXEC = rdflib.Namespace("https://secorolab.github.io/metamodels/execution-context#")
MSRUN = rdflib.Namespace("https://secorolab.github.io/motion-spec/runtime/")
RUNTIME_RDF_CONTRACT_VERSION = 1
# Events are the only trigger kind the runtime actually emits; state/constraint/monitor edges
# are synthesized here from the per-tick frame scan (see _project_occurrences).
KIND_EVENT = 1


def _node(identifier: str) -> rdflib.URIRef:
    safe = identifier.replace(":", "/")
    return MSRUN[safe]


def _scoped(family: str, run_id: str, *parts) -> rdflib.URIRef:
    """Per-run sample/occurrence node under `<family>/<run_id>/`, with a slash-free local
    (parts joined by '-') so it collapses to a CURIE against the bound family prefix."""
    local = "-".join(str(p) for p in parts)
    return MSRUN[f"{family}/{run_id}/{local}"]


def _literal(g: rdflib.Graph, subject: rdflib.URIRef, predicate: rdflib.URIRef, value) -> None:
    if value is None:
        return
    # Floats are float64 (double) throughout the pipeline (cpp/bin/replay). Preserve the exact
    # value: Decimal(repr(x)) round-trips to the same double and serializes as a plain decimal.
    # (rdflib's Turtle writer emits xsd:double in scientific notation AND truncates it to ~7
    # sig figs, which would silently lose precision — so xsd:double is not usable here.)
    if isinstance(value, float):
        if math.isfinite(value):
            value = Decimal(repr(value))
        # non-finite inf/nan fall through as a Python float -> xsd:double, which represents them
    g.add((subject, predicate, rdflib.Literal(value)))


def _named(rows) -> dict[int, dict]:
    """Index -> {id, uri} for a header table (FSM states or events)."""
    return {row.number: {"id": row.id, "uri": row.iri or None} for row in rows}


def _state_maps(header) -> tuple[dict[int, dict], dict[int, dict]]:
    return _named(header.fsm_states), _named(header.fsm_events)


def _slot_rows(rows) -> list[dict]:
    """Header slot messages as the dicts the occurrence builders read."""
    return [
        {
            "index": row.number,
            "id": row.id,
            "uri": row.iri or None,
            "constraint_uri": row.constraint_iri or None,
            "event_uri": row.event_iri or None,
        }
        for row in rows
    ]


def _motion_meta(header) -> dict[int, dict]:
    """Motion index -> its slot metadata. The frame's active_motion selects it, so the same
    resolution works whatever coordinator drove the run."""
    return {
        motion.index: {
            "index": motion.index,
            "id": motion.id,
            "uri": motion.iri or None,
            "controllers": _slot_rows(motion.controllers),
            "monitors": _slot_rows(motion.monitors),
        }
        for motion in header.motions
    }


def _transition_rows(header) -> list[dict]:
    """Header transition messages as dicts; -1 endpoints mean 'not declared'."""
    return [
        {
            "id": row.id,
            "uri": row.iri or None,
            "from": row.from_state if row.from_state >= 0 else None,
            "to": row.to_state if row.to_state >= 0 else None,
            "event_index": row.event_index if row.event_index >= 0 else None,
            "event_indices": list(row.event_indices),
        }
        for row in header.fsm_transitions
    ]


def _state_meta(states: dict[int, dict], state_idx: int) -> dict | None:
    """The FSM state a frame was in -- coordinator context for the occurrence, not slot identity."""
    return states.get(state_idx)


def _slot_uri(slot: dict, *keys: str) -> rdflib.URIRef | None:
    for key in keys:
        value = slot.get(key)
        if value:
            return rdflib.URIRef(value)
    return None


def _occurrence(g: rdflib.Graph, run_id: str, typename: str, disc, wall_ns, step: int) -> rdflib.URIRef:
    """Create a discrete occurrence node, anchored to its frame and stamped with wall time."""
    node = _scoped("occurrence", run_id, typename, wall_ns if wall_ns is not None else 0, disc)
    g.add((node, rdflib.RDF.type, MSRUN[typename]))
    g.add((node, MSRUN.atFrame, _node(f"frame:{run_id}:{step}")))
    dt = _dt_literal(wall_ns)
    if dt is not None:
        g.add((node, PROV.generatedAtTime, dt))
    return node


def _fired_transition(candidates: list, observed: set):
    """Which of the transitions between one pair of states actually fired.

    Transitions are identified by their own id, so two transitions between the same pair of states
    stay distinct. When a pair has several, the events observed around the state change decide
    which one it was; if that does not single one out, the caller records the state change without
    naming a transition rather than guessing.
    """
    if len(candidates) == 1:
        return candidates[0]
    matched = [
        transition
        for transition in candidates
        if observed & set(transition.get("event_indices") or [])
    ]
    return matched[0] if len(matched) == 1 else None


def _state_change_occurrences(
    g: rdflib.Graph,
    run_id: str,
    states: dict[int, dict],
    events: dict[int, dict],
    transitions: dict,
    prev_state,
    cur,
    state,
    entry,
    step,
    observed_events: set | None = None,
) -> set:
    anchors = set()
    if state and state.get("uri"):
        occ = _occurrence(g, run_id, "StateOccurrence", cur, entry, step)
        g.add((occ, MSRUN.state, rdflib.URIRef(state["uri"])))
        anchors.add(step)
    if prev_state is None:
        return anchors
    tr = _fired_transition(transitions.get((prev_state, cur)) or [], observed_events or set())
    if not tr or not tr.get("uri"):
        return anchors
    occ = _occurrence(g, run_id, "TransitionOccurrence", tr.get("id", f"{prev_state}-{cur}"), entry, step)
    g.add((occ, MSRUN.transition, rdflib.URIRef(tr["uri"])))
    frm_state = states.get(prev_state) or {}
    if frm_state.get("uri"):
        g.add((occ, MSRUN.fromState, rdflib.URIRef(frm_state["uri"])))
    if state and state.get("uri"):
        g.add((occ, MSRUN.toState, rdflib.URIRef(state["uri"])))
    # Prefer the event actually seen; fall back to the declared one when the transition has only
    # one, which is how a transition driven by an unlogged event (the heartbeat) stays attributed.
    fired = observed_events & set(tr.get("event_indices") or []) if observed_events else set()
    ev = events.get(next(iter(fired), None) if fired else tr.get("event_index")) or {}
    if ev.get("uri"):
        g.add((occ, MSRUN.event, rdflib.URIRef(ev["uri"])))
    anchors.add(step)
    return anchors


def _constraint_edge_occurrences(
    g: rdflib.Graph,
    run_id: str,
    state,
    controllers: list,
    prev_csat: list | None,
    csat: list,
    frame: dict,
    wall,
    step: int,
) -> set:
    anchors = set()
    if prev_csat is None:
        return anchors
    for idx, now in enumerate(csat):
        if idx >= len(prev_csat) or idx >= len(controllers) or now == prev_csat[idx]:
            continue
        constraint_uri = _slot_uri(controllers[idx], "constraint_uri")
        if constraint_uri is None:  # only goal constraints, not pure regulation
            continue
        typename = "ConstraintSatisfiedOccurrence" if now else "ConstraintUnsatisfiedOccurrence"
        occ = _occurrence(g, run_id, typename, idx, wall, step)
        controller_uri = _slot_uri(controllers[idx], "uri", "controller_uri")
        if controller_uri is not None:
            g.add((occ, MSRUN.controller, controller_uri))
        g.add((occ, MSRUN.constraint, constraint_uri))
        if state and state.get("uri"):
            g.add((occ, MSRUN.fsmState, rdflib.URIRef(state["uri"])))
        _literal(g, occ, MSRUN.slotIndex, idx)
        value = frame["constraints"][idx].get("error")
        _literal(g, occ, MSRUN.value, None if value is None else float(value))
        anchors.add(step)
    return anchors


def _monitor_edge_occurrences(
    g: rdflib.Graph,
    run_id: str,
    state,
    monitors: list,
    prev_msat: list | None,
    msat: list,
    frame: dict,
    cond_map: dict,
    wall,
    step: int,
) -> set:
    anchors = set()
    if prev_msat is None:
        return anchors
    for idx, now in enumerate(msat):
        if idx >= len(prev_msat) or idx >= len(monitors) or not (now and not prev_msat[idx]):
            continue
        monitor_uri = _slot_uri(monitors[idx], "uri", "monitor_uri")
        if monitor_uri is None:
            continue
        occ = _occurrence(g, run_id, "MonitorOccurrence", idx, wall, step)
        g.add((occ, MSRUN.monitor, monitor_uri))
        condition = cond_map.get(str(monitor_uri))
        if condition is not None:
            g.add((occ, MSRUN.constraint, condition))
        if monitors[idx].get("event_uri"):
            g.add((occ, MSRUN.event, rdflib.URIRef(monitors[idx]["event_uri"])))
        if state and state.get("uri"):
            g.add((occ, MSRUN.fsmState, rdflib.URIRef(state["uri"])))
        _literal(g, occ, MSRUN.slotIndex, idx)
        value = frame["monitors"][idx].get("value")
        _literal(g, occ, MSRUN.value, None if value is None else float(value))
        anchors.add(step)
    return anchors


def _trigger_occurrences(
    g: rdflib.Graph,
    run_id: str,
    states: dict[int, dict],
    events: dict[int, dict],
    frame: dict,
    seen_events: set,
    step: int,
) -> set:
    anchors = set()
    for trigger in frame.get("triggers", []):
        if trigger.get("kind") != KIND_EVENT:
            continue
        eidx = trigger.get("idx")
        event = events.get(eidx) or {}
        ekey = (eidx, trigger.get("wall_ns"))
        if ekey in seen_events:
            continue
        seen_events.add(ekey)
        occ = _occurrence(g, run_id, "EventOccurrence", eidx, trigger.get("wall_ns"), step)
        if event.get("uri"):
            g.add((occ, MSRUN.event, rdflib.URIRef(event["uri"])))
        tstate = states.get(trigger.get("fsm_state", -1))
        if tstate and tstate.get("uri"):
            g.add((occ, MSRUN.fsmState, rdflib.URIRef(tstate["uri"])))
        _literal(g, occ, MSRUN.slotIndex, eidx)
        anchors.add(step)
    return anchors


def _project_occurrences(
    g: rdflib.Graph, run_id: str, header, frames: list[dict], cond_map: dict
) -> set:
    """Synthesize the discrete event graph from the per-tick frame scan; return the set of steps
    that carry an occurrence (the frames worth materializing). Continuous scalars stay in
    the frame log; only semantic edges land in the graph:

      * StateOccurrence / TransitionOccurrence on FSM state changes,
      * ConstraintSatisfied/UnsatisfiedOccurrence on a goal constraint's satisfied edge (both
        directions - a falling edge is a goal lost, e.g. what fires E_GRASP_LOST_*),
      * MonitorOccurrence on a monitor's rising (fired) edge,
      * EventOccurrence from the runtime's event triggers. Nothing needs filtering here: the
        heartbeat that drives the FSM is deliberately not recorded, because the frame log is a
        time series and every frame already is the tick.

    Edge detection resets at state boundaries: slot indices are motion-local (slot i is a
    different controller under a different motion), so only intra-state comparison is valid.
    """
    states, events = _state_maps(header)
    motions = _motion_meta(header)
    # Indexed by state pair but holding every transition between that pair, so two transitions
    # between the same states driven by different events stay distinct.
    transitions: dict = {}
    for transition in _transition_rows(header):
        transitions.setdefault((transition.get("from"), transition.get("to")), []).append(
            transition
        )
    anchors: set = set()
    prev_state = None
    prev_csat: list | None = None
    prev_msat: list | None = None
    seen_events: set = set()
    # An event fires on one tick and the state change lands on the next, so the events that could
    # have caused a change span this frame and the previous one.
    prev_frame_events: set = set()
    for frame in frames:
        step = frame["step"]
        wall = frame.get("timing", {}).get("wall_ns")
        cur = frame.get("fsm_state", -1)
        frame_events = {
            trigger.get("idx")
            for trigger in frame.get("triggers", [])
            if trigger.get("kind") == KIND_EVENT
        }
        state = _state_meta(states, cur)
        meta = motions.get(frame.get("active_motion", -1), {})
        controllers = meta.get("controllers") or meta.get("constraints") or []
        monitors = meta.get("monitors") or []
        csat = [bool(c.get("active")) and bool(c.get("satisfied")) for c in frame.get("constraints", [])]
        msat = [bool(m.get("active")) and bool(m.get("satisfied")) for m in frame.get("monitors", [])]

        if cur != prev_state:
            anchors.update(
                _state_change_occurrences(
                    g, run_id, states, events, transitions, prev_state, cur, state,
                    frame.get("state_since_wall_ns") or wall, step,
                    observed_events=frame_events | prev_frame_events,
                )
            )
        else:
            anchors.update(_constraint_edge_occurrences(
                g, run_id, state, controllers, prev_csat, csat, frame, wall, step
            ))
            anchors.update(_monitor_edge_occurrences(
                g, run_id, state, monitors, prev_msat, msat, frame, cond_map, wall, step
            ))

        anchors.update(_trigger_occurrences(g, run_id, states, events, frame, seen_events, step))

        prev_state, prev_csat, prev_msat = cur, csat, msat
        prev_frame_events = frame_events
    return anchors


def _add_rec_timing(g: rdflib.Graph, run_dir: Path, manifest: dict, activity: rdflib.URIRef) -> None:
    rec_rel = manifest.get("files", {}).get("rec", "rec.ld.json")
    rec_path = run_dir / rec_rel
    if not rec_path.exists():
        return
    lifecycle = rec_run_lifecycle(rdflib.Graph().parse(rec_path, format="json-ld"))
    for key, pred in (("started_time", PROV.startedAtTime), ("completed_time", PROV.endedAtTime)):
        if lifecycle.get(key):
            g.add((activity, pred, rdflib.Literal(lifecycle[key], datatype=rdflib.XSD.dateTime)))


def project_runtime(run_dir: Path | str, frames: list[dict], *, frame_count: int | None = None) -> rdflib.Graph:
    run_dir, manifest = load_manifest(run_dir)
    # The run's contract comes from the log itself, not a companion artifact.
    header = frame_log_pb.read_contract(run_dir / manifest["files"]["frame_log"]).header
    g = rdflib.Graph()
    for prefix, ns in {
        "prov": PROV,
        "bdd": BDD,
        "agn": AGN,
        "obs": OBS,
        "exec": EXEC,
        "msrun": MSRUN,
        "ent": rdflib.Namespace(f"{MSRUN}entity/"),
        "run": rdflib.Namespace(f"{MSRUN}run/"),
        "mspact": rdflib.Namespace(f"{MSPROV}activity/"),
        "mspagent": rdflib.Namespace(f"{MSPROV}agent/"),
    }.items():
        g.bind(prefix, ns)
    # Compact the per-run node families and model FSM nodes into CURIEs by binding a prefix at
    # each family's `<family>/<run_id>/` boundary (locals are slash-free — see _scoped).
    run_id = manifest["run_id"]
    for prefix, family in (
        ("frm", "frame"),
        ("cs", "controller-sample"),
        ("mons", "monitor-sample"),
        ("sig", "signal"),
        ("occ", "occurrence"),
    ):
        g.bind(prefix, rdflib.Namespace(f"{MSRUN}{family}/{run_id}/"))
    fsm_namespace = header.fsm_namespace
    if fsm_namespace:
        g.bind("mfsm", rdflib.Namespace(fsm_namespace))

    run = _node(f"run:{manifest['run_id']}")
    # Agents and the execution activity are shared provenance concepts: emit the same
    # canonical msprov IRIs the codegen graph uses so the runtime, codegen and rec graphs
    # join on one node per concept (rather than three parallel ones).
    activity = rdflib.URIRef(prov_uri(header.activity_id or "activity:controller_execution"))
    producer = rdflib.URIRef(prov_uri(header.producer_agent_id or "agent:controller_process"))
    runtime = rdflib.URIRef(prov_uri(header.runtime_agent_id or "agent:runtime"))
    frame_log = _node("entity:frame_log")
    proto_entity = _node("entity:frame_log_proto")
    model_entity = _node("entity:model_jsonld")
    provenance_entity = _node("entity:provenance_jsonld")

    g.add((run, rdflib.RDF.type, PROV.Entity))
    g.add((run, rdflib.RDF.type, EXEC.ExecutionContext))
    g.add((activity, rdflib.RDF.type, PROV.Activity))
    # Simulated vs real is the model's declaration, not an assumption and not a substring match.
    g.add(
        (
            activity,
            rdflib.RDF.type,
            BDD.SimulatedExecution if header.simulated else BDD.ScenarioExecution,
        )
    )
    g.add((activity, PROV.wasAssociatedWith, producer))
    g.add((activity, PROV.used, proto_entity))
    g.add((activity, PROV.used, model_entity))
    g.add((activity, PROV.used, provenance_entity))
    g.add((producer, rdflib.RDF.type, PROV.SoftwareAgent))
    g.add((producer, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((producer, PROV.actedOnBehalfOf, runtime))
    g.add((runtime, rdflib.RDF.type, PROV.SoftwareAgent))
    if header.simulated:
        g.add((runtime, rdflib.RDF.type, EXEC.Simulation))
    g.add((frame_log, rdflib.RDF.type, PROV.Entity))
    g.add((frame_log, PROV.wasGeneratedBy, activity))
    g.add(
        (
            frame_log,
            PROV.atLocation,
            rdflib.URIRef(os.path.relpath(manifest["files"]["frame_log"], "runtime")),
        )
    )
    for entity, key in (
        (proto_entity, "frame_log_proto"),
        (model_entity, "model"),
        (provenance_entity, "provenance"),
    ):
        rel = manifest.get("files", {}).get(key)
        if rel and (run_dir / rel).exists():
            g.add((entity, rdflib.RDF.type, PROV.Entity))
            g.add((entity, PROV.atLocation, rdflib.URIRef(os.path.relpath(rel, "runtime"))))
    g.add((run, MSRUN.contractVersion, rdflib.Literal(RUNTIME_RDF_CONTRACT_VERSION)))
    g.add((run, MSRUN.runId, rdflib.Literal(manifest["run_id"])))
    g.add((run, MSRUN.frameCount, rdflib.Literal(len(frames) if frame_count is None else frame_count)))
    g.add((run, PROV.wasGeneratedBy, activity))
    _add_rec_timing(g, run_dir, manifest, activity)

    # Provenance of this runtime.ttl document itself: recovered from the frame log by the
    # introspection recovery agent (same IRIs the rec graph uses), stamped at generation time.
    runtime_doc = _node("entity:runtime_ttl")
    recovery_activity = rdflib.URIRef(prov_uri("activity:runtime_ttl_recovery"))
    recovery_agent = rdflib.URIRef(prov_uri("agent:replay_process"))
    generated_at = _dt_literal(time.time_ns())
    g.add((runtime_doc, rdflib.RDF.type, PROV.Entity))
    g.add((runtime_doc, PROV.atLocation, rdflib.URIRef("runtime.ttl")))
    g.add((runtime_doc, PROV.wasGeneratedBy, recovery_activity))
    g.add((runtime_doc, PROV.wasDerivedFrom, frame_log))
    g.add((runtime_doc, PROV.generatedAtTime, generated_at))
    g.add((recovery_activity, rdflib.RDF.type, PROV.Activity))
    g.add((recovery_activity, PROV.used, frame_log))
    g.add((recovery_activity, PROV.wasAssociatedWith, recovery_agent))
    g.add((recovery_activity, PROV.endedAtTime, generated_at))
    g.add((recovery_agent, rdflib.RDF.type, PROV.SoftwareAgent))
    g.add((recovery_agent, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((recovery_agent, rdflib.RDFS.label, rdflib.Literal("motion_spec runtime.ttl recovery")))

    # Synthesize the discrete event graph, then materialize a Frame node only for the steps that
    # actually anchor an occurrence (plus the run's first/last for bounds). The dense per-tick
    # curve — every frame, all continuous scalars — stays in the frame log for numeric analysis.
    if frames:
        states, _events = _state_maps(header)
        cond_map = _condition_map(run_dir, manifest)
        anchors = _project_occurrences(g, manifest["run_id"], header, frames, cond_map)
        emit_steps = anchors | {frames[0]["step"], frames[-1]["step"]}
        for frame in frames:
            if frame["step"] not in emit_steps:
                continue
            frame_node = _node(f"frame:{manifest['run_id']}:{frame['step']}")
            g.add((frame_node, rdflib.RDF.type, MSRUN.Frame))
            g.add((frame_node, MSRUN.step, rdflib.Literal(frame["step"])))
            frame_dt = _dt_literal(frame.get("timing", {}).get("wall_ns"))
            if frame_dt is not None:
                g.add((frame_node, PROV.generatedAtTime, frame_dt))
            state = _state_meta(states, frame.get("fsm_state", -1))
            if state and state.get("uri"):
                g.add((frame_node, MSRUN.activeState, rdflib.URIRef(state["uri"])))
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
        manifest.setdefault("files", {})["runtime_ttl"] = runtime_rel
        _record_runtime_ttl_with_rec(run_dir, manifest, out)
        manifest_path.write_text(json.dumps(manifest, indent=4) + "\n")
    return out


def _record_runtime_ttl_with_rec(run_dir: Path, manifest: dict, runtime_ttl: Path) -> None:
    try:
        from motion_spec.introspection.provenance import ensure_local_rec_importable

        ensure_local_rec_importable()
        from rec import Run
        from rec.observers import FileObserver
    except ImportError as exc:
        raise RuntimeError("REC is required to update runtime.ttl provenance") from exc

    rec_path = run_dir / manifest.get("files", {}).get("rec", "rec.ld.json")
    if not rec_path.exists():
        return
    observer = FileObserver(rec_path)
    run = Run(observers=[observer], run_id=manifest.get("run_id"))
    run.add_agent(
        prov_uri("agent:replay_process"),
        rec_types(["prov:SoftwareAgent", "obs:ObservationProvider"]),
    )
    run.add_activity(
        prov_uri("activity:runtime_ttl_recovery"),
        rec_types(["prov:Activity"]),
        associated_with=prov_uri("agent:replay_process"),
    )
    run.add_artefact(
        str(runtime_ttl.resolve()),
        gen_activity=prov_uri("activity:runtime_ttl_recovery"),
        archive_path=str(runtime_ttl.relative_to(run_dir)),
        title="runtime_ttl",
        sha256=sha256_file(runtime_ttl),
        size_bytes=runtime_ttl.stat().st_size,
    )
    observer.close()
