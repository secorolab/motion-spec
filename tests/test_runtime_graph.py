# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from decimal import Decimal
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import create_archive_manifest
from motion_spec.introspection.provenance import prov_uri
from motion_spec.generation.artifacts import field_names_and_format
from motion_spec.introspection.replay import runtime_frames
from motion_spec.introspection.runtime_graph import (
    DCTERMS,
    MEMBER_EDGE_LIMIT,
    MSRUN,
    PROV,
    QKIND,
    QUDT,
    SENS,
    SOSA,
    TIME,
    TRACE,
    UNIT,
    write_runtime_ttl,
)

S_START = rdflib.URIRef("https://example.test/S_START")
S_MOVE = rdflib.URIRef("https://example.test/S_MOVE")
T_START_MOVE = rdflib.URIRef("https://example.test/T_START_MOVE")
E_DONE = rdflib.URIRef("https://example.test/E_DONE")
CONSTRAINT_X = rdflib.URIRef("https://example.test/constraint_x")
DONE_MON = rdflib.URIRef("https://example.test/done_mon")

# The whole ms-exec-trace vocabulary. Everything else composes from prov/time/sosa/dcterms/sens.
TRACE_TERMS = {
    "atFrame",
    "seq",
    "step",
    "slotIndex",
    "rearmCount",
    "Frame",
    "ActivityOccurrence",
    "ControlFlowOccurrence",
}

from frame_log_fixture import write_frame_log_pb, write_frame_log_proto
from support import _hash_doc, _layout, _provenance


def _schema() -> dict:
    schema = {
        "schema_version": 1,
        "frame_layout_version": 1,
        "runtime_rdf_contract_version": 1,
        "generated_by": "test",
        "ir_path": "ir.json",
        "graph": "model.ld.json",
        "context": {},
        "pools": {"constraints": 1, "monitors": 1, "quantities": 0, "triggers": 3},
        "control_period_ns": 1_000_000,
        "timing": {"nominal_period_ns": 1_000_000},
        "fsm": {
            "states": [
                {"index": 0, "id": "S_START", "uri": "https://example.test/S_START"},
                {"index": 1, "id": "S_MOVE", "uri": "https://example.test/S_MOVE"},
            ],
            "events": [{"index": 1, "id": "E_DONE", "uri": "https://example.test/E_DONE"}],
            "transitions": [
                {
                    "id": "T_START_MOVE",
                    "uri": "https://example.test/T_START_MOVE",
                    "from": 0,
                    "to": 1,
                    "event": "E_DONE",
                    "event_index": 1,
                }
            ],
            "end": 1,
        },
        "platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"},
        "by_motion": {
            "move": {
                "index": 0,
                "controllers": [
                    {
                        "index": 0,
                        "id": "ctrl_x",
                        "uri": "https://example.test/ctrl_x",
                        "constraint_uri": "https://example.test/constraint_x",
                        "error_signal_uri": "https://example.test/err_x",
                        "output_signal_uri": "https://example.test/out_x",
                    }
                ],
                "monitors": [
                    {
                        "index": 0,
                        "id": "done_mon",
                        "uri": "https://example.test/done_mon",
                        "event_uri": "https://example.test/E_DONE",
                        "error_signal_uri": "https://example.test/mon_err",
                    }
                ],
            }
        },
        "quantities": [],
        "provenance_contexts": [{"id": "prov", "source": "src/metamodels/prov.json"}],
        "runtime_provenance": {
            "activity_id": "activity:controller_execution",
            "producer_agent_id": "agent:controller_process",
            "runtime_agent_id": "agent:runtime:mujoco",
        },
    }
    schema["schema_hash"] = _hash_doc(schema)
    return schema


def _frame(names: list[str], **values) -> dict:
    flat = {name: 0 for name in names}
    flat.update(
        {
            "t": 1.0,
            "step": 0,
            "fsm_state": 0,
            "active_motion": -1,
            "last_event": -1,
            "period_ns": 1_000_000,
            "compute_ns": 10_000,
        }
    )
    flat.update(values)
    return flat


def _write_frame_log(path: Path, schema: dict) -> None:
    _fmt, names = field_names_and_format(schema["pools"])
    frames = [
        # S_START; nothing active -> ActivityOccurrence(S_START)
        _frame(names, step=0, t=1.0, wall_ns=100, fsm_state=0),
        # 0->1 transition (event E_DONE); constraint satisfied + monitor low are the S_MOVE
        # baseline (edges are only detected intra-state) -> one activity, two control flows
        _frame(
            names,
            step=1,
            t=1.1,
            wall_ns=200,
            fsm_state=1,
            active_motion=0,  # S_MOVE runs the "move" motion; slots resolve through it
            state_since_wall_ns=200,
            **{
                "c0.active": 1,
                "c0.satisfied": 1,
                "m0.active": 1,
                "m0.satisfied": 0,
                "trigger_count": 1,
                "tr0.kind": 1,
                "tr0.idx": 1,
                "tr0.fsm_state": 0,
                "tr0.t": 1.1,
                "tr0.wall_ns": 200,
            },
        ),
        # intra-state: constraint falls (goal lost -> the entry span closes) + monitor fires
        _frame(
            names,
            step=2,
            t=1.2,
            wall_ns=300,
            fsm_state=1,
            active_motion=0,  # S_MOVE runs the "move" motion; slots resolve through it
            state_since_wall_ns=200,
            **{
                "c0.active": 1,
                "c0.satisfied": 0,
                "m0.active": 1,
                "m0.satisfied": 1,
                "m0.value": 0.004,
                "trigger_count": 1,
                "tr0.kind": 1,
                "tr0.idx": 1,
                "tr0.fsm_state": 0,
                "tr0.t": 1.1,
                "tr0.wall_ns": 200,
            },
        ),
        # intra-state: the goal is regained -> a satisfied span opens on the rising edge
        _frame(
            names,
            step=3,
            t=1.3,
            wall_ns=400,
            fsm_state=1,
            active_motion=0,
            state_since_wall_ns=200,
            **{
                "c0.active": 1,
                "c0.satisfied": 1,
                # Integer-valued measurement: JSON serializes a double 1.0 as "1", which
                # json.loads reads back as int — the result must still be xsd:decimal.
                "c0.error": 1,
                "m0.active": 1,
                "m0.satisfied": 1,
            },
        ),
    ]
    write_frame_log_pb(path, schema, frames)


def _source_tree(path: Path) -> Path:
    schema = _schema()
    layout = _layout(schema)
    path.mkdir()
    (path / "schema.json").write_text(json.dumps(schema, indent=4))
    (path / "frame_layout.json").write_text(json.dumps(layout, indent=4))
    write_frame_log_proto(path / "frame_log.proto", schema)
    (path / "provenance.ld.json").write_text(json.dumps(_provenance(), indent=4))
    (path / "model.ld.json").write_text(json.dumps(_provenance(), indent=4))
    (path / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (path / "headers").mkdir()
    (path / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "main.cpp").write_text("// generated\n")
    _write_frame_log(path / "frame_log.pb", schema)
    return path


def _has(graph: rdflib.Graph, subject=None, predicate=None, object_=None) -> bool:
    return (subject, predicate, object_) in graph


def test_runtime_ttl_projects_full_observation_graph(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")

    frames, _frame_count = runtime_frames(run_dir)
    runtime_ttl = write_runtime_ttl(run_dir, frames)
    text = runtime_ttl.read_text()
    assert "ent:runtime_ttl" in text
    assert "run:run-test" in text
    assert "mspact:runtime_ttl_recovery" in text
    assert "mspagent:replay_process" in text

    graph = rdflib.Graph().parse(runtime_ttl, format="turtle")

    # Sparse event graph: a Frame node only where an occurrence anchors (here all 4 steps).
    assert len(list(graph.subjects(rdflib.RDF.type, TRACE.Frame))) == 4
    # Two occurrence classes only: a span for what was in force, an instant for control moving.
    assert _has(graph, None, rdflib.RDF.type, TRACE.ActivityOccurrence)
    assert _has(graph, None, rdflib.RDF.type, TRACE.ControlFlowOccurrence)
    # Every referent is named through the one prov:used, never through a per-kind predicate:
    # the design graph's own rdf:type says which kind of element it is.
    for referent in (S_START, S_MOVE, T_START_MOVE, E_DONE, CONSTRAINT_X, DONE_MON):
        assert _has(graph, None, PROV.used, referent)
    # ... and exactly one per occurrence, so an occurrence never conflates two referents.
    for occ in set(graph.subjects(rdflib.RDF.type, PROV.Activity)):
        if (occ, TRACE.seq, None) in graph:
            assert len(list(graph.objects(occ, PROV.used))) == 1
    # The contract version composes from dcterms; run id and frame count are not restated.
    assert _has(graph, None, DCTERMS.hasVersion, None)
    # Nothing is named after a state machine any more.
    for gone in (
        MSRUN.state,
        MSRUN.transition,
        MSRUN.fsmState,
        MSRUN.fromState,
        MSRUN.toState,
        TRACE.state,
        TRACE.transition,
        TRACE.fsmState,
        TRACE.fromState,
        TRACE.toState,
    ):
        assert not list(graph.triples((None, gone, None)))

    # The Sample node families and the continuous streams are gone (they live in the frame log).
    for gone in (MSRUN.ControllerSample, MSRUN.SignalSample, MSRUN.MonitorSample):
        assert not list(graph.subjects(rdflib.RDF.type, gone))
    for gone in (
        MSRUN.error,
        MSRUN.output,
        MSRUN.measured,
        MSRUN.setpoint,
        MSRUN.t,
        MSRUN.wall_ns,
        MSRUN.satSince,
        MSRUN.compute_ns,
        MSRUN.period_ns,
    ):
        assert not list(graph.triples((None, gone, None)))

    # Each monitor/constraint occurrence carries the live residual as its simple result; the spec
    # (setpoint/threshold/operator) is referenced by URI, not copied in.
    mon = next(graph.subjects(PROV.used, DONE_MON))
    assert graph.value(mon, SOSA.hasSimpleResult) == rdflib.Literal(Decimal("0.004"))
    # Satisfied on entry, lost at step 2, regained at step 3: two spans, and the goal lost is
    # the gap between them. The entry span is what makes the loss visible at all.
    con_spans = sorted(
        graph.subjects(PROV.used, CONSTRAINT_X), key=lambda occ: int(graph.value(occ, TRACE.seq))
    )
    assert len(con_spans) == 2
    assert graph.value(con_spans[0], SOSA.hasSimpleResult) == rdflib.Literal(Decimal("0.0"))
    assert graph.value(graph.value(con_spans[0], TIME.hasEnd), TRACE.step) == rdflib.Literal(2)
    # int-valued measurement coerced to xsd:decimal (not xsd:integer, which the runtime SHACL rejects)
    assert graph.value(con_spans[1], SOSA.hasSimpleResult) == rdflib.Literal(Decimal("1.0"))
    assert graph.value(con_spans[1], SOSA.hasSimpleResult).datatype == rdflib.XSD.decimal
    # The only floats are those bounded occurrence values, and they stay exact-decimal (never double).
    floats = [
        (s, o)
        for s, p, o in graph.triples((None, SOSA.hasSimpleResult, None))
        if isinstance(o, rdflib.Literal)
    ]
    assert floats and all(o.datatype == rdflib.XSD.decimal for _, o in floats)
    assert not [
        o
        for o in graph.objects()
        if isinstance(o, rdflib.Literal) and o.datatype == rdflib.XSD.double
    ]

    # Time is xsd:dateTime; a frame is generated at one, an occurrence starts at one.
    stamps = list(graph.subject_objects(PROV.generatedAtTime)) + list(
        graph.subject_objects(PROV.startedAtTime)
    )
    assert stamps and all(o.datatype == rdflib.XSD.dateTime for _, o in stamps)

    # Runtime.ttl self-provenance: who recovered it, and derived from the frame log.
    recovery = rdflib.URIRef(prov_uri("activity:runtime_ttl_recovery"))
    assert _has(graph, None, PROV.wasGeneratedBy, recovery)
    assert _has(
        graph, recovery, PROV.wasAssociatedWith, rdflib.URIRef(prov_uri("agent:replay_process"))
    )


# Predicates the runtime graph is allowed to carry. It says what happened and points at the
# design IRIs for what was declared -- a band, a gain, a tolerance or a declared dwell copied
# in here would make two sources of truth and this list is what catches it.
ALLOWED_PREDICATES = {
    rdflib.RDF.type,
    rdflib.RDFS.label,
    TRACE.atFrame,
    TRACE.rearmCount,
    TRACE.seq,
    TRACE.slotIndex,
    TRACE.step,
    DCTERMS.hasVersion,
    SOSA.hasSimpleResult,
    TIME.hasBeginning,
    TIME.hasEnd,
    PROV.actedOnBehalfOf,
    PROV.atLocation,
    PROV.endedAtTime,
    PROV.generatedAtTime,
    PROV.startedAtTime,
    PROV.used,
    PROV.wasAssociatedWith,
    PROV.wasDerivedFrom,
    PROV.wasGeneratedBy,
    PROV.wasInformedBy,
    QUDT.hasQuantityKind,
    QUDT.unit,
    QUDT.value,
    SENS["update-rate"],
}


def _graph(tmp_path: Path) -> tuple[rdflib.Graph, Path]:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")
    frames, _frame_count = runtime_frames(run_dir)
    runtime_ttl = write_runtime_ttl(run_dir, frames)
    return rdflib.Graph().parse(runtime_ttl, format="turtle"), run_dir


def _step(graph: rdflib.Graph, frame) -> int:
    return int(graph.value(frame, TRACE.step))


def test_runtime_graph_carries_no_design_values(tmp_path: Path) -> None:
    """The separation invariant, asserted by predicate set rather than by spot check."""
    graph, _run_dir = _graph(tmp_path)
    used = {p for _s, p, _o in graph}
    assert used <= ALLOWED_PREDICATES, sorted(str(p) for p in used - ALLOWED_PREDICATES)


def test_only_the_eight_trace_terms_appear(tmp_path: Path) -> None:
    """The cheapest guard against the vocabulary creeping back."""
    graph, _run_dir = _graph(tmp_path)
    seen = {
        str(term)[len(TRACE) :]
        for triple in graph
        for term in triple
        if isinstance(term, rdflib.URIRef) and str(term).startswith(str(TRACE))
    }
    assert seen and seen <= TRACE_TERMS, sorted(seen - TRACE_TERMS)


def test_spans_are_closed_and_instants_have_no_interval(tmp_path: Path) -> None:
    graph, _run_dir = _graph(tmp_path)
    spans = set(graph.subjects(rdflib.RDF.type, TRACE.ActivityOccurrence))
    assert spans == set(graph.subjects(rdflib.RDF.type, TIME.ProperInterval))
    for span in spans:
        assert graph.value(span, TIME.hasBeginning) is not None
        # Including the run's final occupancy: without its end it drops out of every
        # duration query and the durations no longer sum to the run.
        assert graph.value(span, TIME.hasEnd) is not None
        assert graph.value(span, TRACE.atFrame) is None
    instants = set(graph.subjects(rdflib.RDF.type, TRACE.ControlFlowOccurrence))
    assert instants
    for occ in instants:
        assert graph.value(occ, TRACE.atFrame) is not None
        assert graph.value(occ, TIME.hasBeginning) is None


def test_activity_spans_tile_the_run(tmp_path: Path) -> None:
    """Coordination occupancies are contiguous, so their extents sum to the run's own extent."""
    graph, _run_dir = _graph(tmp_path)
    total = sum(
        _step(graph, graph.value(a, TIME.hasEnd)) - _step(graph, graph.value(a, TIME.hasBeginning))
        for a in graph.subjects(rdflib.RDF.type, TRACE.ActivityOccurrence)
        if graph.value(a, PROV.used) in (S_START, S_MOVE)
    )
    steps = [_step(graph, f) for f in graph.subjects(rdflib.RDF.type, TRACE.Frame)]
    assert total == max(steps) - min(steps)


def test_tick_rate_comes_from_the_header(tmp_path: Path) -> None:
    """A step converts to seconds without opening the frame log."""
    graph, _run_dir = _graph(tmp_path)
    rate = next(graph.objects(None, SENS["update-rate"]))
    assert graph.value(rate, QUDT.hasQuantityKind) == QKIND.Frequency
    assert graph.value(rate, QUDT.unit) == UNIT.HZ
    assert float(graph.value(rate, QUDT.value)) == 1e9 / _schema()["control_period_ns"]


def test_occurrences_link_to_what_informed_them(tmp_path: Path) -> None:
    """The causal chain is a walk, and it names instances rather than definitions."""
    graph, _run_dir = _graph(tmp_path)
    flow = next(graph.subjects(PROV.used, T_START_MOVE))
    # The event that moved control is itself a control-flow instant, named by its own prov:used.
    cause = graph.value(flow, PROV.wasInformedBy)
    assert cause is not None
    assert (cause, rdflib.RDF.type, TRACE.ControlFlowOccurrence) in graph
    assert graph.value(cause, PROV.used) == E_DONE
    entered = next(graph.subjects(PROV.wasInformedBy, flow))
    assert (entered, rdflib.RDF.type, TRACE.ActivityOccurrence) in graph
    assert graph.value(entered, PROV.used) == S_MOVE


def test_recover_runtime_ttl_rewrites_with_the_new_terms(tmp_path: Path) -> None:
    """The migration path for archives written before this vocabulary existed."""
    from motion_spec.introspection.replay import main

    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")
    runtime_ttl = run_dir / "runtime" / "runtime.ttl"
    runtime_ttl.parent.mkdir(parents=True, exist_ok=True)
    runtime_ttl.write_text("# stale\n")
    assert main([str(run_dir), "--recover-runtime-ttl"]) == 0
    graph = rdflib.Graph().parse(runtime_ttl, format="turtle")
    assert _has(graph, None, rdflib.RDF.type, TRACE.ActivityOccurrence)
    assert list(graph.triples((None, TIME.hasEnd, None)))
    assert list(graph.triples((None, TRACE.rearmCount, None)))


def _gate_schema(*, members: bool = True) -> dict:
    """A one-state model whose single monitor watches two member constraints."""
    schema = _schema()
    monitor = schema["by_motion"]["move"]["monitors"][0]
    if members:
        monitor["watched"] = [
            {
                "id": f"member_{axis}",
                "uri": f"https://example.test/member_{axis}",
                "error_id": f"err_{axis}",
                "tolerance_id": "band",
            }
            for axis in ("x", "y")
        ]
    schema["quantities"] = [{"index": 0, "id": "err_x"}, {"index": 1, "id": "err_y"}]
    schema["pools"] = dict(schema["pools"], quantities=2)
    schema["constants"] = [{"id": "band", "value": 1.0}]
    schema["schema_hash"] = _hash_doc(schema)
    return schema


def _gate_frames(names: list[str], *, breaks: int, member_flaps: int) -> list[dict]:
    """S_MOVE throughout: the monitor's condition holds, breaks `breaks` times, then fires."""
    frames = [_frame(names, step=0, t=0.0, wall_ns=100, fsm_state=1, active_motion=0)]
    step = 1
    for cycle in range(breaks + 1):
        for held in (1, 0) if cycle < breaks else (1,):
            for _ in range(2):
                flap = member_flaps and (step % 2)
                frames.append(
                    _frame(
                        names,
                        step=step,
                        t=step / 1000,
                        wall_ns=100 + step,
                        fsm_state=1,
                        active_motion=0,
                        state_since_wall_ns=100,
                        **{
                            "m0.active": 1,
                            "m0.satisfied": held,
                            "q0": 0.5 if not flap else 5.0,
                            "q1": 0.5,
                        },
                    )
                )
                step += 1
    # The firing tick: the monitor's event reaches the trigger ring.
    frames.append(
        _frame(
            names,
            step=step,
            t=step / 1000,
            wall_ns=100 + step,
            fsm_state=1,
            active_motion=0,
            state_since_wall_ns=100,
            **{
                "m0.active": 1,
                "m0.satisfied": 1,
                "q0": 0.5,
                "q1": 0.5,
                "trigger_count": 1,
                "tr0.kind": 1,
                "tr0.idx": 1,
                "tr0.fsm_state": 1,
                "tr0.t": step / 1000,
                "tr0.wall_ns": 100 + step,
            },
        )
    )
    return frames


def _gate_graph(tmp_path: Path, *, breaks: int, member_flaps: int = 1, **kw) -> rdflib.Graph:
    schema = _gate_schema(**kw)
    source = tmp_path / "source"
    source.mkdir()
    (source / "schema.json").write_text(json.dumps(schema, indent=4))
    (source / "frame_layout.json").write_text(json.dumps(_layout(schema), indent=4))
    write_frame_log_proto(source / "frame_log.proto", schema)
    for name in ("provenance.ld.json", "model.ld.json"):
        (source / name).write_text(json.dumps(_provenance(), indent=4))
    (source / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (source / "headers").mkdir()
    (source / "headers" / "runtime.hpp").write_text("// generated\n")
    (source / "main.cpp").write_text("// generated\n")
    _fmt, names = field_names_and_format(schema["pools"])
    write_frame_log_pb(
        source / "frame_log.pb",
        schema,
        _gate_frames(names, breaks=breaks, member_flaps=member_flaps),
    )
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-gate")
    frames, _frame_count = runtime_frames(run_dir)
    return rdflib.Graph().parse(write_runtime_ttl(run_dir, frames), format="turtle")


def _monitor(graph: rdflib.Graph):
    return next(graph.subjects(PROV.used, DONE_MON))


def _members(graph: rdflib.Graph) -> list:
    return [
        occ
        for occ, referent in graph.subject_objects(PROV.used)
        if str(referent).startswith("https://example.test/member_")
    ]


def test_rearm_count_and_first_hold(tmp_path: Path) -> None:
    """A condition that breaks twice before firing records it, and the arming's interval
    begins at the hold that led to the firing -- not at the firing edge."""
    graph = _gate_graph(tmp_path, breaks=2)
    mon = _monitor(graph)
    assert int(graph.value(mon, TRACE.rearmCount)) == 2
    began = _step(graph, graph.value(mon, TIME.hasBeginning))
    fired = _step(graph, graph.value(mon, TIME.hasEnd))
    assert began < fired


def test_boring_gate_emits_no_member_edges(tmp_path: Path) -> None:
    """A gate that armed once and fired says nothing a member lane could show."""
    graph = _gate_graph(tmp_path, breaks=0)
    mon = _monitor(graph)
    assert int(graph.value(mon, TRACE.rearmCount)) == 0
    assert not _members(graph)


def test_interesting_gate_emits_member_edges_linked_to_the_arming(tmp_path: Path) -> None:
    graph = _gate_graph(tmp_path, breaks=2)
    mon = _monitor(graph)
    members = _members(graph)
    assert members
    # Each member span informed the arming, so the lanes hang off the firing that wanted them.
    assert set(members) <= set(graph.objects(mon, PROV.wasInformedBy))
    for occ in members:
        assert graph.value(occ, TIME.hasEnd) is not None


def test_truncated_member_series_keeps_the_true_rearm_count(tmp_path: Path) -> None:
    """The member series is capped; rearmCount is not, so truncation stays visible."""
    breaks = MEMBER_EDGE_LIMIT * 4
    graph = _gate_graph(tmp_path, breaks=breaks)
    mon = _monitor(graph)
    assert int(graph.value(mon, TRACE.rearmCount)) == breaks
    per_member: dict = {}
    for occ in _members(graph):
        per_member.setdefault(str(graph.value(occ, PROV.used)), []).append(occ)
    assert per_member
    for spans in per_member.values():
        assert len(spans) <= MEMBER_EDGE_LIMIT
    # Every break flapped a member, so an uncapped series would carry one span per break.
    assert max(len(spans) for spans in per_member.values()) < breaks


def test_ambiguous_control_flow_records_nothing(tmp_path: Path) -> None:
    """Two candidate transitions both matching the observed events is not a cause.

    A missing link is honest; a guessed one poisons every query built on the chain.
    """
    from motion_spec.introspection.runtime_graph import _fired_transition

    candidates = [
        {"id": "T_A", "uri": "https://example.test/T_A", "event_indices": [1, 2]},
        {"id": "T_B", "uri": "https://example.test/T_B", "event_indices": [1, 3]},
    ]
    assert _fired_transition(candidates, {1}) is None
    assert _fired_transition(candidates, {2})["id"] == "T_A"
    assert _fired_transition(candidates, set()) is None
