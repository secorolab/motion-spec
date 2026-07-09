# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from decimal import Decimal
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import create_archive_manifest
from motion_spec.introspection.artifacts import prov_uri
from motion_spec.introspection.frame_layout_spec import field_names_and_format
from motion_spec.introspection.replay import runtime_frames
from motion_spec.introspection.runtime_graph import MSRUN, PROV, _bind_model_subnamespaces, write_runtime_ttl

from frame_log_fixture import write_frame_log_pb
from test_introspection_archive import _hash_doc, _layout, _provenance


def _schema() -> dict:
    schema = {
        "schema_version": 1,
        "frame_layout_version": 1,
        "runtime_rdf_contract_version": 1,
        "generated_by": "test",
        "ir_path": "ir.json",
        "graph": "model.jsonld",
        "context": {},
        "pools": {"constraints": 1, "monitors": 1, "quantities": 0, "triggers": 3},
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
        "by_state": {
            "S_MOVE": {
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


def _write_frame_log(path: Path, schema: dict, layout: dict) -> None:
    _fmt, names = field_names_and_format(schema["pools"])
    frames = [
        # S_START; nothing active -> StateOccurrence(S_START)
        _frame(names, step=0, t=1.0, wall_ns=100, fsm_state=0),
        # 0->1 transition (event E_DONE); constraint satisfied + monitor low are the S_MOVE
        # baseline (edges are only detected intra-state) -> State/Transition/EventOccurrence
        _frame(
            names,
            step=1,
            t=1.1,
            wall_ns=200,
            fsm_state=1,
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
        # intra-state: constraint falls (goal lost) + monitor fires (rising)
        # -> ConstraintUnsatisfiedOccurrence + MonitorOccurrence
        _frame(
            names,
            step=2,
            t=1.2,
            wall_ns=300,
            fsm_state=1,
            state_since_wall_ns=200,
            **{
                "c0.active": 1,
                # Integer-valued measurement: JSON serializes a double 1.0 as "1", which
                # json.loads reads back as int — msrun:value must still be xsd:decimal.
                "c0.satisfied": 0,
                "c0.error": 1,
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
    ]
    write_frame_log_pb(path, schema, layout, frames)


def _source_tree(path: Path) -> Path:
    schema = _schema()
    layout = _layout(schema)
    path.mkdir()
    (path / "schema.json").write_text(json.dumps(schema, indent=4))
    (path / "frame_layout.json").write_text(json.dumps(layout, indent=4))
    (path / "provenance.jsonld").write_text(json.dumps(_provenance(), indent=4))
    (path / "model.jsonld").write_text(json.dumps(_provenance(), indent=4))
    (path / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (path / "headers").mkdir()
    (path / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "ref_main.cpp").write_text("// generated\n")
    _write_frame_log(path / "frame_log.pb", schema, layout)
    return path


def _has(graph: rdflib.Graph, subject=None, predicate=None, object_=None) -> bool:
    return (subject, predicate, object_) in graph


def test_runtime_ttl_projects_full_observation_graph(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")

    frames, frame_count = runtime_frames(run_dir)
    runtime_ttl = write_runtime_ttl(run_dir, frames, frame_count=frame_count)
    text = runtime_ttl.read_text()
    assert "ent:runtime_ttl" in text
    assert "run:run-test" in text
    assert "mspact:runtime_ttl_recovery" in text
    assert "mspagent:replay_process" in text

    graph = rdflib.Graph().parse(runtime_ttl, format="turtle")

    # Sparse event graph: a Frame node only where an occurrence anchors (here all 3 steps).
    assert len(list(graph.subjects(rdflib.RDF.type, MSRUN.Frame))) == 3
    # Discrete occurrences are synthesized from the frame scan (not the C++ trigger ring).
    assert _has(graph, None, rdflib.RDF.type, MSRUN.StateOccurrence)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.TransitionOccurrence)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.EventOccurrence)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.MonitorOccurrence)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.ConstraintUnsatisfiedOccurrence)
    # Transition carries the full edge: which transition, from/to state, and the firing event.
    assert _has(graph, None, MSRUN.transition, rdflib.URIRef("https://example.test/T_START_MOVE"))
    assert _has(graph, None, MSRUN.fromState, rdflib.URIRef("https://example.test/S_START"))
    assert _has(graph, None, MSRUN.toState, rdflib.URIRef("https://example.test/S_MOVE"))
    assert _has(graph, None, MSRUN.activeState, rdflib.URIRef("https://example.test/S_MOVE"))
    assert _has(graph, None, MSRUN.controller, rdflib.URIRef("https://example.test/ctrl_x"))
    assert _has(graph, None, MSRUN.constraint, rdflib.URIRef("https://example.test/constraint_x"))
    assert _has(graph, None, MSRUN.monitor, rdflib.URIRef("https://example.test/done_mon"))
    assert _has(graph, None, MSRUN.event, rdflib.URIRef("https://example.test/E_DONE"))

    # The Sample node families and the continuous streams are gone (they live in the frame log).
    for gone in (MSRUN.ControllerSample, MSRUN.SignalSample, MSRUN.MonitorSample):
        assert not list(graph.subjects(rdflib.RDF.type, gone))
    for gone in (MSRUN.error, MSRUN.output, MSRUN.measured, MSRUN.setpoint,
                 MSRUN.t, MSRUN.wall_ns, MSRUN.satSince, MSRUN.compute_ns, MSRUN.period_ns):
        assert not list(graph.triples((None, gone, None)))

    # Each monitor/constraint occurrence carries the live residual (msrun:value) and the spec
    # (setpoint/threshold/operator) is referenced by URI, not copied in.
    mon = next(graph.subjects(rdflib.RDF.type, MSRUN.MonitorOccurrence))
    assert graph.value(mon, MSRUN.value) == rdflib.Literal(Decimal("0.004"))
    con = next(graph.subjects(rdflib.RDF.type, MSRUN.ConstraintUnsatisfiedOccurrence))
    # int-valued measurement coerced to xsd:decimal (not xsd:integer, which the runtime SHACL rejects)
    assert graph.value(con, MSRUN.value) == rdflib.Literal(Decimal("1.0"))
    assert graph.value(con, MSRUN.value).datatype == rdflib.XSD.decimal
    # The only floats are those bounded occurrence values, and they stay exact-decimal (never double).
    floats = [(s, o) for s, p, o in graph.triples((None, MSRUN.value, None)) if isinstance(o, rdflib.Literal)]
    assert floats and all(o.datatype == rdflib.XSD.decimal for _, o in floats)
    assert not [o for o in graph.objects() if isinstance(o, rdflib.Literal) and o.datatype == rdflib.XSD.double]

    # Time is prov:generatedAtTime (xsd:dateTime).
    stamps = list(graph.subject_objects(PROV.generatedAtTime))
    assert stamps and all(o.datatype == rdflib.XSD.dateTime for _, o in stamps)

    # Runtime.ttl self-provenance: who recovered it, and derived from the frame log.
    recovery = rdflib.URIRef(prov_uri("activity:runtime_ttl_recovery"))
    assert _has(graph, None, PROV.wasGeneratedBy, recovery)
    assert _has(graph, recovery, PROV.wasAssociatedWith, rdflib.URIRef(prov_uri("agent:replay_process")))


def test_model_subnamespace_bindings_compact_deep_model_iris() -> None:
    graph = rdflib.Graph()
    graph.bind("msrun", MSRUN)
    model_base = "https://example.test/models/demo/"
    condition = rdflib.URIRef(f"{model_base}pick/when/aligned-above")
    graph.add((MSRUN["example"], MSRUN.constraint, condition))

    _bind_model_subnamespaces(graph, model_base)

    text = graph.serialize(format="turtle")
    assert "pick-when:aligned-above" in text
    assert f"<{condition}>" not in text
