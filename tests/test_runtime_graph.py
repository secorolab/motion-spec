# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import create_archive_manifest
from motion_spec.introspection.frame_layout_spec import frame_struct
from motion_spec.introspection.replay import HEADER, MAGIC, runtime_frames
from motion_spec.introspection.runtime_graph import MSRUN, write_runtime_ttl

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
    st, names = frame_struct(schema["pools"])
    frames = [
        _frame(names, step=0, t=1.0, wall_ns=100, fsm_state=0),
        _frame(
            names,
            step=1,
            t=1.1,
            wall_ns=200,
            fsm_state=1,
            **{
                "c0.active": 1,
                "c0.error": 0.2,
                "c0.output": 3.0,
                "c0.satisfied": 1,
                "c0.sat_t": 1.05,
                "c0.measured": 0.8,
                "c0.setpoint": 1.0,
                "trigger_count": 1,
                "tr0.kind": 1,
                "tr0.idx": 1,
                "tr0.fsm_state": 1,
                "tr0.t": 1.1,
                "tr0.wall_ns": 200,
            },
        ),
        _frame(
            names,
            step=2,
            t=1.2,
            wall_ns=300,
            fsm_state=1,
            **{
                "c0.active": 1,
                "c0.error": -0.1,
                "c0.output": 2.5,
                "c0.satisfied": 0,
                "m0.active": 1,
                "m0.value": 0.9,
                "m0.satisfied": 1,
                "m0.sat_t": 1.2,
                "trigger_count": 3,
                "tr0.kind": 1,
                "tr0.idx": 1,
                "tr0.fsm_state": 1,
                "tr0.t": 1.1,
                "tr0.wall_ns": 200,
                "tr1.kind": 4,
                "tr1.idx": 0,
                "tr1.fsm_state": 1,
                "tr1.t": 1.2,
                "tr1.wall_ns": 300,
                "tr2.kind": 3,
                "tr2.idx": 0,
                "tr2.fsm_state": 1,
                "tr2.t": 1.2,
                "tr2.wall_ns": 301,
            },
        ),
    ]
    header = HEADER.pack(
        MAGIC,
        1,
        st.size,
        schema["schema_hash"].encode(),
        layout["frame_layout_hash"].encode(),
        schema["runtime_provenance"]["producer_agent_id"].encode(),
        schema["runtime_provenance"]["activity_id"].encode(),
    )
    path.write_bytes(header + b"".join(st.pack(*(frame[name] for name in names)) for frame in frames))


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
    _write_frame_log(path / "frame_log.bin", schema, layout)
    return path


def _has(graph: rdflib.Graph, subject=None, predicate=None, object_=None) -> bool:
    return (subject, predicate, object_) in graph


def test_runtime_ttl_projects_full_observation_graph(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")

    frames, frame_count = runtime_frames(run_dir)
    runtime_ttl = write_runtime_ttl(run_dir, frames, frame_count=frame_count)

    graph = rdflib.Graph().parse(runtime_ttl, format="turtle")
    assert len(list(graph.subjects(rdflib.RDF.type, MSRUN.Frame))) == 3
    assert _has(graph, None, rdflib.RDF.type, MSRUN.ControllerSample)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.SignalSample)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.MonitorSample)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.EventOccurrence)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.MonitorOccurrence)
    assert _has(graph, None, rdflib.RDF.type, MSRUN.ConstraintUnsatisfiedOccurrence)
    assert _has(graph, None, MSRUN.activeState, rdflib.URIRef("https://example.test/S_MOVE"))
    assert _has(graph, None, MSRUN.controller, rdflib.URIRef("https://example.test/ctrl_x"))
    assert _has(graph, None, MSRUN.signal, rdflib.URIRef("https://example.test/err_x"))
    assert _has(graph, None, MSRUN.monitor, rdflib.URIRef("https://example.test/done_mon"))
    assert _has(graph, None, MSRUN.event, rdflib.URIRef("https://example.test/E_DONE"))
