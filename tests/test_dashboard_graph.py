# SPDX-License-Identifier: MPL-2.0
"""The dashboard graph service: live values, sampled history, and no new vocabulary."""

from __future__ import annotations

import json
from decimal import Decimal

import rdflib
from rdflib.namespace import split_uri

from motion_spec.dashboard.graph import LIVE_GRAPH, RUNTIME_GRAPH, GraphService
from motion_spec.dashboard.store import RunStore
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.introspection import frame_log_pb

from dashboard_fixture import (
    CONSTRAINT,
    ERROR_SIGNAL,
    MONITOR,
    OUTPUT_SIGNAL,
    QUANTITY,
    model_jsonld,
    schema,
)
from frame_log_fixture import flat_frame, write_frame_log_pb

SOSA = "http://www.w3.org/ns/sosa/"
PROV = "http://www.w3.org/ns/prov#"
MSRUN = "https://secorolab.github.io/motion-spec/runtime/"
TRACE = "https://secorolab.github.io/metamodels/motion-spec/execution-trace/"
TIME = "http://www.w3.org/2006/time#"
ERROR_VALUE = 0.125
OBSERVATIONS = f"""
PREFIX sosa: <{SOSA}>
SELECT ?p ?v WHERE {{
    ?obs a sosa:Observation ; sosa:observedProperty ?p ; sosa:hasSimpleResult ?v .
}}
"""


def _generation(tmp_path, doc):
    gen = tmp_path / "gen"
    (gen / "generated" / "model").mkdir(parents=True)
    (gen / "generated" / "contract").mkdir(parents=True)
    (gen / "generated" / "model" / "test-app.ld.json").write_text(json.dumps(model_jsonld()))
    (gen / "generated" / "contract" / "frame_layout.json").write_text(
        json.dumps(build_frame_layout(doc))
    )
    return gen


def _frames(doc, count=101):
    """`count` ticks at 100 Hz -- 10 s of sim time, with one satisfaction edge in the middle."""
    return [
        flat_frame(
            doc,
            t=step * 0.1,
            step=step,
            fsm_state=0,
            active_motion=0,
            last_event=-1,
            wall_ns=1_000_000_000 + step,
            period_ns=1_000_000,
            **{
                "c0.active": 1,
                "c0.error": ERROR_VALUE,
                "c0.output": -3.5,
                "c0.satisfied": 1 if step > 50 else 0,
                "m0.active": 1,
                "m0.value": 0.004,
                "q0": 42.5,
                "pose0.active": 1,
            },
        )
        for step in range(count)
    ]


def _store(tmp_path, doc):
    log = tmp_path / "frame_log.pb"
    write_frame_log_pb(log, doc, _frames(doc))
    contract = frame_log_pb.read_contract(log)
    store = RunStore("run-1", contract)
    store.add_frames(frame_log_pb.frame_records(log, contract))
    return store


def _service(tmp_path, **kwargs):
    doc = schema()
    return GraphService(_generation(tmp_path, doc), _store(tmp_path, doc), **kwargs)


def test_a_live_query_returns_the_current_value_of_every_active_slot(tmp_path):
    service = _service(tmp_path)
    _headers, rows = service.query(OBSERVATIONS)
    observed = {(str(p), v.toPython()) for p, v in rows}

    assert (ERROR_SIGNAL, Decimal(repr(ERROR_VALUE))) in observed
    assert (OUTPUT_SIGNAL, Decimal("-3.5")) in observed
    assert (MONITOR, Decimal("0.004")) in observed
    assert (QUANTITY, Decimal("42.5")) in observed
    assert (CONSTRAINT, True) in observed  # constraint satisfaction, boolean


def test_the_live_graph_is_replaced_not_accumulated(tmp_path):
    service = _service(tmp_path)
    sizes = []
    for _ in range(4):
        service.query(OBSERVATIONS)
        sizes.append(len(service.dataset.graph(LIVE_GRAPH)))

    assert sizes[0] > 0
    assert len(set(sizes)) == 1, f"live graph grew across queries: {sizes}"


def test_history_is_sampled_at_the_declared_interval_and_excludes_quantities(tmp_path):
    service = _service(tmp_path, sample_interval_s=1.0)
    service.sync()
    runtime = service.dataset.graph(RUNTIME_GRAPH)

    def sampled(prop):
        return len(list(runtime.subjects(rdflib.URIRef(SOSA + "observedProperty"), prop)))

    # 10 s of sim time at 1.0 s spacing.
    assert abs(sampled(rdflib.URIRef(ERROR_SIGNAL)) - 10) <= 1
    assert abs(sampled(rdflib.URIRef(OUTPUT_SIGNAL)) - 10) <= 1
    assert abs(sampled(rdflib.URIRef(MONITOR)) - 10) <= 1
    # Quantities would be 595/frame on a real model; they live in the frame log and the charts.
    assert sampled(rdflib.URIRef(QUANTITY)) == 0


def test_history_carries_no_values_when_sampling_is_off(tmp_path):
    service = _service(tmp_path, sample_interval_s=None)
    service.sync()
    runtime = service.dataset.graph(RUNTIME_GRAPH)

    assert list(runtime.subjects(rdflib.RDF.type, rdflib.URIRef(SOSA + "Observation"))) == []
    # ...but the semantic edges are still there: the constraint became satisfied at step 51.
    assert (None, rdflib.URIRef(PROV + "used"), rdflib.URIRef(CONSTRAINT)) in runtime


def test_occurrences_and_live_values_answer_one_query_together(tmp_path):
    service = _service(tmp_path)
    _headers, rows = service.query(f"""
        PREFIX trace: <{TRACE}>
        SELECT ?occ WHERE {{ ?occ a trace:ActivityOccurrence }}
    """)
    assert rows, "the satisfaction edge should be projected into urn:runtime"

    assert service.namespaces()["sosa"] == SOSA
    assert "msrun" in service.namespaces()


def test_the_dashboard_mints_no_vocabulary(tmp_path):
    """Every predicate and class the dashboard emits must already exist in a standard or
    vendored vocabulary. A new term here means the graph stopped being composable."""
    service = _service(tmp_path, sample_interval_s=1.0)
    service.sync()
    live = service.dataset.graph(LIVE_GRAPH)
    runtime = service.dataset.graph(RUNTIME_GRAPH)

    def namespaces(nodes):
        return {split_uri(str(node))[0] for node in nodes}

    # The value overlay is the dashboard's own emission: SOSA plus rdf:type, nothing else.
    live_predicates = {str(p) for p in set(live.predicates())}
    assert live_predicates - {str(rdflib.RDF.type)} == {
        SOSA + name
        for name in (
            "observedProperty",
            "hasSimpleResult",
            "hasFeatureOfInterest",
            "madeBySensor",
            "resultTime",
        )
    }
    assert {str(o) for o in live.objects(None, rdflib.RDF.type)} == {SOSA + "Observation"}

    allowed = {SOSA, PROV, MSRUN, TRACE, TIME, str(rdflib.RDF)}
    for graph in (live, runtime):
        assert namespaces(graph.predicates()) <= allowed
        assert namespaces(graph.objects(None, rdflib.RDF.type)) <= allowed
        datatypes = {
            o.datatype
            for o in graph.objects()
            if isinstance(o, rdflib.Literal) and o.datatype is not None
        }
        assert namespaces(datatypes) <= {str(rdflib.XSD)}
