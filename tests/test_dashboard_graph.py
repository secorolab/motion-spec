# SPDX-License-Identifier: MPL-2.0
"""The dashboard graph service: live values, sampled history, and no new vocabulary."""

from __future__ import annotations

import json
from decimal import Decimal
from pathlib import Path

import rdflib
from rdflib.namespace import split_uri

from motion_spec.dashboard.catalog import provenance_graph, rdf_name
from motion_spec.dashboard.graph import LIVE_GRAPH, RUNTIME_GRAPH, GraphService
from motion_spec.dashboard.store import RunStore
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.runtime_graph import write_runtime_ttl

from dashboard_fixture import (
    CONSTRAINT,
    ERROR_SIGNAL,
    MONITOR,
    MOVE,
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
# What was active, and for how long: an activity spans two frames, each carrying its step.
SPANS = f"""
PREFIX prov: <{PROV}>
PREFIX time: <{TIME}>
PREFIX trace: <{TRACE}>
SELECT ?element ?from ?to WHERE {{
    ?occ a trace:ActivityOccurrence ;
         prov:used ?element ;
         time:hasBeginning ?begin ;
         time:hasEnd ?end .
    ?begin trace:step ?from .
    ?end trace:step ?to .
}}
"""
OCCURRENCES = f"""
PREFIX trace: <{TRACE}>
SELECT ?occ WHERE {{ ?occ a trace:ActivityOccurrence }}
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


def _archived_ttl(tmp_path, doc, frames) -> Path:
    """A run directory that kept its own runtime.ttl -- the archived record, not a projection."""
    run = tmp_path / "run"
    (run / "logs").mkdir(parents=True)
    write_frame_log_pb(run / "logs" / "frame_log.pb", doc, _frames(doc))
    (run / "manifest.json").write_text(
        json.dumps({"run_id": "run-1", "files": {"frame_log": "logs/frame_log.pb"}})
    )
    return write_runtime_ttl(run, frames)


def _archived_service(tmp_path, **kwargs):
    doc = schema()
    store = _store(tmp_path, doc)
    ttl = _archived_ttl(tmp_path, doc, store.snapshot())
    return GraphService(_generation(tmp_path, doc), store, runtime_ttl=ttl, **kwargs)


def test_a_live_query_returns_the_current_value_of_every_active_slot(tmp_path):
    service = _service(tmp_path)
    _type, (_headers, rows) = service.query(OBSERVATIONS)
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
    _type, (_headers, rows) = service.query(OCCURRENCES)
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


def test_an_archived_run_answers_the_state_timeline_from_its_runtime_ttl(tmp_path):
    """The whole plan in one query: what was active, from which step to which. The archived
    record is the source; the frame log only produced it."""
    service = _archived_service(tmp_path)
    assert service.runtime_source == "archive"
    _type, (_headers, rows) = service.query(SPANS)
    spans = {(str(element), int(begin), int(end)) for element, begin, end in rows}

    # S_MOVE runs the whole 101-frame log; the constraint's goal is only reached at step 51.
    assert (MOVE, 0, 100) in spans
    assert (CONSTRAINT, 51, 100) in spans


def test_an_archived_run_is_never_also_projected(tmp_path):
    """Loading the record and projecting the frames would emit every occurrence twice, and
    silently double every duration read off one."""
    doc = schema()
    store = _store(tmp_path, doc)
    ttl = _archived_ttl(tmp_path, doc, store.snapshot())
    archived = rdflib.Graph().parse(ttl, format="turtle")
    on_record = set(archived.subjects(rdflib.RDF.type, rdflib.URIRef(TRACE + "ActivityOccurrence")))

    service = GraphService(_generation(tmp_path, doc), store, runtime_ttl=ttl)
    size = len(service.dataset.graph(RUNTIME_GRAPH))
    _type, (_headers, rows) = service.query(OCCURRENCES)

    assert {occ for (occ,) in rows} == on_record
    assert len(rows) == len(on_record), "an occurrence was projected on top of the record"
    assert len(service.dataset.graph(RUNTIME_GRAPH)) == size, "sync() projected onto the archive"
    assert service._fed == 0, "frames reached a projector for an archived run"


def test_a_live_run_projects_because_it_has_no_record_yet(tmp_path):
    service = _service(tmp_path)
    service.sync()

    assert service.runtime_source == "projected"
    assert len(service.dataset.graph(RUNTIME_GRAPH)) > 0


def test_a_construct_query_answers_with_triples_and_a_select_keeps_its_shape(tmp_path):
    service = _service(tmp_path)
    kind, triples = service.query("CONSTRUCT { ?s ?p ?o } WHERE { ?s ?p ?o } LIMIT 5")

    assert kind == "CONSTRUCT"
    assert len(triples) == 5
    assert all(len(triple) == 3 for triple in triples)

    kind, (headers, rows) = service.query(OBSERVATIONS)
    assert kind == "SELECT"
    assert headers == ["p", "v"]
    assert rows and all(len(row) == 2 for row in rows)


def _payload(tmp_path):
    service = _archived_service(tmp_path)
    return service, provenance_graph(service)


def test_the_graph_payload_folds_type_edges_into_node_types(tmp_path):
    service, payload = _payload(tmp_path)
    by_id = {node["id"]: node for node in payload["nodes"]}

    assert not [link for link in payload["links"] if link["value"] == str(rdflib.RDF.type)]
    for subject, _p, obj, _g in service.dataset.quads((None, rdflib.RDF.type, None, None)):
        assert rdf_name(obj) in by_id[str(subject)]["types"]


def test_the_graph_payload_folds_literals_into_node_attributes(tmp_path):
    service, payload = _payload(tmp_path)
    by_id = {node["id"]: node for node in payload["nodes"]}
    drawn = {(link["source"], link["target"]) for link in payload["links"]}

    for subject, predicate, obj, _g in service.dataset.quads((None, None, None, None)):
        if not isinstance(obj, rdflib.Literal):
            continue
        assert (str(subject), str(obj)) not in drawn, "a literal was drawn as a node"
        assert str(obj) in by_id[str(subject)]["attributes"][rdf_name(predicate)]


def test_every_folded_edge_is_accounted_for(tmp_path):
    """A reader who cannot account for the difference between triples and links will not
    trust the picture, so the folded classes are counted rather than dropped."""
    service, payload = _payload(tmp_path)
    raw = len(list(service.dataset.quads((None, None, None, None))))
    hidden = payload["hidden"]

    assert hidden["type_edges"] + hidden["literal_edges"] + len(payload["links"]) == raw
    # Provenance edges stay in `links`, marked for an overlay the view can switch off.
    assert hidden["provenance_edges"] == len(
        [link for link in payload["links"] if link["kind"] == "provenance"]
    )


def test_folding_leaves_no_hub_a_force_layout_cannot_separate(tmp_path):
    payload = _payload(tmp_path)[1]

    assert payload["nodes"]
    assert max(node["degree"] for node in payload["nodes"]) < 200


def test_the_legend_carries_every_type_the_graph_declares(tmp_path):
    service, payload = _payload(tmp_path)
    declared = {rdf_name(obj) for obj in service.dataset.objects(None, rdflib.RDF.type)}

    assert payload["types"]
    assert set(payload["types"]) == declared
    assert payload["runtime_source"] == "archive"
    assert {node["graphs"][0] for node in payload["nodes"]} <= {"model", "runtime", "live"}


def test_node_ids_are_stable_across_calls(tmp_path):
    service = _archived_service(tmp_path)

    first = [node["id"] for node in provenance_graph(service)["nodes"]]
    second = [node["id"] for node in provenance_graph(service)["nodes"]]
    assert first == second
