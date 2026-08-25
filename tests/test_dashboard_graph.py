# SPDX-License-Identifier: MPL-2.0
"""The dashboard graph service: live values, sampled history, and no new vocabulary."""

from __future__ import annotations

import json
from decimal import Decimal

import rdflib
from rdflib.namespace import split_uri

from motion_spec.dashboard.graph import LIVE_GRAPH, RUNTIME_GRAPH, GraphService
from motion_spec.dashboard.queries import model_lint
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


# The design-graph lint, on a model shaped like the one that motivated it: two zero-valued
# tolerances that look redundant and are not, one band declared and read by nothing.
LINT_APP = "https://example.test/exec/"
LINT_MODEL = {
    "@context": {
        "app": LINT_APP,
        "cstr": "https://comp-rob2b.github.io/metamodels/task/constraint#",
        "cstr-ext": "https://secorolab.github.io/metamodels/task/constraint#",
        "qudt": "http://qudt.org/schema/qudt/",
        "qudt:value": {"@id": "http://qudt.org/schema/qudt/value", "@type": "xsd:double"},
        "xsd": "http://www.w3.org/2001/XMLSchema#",
    },
    "@graph": [
        {"@id": "app:shared/spec/zero-length", "@type": "qudt:Quantity", "qudt:value": "0.0"},
        {"@id": "app:shared/spec/zero-angle", "@type": "qudt:Quantity", "qudt:value": "0.0"},
        {"@id": "app:shared/spec/lift-height-z", "@type": "qudt:Quantity", "qudt:value": "0.2"},
        {"@id": "app:shared/spec/pose-band", "@type": "qudt:Quantity", "qudt:value": "0.03"},
        # `within <zero-length>`: a gate saying "no band" rather than inheriting a tolerance.
        {
            "@id": "app:lift-up/until/above-lift-up-z",
            "@type": "cstr:Constraint",
            "cstr-ext:tolerance": {"@id": "app:shared/spec/zero-length"},
            "cstr:threshold": {"@id": "app:shared/spec/lift-height-z"},
        },
        {
            "@id": "app:hold-close/until/grasped",
            "@type": "cstr:Constraint",
            "cstr-ext:tolerance": {"@id": "app:shared/spec/zero-angle"},
        },
    ],
}
LINT_SOURCE = """context (ns=app) shared {
    spec {
        length zero-length   = 0.0 m,
        angle  zero-angle    = 0.0 rad,
        length lift-height-z = 0.2 m,
        length pose-band     = 0.03 m
    }
}
"""


def _lint_generation(tmp_path, model=LINT_MODEL, source=LINT_SOURCE, name="lint-gen"):
    gen = tmp_path / name
    (gen / "generated" / "model").mkdir(parents=True)
    (gen / "generated" / "source").mkdir(parents=True)
    if model is not None:
        (gen / "generated/model/test-app.ld.json").write_text(json.dumps(model))
    (gen / "generated/source/test.robmot").write_text(source)
    return gen


def test_a_declared_value_nothing_reads_is_flagged_with_its_iri_and_line(tmp_path):
    items = model_lint(_lint_generation(tmp_path))["items"]

    assert [item["name"] for item in items] == ["pose-band"]
    assert items[0]["iri"] == LINT_APP + "shared/spec/pose-band"
    assert items[0]["rule"] == "unused-declaration"
    assert items[0]["source_line"] == 6
    assert LINT_SOURCE.splitlines()[5].strip().startswith("length pose-band")


def test_a_zero_tolerance_is_referenced_and_must_never_be_flagged(tmp_path):
    """`zero-length` and `zero-angle` exist so a gate can say "no band" instead of inheriting
    the quantity's tolerance. They look redundant and are load-bearing: a constraint names
    them, so a lint that reads references rather than values leaves them alone. Flagging one
    means the query stopped asking about references -- fix the query, never the name."""
    flagged = {item["name"] for item in model_lint(_lint_generation(tmp_path))["items"]}

    assert "zero-length" not in flagged
    assert "zero-angle" not in flagged
    # ...and not because zero-valued terms are excluded: the same 0.0 unreferenced is flagged.
    orphan = json.loads(json.dumps(LINT_MODEL))
    orphan["@graph"][4].pop("cstr-ext:tolerance")
    assert "zero-length" in {
        item["name"]
        for item in model_lint(_lint_generation(tmp_path, orphan, name="orphan-gen"))["items"]
    }


def test_the_lint_never_raises_above_a_warning(tmp_path):
    """The graph can see that nothing reads a value; only the author knows whether that is a
    mistake, so no finding is ever an error."""
    assert {item["severity"] for item in model_lint(_lint_generation(tmp_path))["items"]} == {
        "warn"
    }


def test_a_generation_with_no_model_graph_lints_to_nothing(tmp_path):
    assert model_lint(_lint_generation(tmp_path, model=None)) == {"items": []}


def test_a_term_the_source_does_not_declare_is_still_reported(tmp_path):
    """An imported graph -- a scene, an FSM -- has no line in the model being read. The finding
    is still true, so it is still listed, with no line to jump to."""
    items = model_lint(_lint_generation(tmp_path, source="context (ns=app) shared {}\n"))["items"]

    assert [item["name"] for item in items] == ["pose-band"]
    assert items[0]["source_line"] is None
