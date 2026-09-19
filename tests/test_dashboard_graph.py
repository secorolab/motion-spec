# SPDX-License-Identifier: MPL-2.0
"""The dashboard graph service: the design graph, read and classified, with nothing added."""

from __future__ import annotations

import json

import rdflib
from dashboard_fixture import CTRL, ERROR_SIGNAL, model_jsonld, schema
from frame_log_fixture import flat_frame, write_frame_log_pb
from rdflib.namespace import split_uri

from motion_spec.dashboard.catalog import graph_name, provenance_graph, rdf_name
from motion_spec.dashboard.graph import GraphService
from motion_spec.dashboard.queries import model_lint
from motion_spec.dashboard.store import RunStore
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.introspection import frame_log_pb

ERROR_VALUE = 0.125
# What the model declares a controller drives, which is what the Explore page is asked for.
SIGNALS = """
PREFIX cstr-hdl: <https://comp-rob2b.github.io/metamodels/task/constraint-handler#>
SELECT ?controller ?signal WHERE { ?controller cstr-hdl:error-signal ?signal }
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


def test_the_service_answers_from_the_model_graph_alone(tmp_path):
    """One graph, the design's. Nothing the run produced is projected into RDF any more."""
    service = _service(tmp_path)
    assert [name for name in service.dataset.graphs() if len(name)] == [service.model]
    _type, (_headers, rows) = service.query(SIGNALS)
    assert (rdflib.URIRef(CTRL), rdflib.URIRef(ERROR_SIGNAL)) in {tuple(row) for row in rows}


def test_the_dashboard_mints_no_vocabulary(tmp_path):
    """The dashboard reads the design graph and adds nothing of its own to it."""
    service = _service(tmp_path)
    model = service.dataset.graph(rdflib.URIRef("urn:model"))
    declared = rdflib.Graph().parse(data=json.dumps(model_jsonld()), format="json-ld")

    def namespaces(nodes):
        return {split_uri(str(node))[0] for node in nodes}

    assert namespaces(model.predicates()) == namespaces(declared.predicates())
    assert set(model) == set(declared)


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


def test_a_term_the_source_does_not_declare_is_still_reported(tmp_path):
    """An imported graph -- a scene, an FSM -- has no line in the model being read. The finding
    is still true, so it is still listed, with no line to jump to."""
    items = model_lint(_lint_generation(tmp_path, source="context (ns=app) shared {}\n"))["items"]

    assert [item["name"] for item in items] == ["pose-band"]
    assert items[0]["source_line"] is None


def test_a_construct_query_answers_with_triples_and_a_select_keeps_its_shape(tmp_path):
    service = _service(tmp_path)
    kind, triples = service.query("CONSTRUCT { ?s ?p ?o } WHERE { ?s ?p ?o } LIMIT 5")

    assert kind == "CONSTRUCT"
    assert len(triples) == 5
    assert all(len(triple) == 3 for triple in triples)

    kind, (headers, rows) = service.query(SIGNALS)
    assert kind == "SELECT"
    assert headers == ["controller", "signal"]
    assert rows and all(len(row) == 2 for row in rows)


def _payload(tmp_path):
    service = _service(tmp_path)
    return service, provenance_graph(service)


def test_the_graph_payload_folds_type_edges_into_node_types(tmp_path):
    service, payload = _payload(tmp_path)
    by_id = {node["value"]: node for node in payload["nodes"]}

    assert not [link for link in payload["links"] if link["value"] == str(rdflib.RDF.type)]
    for subject, _p, obj, _g in service.dataset.quads((None, rdflib.RDF.type, None, None)):
        assert rdf_name(obj) in by_id[str(subject)]["types"]


def test_the_graph_payload_folds_literals_into_node_attributes(tmp_path):
    service, payload = _payload(tmp_path)
    by_id = {node["value"]: node for node in payload["nodes"]}
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


def test_the_legend_carries_every_type_the_graph_declares(tmp_path):
    service, payload = _payload(tmp_path)
    declared = {rdf_name(obj) for obj in service.dataset.objects(None, rdflib.RDF.type)}

    assert payload["types"]
    assert set(payload["types"]) == declared
    # Against the dataset's own graphs, not a fixed three: a JSON-LD file that declares a graph
    # of its own keeps that graph's name, so the allowed set is whatever was actually loaded.
    assert {name for node in payload["nodes"] for name in node["graphs"]} <= {
        graph_name(context) for context in service.dataset.graphs()
    }


def test_node_ids_are_stable_across_calls(tmp_path):
    """A second read names the same nodes: what the id identifies never moves under a reader.

    Which order they come back in does not: every call rebuilds the live overlay, and the
    nodes it puts back are ordered by a set the interpreter's hash seed decides. The force
    layout the ids are drawn by asks for no order, so nothing here asserts one.
    """
    service = _service(tmp_path)

    first = {node["id"] for node in provenance_graph(service)["nodes"]}
    second = {node["id"] for node in provenance_graph(service)["nodes"]}
    assert first == second
