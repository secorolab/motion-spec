# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import pytest
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.ir_gen import Parser, _scene_from_graph
from motion_spec.namespace import CSTR_HDL, ENV, GEOM_ENT, MJ, QUDT_QKIND, QUDT_SCHEMA, SIM, SLV


def _quantity(graph: Graph, name: str) -> URIRef:
    node = URIRef(f"https://example.test/{name}")
    graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((node, RDF.type, QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA.unit, URIRef("https://qudt.org/vocab/unit/N")))
    return node


def _pid_graph(*, kp: float | None = 1.0) -> tuple[Graph, URIRef]:
    graph = Graph()
    controller = URIRef("https://example.test/controller")
    graph.add((controller, RDF.type, CSTR_HDL.Controller))
    graph.add((controller, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
    graph.add((controller, CSTR_HDL["error-signal"], _quantity(graph, "error")))
    graph.add((controller, CSTR_HDL["control-signal"], _quantity(graph, "control")))
    if kp is not None:
        graph.add((controller, CSTR_HDL["proportional-gain"], Literal(kp, datatype=XSD.double)))
    return graph, controller


def test_parser_rejects_pid_missing_required_integral_gain() -> None:
    graph, controller_node = _pid_graph(kp=2.0)

    with pytest.raises(ValueError, match="integral_gain"):
        Parser(graph).controller(controller_node)


def test_parser_rejects_pid_missing_required_proportional_gain() -> None:
    graph, controller_node = _pid_graph(kp=None)

    with pytest.raises(ValueError, match="proportional_gain"):
        Parser(graph).controller(controller_node)


def test_scene_object_site_attach_target_is_prefixed_for_runtime_scene_name() -> None:
    graph = Graph()
    env = URIRef("https://example.test/env")
    table = URIRef("https://example.test/env/table")
    robot = URIRef("https://example.test/env/robot")
    robot_model = URIRef("https://example.test/assets/robot-model")
    chain = URIRef("https://example.test/env/robot.chain")

    graph.add((env, RDF.type, ENV.Workspace))
    graph.add((env, ENV["has-object"], table))
    graph.add((env, ENV["has-object"], robot))
    graph.add((table, RDF.type, ENV.RigidObject))
    graph.add((table, RDF.type, ENV.Object))
    graph.add((robot, RDF.type, ENV.RigidObject))
    graph.add((robot, ENV["has-object-model"], robot_model))
    graph.add((robot_model, SIM.path, Literal("robot.xml")))
    graph.add((robot, GEOM_ENT["kinematic-chain"], chain))
    graph.add((robot, MJ["attach-kind"], Literal("site")))
    graph.add((robot, MJ["attach-name"], Literal("table_top")))
    graph.add((robot, SLV["attached-to"], table))

    scene = _scene_from_graph(graph)

    assert scene.robots[0].attach_kind == "Site"
    assert scene.robots[0].attach_name == "table_table_top"


def test_uris_table_maps_each_id_to_full_uri() -> None:
    graph, controller_node = _pid_graph(kp=1.0)

    parser = Parser(graph)
    node_by_id = {parser.id(node): node for node in graph.subjects()}
    uris = {
        id_: str(node)
        for id_, node in sorted(node_by_id.items())
        if isinstance(node, URIRef)
    }

    assert uris[parser.id(controller_node)] == str(controller_node)
    assert uris[parser.id(controller_node)] == "https://example.test/controller"
    assert all(uri.startswith("https://example.test/") for uri in uris.values())


@pytest.mark.parametrize(
    "event_uri, expected_name",
    [
        ("http://example.org/coord/E_OBJ_REACHED", "E_OBJ_REACHED"),
        ("http://example.org/coord/e-step", "E_STEP"),
    ],
)
def test_edge_monitor_carries_full_event_uri_and_enum_token(event_uri: str, expected_name: str) -> None:
    graph = Graph()
    monitor = URIRef("https://example.test/mon")
    event = URIRef(event_uri)
    graph.add((monitor, RDF.type, CSTR_HDL.Monitor))
    graph.add((monitor, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    graph.add((monitor, CSTR_HDL.event, event))

    entry = Parser(graph).monitor_entry(monitor)

    assert entry.event_uri == event_uri
    # event_name is the coord-dsl FSM enum token (local name, upper-cased, '-' -> '_').
    assert entry.event_name == expected_name
