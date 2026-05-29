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


def test_parser_preserves_missing_optional_pid_gains() -> None:
    graph, controller_node = _pid_graph(kp=2.0)

    controller = Parser(graph).controller(controller_node)

    assert controller.proportional_gain == 2.0
    assert controller.integral_gain is None
    assert controller.derivative_gain is None


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
