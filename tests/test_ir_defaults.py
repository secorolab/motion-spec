# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import shutil
import subprocess

import pytest
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.codegen import render_template
from motion_spec.ir_gen import Parser, _scene_from_graph, ops_generic
from motion_spec.namespace import (
    CSTR_HDL,
    CSTR_HDL_EXT,
    ENV,
    EXEC,
    GEOM_ENT,
    GEOM_COORD,
    MJ,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)


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
    graph.add((robot_model, EXEC.path, Literal("robot.xml")))
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


def test_velocity_profile_operator_closure_exposes_codegen_fields() -> None:
    graph = Graph()
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, CSTR_HDL_EXT.VelocityProfile))
    for pred, name in (
        (CSTR_HDL_EXT["goal"], "goal"),
        (CSTR_HDL_EXT["measured"], "measured"),
        (CSTR_HDL_EXT["measured-velocity"], "measured-velocity"),
        (CSTR_HDL_EXT["max-velocity"], "max-velocity"),
        (CSTR_HDL_EXT["max-acceleration"], "max-acceleration"),
        (CSTR_HDL_EXT["max-jerk"], "max-jerk"),
        (CSTR_HDL_EXT["reference"], "reference"),
        (CSTR_HDL_EXT["controller"], "controller"),
    ):
        graph.add((op, pred, URIRef(f"https://example.test/{name}")))
    graph.add((op, CSTR_HDL_EXT["shape"], Literal("SCurve")))

    closure = Parser(graph).closures(ops_generic)["profile_op"]

    assert closure["type"] == "VelocityProfile"
    assert closure["goal"] == "goal"
    assert closure["measured"] == "measured"
    assert closure["measured_velocity"] == "measured_velocity"
    assert closure["max_velocity"] == "max_velocity"
    assert closure["max_acceleration"] == "max_acceleration"
    assert closure["max_jerk"] == "max_jerk"
    assert closure["reference"] == "reference"
    assert closure["controller"] == "controller"
    assert str(closure["shape"]) == "SCurve"


def test_solver_ir_carries_rne_algorithm_and_gravity() -> None:
    graph = Graph()
    solver = URIRef("https://example.test/solver")
    gravity = URIRef("https://example.test/gravity")
    graph.add((solver, RDF.type, SLV.SolverWithInputAndOutput))
    graph.add((solver, SLV.solver, SLV.RecursiveNewtonEulerAlgorithm))
    graph.add((solver, SLV_EXT["gravity-value"], gravity))
    graph.add((gravity, GEOM_COORD["x"], Literal(0.0, datatype=XSD.double)))
    graph.add((gravity, GEOM_COORD["y"], Literal(0.0, datatype=XSD.double)))
    graph.add((gravity, GEOM_COORD["z"], Literal(-9.81, datatype=XSD.double)))

    entry = Parser(graph).solver_with_input_and_output(solver)

    assert entry.algorithm == "RNE"
    assert entry.algorithm_is_rne is True
    assert entry.gravity == [0.0, 0.0, -9.81]


def test_generated_velocity_profile_runtime_respects_authored_bounds(tmp_path) -> None:
    if shutil.which("stst") is None or shutil.which("c++") is None:
        pytest.skip("requires stst and c++")

    payload = tmp_path / "ir.json"
    payload.write_text(json.dumps({"has_mobile_base": False, "control_period_ns": 1_000_000}))
    runtime_hpp = tmp_path / "runtime.hpp"
    render_template("stst", "runtime_header", payload, runtime_hpp)

    source = tmp_path / "check.cpp"
    source.write_text(
        r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include "runtime.hpp"

void check(motion_spec::runtime::VelocityProfileShape shape) {
    constexpr double dt = motion_spec::runtime::kControlPeriodS;
    constexpr double vmax = 0.10;
    constexpr double amax = 0.30;
    constexpr double jmax = 2.0;
    constexpr double goal = 0.08;
    double x = 0.50;
    double v = 0.0;
    double a = 0.0;
    double prev_a = 0.0;
    for (int i = 0; i < 5000; ++i) {
        const double before = x;
        x = motion_spec::runtime::velocity_profile_step(x, v, a, goal, vmax, amax, jmax, dt, shape);
        assert(std::abs(v) <= vmax + 1e-9);
        assert(std::abs(a) <= amax + 1e-9);
        if (shape == motion_spec::runtime::VelocityProfileShape::SCurve && !(x == goal && v == 0.0 && a == 0.0)) {
            assert(std::abs(a - prev_a) <= jmax * dt + 1e-9);
        }
        assert((goal - x) * (goal - before) >= -1e-12 || x == goal);
        if (x == goal) break;
        prev_a = a;
    }
    assert(x == goal);
}

int main() {
    check(motion_spec::runtime::VelocityProfileShape::Trapezoidal);
    check(motion_spec::runtime::VelocityProfileShape::SCurve);

    // Seeded from a nonzero measured velocity (the online-generator initial
    // condition set in the controller init): still respects bounds and converges.
    double x = 0.50;
    double v = -0.05;
    double a = 0.0;
    for (int i = 0; i < 5000; ++i) {
        x = motion_spec::runtime::velocity_profile_step(
            x, v, a, 0.08, 0.10, 0.30, 2.0, motion_spec::runtime::kControlPeriodS,
            motion_spec::runtime::VelocityProfileShape::Trapezoidal);
        assert(std::abs(v) <= 0.10 + 1e-9);
        if (x == 0.08) break;
    }
    assert(x == 0.08);
}
'''
    )
    exe = tmp_path / "check"
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-I",
            str(tmp_path),
            "-I",
            "/usr/include/eigen3",
            str(source),
            "-lorocos-kdl",
            "-o",
            str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True)


def test_generated_runtime_resolves_constraint_acceleration(tmp_path) -> None:
    if shutil.which("stst") is None or shutil.which("c++") is None:
        pytest.skip("requires stst and c++")

    payload = tmp_path / "ir.json"
    payload.write_text(json.dumps({"has_mobile_base": False, "control_period_ns": 1_000_000}))
    runtime_hpp = tmp_path / "runtime.hpp"
    render_template("stst", "runtime_header", payload, runtime_hpp)

    source = tmp_path / "check_rne.cpp"
    source.write_text(
        r'''
#include <cassert>
#include <cmath>
#include "runtime.hpp"

int main() {
    KDL::Jacobian jac(1);
    KDL::Jacobian alpha(1);
    KDL::JntArray e_acc(1);
    KDL::JntArray qdd(1);
    KDL::SetToZero(jac);
    KDL::SetToZero(alpha);
    jac.data(0, 0) = 1.0;
    alpha.data(0, 0) = 1.0;
    e_acc(0) = 2.0;
    motion_spec::runtime::resolve_constraint_acceleration(jac, alpha, e_acc, nullptr, 1e-9, qdd);
    assert(std::abs(qdd(0) - 2.0) < 1e-6);
}
'''
    )
    exe = tmp_path / "check_rne"
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-I",
            str(tmp_path),
            "-I",
            "/usr/include/eigen3",
            str(source),
            "-lorocos-kdl",
            "-o",
            str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True)


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
