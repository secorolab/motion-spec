# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.codegen import render_template
from motion_spec.ir_gen import (
    _motion_done_terms,
    GuardedMotionBlock,
    Parser,
    SceneRobot,
    SceneSpec,
    _build_introspection,
    _scene_from_graph,
    ops_cstr_hdl,
    ops_generic,
)
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
    TRAJ,
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


def test_parser_reads_pid_measured_derivative() -> None:
    graph, controller_node = _pid_graph(kp=1.0)
    graph.add((controller_node, CSTR_HDL["integral-gain"], Literal(0.0, datatype=XSD.double)))
    graph.add((controller_node, CSTR_HDL["derivative-gain"], Literal(1.0, datatype=XSD.double)))
    measured_derivative = _quantity(graph, "measured_derivative")
    graph.add((controller_node, CSTR_HDL_EXT["measured-derivative"], measured_derivative))

    parser = Parser(graph)
    controller = parser.controller(controller_node)
    closure = parser.closures(ops_generic + ops_cstr_hdl)["controller"]

    assert controller.measured_derivative is not None
    assert controller.measured_derivative.id == "measured_derivative"
    assert closure["measured_derivative"] == "measured_derivative"


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


def test_scene_ids_are_scoped_when_local_names_collide() -> None:
    graph = Graph()
    graph.bind("demo1", "https://example.test/models/demo1/")
    graph.bind("demo2", "https://example.test/models/demo2/")
    env = URIRef("https://example.test/env")
    first = URIRef("https://example.test/models/demo1/pick_object")
    second = URIRef("https://example.test/models/demo2/pick_object")

    graph.add((env, RDF.type, ENV.Workspace))
    for node in (first, second):
        graph.add((env, ENV["has-object"], node))
        graph.add((node, RDF.type, ENV.RigidObject))
        graph.add((node, RDF.type, ENV.Object))

    scene = _scene_from_graph(graph)

    assert {obj.id for obj in scene.objects} == {"demo1_pick_object", "demo2_pick_object"}


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


def test_introspection_contract_carries_control_and_provenance() -> None:
    graph, controller_node = _pid_graph(kp=2.0)
    graph.add((controller_node, CSTR_HDL["integral-gain"], Literal(0.1, datatype=XSD.double)))
    graph.add((controller_node, CSTR_HDL["derivative-gain"], Literal(0.3, datatype=XSD.double)))
    monitor_node = URIRef("https://example.test/monitor")
    event_node = URIRef("https://example.test/events/complete")
    graph.add((monitor_node, RDF.type, CSTR_HDL.Monitor))
    graph.add((monitor_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    graph.add((monitor_node, CSTR_HDL.event, event_node))

    parser = Parser(graph)
    controller = parser.controller(controller_node)
    monitor = parser.monitor_entry(monitor_node)
    motion = GuardedMotionBlock(
        id="move",
        handler="move_handler",
        control_mode="JointTorque",
        when_evaluators=[],
        while_evaluators=[],
        until_evaluators=[],
        controllers=[controller],
        when_monitors=[],
        while_monitors=[],
        until_monitors=[monitor],
        when_schedule=[],
        while_schedule=[],
        until_schedule=[],
    )

    introspection = _build_introspection(
        app_model_path=Path("/tmp/app.json"),
        imported_models=["https://example.test/imported.json"],
        imported_provenance=["/tmp/generated/provenance/dsl.jsonld"],
        id_nodes=[(parser.id(node), node) for node in graph.subjects()],
        node_by_id={parser.id(node): node for node in graph.subjects()},
        motions=[motion],
        data_structures=[controller.error_signal, controller.control_signal],
        control_period_ns=2_000_000,
        backend="mj_kdl",
        scene=SceneSpec(robots=[SceneRobot(id="robot", path="robot.xml")]),
        closures={},
        views={},
        shared_data=[],
    )

    assert introspection["contract_version"] == 1
    assert introspection["controllers"][0]["proportional_gain"] == 2.0
    assert introspection["controllers"][0]["output_signal"] == "control"
    assert introspection["monitors"][0]["trigger"] == "edge"
    assert introspection["monitors"][0]["event_uri"] == "https://example.test/events/complete"
    assert {"id": "control", "uri": "https://example.test/control"} in introspection["uris"]
    assert any(entity["role"] == "app_manifest" for entity in introspection["provenance"]["entities"])
    assert any(
        entity["role"] == "imported_provenance"
        and entity["source"] == "/tmp/generated/provenance/dsl.jsonld"
        for entity in introspection["provenance"]["entities"]
    )
    assert any(
        activity["wasAssociatedWith"] == "agent:motion_spec_ir_gen"
        for activity in introspection["provenance"]["activities"]
    )
    runtime_activity = next(
        activity
        for activity in introspection["provenance"]["activities"]
        if activity["id"] == "activity:controller_execution"
    )
    assert runtime_activity["role"] == "controller_execution"
    assert runtime_activity["wasAssociatedWith"] == "agent:controller_process"
    assert "bdd:SimulatedExecution" in runtime_activity["types"]
    agents = {agent["id"]: agent for agent in introspection["provenance"]["agents"]}
    assert {
        "agent:runtime:mujoco",
        "agent:controller_process",
        "agent:modelled:robot",
    } <= set(agents)
    assert "rt:MuJoCoRuntime" in agents["agent:runtime:mujoco"]["types"]
    assert "agn:ModelledAgent" in agents["agent:modelled:robot"]["types"]


def test_velocity_profile_operator_closure_exposes_codegen_fields() -> None:
    graph = Graph()
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, CSTR_HDL_EXT.VelocityProfile))
    for pred, name in (
        (CSTR_HDL_EXT["goal"], "goal"),
        (CSTR_HDL_EXT["measured"], "measured"),
        (TRAJ["measured-velocity"], "measured-velocity"),
        (TRAJ["max-velocity"], "max-velocity"),
        (TRAJ["max-acceleration"], "max-acceleration"),
        (TRAJ["max-jerk"], "max-jerk"),
        (CSTR_HDL_EXT["reference"], "reference"),
        (CSTR_HDL_EXT["controller"], "controller"),
    ):
        graph.add((op, pred, URIRef(f"https://example.test/{name}")))
    graph.add((op, TRAJ["shape"], Literal("SCurve")))

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
    assert entry.root_acc == [0.0, 0.0, -9.81]


def test_generated_velocity_profile_runtime_respects_authored_bounds(tmp_path) -> None:
    if shutil.which("stst") is None or shutil.which("c++") is None:
        pytest.skip("requires stst and c++")

    payload = tmp_path / "ir.json"
    payload.write_text(
        json.dumps(
            {
                "has_mobile_base": False,
                "control_period_ns": 1_000_000,
                "rne_damping_lambda": 0.05,
                "beta_max_lin": 1e6,
                "beta_max_rot": 1e6,
                "tau_max_override": None,
            }
        )
    )
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

    // Passing an authored derivative of zero must still mean "use the authored
    // derivative", not "fall back to numerical error differentiation".
    motion_spec::runtime::PIDControl pid(0.0, 0.0, 1.0, 0.0, 1.0);
    assert(pid.control(1.0, 0.0) == 0.0);
    assert(pid.control(2.0, 0.0) == 0.0);
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
    payload.write_text(
        json.dumps(
            {
                "has_mobile_base": False,
                "control_period_ns": 1_000_000,
                "rne_damping_lambda": 0.05,
                "beta_max_lin": 1e6,
                "beta_max_rot": 1e6,
                "tau_max_override": None,
            }
        )
    )
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


def test_done_terms_are_until_only_event_or_flag() -> None:
    """done_terms are the UNTIL members alone (edge monitor -> event flag, level monitor ->
    boolean flag), with no trajectory-alpha coupling. The any/all/paren/empty folding is
    done by the bool-condition template and covered by the codegen golden diff."""
    monitors = [
        {"id": "mon_a", "is_edge_triggered": True},
        {"id": "mon_b", "flag": "flag_b"},
    ]

    terms = _motion_done_terms({"id": "m", "until_monitors": monitors})
    assert terms == [
        {"kind": "event", "motion_id": "m", "monitor_id": "mon_a"},
        {"kind": "flag", "motion_id": "m", "flag": "flag_b"},
    ]
    # Trajectory completion is NOT folded in: only event/flag member terms.
    assert all(t["kind"] in ("event", "flag") for t in terms)

    # No UNTIL monitors: no stop terms -> the template renders the "true" default.
    assert _motion_done_terms({"id": "m", "until_monitors": []}) == []
