# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
from rdf_utils.models.vocab import URI_KC_TYPE_SERIAL
from rdf_utils.namespace import NS_MM_GEOM, NS_MM_KC_EXT
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.codegen import render_template
from motion_spec.entities import PIDController
from motion_spec.ir_gen import (
    _motion_done_terms,
    _validate_motion_progress_objectives,
    GuardedMotionBlock,
    Parser,
    SceneRobot,
    SceneSpec,
    _build_introspection,
    _fixed_attachments,
    _robot_setups_from_graph,
    ops_generic,
)
from motion_spec.namespace import (
    AGN,
    ALGO_EXT,
    CSTR,
    CSTR_HDL,
    EXEC,
    GEOM_COORD,
    KC,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SLV,
)


def test_fixed_attachments_root_a_branched_multi_robot_scene_at_world() -> None:
    graph = Graph()
    world = URIRef("https://example.test/world")
    table = URIRef("https://example.test/table")
    table_top = URIRef(f"{table}/top")
    trees = {
        URIRef("https://example.test/arm1"),
        URIRef("https://example.test/arm2"),
    }

    def fixed(name: str, frame_a: URIRef, frame_b: URIRef) -> None:
        joint = URIRef(f"https://example.test/{name}")
        graph.add((joint, RDF.type, KC.Joint))
        graph.add((joint, KC["between-attachments"], frame_a))
        graph.add((joint, KC["between-attachments"], frame_b))

    fixed("world-table", URIRef(f"{world}/origin"), table_top)
    for tree in trees:
        root = URIRef(f"{tree}/base")
        root_frame = URIRef(f"{root}/origin")
        tip = URIRef(f"{tree}/tip")
        tip_frame = URIRef(f"{tip}/origin")
        graph.add((tree, NS_MM_KC_EXT["tip"], tip_frame))
        fixed(f"{tree.rsplit('/', 1)[-1]}-table", table_top, root_frame)
        joint = URIRef(f"{tree}/moving")
        graph.add((joint, RDF.type, KC.Joint))
        graph.add((joint, RDF.type, KC.RevoluteJoint))
        graph.add((joint, KC["between-attachments"], root_frame))
        graph.add((joint, KC["between-attachments"], tip_frame))

    attachments, root = _fixed_attachments(graph, trees)

    assert root == world
    assert attachments[table][:2] == ("World", "")
    assert {attachments[URIRef(f"{tree}/base")][:2] for tree in trees} == {
        ("Site", "top")
    }
def _quantity(graph: Graph, name: str) -> URIRef:
    node = URIRef(f"https://example.test/{name}")
    graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((node, RDF.type, QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA.unit, URIRef("https://qudt.org/vocab/unit/N")))
    return node


def test_parser_scopes_repeated_nested_reference_ids() -> None:
    graph = Graph()
    graph.bind("example", "https://example.test/")
    first = URIRef("https://example.test/motion/Spec/spec/traj1/reference")
    second = URIRef("https://example.test/motion/Spec/spec/traj2/reference")
    graph.add((first, RDF.type, RDF.Property))
    graph.add((second, RDF.type, RDF.Property))

    parser = Parser(graph)

    assert parser.id(first) == "motion_traj1_reference"
    assert parser.id(second) == "motion_traj2_reference"


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


def test_agent_model_may_bind_the_assembled_kinematic_tree() -> None:
    graph = Graph()
    base = URIRef("https://example.test/arm/base")
    root = URIRef(f"{base}/root")
    tool = URIRef("https://example.test/gripper/tool")
    tcp = URIRef(f"{tool}/tcp")
    tree = URIRef("https://example.test/assembled")
    joint = URIRef(f"{tree}/fixed")
    agent = URIRef("https://example.test/robot")
    modelled = URIRef("https://example.test/modelled-robot")
    model = URIRef("https://example.test/robot-model")

    graph.add((modelled, RDF.type, AGN.ModelledAgent))
    graph.add((modelled, AGN["of-agent"], agent))
    graph.add((modelled, AGN["has-agent-model"], model))
    graph.add((model, EXEC["has-kinematic-tree"], tree))
    graph.add((model, EXEC.path, Literal("kinova_gen3.xml")))
    graph.add((tree, RDF.type, NS_MM_GEOM["KinematicTree"]))
    graph.add((tree, RDF.type, URI_KC_TYPE_SERIAL))
    graph.add((tree, NS_MM_KC_EXT["root"], root))
    graph.add((tree, NS_MM_KC_EXT["tip"], tcp))
    graph.add((joint, RDF.type, KC.Joint))
    graph.add((joint, KC["between-attachments"], root))
    graph.add((joint, KC["between-attachments"], tcp))

    setups, _ordered = _robot_setups_from_graph(graph)

    assert setups[agent][1:7] == ("base", "tool", "tool", "KinovaGen3", "", "")


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
    controller = PIDController(
        id=parser.id(controller_node),
        control_signal=parser.quantity(graph.value(controller_node, CSTR_HDL["control-signal"])),
        error_signal=parser.quantity(graph.value(controller_node, CSTR_HDL["error-signal"])),
        measured_derivative=None,
        proportional_gain=2.0,
        integral_gain=0.1,
        derivative_gain=0.3,
        decay_rate=None,
        output_saturation=None,
        integral_saturation=None,
        type=parser.id(CSTR_HDL.ProportionalIntegralDerivative),
    )
    monitor = parser.monitor_entry(monitor_node)
    motion = GuardedMotionBlock(
        id="move",
        handler="move_handler",
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
        imported_provenance=["/tmp/generated/provenance/dsl.ld.json"],
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
        and entity["source"] == "/tmp/generated/provenance/dsl.ld.json"
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
    assert "exec:Simulation" in agents["agent:runtime:mujoco"]["types"]
    assert "agn:ModelledAgent" in agents["agent:modelled:robot"]["types"]


def test_velocity_profile_operator_closure_exposes_codegen_fields() -> None:
    graph = Graph()
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, ALGO_EXT.VelocityProfile))
    for pred, name in (
        (ALGO_EXT["target"], "goal"),
        (ALGO_EXT["in"], "measured-velocity"),
        (ALGO_EXT["maximum-velocity"], "max-velocity"),
        (ALGO_EXT["maximum-acceleration"], "max-acceleration"),
        (ALGO_EXT["maximum-jerk"], "max-jerk"),
        (ALGO_EXT["out"], "reference"),
    ):
        graph.add((op, pred, URIRef(f"https://example.test/{name}")))
    constraint = URIRef("https://example.test/constraint")
    controller = URIRef("https://example.test/controller")
    graph.add((constraint, CSTR["reference-value"], URIRef("https://example.test/reference")))
    # The value the profile starts from is the constraint's own quantity.
    graph.add((constraint, CSTR.quantity, URIRef("https://example.test/measured")))
    graph.add((controller, CSTR_HDL.constraint, constraint))
    graph.add((op, ALGO_EXT["shape"], ALGO_EXT["s-curve"]))

    closure = Parser(graph).closures(ops_generic)["profile_op"]

    assert closure["type"] == "VelocityProfile"
    assert closure["goal"] == "goal"
    assert closure["measured"] == "measured"
    assert closure["in"] == "measured_velocity"
    assert closure["maximum_velocity"] == "max_velocity"
    assert closure["maximum_acceleration"] == "max_acceleration"
    assert closure["maximum_jerk"] == "max_jerk"
    assert closure["out"] == "reference"
    assert closure["controller"] == "controller"
    assert str(closure["shape"]) == "s_curve"


def test_velocity_profile_without_controller_fails_clearly() -> None:
    graph = Graph()
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, ALGO_EXT.VelocityProfile))
    graph.add((op, ALGO_EXT["out"], URIRef("https://example.test/reference")))

    with pytest.raises(ValueError, match="not bound to a constraint"):
        Parser(graph).closures(ops_generic)


def test_progress_without_derived_tracking_error_fails_before_codegen() -> None:
    motion = SimpleNamespace(
        id="motion_path",
        progress_objectives=[SimpleNamespace(id="progress_path_0", errors=[])],
    )

    with pytest.raises(ValueError, match="no derived tracking-controller error signal"):
        _validate_motion_progress_objectives([motion])


def test_solver_ir_carries_rne_algorithm_and_gravity() -> None:
    graph = Graph()
    solver = URIRef("https://example.test/solver")
    gravity = URIRef("https://example.test/gravity")
    graph.add((solver, RDF.type, SLV.SolverWithInputAndOutput))
    graph.add((solver, SLV.solver, SLV.RecursiveNewtonEulerAlgorithm))
    graph.add((solver, SLV.gravity, gravity))
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

    KDL::Chain prefixed_chain;
    prefixed_chain.addSegment(KDL::Segment("r2_link_1"));
    assert(motion_spec::runtime::find_segment_index(
               prefixed_chain, "base_link", "r2_base_link") == 0);
    assert(motion_spec::runtime::find_segment_index(
               prefixed_chain, "link_1", "r2_base_link") == 1);

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

    double progress = 0.0; // activation reset
    assert(motion_spec::runtime::path_progress_step(
        progress, false, 1.0, motion_spec::runtime::kControlPeriodS) == 0.0);
    double previous = progress;
    for (int i = 0; i < 2000; ++i) {
        progress = motion_spec::runtime::path_progress_step(
            progress, true, 1.0, motion_spec::runtime::kControlPeriodS);
        assert(progress >= previous && progress <= 1.0);
        previous = progress;
    }
    assert(progress == 1.0);
    assert(motion_spec::runtime::path_progress_step(
        progress, false, 1.0, motion_spec::runtime::kControlPeriodS) == 1.0);
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
    boolean flag), with no path-parameter coupling. The any/all/paren/empty folding is
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
    # Path-parameter completion is NOT folded in: only event/flag member terms.
    assert all(t["kind"] in ("event", "flag") for t in terms)

    # No UNTIL monitors: no stop terms -> the template renders the "true" default.
    assert _motion_done_terms({"id": "m", "until_monitors": []}) == []
