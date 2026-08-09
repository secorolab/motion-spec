# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Integration checks for the reduced motion-spec RDF contract."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from motion_spec_dsl.gens import _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.model import ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import (
    CSTR,
    CSTR_HDL,
    GEOM_OP_EXT,
    MAP,
    MAP_EXT,
    QKIND_EXT,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)
from rdflib import Graph, Namespace, URIRef
from rdflib.namespace import RDF

from motion_spec.rdf_parser.ir import generate_ir
from motion_spec.rdf_parser.model import load_model

from conftest import requires_interfaces

pytestmark = requires_interfaces("aruco_perception/action/LocateObjects")

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"


@pytest.fixture(scope="module")
def generated_model(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the representative model once per module; consumers load their own
    fresh graph from the immutable result and must not mutate the manifest on disk."""
    tmp_path = tmp_path_factory.mktemp("pick_place_single")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(MODELS / "pick_place_single" / "pick_place_single.robmot")
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single-app.ld.json"


@pytest.fixture(scope="module")
def generated_dual_model(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the dual-arm model once per module; consumers load their own fresh
    graph from the immutable result and must not mutate the manifest on disk."""
    tmp_path = tmp_path_factory.mktemp("pick_place_dual")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(MODELS / "pick_place_dual" / "pick_place_dual.robmot")
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_dual-app.ld.json"


def test_dual_arm_physical_profiles_and_path_progress_reach_ir(generated_dual_model: Path) -> None:
    graph = load_model(generated_dual_model).graph
    assert len(set(graph.subjects(QUDT_SCHEMA.hasQuantityKind, QKIND_EXT.LinearJerk))) == 1
    # Each arm follows its own path, so each gets its own projection and its own local frame.
    projections = set(graph.subjects(RDF.type, GEOM_OP_EXT.PathProjection))
    assert len(projections) == 2
    assert len({graph.value(node, GEOM_OP_EXT.path) for node in projections}) == 2
    frames = set(graph.subjects(RDF.type, GEOM_OP_EXT.PathTangentFrame))
    assert len(frames) == 2
    assert len({graph.value(node, GEOM_OP_EXT.tangent) for node in frames}) == 2

    ir = generate_ir(generated_dual_model)
    profiles = [
        value
        for value in ir["computation"]["closures"].values()
        if value.get("type") == "VelocityProfile"
    ]

    motion_profiles = profiles
    assert len(motion_profiles) == 2
    assert all(str(profile["shape"]) == "s_curve" and profile["in"] for profile in motion_profiles)
    assert all(profile["maximum_jerk"] == "max_lower_jerk" for profile in motion_profiles)
    pick_above = next(
        motion for motion in ir["coordination"]["motions"] if motion.id == "motion_pick_above"
    )
    assert {entry["parameter"] for entry in pick_above.path_projections} == {
        "arm1_approach_path_s",
        "arm2_approach_path_s",
    }
    assert {entry["along_speed"] for entry in pick_above.path_projections} == {
        "arm1_approach_path_along_speed",
        "arm2_approach_path_along_speed",
    }


def test_generation_keeps_scene_fsm_and_provenance_separate(generated_model: Path) -> None:
    output = generated_model.parent
    assert (output / "pick_place_single.ld.json").exists()
    assert (output / "pick_place_single.scenex.ld.json").exists()
    header = output / "pick_place_single.kdl.hpp"
    assert header.exists()
    assert "make_tree_" in header.read_text()
    assert (output / "pick_place_single_fsm.ld.json").exists()

    provenance = Graph().parse(output / "provenance" / "dsl.ld.json", format="json-ld")
    prov = Namespace("http://www.w3.org/ns/prov#")
    activity = URIRef(
        "https://secorolab.github.io/motion-spec-dsl/provenance/"
        "activity/jsonld_generation/pick_place_single"
    )
    assert (activity, RDF.type, prov.Activity) in provenance


def test_non_pose_component_views_keep_their_subspace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Wrench and twist axes retain their authored non-pose subspaces."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(
        MODELS / "admittance_arc_single" / "admittance_arc_single.robmot"
    )
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    graph = load_model(tmp_path / "admittance_arc_single-app.ld.json").graph
    wrench_views = set(graph.subjects(RDF.type, MAP_EXT.WrenchCoordinateView))
    twist_views = set(graph.subjects(RDF.type, MAP_EXT.VelocityTwistCoordinateView))
    assert wrench_views and twist_views
    assert {graph.value(view, MAP.subspace) for view in wrench_views} == {MAP.force}
    assert {graph.value(view, MAP.subspace) for view in twist_views} == {MAP["linear-velocity"]}


def test_monitor_publishes_to_ros_topic(monkeypatch: pytest.MonkeyPatch) -> None:
    """A monitor's `publish` marks it as a ROS topic naming a channel, a message type, and one
    field-path row per authored assignment -- the only ROS vocabulary the graph carries."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(
        MODELS / "admittance_arc_single" / "admittance_arc_single.robmot"
    )
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    graph = dataset.default_graph

    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    assert graph.value(monitor, CSTR_HDL.event) is not None
    assert str(graph.value(monitor, ROS["channel-name"])) == "/motion/forward_done"
    assert str(graph.value(monitor, ROS["type-name"])) == "bdd_ros2_interfaces/msg/TrinaryStamped"
    assert {p for _s, p, _o in graph if str(p).startswith(str(ROS))} == {
        ROS["channel-name"],
        ROS["type-name"],
        ROS["field-path"],
    }

    document = graph.serialize(format="json-ld", context=context)
    document = document.decode() if isinstance(document, bytes) else document
    assert '"ros": "https://index.ros.org/p/"' in document


def test_ir_derives_forwarded_commands_and_monitors(generated_model: Path) -> None:
    ir = generate_ir(generated_model)
    forwarded = [
        command for motion in ir["coordination"]["motions"] for command in motion.forwarded_commands
    ]
    assert len(forwarded) == 6
    assert all(command.target for command in forwarded)
    graph = load_model(generated_model).graph
    # The progress guard is a lower bound on the measured speed along the path, so it names
    # the same path as the projection and never produces the parameter itself.
    (guard,) = [
        subject
        for subject in graph.subjects(GEOM_OP_EXT.path, None)
        if CSTR.GreaterThanConstraint in graph[subject : RDF.type]
    ]
    (projection,) = list(graph.subjects(RDF.type, GEOM_OP_EXT.PathProjection))
    (along,) = list(graph.subjects(RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong))
    assert graph.value(guard, GEOM_OP_EXT.path) == graph.value(projection, GEOM_OP_EXT.path)
    assert graph.value(guard, CSTR.quantity) == graph.value(along, GEOM_OP_EXT["along-speed"])
    assert CSTR.GreaterThanConstraint in graph[guard : RDF.type]
    forwarding_solvers = set(graph.subjects(RDF.type, SLV_EXT.CommandForwardingSolver))
    assert forwarding_solvers
    assert all(graph.value(solver, SLV.output) is not None for solver in forwarding_solvers)
    assert all(
        graph.value(monitor, CSTR_HDL.constraint) is not None
        for monitor in graph.subjects(RDF.type, CSTR_HDL.Monitor)
    )

    pick_above = next(
        motion for motion in ir["coordination"]["motions"] if motion.id == "motion_pick_above"
    )
    scheduled = [ir["computation"]["closures"][step] for step in pick_above.while_schedule]
    # The parameter is measured by the projection; the pose the motion tracks is the
    # evaluator sampling the curve there.
    projection_call = next(closure for closure in scheduled if closure["type"] == "PathProjection")
    evaluator_call = next(closure for closure in scheduled if closure["type"] == "PathEvaluator")
    assert projection_call["shape"] == "LinearPath"
    assert (evaluator_call["setpoint"], projection_call["path_parameter"]) == (
        "reference",
        "approach_path_s",
    )
    assert evaluator_call["path_parameter"] == projection_call["path_parameter"]
    # Only the projection writes the shared goal; it is scheduled before the others read it.
    assert projection_call["assign_goal"]
    assert not evaluator_call.get("assign_goal")
    assert scheduled.index(projection_call) < scheduled.index(evaluator_call)
    assert any(closure["type"] == "PoseDiffEvaluator" for closure in scheduled)
    assert any(component["id"] == "goal_pose" for component in pick_above.declared_pose_components)
    # The progress guard is a monitored condition, never a solver row: the schedule holds
    # exactly the 1 tangent + 2 normal + 3 angular controllers, and the guard appears only as
    # a while monitor over its along-speed error.
    assert [closure["type"] for closure in scheduled].count("Controller") == 6
    assert not any(
        "advance" in closure["id"] and closure["type"] == "Controller" for closure in scheduled
    )
    assert len(pick_above.while_monitors) == 1
    assert pick_above.while_monitors[0].monitor_type == "LevelTriggeredMonitor"
    assert all(closure["type"] != "VelocityProfile" for closure in scheduled)


def test_generated_manifest_is_portable(generated_model: Path) -> None:
    document = json.loads(generated_model.read_text())
    text = json.dumps(document)
    assert str(generated_model.parent) not in text
    assert "https://secorolab.github.io/" in text
