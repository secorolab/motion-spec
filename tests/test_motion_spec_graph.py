# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Integration checks for the reduced motion-spec RDF contract."""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest
from conftest import requires_interfaces, requires_workspace
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
from rdflib import Namespace
from rdflib.namespace import RDF, SDO
from support import DSL_MODELS, example, load_model

from motion_spec.generation.pipeline import generate_model
from motion_spec.rdf_parser.ir import generate_ir
from motion_spec.runs import provenance

MODELS = DSL_MODELS
FIXTURES = Path(__file__).parent / "fixtures"
METAMODELS = Path(
    os.environ.get("METAMODELS_PATH", Path(__file__).resolve().parents[2] / "metamodels")
)
pytestmark = [
    requires_interfaces("aruco_perception/action/LocateObjects"),
    requires_workspace(METAMODELS / "prov.shacl.ttl"),
]
PICK_AND_PLACE = example("pick_and_place") / "pick_and_place.robmot"
DUAL_ARM = example("dual_arm_pick_and_place") / "dual_arm_pick_and_place.robmot"


@pytest.fixture(scope="module")
def generation(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The representative model generated through the IR once per module."""
    generation = tmp_path_factory.mktemp("pick_and_place")
    generate_model(PICK_AND_PLACE, generation, stage="ir")
    return generation


def test_dual_arm_physical_profiles_and_path_progress_reach_ir(tmp_path: Path) -> None:
    model, fsm = load_model(DUAL_ARM, tmp_path)
    graph = model.graph
    assert len(set(graph.subjects(QUDT_SCHEMA.hasQuantityKind, QKIND_EXT.LinearJerk))) == 1
    # Each arm follows its own path, so each gets its own projection and its own local frame.
    projections = set(graph.subjects(RDF.type, GEOM_OP_EXT.PathProjection))
    assert len(projections) == 2
    assert len({graph.value(node, GEOM_OP_EXT.path) for node in projections}) == 2
    frames = set(graph.subjects(RDF.type, GEOM_OP_EXT.PathTangentFrame))
    assert len(frames) == 2
    assert len({graph.value(node, GEOM_OP_EXT.tangent) for node in frames}) == 2

    ir = generate_ir(model, fsm)
    profiles = [
        value
        for value in ir["computation"]["closures"].values()
        if value.get("type") == "VelocityProfile"
    ]

    # Two kinds, one pair per arm: a profile shaping a measured input, and a path driver that
    # owns its tangent speed and so reads none.
    motion_profiles = [profile for profile in profiles if profile["in"]]
    assert len(motion_profiles) == 2
    assert all(str(profile["shape"]) == "s_curve" for profile in motion_profiles)
    assert all(profile["maximum_jerk"] == "max_lower_jerk" for profile in motion_profiles)
    tangent_profiles = [profile for profile in profiles if not profile["in"]]
    assert len(tangent_profiles) == 2
    assert all(profile["profile_shape"] == "trapezoidal" for profile in tangent_profiles)
    pick_above = next(
        motion
        for motion in ir["coordination"]["motions"]
        if motion.motion_id == "motion_pick_above"
    )
    assert {entry["parameter"] for entry in pick_above.path_projections} == {
        "arm1_approach_path_s",
        "arm2_approach_path_s",
    }
    assert {entry["along_speed"] for entry in pick_above.path_projections} == {
        "arm1_approach_path_along_speed",
        "arm2_approach_path_along_speed",
    }


def test_generation_keeps_scene_fsm_and_provenance_separate(generation: Path) -> None:
    output = generation / "generated" / "model"
    assert (output / "pick_and_place.ld.json").exists()
    assert (output / "pick_and_place.scenex.ld.json").exists()
    assert (output / "pick_and_place.fsm.ld.json").exists()

    dataset = provenance.read_generation_dataset(
        generation / "generated" / provenance.GENERATION_DOCUMENT
    )
    activity = provenance.generation_scope(generation)["activity/jsonld_generation/pick_and_place"]
    prov = Namespace("http://www.w3.org/ns/prov#")
    assert (activity, RDF.type, prov.Activity) in dataset.graph(provenance.GRAPH_DSL)


def test_generation_documents_share_one_node_per_tool_and_per_file(generation: Path) -> None:
    """Every tool is one agent per release, and every file one node, across the tool graphs."""
    prov_ext = Namespace("https://secorolab.github.io/metamodels/prov#")
    prov = Namespace("http://www.w3.org/ns/prov#")
    dataset = provenance.read_generation_dataset(
        generation / "generated" / provenance.GENERATION_DOCUMENT
    )
    dsl = dataset.graph(provenance.GRAPH_DSL)
    motion_spec = dataset.graph(provenance.GRAPH_MOTION_SPEC)

    agents = set(dataset.subjects(RDF.type, prov.SoftwareAgent))
    assert {str(dataset.value(agent, SDO.name)) for agent in agents} == {
        "motion_spec_dsl",
        "motion_spec",
        "coord_dsl",
        "scene-dsl",
    }
    # The .fsm the DSL read and the one its FSM graph was built from are one node.
    fsm = provenance.generation_scope(generation)["entity/generated/source/pick_and_place.fsm"]
    assert fsm in set(dsl.objects(None, prov.used))
    assert fsm in set(motion_spec.objects(None, prov.used))

    # Turning one model into another is what these activities do; the class says so in both.
    assert set(dsl.subjects(RDF.type, prov_ext.Transformation))
    assert set(motion_spec.subjects(RDF.type, prov_ext.Transformation))


def test_non_pose_component_views_keep_their_subspace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Wrench and twist axes retain their authored non-pose subspaces."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    robmot = example("arc_tracing_with_admittance") / "arc_tracing_with_admittance.robmot"
    graph = load_model(robmot, tmp_path)[0].graph
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
    model = metamodel.model_from_file(FIXTURES / "perception" / "perception.robmot")
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    graph = dataset.default_graph

    # The subscription is a topic too, so the monitor is the one that fires an event.
    monitor = next(
        node
        for node in graph.subjects(RDF.type, ROS.Topic)
        if graph.value(node, CSTR_HDL.event) is not None
    )
    assert str(graph.value(monitor, ROS["channel-name"])) == "/perception/table_moved"
    assert str(graph.value(monitor, ROS["type-name"])) == "action_msgs/msg/GoalStatus"
    assert {p for _s, p, _o in graph if str(p).startswith(str(ROS))} == {
        ROS["channel-name"],
        ROS["type-name"],
        ROS["field-path"],
    }

    document = graph.serialize(format="json-ld", context=context)
    document = document.decode() if isinstance(document, bytes) else document
    assert '"ros": "https://index.ros.org/p/"' in document


def test_ir_derives_forwarded_commands_and_monitors(tmp_path: Path) -> None:
    model, fsm = load_model(PICK_AND_PLACE, tmp_path)
    ir = generate_ir(model, fsm)
    forwarded = [
        command for motion in ir["coordination"]["motions"] for command in motion.forwarded_commands
    ]
    assert len(forwarded) == 6
    assert all(command.target for command in forwarded)
    graph = model.graph
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
        motion
        for motion in ir["coordination"]["motions"]
        if motion.motion_id == "motion_pick_above"
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
    # exactly the 1 tangent + 2 normal + 3 angular controllers and the elbow's joint-space
    # hold; the guard appears only as a while monitor over
    # its along-speed error.
    assert [closure["type"] for closure in scheduled].count("Controller") == 7
    assert not any(
        "advance" in closure["id"] and closure["type"] == "Controller" for closure in scheduled
    )
    assert len(pick_above.while_monitors) == 1
    assert pick_above.while_monitors[0].monitor_type == "LevelTriggeredMonitor"
    # The path driver owns its tangent speed, so its profile runs once the projection has
    # measured the parameter and before the evaluator samples the curve there.
    (profile_call,) = [closure for closure in scheduled if closure["type"] == "VelocityProfile"]
    assert (
        scheduled.index(projection_call)
        < scheduled.index(profile_call)
        < scheduled.index(evaluator_call)
    )


def test_generated_manifest_is_portable(generation: Path) -> None:
    manifest = generation / "generated" / "model" / "pick_and_place-app.ld.json"
    text = json.dumps(json.loads(manifest.read_text()))
    assert str(manifest.parent) not in text
    assert "https://secorolab.github.io/" in text
