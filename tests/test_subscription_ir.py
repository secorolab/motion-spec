# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Lowering a standing subscription: which topics the model reads poses off, how one message is
read, and who the blackboard says wrote the pose.

Direction is derived, not declared: a channel the model reads states the features of interest it
informs the model about, where a published one carries field rows and none. The graph cases build
the topic by hand; the message case reads a real type, so it needs the workspace. The model cases
lower the `perception` fixture, where the whole chain -- subscription -> written pose -> the chain
that no longer computes it -- can be read at once.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, SOSA, split_uri
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.motion import BlackboardValue
from motion_spec.generation.pipeline import generate_model
from motion_spec.rdf_parser import communication, quantities
from motion_spec.rdf_parser.coordination import observation_shape
from motion_spec.rdf_parser.model import Model

from conftest import requires_interfaces

NS = "https://example.test/"
TYPE_NAME = "vision_msgs/msg/Detection3DArray"
MODEL = Path(__file__).parent / "fixtures" / "perception"
SCENE = Path(__file__).parents[2] / "motion-spec-dsl" / "models" / "admittance_arc_single"


def _model(graph: Graph) -> Model:
    return Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _topic(graph: Graph, *observes: str) -> URIRef:
    """A topic node, reading the objects it is given and publishing when given none."""
    node = URIRef(f"{NS}object-poses")
    graph.add((node, RDF.type, NS_MM_ROS["Topic"]))
    graph.add((node, NS_MM_ROS["channel-name"], Literal("/perception/objects")))
    graph.add((node, NS_MM_ROS["type-name"], Literal(TYPE_NAME)))
    graph.add((node, NS_MM_ROS["field-path"], Literal("results.pose")))
    for target in observes:
        graph.add((node, SOSA.hasFeatureOfInterest, URIRef(target)))
    return node


def test_a_topic_stating_no_feature_of_interest_is_one_the_model_publishes():
    """It says nothing about the world it wants told, so nothing subscribes to it."""
    graph = Graph()
    node = _topic(graph)
    model = _model(graph)
    assert quantities.perceived_written_poses(model)[str(node)] == []
    assert communication.ros_subscriptions(model, {}) == []


def test_a_topic_observing_an_object_is_rejected():
    """A subscription writes a declared world pose, not an object it happens to describe."""
    graph = Graph()
    _topic(graph, f"{NS}pick_place_graph/cube/cube_origin")
    with pytest.raises(ConstraintViolation, match="PoseCoordinate"):
        quantities.perceived_written_poses(_model(graph))


@requires_interfaces(TYPE_NAME)
def test_a_subscription_reads_every_path_off_the_message_it_names():
    """Only the pose is stated by the model, because only it is ambiguous; and a message standing
    on its own carries no goal, so nothing in it names the targets."""
    shape = observation_shape(TYPE_NAME, "results.pose")
    assert shape["detections_path"] == "detections"
    assert shape["id_path"] == "id"
    assert shape["frame_path"] == "header.frame_id"
    assert shape["pose_path"] == "results[0].pose.pose"
    assert shape["pose_container"] == "results"
    assert "targets_path" not in shape
    assert shape["cpp_type"] == "vision_msgs::msg::Detection3DArray"
    assert shape["include"] == "vision_msgs/msg/detection3_d_array.hpp"
    # What the build has to find, not merely the package the model named.
    assert "vision_msgs" in shape["packages"]


def test_a_subscribed_pose_names_the_subscription_as_its_producer():
    """Written from the executor thread rather than inside a motion's step, so it is live in every
    state -- and the artifact says which mechanism produced it, not merely that ROS did."""
    shared_data = [BlackboardValue(id="pose_cube_base", type="Pose")]
    subscriptions = [{"sub_id": "object_poses", "written_poses": [{"pose_id": "pose_cube_base"}]}]
    introspection: dict = {}
    quantities.annotate_dataflow(introspection, shared_data, {}, [], [], {}, subscriptions)
    assert introspection["dataflow"]["pose_cube_base"] == {
        "producer": {"kind": "subscription", "id": "object_poses"},
        "cadence": "tick",
        "storage": "log",
    }


@pytest.fixture(scope="module")
def subscription_ir(tmp_path_factory) -> dict:
    generation = tmp_path_factory.mktemp("subscription_generation")
    generated = generate_model(MODEL / "perception.robmot", generation, stage="ir")
    return json.loads((generated / "model" / "ir.json").read_text())


@requires_interfaces(TYPE_NAME)
def test_the_subscription_names_the_poses_it_writes_and_the_frame_they_must_arrive_in(
    subscription_ir,
):
    (subscription,) = subscription_ir["communication"]["ros"]["subscriptions"]
    assert subscription["channel"] == "/recognized_objects"
    assert subscription["cpp_type"] == "vision_msgs::msg::Detection3DArray"
    written = subscription["written_poses"]
    assert {row["pose_id"] for row in written} == {"pose_table_cam"}
    # The frame the pose is stated against. What frame a detection arrives in is the sender's to
    # say, so it is read off the header at run time and is not here.
    assert {row["frame_id"] for row in written} == {"wrist_ft_site"}
    # The pose is written onto the frame the model says the detection is of.
    assert {split_uri(URIRef(row["target_iri"]))[1] for row in written} == {"table_top"}


@requires_interfaces(TYPE_NAME)
def test_the_frame_a_written_pose_is_stated_against_resolves_to_a_world_model_segment(
    subscription_ir,
):
    """The runtime composes the arriving pose into this frame, so it is handed the segment it is
    read off -- a name it looks up once, never searches for on a tick."""
    (subscription,) = subscription_ir["communication"]["ros"]["subscriptions"]
    assert {row["frame_segment"] for row in subscription["written_poses"]} == {
        "ft_tree/wrist_ft_body/wrist_ft_site"
    }


def test_a_frame_the_tree_has_no_segment_for_is_rejected():
    """A frame the world model does not hold is one the runtime could not ask about, and a
    detection composed against nothing would land wherever the identity puts it."""
    with pytest.raises(ConstraintViolation, match="no segment standing for it"):
        communication._segment_of({}, f"{NS}camera", "object-poses")


@requires_interfaces(TYPE_NAME)
def test_no_chain_computes_a_pose_the_subscription_writes(subscription_ir):
    """One producer: the topic writes it, so the per-tick scene-object sync must not."""
    outputs = {
        out["id"]
        for solver in subscription_ir["resources"]["by_kind"]["serial_chain"]
        for out in solver["output"]
    }
    assert "pose_table_cam" not in outputs
    dataflow = subscription_ir["communication"]["introspection"]["dataflow"]
    assert dataflow["pose_table_cam"]["producer"] == {"kind": "subscription", "id": "table_top"}
