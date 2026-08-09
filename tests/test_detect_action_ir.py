# SPDX-License-Identifier: MPL-2.0
"""Lowering a detect act: the status a goal reaches, and the pose its result -- not the
kinematics -- writes.

The graph cases build the act and its status slot by hand; the model case lowers the maintained
`detect_pick_single` model, which is the only place the whole chain (act -> written pose -> the
chain that no longer computes it) can be read at once. It lives with the BDD package that drives
it, since a scenario goal and a perception mock are what it needs to run.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from motion_spec.rdf_parser.coordination import detect_shape
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_HDL, MOT
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import PROV, RDF
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.constraints import Constraint, GoalStatus
from motion_spec.classes.handlers import ConstraintEvaluator, EvaluatorType
from motion_spec.generation.pipeline import generate_model
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.communication import _act_motion, _act_status_slot
from motion_spec.rdf_parser.coordination import constraint_evaluator, evaluator_term
from motion_spec.rdf_parser.model import Model

NS = "https://example.test/"
MODEL = Path(__file__).parents[2] / "bdd_collab_bhv_cpp" / "models" / "detect_pick_single"


def _model(graph: Graph) -> Model:
    return Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _act(graph: Graph) -> URIRef:
    """An act node and the status slot derived from it."""
    act = URIRef(f"{NS}locate-cube")
    graph.add((act, RDF.type, NS_MM_ROS["Action"]))
    graph.add((URIRef(f"{act}.status"), PROV.wasDerivedFrom, act))
    return act


def _status_equality(graph: Graph, act: URIRef, name: str, status: str) -> URIRef:
    node = URIRef(f"{NS}{name}")
    graph.add((node, RDF.type, CSTR["Constraint"]))
    graph.add((node, RDF.type, CSTR["EqualityConstraint"]))
    graph.add((node, CSTR["quantity"], URIRef(f"{act}.status")))
    reference = URIRef(f"{node}-reference")
    graph.add((reference, RDF.value, Literal(status)))
    graph.add((node, CSTR["reference-value"], reference))
    return node


def _evaluator(graph: Graph, constraint: URIRef) -> ConstraintEvaluator:
    node = URIRef(f"{constraint}-eval")
    graph.add((node, RDF.type, CSTR_HDL["ConstraintEvaluator"]))
    graph.add((node, CSTR_HDL["constraint"], constraint))
    graph.add((node, CSTR_HDL["error"], graph.value(constraint, CSTR["quantity"])))
    return constraint_evaluator(_model(graph), node)


def test_a_status_slot_reads_as_a_goal_status_and_its_constraint_states_nothing_else():
    """The status has no kind and no unit: the act it came from is what says what it is."""
    graph = Graph()
    act = _act(graph)
    node = _status_equality(graph, act, "located", "STATUS_SUCCEEDED")
    model = _model(graph)
    assert quantities.goal_status_act(model, URIRef(f"{act}.status")) == act
    read = quantities.constraint(model, node)
    assert isinstance(read.quantity, GoalStatus)
    assert read.parameter is None


def test_the_evaluator_carries_the_status_the_model_named():
    graph = Graph()
    act = _act(graph)
    evaluator = _evaluator(graph, _status_equality(graph, act, "located", "STATUS_SUCCEEDED"))
    assert evaluator.goal_status == "STATUS_SUCCEEDED"


def test_the_term_compares_the_status_slot_against_that_constant():
    evaluator = ConstraintEvaluator(
        "eval-located",
        EvaluatorType.ErrorEvaluator,
        Constraint("located", GoalStatus("locate_cube_status"), None),
        GoalStatus("locate_cube_status"),
        goal_status="STATUS_SUCCEEDED",
    )
    assert evaluator_term(evaluator) == {
        "kind": "goal-status",
        "status_id": "locate_cube_status",
        "value": "STATUS_SUCCEEDED",
    }


def test_the_act_belongs_to_the_motion_whose_until_reads_its_status():
    graph = Graph()
    act = _act(graph)
    located = _status_equality(graph, act, "located", "STATUS_SUCCEEDED")
    motion = URIRef(f"{NS}motion-detect-cube")
    graph.add((motion, RDF.type, MOT["GuardedMotion"]))
    graph.add((motion, MOT["until"], located))
    model = _model(graph)
    assert _act_motion(model, _act_status_slot(model, act)) == "motion_detect_cube"


def test_an_act_no_motion_waits_on_is_rejected():
    """Nothing would send the goal, and nothing would say when it stopped mattering."""
    graph = Graph()
    act = _act(graph)
    _status_equality(graph, act, "located", "STATUS_SUCCEEDED")
    model = _model(graph)
    with pytest.raises(ConstraintViolation, match="nothing sends the goal"):
        _act_motion(model, _act_status_slot(model, act))


@pytest.fixture(scope="module")
def detect_ir(tmp_path_factory) -> dict:
    generation = tmp_path_factory.mktemp("detect_generation")
    generated = generate_model(MODEL / "detect_pick_single.robmot", generation, stage="ir")
    return json.loads((generated / "model" / "ir.json").read_text())


def test_the_client_names_the_pose_it_writes_and_the_frame_it_must_arrive_in(detect_ir):
    (client,) = detect_ir["communication"]["ros"]["action_clients"]
    assert client["channel"] == "/perception/locate"
    assert client["cpp_type"] == "aruco_perception::action::LocateObjects"
    assert client["motion"] == "motion_detect_cube"
    assert client["status_id"] == "locate_cube_status"
    written = client["written_poses"]
    assert [row["pose_id"] for row in written] == ["pose_cube_base", "pose_cube2_base"]
    assert {row["frame_id"] for row in written} == {"base_link"}
    # One goal asks about every object the act names, and one result answers for all of them.
    assert client["target_iris"] == sorted(row["target_iri"] for row in written)


def test_no_chain_computes_a_pose_the_detect_result_writes(detect_ir):
    """One producer: the act writes it, so the per-tick scene-object sync must not."""
    outputs = {
        out["id"]
        for solver in detect_ir["resources"]["by_kind"]["serial_chain"]
        for out in solver["output"]
    }
    assert "pose_cube_base" not in outputs
    dataflow = detect_ir["communication"]["introspection"]["dataflow"]
    assert dataflow["pose_cube_base"]["producer"] == {"kind": "action", "id": "locate_cube"}


def test_the_client_reads_every_path_off_the_action_it_names(detect_ir):
    """Only the pose is stated by the model, because only it is ambiguous: a detection reaches
    one through its hypotheses and another through its bounding box."""
    (client,) = detect_ir["communication"]["ros"]["action_clients"]
    assert client["targets_path"] == "target_iris"
    assert client["detections_path"] == "detections.detections"
    assert client["id_path"] == "id"
    assert client["frame_path"] == "header.frame_id"
    assert client["pose_path"] == "results[0].pose.pose"
    assert client["pose_container"] == "results"


def test_an_act_that_says_no_pose_is_rejected():
    """`results` and `bbox` both reach a pose, so the act has to say which answers the question."""
    with pytest.raises(ConstraintViolation, match="reaches more than one pose"):
        detect_shape("aruco_perception/action/LocateObjects", "")


def test_a_pose_read_from_a_field_the_detection_does_not_repeat_is_rejected():
    with pytest.raises(ConstraintViolation, match="no repeated message field 'bbox'"):
        detect_shape("aruco_perception/action/LocateObjects", "bbox.center")


def test_a_pose_field_the_hypothesis_does_not_carry_is_rejected():
    with pytest.raises(ConstraintViolation, match="no message field 'invented'"):
        detect_shape("aruco_perception/action/LocateObjects", "results.invented")


def test_the_last_hop_to_the_pose_is_derived():
    """The model names `pose`, which is a covariance wrapper; the pose inside it is the only one
    down there, so the generator reaches it rather than the model spelling it."""
    assert (
        detect_shape("aruco_perception/action/LocateObjects", "results.pose")["pose_path"]
        == "results[0].pose.pose"
    )
