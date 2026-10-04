# SPDX-License-Identifier: MPL-2.0
"""Lowering a monitor's ROS publish: what rosidl says the message is, and what the IR carries.

A row's condition is what gives it its polarity: the constraint the monitor watches, or no
condition at all -- the otherwise. Every type read here is one a ROS install already carries,
so the cases need no interface package of their own.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR_EXT, CSTR_HDL, QUDT_SCHEMA, SENSORS
from rdf_utils.namespace import NS_MM_QUDT_QTY
from rdflib import Dataset, URIRef
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.dynamics import JointQuantity
from motion_spec.classes.handlers import LevelMonitor
from motion_spec.rdf_parser.communication import annotate_publish_rates, ros_standing
from motion_spec.rdf_parser.coordination import _ros_publication
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import ros_joint_states

pytest.importorskip("rosidl_runtime_py", reason="source the ROS distribution")

NS = "https://example.test/"
MONITOR = URIRef(f"{NS}mon-x")

PREFIXES = f"""
@prefix ex: <{NS}> .
@prefix cstr-ext: <{CSTR_EXT}> .
@prefix cstr-hdl: <{CSTR_HDL}> .
@prefix qk: <{NS_MM_QUDT_QTY}> .
@prefix qudt: <{QUDT_SCHEMA}> .
@prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
@prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> .
@prefix ros: <{NS_MM_ROS}> .
@prefix sensors: <{SENSORS}> .
@prefix sosa: <http://www.w3.org/ns/sosa/> .
"""

# A publishing rate on `{subject}`, in Hz.
RATE = """
ex:{subject} sensors:update-rate ex:{subject}.rate .
ex:{subject}.rate a qudt:Quantity ; qudt:hasQuantityKind qk:Frequency ;
    qudt:unit <http://qudt.org/vocab/unit/HZ> ; qudt:value {rate:e} .
"""

# A monitor watching `ex:reached` that publishes a GoalStatus; its rows are added per case.
STATUS_MONITOR = """
ex:mon-x ros:channel-name "/probe" ; ros:type-name "action_msgs/msg/GoalStatus" ;
    cstr-hdl:constraint ex:reached ; rdfs:member ex:mon-x.f0 .
ex:mon-x.f0 cstr-ext:has-constraint ex:reached ;
    ros:field-path "status" ; rdf:value "STATUS_SUCCEEDED" .
"""


@pytest.mark.parametrize(("rate", "divider"), [(20.0, 50), (None, None), (4000.0, 1)])
def test_a_monitor_rate_becomes_the_cycles_between_messages(rate, divider):
    """Off a 1 kHz loop: no rate is no divider at all, a rate at or above the loop is every cycle."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=PREFIXES
        + STATUS_MONITOR
        + (RATE.format(subject="mon-x", rate=rate) if rate is not None else ""),
        format="turtle",
    )
    ros = _ros_publication(Model(graph=graph, app_path=Path("model-app.ld.json")), MONITOR)["ros"]
    motion = SimpleNamespace(
        when_monitors=[],
        while_monitors=[
            LevelMonitor(
                id="mon-x", monitor_type="LevelTriggeredMonitor", error=None, flag=None, ros=ros
            )
        ],
        until_monitors=[],
    )
    annotate_publish_rates([motion], 1_000_000)
    assert ros.divider == divider


def test_a_rows_condition_gives_it_its_polarity():
    """The watched constraint means satisfied; no condition at all means violated -- the
    otherwise. TRUE/FALSE render as the constants of the message that owns them."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=PREFIXES
        + STATUS_MONITOR
        + """
ex:mon-x rdfs:member ex:mon-x.f1 .
ex:mon-x.f1 ros:field-path "status" ; rdf:value "STATUS_ABORTED" .
""",
        format="turtle",
    )
    ros = _ros_publication(Model(graph=graph, app_path=Path("model-app.ld.json")), MONITOR)["ros"]
    assert ros.cpp_type == "action_msgs::msg::GoalStatus"
    assert ros.include == "action_msgs/msg/goal_status.hpp"
    assert ros.on_satisfied == [
        {"path": "status", "cpp_value": "action_msgs::msg::GoalStatus::STATUS_SUCCEEDED"}
    ]
    assert ros.on_violated == [
        {"path": "status", "cpp_value": "action_msgs::msg::GoalStatus::STATUS_ABORTED"}
    ]
    assert ros.auto_time == ["goal_info.stamp"]


def test_a_standing_publish_reports_its_quantity_whole_against_its_own_frame():
    """The payload is reached by descending the message to the ROS type the quantity maps to --
    no field name the generator assumed -- and the frame is the quantity's own."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=PREFIXES
        + """
ex:wrist-ft a ros:Topic ; ros:channel-name "/wrist_ft" ;
    ros:type-name "geometry_msgs/msg/WrenchStamped" ; rdfs:member ex:wrist-ft.e0 .
ex:wrist-ft.e0 rdf:value ex:ext-force .
"""
        + RATE.format(subject="wrist-ft", rate=100.0),
        format="turtle",
    )
    seen_by = SimpleNamespace(id="base_link", uri=f"{NS}frames/base_link")
    # 1 kHz control loop, so 100 Hz is every tenth cycle.
    (publish,) = ros_standing(
        Model(graph=graph, app_path=Path("model-app.ld.json")),
        [SimpleNamespace(id="ext_force", type="Wrench", as_seen_by=seen_by)],
        1_000_000,
        {seen_by.uri: seen_by.id},
    )
    assert publish["divider"] == 10
    (entry,) = publish["entries"]
    assert (entry["value_id"], entry["value_type"]) == ("ext_force", "Wrench")
    assert entry["payload_path"] == "wrench."
    assert publish["auto_time"] == ["header.stamp"]
    assert (publish["frame_path"], publish["frame_id"]) == ("header.frame_id", "base_link")


def test_a_message_holding_an_array_reports_one_quantity_per_entry_in_its_own_frame():
    """Every path is walked off the message class: which field holds the entries, where in one
    the pose goes, and which fields say what that entry is and when it was taken."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=PREFIXES
        + """
ex:wrist-ft a ros:Topic ; ros:channel-name "/wrist_ft" ;
    ros:type-name "vision_msgs/msg/Detection3DArray" ; rdfs:member ex:wrist-ft.e0, ex:wrist-ft.e1 .
ex:wrist-ft.e0 rdf:value ex:pose-drawer ; sosa:hasFeatureOfInterest <https://example.test/scene/drawer> .
ex:wrist-ft.e1 rdf:value ex:pose-table ; sosa:hasFeatureOfInterest <https://example.test/scene/table> .
"""
        + RATE.format(subject="wrist-ft", rate=10.0),
        format="turtle",
    )
    camera = SimpleNamespace(id="camera_optical", uri=f"{NS}frames/camera_optical")
    (publish,) = ros_standing(
        Model(graph=graph, app_path=Path("model-app.ld.json")),
        [
            SimpleNamespace(id=name, type="Pose", as_seen_by=camera)
            for name in ("pose_drawer", "pose_table")
        ],
        1_000_000,
        {camera.uri: camera.id},
    )
    assert publish["resize"] == [{"path": "detections", "size": 2}]
    first, second = publish["entries"]
    assert first["payload_path"] == "detections[0].bbox.center."
    assert (second["id_path"], second["id_value"]) == ("detections[1].id", f"{NS}scene/table")
    assert (first["frame_path"], first["frame_id"]) == (
        "detections[0].header.frame_id",
        "camera_optical",
    )
    assert first["auto_time"] == ["detections[0].header.stamp"]
    assert (publish["frame_path"], publish["frame_id"]) == ("header.frame_id", "camera_optical")


def test_a_tf_message_carries_a_pose_as_a_transform():
    """`/tf` holds transforms, so a pose goes in as one: the frame it is against in the header,
    the frame it is of as the child, and the fields spelled the way a Transform does."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=PREFIXES
        + """
ex:wrist-ft a ros:Topic ; ros:channel-name "/wrist_ft" ;
    ros:type-name "tf2_msgs/msg/TFMessage" ; rdfs:member ex:wrist-ft.e0 .
ex:wrist-ft.e0 rdf:value ex:pose-ee .
"""
        + RATE.format(subject="wrist-ft", rate=100.0),
        format="turtle",
    )
    record = SimpleNamespace(
        id="pose_ee",
        type="Pose",
        as_seen_by=SimpleNamespace(id="base_link", uri=f"{NS}scene/base_link_origin"),
        of=SimpleNamespace(id="g_pinch", uri=f"{NS}scene/g_pinch"),
    )
    (publish,) = ros_standing(
        Model(graph=graph, app_path=Path("model-app.ld.json")),
        [record],
        1_000_000,
        {
            f"{NS}scene/base_link_origin": "kinova/base_link",
            f"{NS}scene/g_pinch": "gripper/g_base/g_pinch",
        },
    )
    (entry,) = publish["entries"]
    assert (entry["value_type"], entry["carrier"]) == ("Pose", "Transform")
    assert entry["payload_path"] == "transforms[0].transform."
    assert (entry["id_path"], entry["id_value"]) == (
        "transforms[0].child_frame_id",
        "gripper/g_base/g_pinch",
    )
    assert (entry["frame_path"], entry["frame_id"]) == (
        "transforms[0].header.frame_id",
        "kinova/base_link",
    )


@pytest.mark.parametrize("reporter", ("output", "device_output"))
def test_a_joint_the_chain_does_not_articulate_is_still_published(reporter: str):
    """A gripper's driver joint is a mimic the chain never articulates, so whoever answers for it
    -- the bound device on hardware, the simulator otherwise -- is the only route to it, and
    iterating the chain alone drops it from the message it belongs in."""
    driver = [JointQuantity("gripper_pos", "r1_g_left_driver_joint", "JointPosition")]
    # A serial chain whose joint-space channels are already mirrored onto the blackboard.
    chain = SimpleNamespace(
        id="r1_solver",
        runtime=SimpleNamespace(owner=True, prefix="r1_"),
        chain=SimpleNamespace(joints=["j1"], tree_root="world_tree/world_body"),
        joint_space_samples=[
            {"id": f"arm_{channel}_r1_j1", "channel": channel, "index": 0}
            for channel in ("q", "qd", "tau_ctrl")
        ],
        output=driver if reporter == "output" else [],
        devices=[SimpleNamespace(joint_outputs=driver if reporter == "device_output" else [])],
    )
    section = ros_joint_states({"config": "robot.toml"}, {"ros": {"joint_states": {}}}, [chain])
    assert [joint["name"] for joint in section["joints"]] == ["r1_j1", "r1_g_left_driver_joint"]
    gripper = section["joints"][1]
    assert gripper["position"] == "gripper_pos"
    # Neither gripper route measures one, and a zero would claim it is still and unloaded.
    assert "velocity" not in gripper and "effort" not in gripper


@pytest.mark.parametrize(
    ("outcome", "condition", "satisfied", "method"),
    [
        ("STATUS_SUCCEEDED", "cstr-ext:has-constraint ex:reached ;", True, "succeed"),
        ("STATUS_ABORTED", "", False, "abort"),
    ],
)
def test_an_answer_states_which_polarity_of_its_monitor_answers(
    outcome, condition, satisfied, method
):
    """A satisfied answer holds under the constraint the monitor watches; one that states no
    condition is the otherwise, and answers when the constraint fails."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=PREFIXES
        + f"""
ex:mon-x cstr-hdl:constraint ex:reached ; rdfs:member ex:mon-x.answer .
ex:mon-x.answer a ros:Action ; ros:type-name "control_msgs/action/GripperCommand" ;
    ros:channel-name "pick_place" ; rdfs:member ex:mon-x.answer.outcome, ex:mon-x.answer.f0 .
ex:mon-x.answer.outcome {condition} rdf:value "{outcome}" .
ex:mon-x.answer.f0 ros:field-path "position" ; rdf:value "0.5" .
""",
        format="turtle",
    )
    lowered = _ros_publication(Model(graph=graph, app_path=Path("model-app.ld.json")), MONITOR)[
        "answer"
    ]
    assert (lowered.satisfied, lowered.method) == (satisfied, method)
    assert lowered.fields == [{"path": "position", "cpp_value": "0.5"}]
