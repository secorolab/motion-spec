# SPDX-License-Identifier: MPL-2.0
"""Lowering a monitor's ROS publish: what rosidl says the message is, and what the IR carries.

A row's condition is what gives it its polarity: the constraint the monitor watches, or no
condition at all -- the otherwise. The acceptance cases need `bdd_ros2_interfaces` on
AMENT_PREFIX_PATH; the rest use stock packages.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR_EXT, CSTR_HDL
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, RDFS
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.handlers import LevelMonitor, RosPublication
from motion_spec.rdf_parser.communication import ros_publishers
from motion_spec.rdf_parser.coordination import _message_shape, _ros_publication
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import ros_joint_states

NS = "https://example.test/"
MONITOR = URIRef(f"{NS}mon-x")
WATCHED = URIRef(f"{NS}reached")
OTHERWISE = None  # a row with no condition
UNKNOWN = URIRef(f"{NS}some-other-constraint")


def _model(graph: Graph) -> Model:
    return Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _publication(type_name: str, *rows: tuple[URIRef, str, str]):
    """Lower a monitor node: its channel, its message, and the rows it states."""
    graph = Graph()
    graph.add((MONITOR, NS_MM_ROS["channel-name"], Literal("/probe")))
    graph.add((MONITOR, NS_MM_ROS["type-name"], Literal(type_name)))
    graph.add((MONITOR, CSTR_HDL["constraint"], WATCHED))
    for index, (condition, path, value) in enumerate(rows):
        row = URIRef(f"{NS}mon-x.f{index}")
        graph.add((MONITOR, RDFS.member, row))
        if condition is not None:
            graph.add((row, CSTR_EXT["has-constraint"], condition))
        graph.add((row, NS_MM_ROS["field-path"], Literal(path)))
        graph.add((row, RDF.value, Literal(value)))
    return _ros_publication(_model(graph), MONITOR)["ros"]


def _trinary(*rows: tuple[URIRef, str, str]):
    return _publication("bdd_ros2_interfaces/msg/TrinaryStamped", *rows)


def test_message_shape_separates_authorable_fields_from_auto_filled_ones():
    """A Header's stamp is the node's to fill; its frame_id is the model's to state."""
    shape = _message_shape("sensor_msgs/msg/JointState")
    assert shape["cpp_type"] == "sensor_msgs::msg::JointState"
    assert shape["include"] == "sensor_msgs/msg/joint_state.hpp"
    assert shape["auto"] == {"header.stamp": "time"}
    # name/position/velocity/effort are sequences: leaves, not messages to descend into.
    assert set(shape["leaves"]) == {"header.frame_id", "name", "position", "velocity", "effort"}


def test_include_stem_comes_from_the_rosidl_converter():
    """The generated header name matches rosidl's own case conversion by construction."""
    assert convert_camel_case_to_lower_case_underscore("TrinaryStamped") == "trinary_stamped"


def test_a_rows_condition_gives_it_its_polarity():
    """The watched constraint means satisfied; no condition at all means violated -- the
    otherwise. The sugar's empty path resolves to the one payload leaf, and TRUE/FALSE render as
    the constants of the message that owns them."""
    ros = _trinary((WATCHED, "", "TRUE"), (OTHERWISE, "", "FALSE"))
    assert ros.cpp_type == "bdd_ros2_interfaces::msg::TrinaryStamped"
    assert ros.include == "bdd_ros2_interfaces/msg/trinary_stamped.hpp"
    assert ros.pub_id == "mon_x_pub"
    assert ros.on_satisfied == [
        {"path": "trinary.value", "cpp_value": "bdd_ros2_interfaces::msg::Trinary::TRUE"}
    ]
    assert ros.on_violated == [
        {"path": "trinary.value", "cpp_value": "bdd_ros2_interfaces::msg::Trinary::FALSE"}
    ]
    assert (ros.has_satisfied, ros.has_violated) == (True, True)
    assert ros.auto_time == ["stamp"]
    assert ros.auto_context_id == ["scenario_context_id"]


def test_a_satisfied_only_publish_leaves_the_violated_branch_empty():
    ros = _trinary((WATCHED, "trinary.value", "TRUE"))
    assert (ros.has_satisfied, ros.has_violated) == (True, False)
    assert ros.on_violated == []


def test_an_authored_string_is_quoted_and_a_number_typed_by_its_field():
    """A value the message class does not define as a constant is data: a string field quotes
    it, a numeric field takes it as written."""
    assert _publication(
        "std_msgs/msg/String", (WATCHED, "data", "forward reached")
    ).on_satisfied == [{"path": "data", "cpp_value": '"forward reached"'}]
    assert _publication("std_msgs/msg/Float64", (WATCHED, "data", "0.5")).on_satisfied == [
        {"path": "data", "cpp_value": "0.5"}
    ]


def test_a_condition_that_is_not_the_watched_constraint_is_rejected():
    with pytest.raises(ConstraintViolation, match="not the constraint its monitor watches"):
        _trinary((UNKNOWN, "", "TRUE"))


def test_a_path_the_message_does_not_offer_is_rejected():
    with pytest.raises(ConstraintViolation, match="not a payload field"):
        _trinary((WATCHED, "trinary.verdict", "TRUE"))


def test_a_value_a_numeric_field_cannot_take_is_rejected():
    with pytest.raises(ConstraintViolation, match="neither a constant"):
        _trinary((WATCHED, "trinary.value", "MAYBE"))


def test_the_sugar_needs_a_type_with_exactly_one_payload_field():
    with pytest.raises(ConstraintViolation, match="must name the field it writes"):
        _publication("sensor_msgs/msg/JointState", (WATCHED, "", "0"))


def test_publishers_dedupe_by_channel():
    """Two monitors on one topic share one publisher member, so both fill the same message."""
    shared = [
        LevelMonitor(
            f"mon-{index}",
            "LevelTriggeredMonitor",
            None,
            None,
            ros=RosPublication(
                channel="/probe",
                cpp_type="std_msgs::msg::Bool",
                include="std_msgs/msg/bool.hpp",
                pkg="std_msgs",
                pub_id=f"mon_{index}_pub",
            ),
        )
        for index in (1, 2)
    ]
    motion = type("M", (), {"when_monitors": [], "while_monitors": shared, "until_monitors": []})()
    publishers = ros_publishers([motion])
    assert [publisher["channel"] for publisher in publishers] == ["/probe"]
    assert {monitor.ros.pub_id for monitor in shared} == {"mon_1_pub"}


def _chain(prefix: str, joints: list[str]):
    """A serial chain whose joint-space channels are already mirrored onto the blackboard."""
    samples = [
        {"id": f"arm_{channel}_{prefix}{joint}", "channel": channel, "index": index}
        for channel in ("q", "qd", "tau_ctrl")
        for index, joint in enumerate(joints)
    ]
    return type(
        "C",
        (),
        {
            "runtime": type("R", (), {"owner": True, "prefix": prefix})(),
            "chain": type("Ch", (), {"joints": joints})(),
            "joint_space_samples": samples,
        },
    )()


def test_joint_states_are_gated_on_the_config_section():
    platform = {"config": "robot.toml"}
    chains = [_chain("r1_", ["j1", "j2"])]
    assert ros_joint_states(platform, {}, chains) is None
    section = ros_joint_states(platform, {"ros": {"joint_states": {}}}, chains)
    assert section["config_key"] == "ros.joint_states"
    # Chain joints are stored unprefixed; what is published is runtime-scoped.
    assert [joint["name"] for joint in section["joints"]] == ["r1_j1", "r1_j2"]
    assert section["joints"][0]["position"] == "arm_q_r1_j1"
    assert section["joints"][1]["effort"] == "arm_tau_ctrl_r1_j2"


def test_joint_states_need_a_declared_config_to_read_at_runtime():
    with pytest.raises(ConstraintViolation, match="declare `config:`"):
        ros_joint_states({}, {"ros": {"joint_states": {}}}, [_chain("r1_", ["j1"])])
