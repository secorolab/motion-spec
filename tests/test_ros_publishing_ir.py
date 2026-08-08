# SPDX-License-Identifier: MPL-2.0
"""Lowering a monitor's ROS publish: what rosidl says the message is, and what the IR carries.

The trinary contract is what makes a type publishable at all, so the acceptance cases need
`bdd_ros2_interfaces` on AMENT_PREFIX_PATH; the rejection cases use stock packages.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph, Literal, URIRef
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.handlers import LevelMonitor, RosPublication
from motion_spec.rdf_parser.communication import ros_publishers
from motion_spec.rdf_parser.coordination import _message_shape, _ros_publication
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import ros_joint_states

NS = "https://example.test/"
MONITOR = URIRef(f"{NS}mon-x")


def _model(graph: Graph) -> Model:
    return Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _publication(type_name: str):
    """Lower a monitor node stating only what the graph may state: its channel and its message."""
    graph = Graph()
    graph.add((MONITOR, NS_MM_ROS["channel-name"], Literal("/probe")))
    graph.add((MONITOR, NS_MM_ROS["type-name"], Literal(type_name)))
    return _ros_publication(_model(graph), MONITOR)["ros"]


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


def test_a_trinary_type_resolves_to_its_payload_and_the_class_owning_its_constants():
    """Stamp and scenario id are auto-filled away; the one leaf left is the verdict, and its
    TRUE/FALSE live on the nested message, not on the one published."""
    ros = _publication("bdd_ros2_interfaces/msg/TrinaryStamped")
    assert ros.cpp_type == "bdd_ros2_interfaces::msg::TrinaryStamped"
    assert ros.include == "bdd_ros2_interfaces/msg/trinary_stamped.hpp"
    assert ros.pub_id == "mon_x_pub"
    assert ros.payload_path == "trinary.value"
    assert ros.payload_cpp_type == "bdd_ros2_interfaces::msg::Trinary"
    assert ros.auto_time == ["stamp"]
    assert ros.auto_context_id == ["scenario_context_id"]


def test_an_unstamped_trinary_owns_its_own_constants():
    ros = _publication("bdd_ros2_interfaces/msg/Trinary")
    assert (ros.payload_path, ros.payload_cpp_type) == (
        "value",
        "bdd_ros2_interfaces::msg::Trinary",
    )
    assert ros.auto_time == [] and ros.auto_context_id == []


@pytest.mark.parametrize("type_name", ["std_msgs/msg/String", "sensor_msgs/msg/JointState"])
def test_a_type_without_trinary_constants_is_rejected(type_name):
    """A monitor publishes a verdict; a type that cannot say TRUE/FALSE cannot carry one."""
    with pytest.raises(ConstraintViolation, match="trinary constants"):
        _publication(type_name)


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
