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
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.handlers import LevelMonitor, RosPublication
from motion_spec.rdf_parser.communication import action_server, ros_publishers
from motion_spec.rdf_parser.coordination import _message_shape, _ros_publication
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import ros_joint_states

from conftest import requires_interfaces

pytestmark = requires_interfaces(
    "bdd_ros2_interfaces/msg/TrinaryStamped", "bdd_ros2_interfaces/action/Behaviour"
)
# A mark skips the cases, not the import, so this one has to skip the module itself.
convert_camel_case_to_lower_case_underscore = pytest.importorskip(
    "rosidl_pycommon"
).convert_camel_case_to_lower_case_underscore

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


def _publishing_motion(*cpp_types: str):
    """A motion whose monitors all publish on `/probe`, each carrying its own message type."""
    monitors = [
        LevelMonitor(
            f"mon-{index}",
            "LevelTriggeredMonitor",
            None,
            None,
            ros=RosPublication(
                channel="/probe",
                cpp_type=cpp_type,
                include="std_msgs/msg/bool.hpp",
                pkg="std_msgs",
                pub_id=f"mon_{index}_pub",
            ),
        )
        for index, cpp_type in enumerate(cpp_types, start=1)
    ]
    motion = type(
        "M", (), {"when_monitors": [], "while_monitors": monitors, "until_monitors": []}
    )()

    return monitors, motion


def test_publishers_dedupe_by_channel():
    """Two monitors on one topic share one publisher member, so both fill the same message."""
    monitors, motion = _publishing_motion("std_msgs::msg::Bool", "std_msgs::msg::Bool")
    publishers = ros_publishers([motion])
    assert [publisher["channel"] for publisher in publishers] == ["/probe"]
    assert {monitor.ros.pub_id for monitor in monitors} == {"mon_1_pub"}


def test_one_channel_published_as_two_message_types_is_rejected():
    """The shared member is typed once, so the second type could only be written into the
    first's message."""
    _monitors, motion = _publishing_motion("std_msgs::msg::Bool", "std_msgs::msg::Float64")
    with pytest.raises(ConstraintViolation, match="one channel carries one message type"):
        ros_publishers([motion])


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


FSM_NS = "https://example.test/fsm/"
# Event order is the FSM's own -- the indices the generated coord2b header was built against.
FSM = {
    "name": "demo_fsm",
    "events": ["E_DONE", "E_GOAL", "E_STEP"],
    "event_uris": {name: f"{FSM_NS}{name}" for name in ("E_DONE", "E_GOAL", "E_STEP")},
    "transitions_table": [{"id": "T_IDLE_RUN", "from_state": "S_IDLE", "to_state": "S_RUN"}],
    "reactions_table": [
        {"id": "R_GOAL", "when_event": "E_GOAL", "do_transition": "T_IDLE_RUN", "fires_events": []}
    ],
}
SERVER = URIRef(f"{NS}pick-place")
ACTION_TYPE = "bdd_ros2_interfaces/action/Behaviour"


def _served(goal_event: str = "E_GOAL", *, result: tuple | None = ("result.trinary.value", "TRUE")):
    """A served action: the goal event it produces, and what a completed run answers with."""
    graph = Graph()
    graph.add((SERVER, RDF.type, NS_MM_ROS["Action"]))
    graph.add((SERVER, NS_MM_ROS["type-name"], Literal(ACTION_TYPE)))
    graph.add((SERVER, NS_MM_ROS["channel-name"], Literal("pick_place")))
    graph.add((SERVER, RDFS.member, URIRef(f"{FSM_NS}{goal_event}")))
    if result is not None:
        row = URIRef(f"{SERVER}.r0")
        graph.add((SERVER, RDFS.member, row))
        graph.add((row, NS_MM_ROS["field-path"], Literal(result[0])))
        graph.add((row, RDF.value, Literal(result[1])))
    return graph


def test_a_model_without_a_server_states_none():
    assert action_server(_model(Graph()), FSM) is None


def test_the_server_carries_the_fsms_own_event_tokens():
    """A token, never an index: the generated enum is declaration-ordered, this reader's tables
    are sorted, so only the symbol survives the crossing."""
    server = action_server(_model(_served()), FSM)
    assert server["action_name"] == "pick_place"
    assert server["goal_event"] == "E_GOAL"
    assert server["goal_states"] == ["S_IDLE"]


def test_the_server_reads_its_whole_shape_off_the_action_it_names():
    """Nothing about the action is assumed: the C++ type, the header, the packages and the
    node-owned result fields all come from the type the model stated."""
    server = action_server(_model(_served()), FSM)
    assert server["cpp_type"] == "bdd_ros2_interfaces::action::Behaviour"
    assert server["result_cpp_type"] == "bdd_ros2_interfaces::action::Behaviour_Result"
    assert server["include"] == "bdd_ros2_interfaces/action/behaviour.hpp"
    assert server["goal_context_id"] == ["scenario_context_id"]
    assert server["result_auto_time"] == ["result.stamp"]
    assert server["result_auto_context_id"] == ["result.scenario_context_id"]
    assert sorted(server["ignored_goal_fields"]) == ["configs", "parameters"]
    assert "bdd_ros2_interfaces" in server["packages"]


def test_a_completed_run_answers_with_the_fields_the_model_authored():
    server = action_server(_model(_served()), FSM)
    assert server["result_fields"] == [
        {"path": "result.trinary.value", "cpp_value": "bdd_ros2_interfaces::msg::Trinary::TRUE"}
    ]


def test_a_server_authoring_no_result_answers_with_the_types_own_defaults():
    """A run that never finishes states nothing it did not establish; so does one whose model
    authored no result at all."""
    assert action_server(_model(_served(result=None)), FSM)["result_fields"] == []


def test_a_result_field_the_action_does_not_offer_is_rejected():
    graph = _served(result=("result.trinary.invented", "TRUE"))
    with pytest.raises(ConstraintViolation, match="not a payload field"):
        action_server(_model(graph), FSM)


def test_an_event_the_fsm_does_not_declare_is_rejected():
    with pytest.raises(ConstraintViolation, match="E_INVENTED"):
        action_server(_model(_served("E_INVENTED")), FSM)


def test_a_goal_event_no_reaction_consumes_is_rejected():
    """Nothing reacts to E_DONE, so an accepted goal would start nothing."""
    with pytest.raises(ConstraintViolation, match="no FSM reaction"):
        action_server(_model(_served("E_DONE")), FSM)


def test_serving_goals_without_an_fsm_is_rejected():
    with pytest.raises(ConstraintViolation, match="imports no FSM"):
        action_server(_model(_served()), None)


def test_two_served_actions_are_rejected():
    """One runtime answers one action."""
    graph = _served()
    other = URIRef(f"{NS}other")
    graph.add((other, RDF.type, NS_MM_ROS["Action"]))
    graph.add((other, NS_MM_ROS["type-name"], Literal(ACTION_TYPE)))
    graph.add((other, NS_MM_ROS["channel-name"], Literal("other")))
    graph.add((other, RDFS.member, URIRef(f"{FSM_NS}E_GOAL")))
    with pytest.raises(ConstraintViolation, match="serves 2 actions"):
        action_server(_model(graph), FSM)


EVENT = URIRef(f"{FSM_NS}E_DONE")


def _occurrence(type_name: str, event=EVENT) -> RosPublication:
    """Lower a monitor that publishes the event it triggers, rather than authored fields."""
    graph = Graph()
    graph.add((MONITOR, NS_MM_ROS["channel-name"], Literal("/bdd/events")))
    graph.add((MONITOR, NS_MM_ROS["type-name"], Literal(type_name)))
    graph.add((MONITOR, CSTR_HDL["constraint"], WATCHED))
    graph.add((MONITOR, CSTR_HDL["event"], EVENT))
    graph.add((MONITOR, RDFS.member, event))
    return _ros_publication(_model(graph), MONITOR)["ros"]


def test_an_occurrence_resolves_its_field_off_the_message_type():
    """The payload field is the message's own sole leaf, not a name the generator assumes; the
    authored-row branches stay empty, since the event is the whole payload."""
    ros = _occurrence("bdd_ros2_interfaces/msg/Event")
    assert ros.occurrence_path == "uri"
    assert (ros.on_satisfied, ros.on_violated) == ([], [])
    assert (ros.has_satisfied, ros.has_violated) == (False, False)
    assert ros.auto_time == ["stamp"]
    assert ros.auto_context_id == ["scenario_context_id"]


def test_an_occurrence_of_an_event_the_monitor_does_not_trigger_is_rejected():
    with pytest.raises(ConstraintViolation, match="not the event it triggers"):
        _occurrence("bdd_ros2_interfaces/msg/Event", event=URIRef(f"{FSM_NS}E_OTHER"))


def test_an_occurrence_needs_a_field_that_can_hold_an_iri():
    with pytest.raises(ConstraintViolation, match="must offer a string"):
        _occurrence("std_msgs/msg/Float64")
