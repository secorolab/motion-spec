# SPDX-License-Identifier: MPL-2.0
"""Lowering a monitor's ROS publish: what rosidl says the message is, and what the IR carries.

A row's condition is what gives it its polarity: the constraint the monitor watches, or no
condition at all -- the otherwise. Every type read here is one a ROS install already carries,
so the cases need no interface package of their own.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR_EXT, CSTR_HDL, QUDT_SCHEMA, SENSORS
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.namespace import NS_MM_QUDT_QTY
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, RDFS, SOSA
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.dynamics import JointPosition
from motion_spec.classes.handlers import LevelMonitor, RosPublication
from motion_spec.rdf_parser.communication import (
    action_server,
    annotate_publish_rates,
    ros_publishers,
    ros_standing,
)
from motion_spec.rdf_parser.coordination import _message_shape, _ros_publication
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import ros_joint_states

from conftest import requires_interfaces

pytestmark = requires_interfaces("action_msgs/msg/GoalStatus", "control_msgs/action/GripperCommand")
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


def _publication(type_name: str, *rows: tuple[URIRef, str, str], rate: float | None = None):
    """Lower a monitor node: its channel, its message, and the rows it states."""
    graph = Graph()
    graph.add((MONITOR, NS_MM_ROS["channel-name"], Literal("/probe")))
    graph.add((MONITOR, NS_MM_ROS["type-name"], Literal(type_name)))
    graph.add((MONITOR, CSTR_HDL["constraint"], WATCHED))
    if rate is not None:
        rate_node = URIRef(f"{MONITOR}.rate")
        graph.add((MONITOR, SENSORS["update-rate"], rate_node))
        graph.add((rate_node, RDF["type"], QUDT_SCHEMA.Quantity))
        graph.add((rate_node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Frequency"]))
        graph.add((rate_node, QUDT_SCHEMA["unit"], URIRef("http://qudt.org/vocab/unit/HZ")))
        graph.add((rate_node, QUDT_SCHEMA["value"], Literal(rate)))
    for index, (condition, path, value) in enumerate(rows):
        row = URIRef(f"{NS}mon-x.f{index}")
        graph.add((MONITOR, RDFS.member, row))
        if condition is not None:
            graph.add((row, CSTR_EXT["has-constraint"], condition))
        graph.add((row, NS_MM_ROS["field-path"], Literal(path)))
        graph.add((row, RDF.value, Literal(value)))
    return _ros_publication(_model(graph), MONITOR)["ros"]


def _status(*rows: tuple[URIRef, str, str], rate: float | None = None):
    """A verdict published as a goal status: a constant-bearing field beside a stamp the node
    fills, which is what a monitor's publish has to render."""
    return _publication("action_msgs/msg/GoalStatus", *rows, rate=rate)


def _monitor_motion(publication):
    """A motion with one publishing monitor, as the divider pass walks it."""
    monitor = LevelMonitor("mon-x", "LevelTriggeredMonitor", None, None, ros=publication)

    return type("M", (), {"when_monitors": [], "while_monitors": [monitor], "until_monitors": []})()


def test_a_monitor_publishing_at_a_rate_counts_the_cycles_between_messages():
    """A verdict a reader wants twenty times a second, off a loop running a thousand."""
    ros = _status((WATCHED, "status", "STATUS_SUCCEEDED"), rate=20.0)
    assert ros.rate_hz == 20.0
    annotate_publish_rates([_monitor_motion(ros)], 1_000_000)
    assert ros.divider == 50


def test_a_monitor_stating_no_rate_publishes_every_cycle():
    """No divider at all rather than one, so the loop tests nothing it need not."""
    ros = _status((WATCHED, "status", "STATUS_SUCCEEDED"))
    assert ros.rate_hz is None
    annotate_publish_rates([_monitor_motion(ros)], 1_000_000)
    assert ros.divider is None


def test_a_monitor_rate_at_or_above_the_loop_rate_publishes_every_cycle():
    ros = _status((WATCHED, "status", "STATUS_SUCCEEDED"), rate=4000.0)
    annotate_publish_rates([_monitor_motion(ros)], 1_000_000)
    assert ros.divider == 1


def test_a_monitor_rate_that_is_not_positive_is_rejected():
    with pytest.raises(ConstraintViolation, match="a rate says how often"):
        _status((WATCHED, "status", "STATUS_SUCCEEDED"), rate=0.0)


def test_a_monitors_rate_does_not_make_it_a_standing_publish():
    """Both state a rate with the same term. What a monitor publishes belongs to its motion, so
    it is published where the motion is and never from the run."""
    graph = Graph()
    graph.add((MONITOR, RDF["type"], NS_MM_ROS["Topic"]))
    graph.add((MONITOR, RDF["type"], CSTR_HDL["Monitor"]))
    graph.add((MONITOR, NS_MM_ROS["channel-name"], Literal("/probe")))
    graph.add((MONITOR, NS_MM_ROS["type-name"], Literal("std_msgs/msg/Bool")))
    rate_node = URIRef(f"{MONITOR}.rate")
    graph.add((MONITOR, SENSORS["update-rate"], rate_node))
    assert ros_standing(_model(graph), [], 1_000_000) == []


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
    assert convert_camel_case_to_lower_case_underscore("GoalStatus") == "goal_status"


def test_a_rows_condition_gives_it_its_polarity():
    """The watched constraint means satisfied; no condition at all means violated -- the
    otherwise. TRUE/FALSE render as the constants of the message that owns them."""
    ros = _status((WATCHED, "status", "STATUS_SUCCEEDED"), (OTHERWISE, "status", "STATUS_ABORTED"))
    assert ros.cpp_type == "action_msgs::msg::GoalStatus"
    assert ros.include == "action_msgs/msg/goal_status.hpp"
    assert ros.pub_id == "mon_x_pub"
    assert ros.on_satisfied == [
        {"path": "status", "cpp_value": "action_msgs::msg::GoalStatus::STATUS_SUCCEEDED"}
    ]
    assert ros.on_violated == [
        {"path": "status", "cpp_value": "action_msgs::msg::GoalStatus::STATUS_ABORTED"}
    ]
    assert (ros.has_satisfied, ros.has_violated) == (True, True)
    assert ros.auto_time == ["goal_info.stamp"]
    assert ros.auto_context_id == []


def test_a_satisfied_only_publish_leaves_the_violated_branch_empty():
    ros = _status((WATCHED, "status", "STATUS_SUCCEEDED"))
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
        _status((UNKNOWN, "", "STATUS_SUCCEEDED"))


def test_a_path_the_message_does_not_offer_is_rejected():
    with pytest.raises(ConstraintViolation, match="not a payload field"):
        _status((WATCHED, "goal_info.verdict", "STATUS_SUCCEEDED"))


def test_a_value_a_numeric_field_cannot_take_is_rejected():
    with pytest.raises(ConstraintViolation, match="neither a constant"):
        _status((WATCHED, "status", "MAYBE"))


def test_the_sugar_resolves_to_the_one_payload_field_a_type_offers():
    """An empty path is the author saying the message is the value: it holds only when the type
    leaves no choice about which field that is."""
    assert _publication("std_msgs/msg/Bool", (WATCHED, "", "1")).on_satisfied == [
        {"path": "data", "cpp_value": "1"}
    ]


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


STANDING = URIRef(f"{NS}wrist-ft")
REPORTED = URIRef(f"{NS}ext-force")


def _reported(name: str = "ext_force", type_: str = "Wrench", frame: str = "base_link"):
    """A data structure on the blackboard, as the standing publish finds it."""
    return type(
        "Q", (), {"id": name, "type": type_, "as_seen_by": type("F", (), {"id": frame})()}
    )()


def _standing(type_name: str = "geometry_msgs/msg/WrenchStamped", rate: float = 100.0, **kwargs):
    """A topic published for the whole run: the quantity it reports, and how often."""
    return _standing_many(type_name, rate, (REPORTED, None, _reported(**kwargs)))


def _standing_many(type_name: str, rate: float, *entries):
    """The same, reporting several quantities -- each stating the entity it is an entry for."""
    graph = Graph()
    graph.add((STANDING, RDF.type, NS_MM_ROS["Topic"]))
    graph.add((STANDING, NS_MM_ROS["channel-name"], Literal("/wrist_ft")))
    graph.add((STANDING, NS_MM_ROS["type-name"], Literal(type_name)))
    for index, (quantity, subject, _record) in enumerate(entries):
        row = URIRef(f"{STANDING}.e{index}")
        graph.add((STANDING, RDFS.member, row))
        graph.add((row, RDF.value, quantity))
        if subject is not None:
            graph.add((row, SOSA.hasFeatureOfInterest, subject))
    rate_node = URIRef(f"{STANDING}.rate")
    graph.add((STANDING, SENSORS["update-rate"], rate_node))
    graph.add((rate_node, RDF["type"], QUDT_SCHEMA.Quantity))
    graph.add((rate_node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Frequency"]))
    graph.add((rate_node, QUDT_SCHEMA["unit"], URIRef("http://qudt.org/vocab/unit/HZ")))
    graph.add((rate_node, QUDT_SCHEMA["value"], Literal(rate)))
    # 1 kHz control loop, so 100 Hz is every tenth cycle.
    return ros_standing(_model(graph), [record for _q, _s, record in entries], 1_000_000)


def test_a_standing_publish_reports_its_quantity_whole_at_its_own_rate():
    """The rate becomes cycles between messages, and the payload is reached by descending the
    message to the ROS type the quantity maps to -- no field name the generator assumed."""
    (publish,) = _standing()
    assert publish["channel"] == "/wrist_ft"
    assert publish["pub_id"] == "wrist_ft_pub"
    assert publish["divider"] == 10
    (entry,) = publish["entries"]
    assert (entry["value_id"], entry["value_type"]) == ("ext_force", "Wrench")
    assert entry["payload_path"] == "wrench."
    assert publish["cpp_type"] == "geometry_msgs::msg::WrenchStamped"
    # Nothing to size and nothing to tell apart: the message is the quantity.
    assert "resize" not in publish
    assert "id_path" not in entry
    # The node stamps it; the frame it is stated against is the quantity's own.
    assert publish["auto_time"] == ["header.stamp"]
    assert (publish["frame_path"], publish["frame_id"]) == ("header.frame_id", "base_link")


def test_a_rate_at_or_above_the_loop_rate_publishes_every_cycle():
    assert _standing(rate=4000.0)[0]["divider"] == 1


def test_a_standing_publish_shares_the_channels_publisher_member():
    """A monitor and a standing publish on one topic fill one message through one member."""
    monitors, motion = _publishing_motion("geometry_msgs::msg::WrenchStamped")
    standing = _standing()
    publishers = ros_publishers([motion], standing)
    assert [publisher["channel"] for publisher in publishers] == ["/probe", "/wrist_ft"]
    assert monitors[0].ros.pub_id == "mon_1_pub"
    assert standing[0]["pub_id"] == "wrist_ft_pub"


def test_a_quantity_no_ros_type_carries_whole_is_rejected():
    with pytest.raises(ConstraintViolation, match="no ROS type that carries it whole"):
        _standing(type_="JointPosition")


def test_a_message_that_does_not_carry_the_quantity_is_rejected():
    with pytest.raises(ConstraintViolation, match="geometry_msgs/Wrench"):
        _standing("geometry_msgs/msg/PoseStamped")


def test_a_rate_that_is_not_positive_is_rejected():
    with pytest.raises(ConstraintViolation, match="a rate says how often"):
        _standing(rate=0.0)


def test_a_message_with_no_header_states_no_frame():
    """The quantity is against a frame either way; a message with nowhere to say so does not."""
    (publish,) = _standing("geometry_msgs/msg/Wrench")
    # The message is the quantity, so there is nothing to descend through to reach it.
    assert publish["entries"][0]["payload_path"] == ""
    assert "frame_path" not in publish
    assert "frame_id" not in publish


# A message holding an array of what it reports: one entry per quantity, each stating what it is
# an entry for. `vision_msgs` is stock, so these need no workspace package.
DETECTIONS = "vision_msgs/msg/Detection3DArray"
DRAWER = URIRef(f"{NS}scene/drawer")
TABLE = URIRef(f"{NS}scene/table")


def _entry(name: str, subject, type_: str = "Pose", frame: str = "camera_optical"):
    """One reported quantity: the node the publish names, and the record it resolves to."""
    return (URIRef(f"{NS}{name.replace('_', '-')}"), subject, _reported(name, type_, frame))


def _detections(*entries, rate: float = 10.0):
    return _standing_many(DETECTIONS, rate, *entries)


def test_a_message_holding_an_array_reports_one_quantity_per_entry():
    """Every path is walked off the message class: which field holds the entries, where in one
    the pose goes, and which fields say what that entry is and when it was taken."""
    (publish,) = _detections(_entry("pose_drawer", DRAWER), _entry("pose_table", TABLE))
    assert publish["resize"] == [{"path": "detections", "size": 2}]
    first, second = publish["entries"]
    assert first["payload_path"] == "detections[0].bbox.center."
    assert second["payload_path"] == "detections[1].bbox.center."
    assert (first["id_path"], first["id_value"]) == ("detections[0].id", str(DRAWER))
    assert (second["id_path"], second["id_value"]) == ("detections[1].id", str(TABLE))
    # Each entry states the frame its own quantity is against, and is stamped in its own header.
    assert (first["frame_path"], first["frame_id"]) == (
        "detections[0].header.frame_id",
        "camera_optical",
    )
    assert first["auto_time"] == ["detections[0].header.stamp"]
    # The array's own header, and the packages the entries reach into.
    assert (publish["frame_path"], publish["frame_id"]) == ("header.frame_id", "camera_optical")
    assert "geometry_msgs" in publish["packages"]


def test_the_array_states_no_frame_when_its_entries_disagree():
    """Two cameras on one topic: each detection says where it was seen, the message cannot."""
    (publish,) = _detections(
        _entry("pose_drawer", DRAWER, frame="left_optical"),
        _entry("pose_table", TABLE, frame="right_optical"),
    )
    assert "frame_id" not in publish


def test_an_entry_naming_no_entity_is_rejected():
    with pytest.raises(ConstraintViolation, match="could not say what it is an entry for"):
        _detections(_entry("pose_drawer", None))


def test_reporting_several_quantities_on_a_message_carrying_one_is_rejected():
    with pytest.raises(ConstraintViolation, match="carries one"):
        _standing_many(
            "geometry_msgs/msg/PoseStamped",
            10.0,
            _entry("pose_drawer", DRAWER),
            _entry("pose_table", TABLE),
        )


def test_reporting_two_kinds_of_quantity_on_one_message_is_rejected():
    with pytest.raises(ConstraintViolation, match="one kind of quantity"):
        _detections(_entry("pose_drawer", DRAWER), _entry("ext_force", TABLE, "Wrench"))


def _chain(prefix: str, joints: list[str], output=(), device_output=()):
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
            "output": list(output),
            "devices": [type("D", (), {"joint_outputs": list(device_output)})()],
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


@pytest.mark.parametrize("reporter", ("output", "device_output"))
def test_a_joint_the_chain_does_not_articulate_is_still_published(reporter: str):
    """A gripper's driver joint is a mimic the chain never articulates, so whoever answers for it
    -- the bound device on hardware, the simulator otherwise -- is the only route to it, and
    iterating the chain alone drops it from the message it belongs in."""
    driver = JointPosition("gripper_pos", "r1_g_left_driver_joint")
    chain = _chain("r1_", ["j1"], **{reporter: [driver]})
    section = ros_joint_states({"config": "robot.toml"}, {"ros": {"joint_states": {}}}, [chain])
    assert [joint["name"] for joint in section["joints"]] == ["r1_j1", "r1_g_left_driver_joint"]
    gripper = section["joints"][1]
    assert gripper["position"] == "gripper_pos"
    # Neither gripper route measures one, and a zero would claim it is still and unloaded.
    assert "velocity" not in gripper and "effort" not in gripper


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
ACTION_TYPE = "control_msgs/action/GripperCommand"


def _served(goal_event: str = "E_GOAL"):
    """A served action: the channel goals arrive on, and the event an accepted one produces."""
    graph = Graph()
    graph.add((SERVER, RDF.type, NS_MM_ROS["Action"]))
    graph.add((SERVER, NS_MM_ROS["type-name"], Literal(ACTION_TYPE)))
    graph.add((SERVER, NS_MM_ROS["channel-name"], Literal("pick_place")))
    graph.add((SERVER, RDFS.member, URIRef(f"{FSM_NS}{goal_event}")))
    return graph


def _answer(
    outcome: str = "STATUS_SUCCEEDED",
    *,
    result: tuple | None = ("position", "0.5"),
    satisfied: bool = True,
):
    """A monitor that answers the goal in flight: the status it reports, and the result fields
    it states. The answer is a member of the monitor, and its own outcome member carries the
    constraint it holds under when the answering state is the satisfied one."""
    graph = Graph()
    answer = URIRef(f"{MONITOR}.answer")
    graph.add((MONITOR, CSTR_HDL["constraint"], WATCHED))
    graph.add((MONITOR, RDFS.member, answer))
    graph.add((answer, RDF.type, NS_MM_ROS["Action"]))
    graph.add((answer, NS_MM_ROS["type-name"], Literal(ACTION_TYPE)))
    graph.add((answer, NS_MM_ROS["channel-name"], Literal("pick_place")))
    node = URIRef(f"{answer}.outcome")
    graph.add((answer, RDFS.member, node))
    graph.add((node, RDF.value, Literal(outcome)))
    if satisfied:
        graph.add((node, CSTR_EXT["has-constraint"], WATCHED))
    if result is not None:
        row = URIRef(f"{answer}.f0")
        graph.add((answer, RDFS.member, row))
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
    assert server["cpp_type"] == "control_msgs::action::GripperCommand"
    assert server["result_cpp_type"] == "control_msgs::action::GripperCommand_Result"
    assert server["include"] == "control_msgs/action/gripper_command.hpp"
    # This action carries none of the fields the node fills itself: an interface that names no
    # stamp and no scenario is answered with what the model states and nothing more.
    assert server["goal_context_id"] == []
    assert server["result_auto_time"] == []
    assert server["result_auto_context_id"] == []
    assert server["ignored_goal_fields"] == []
    assert "control_msgs" in server["packages"]


def test_the_monitor_that_answers_states_the_status_and_the_fields():
    """The run reaches its finish at a monitor, so the answer is read there: the status it
    reports, the handle call that reports it, and the result type it fills."""
    answer = _ros_publication(_model(_answer()), MONITOR)["answer"]
    assert (answer.outcome, answer.method) == ("STATUS_SUCCEEDED", "succeed")
    assert answer.result_cpp_type == "control_msgs::action::GripperCommand_Result"
    assert answer.fields == [{"path": "position", "cpp_value": "0.5"}]
    assert answer.auto_time == []
    assert answer.auto_context_id == []


def test_an_answer_states_which_polarity_of_its_monitor_answers():
    """A satisfied answer holds under the constraint the monitor watches; one that states no
    condition is the otherwise, and answers when the constraint fails."""
    assert _ros_publication(_model(_answer()), MONITOR)["answer"].satisfied
    violated = _ros_publication(_model(_answer("STATUS_ABORTED", satisfied=False)), MONITOR)
    assert violated["answer"].satisfied is False
    assert violated["answer"].method == "abort"


def test_an_answer_stating_no_field_answers_with_the_types_own_defaults():
    assert _ros_publication(_model(_answer(result=None)), MONITOR)["answer"].fields == []


def test_a_result_field_the_action_does_not_offer_is_rejected():
    graph = _answer(result=("position.invented", "0.5"))
    with pytest.raises(ConstraintViolation, match="not a payload field"):
        _ros_publication(_model(graph), MONITOR)


def test_a_status_a_run_cannot_reach_on_its_own_is_rejected():
    """A cancel is the client's to ask for; the runtime reports it wherever it stops."""
    with pytest.raises(ConstraintViolation, match="answers its goal"):
        _ros_publication(_model(_answer("STATUS_CANCELED")), MONITOR)


def test_a_monitor_answering_is_not_read_as_the_served_action():
    """Every member of an answer carries a value -- each is one field of the result -- so the
    action a goal arrives on stays the one with the valueless member."""
    graph = _served()
    for triple in _answer():
        graph.add(triple)
    assert action_server(_model(graph), FSM)["action_name"] == "pick_place"


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


def _occurrence(type_name: str, *events: URIRef) -> RosPublication:
    """Lower a monitor that announces events, rather than authored fields."""
    graph = Graph()
    graph.add((MONITOR, NS_MM_ROS["channel-name"], Literal("/events")))
    graph.add((MONITOR, NS_MM_ROS["type-name"], Literal(type_name)))
    graph.add((MONITOR, CSTR_HDL["constraint"], WATCHED))
    for event in events or (EVENT,):
        graph.add((MONITOR, RDFS.member, event))
    return _ros_publication(_model(graph), MONITOR)["ros"]


def test_an_occurrence_resolves_its_field_off_the_message_type():
    """The payload field is the message's own sole leaf, not a name the generator assumes; the
    authored-row branches stay empty, since the event is the whole payload."""
    ros = _occurrence("std_msgs/msg/String")
    assert ros.occurrence_path == "data"
    assert ros.occurrence_events == [str(EVENT)]
    assert (ros.on_satisfied, ros.on_violated) == ([], [])
    assert (ros.has_satisfied, ros.has_violated) == (False, False)
    assert ros.auto_time == []
    assert ros.auto_context_id == []


def test_every_announced_event_is_carried_whatever_the_monitor_triggers():
    """The set is the author's choice, so an event the monitor never fires is carried too; each
    one becomes its own message at generation."""
    other = URIRef(f"{FSM_NS}E_OTHER")
    ros = _occurrence("std_msgs/msg/String", EVENT, other)
    assert sorted(ros.occurrence_events) == sorted([str(EVENT), str(other)])
    assert ros.occurrence_path == "data"


def test_an_occurrence_needs_a_field_that_can_hold_an_iri():
    with pytest.raises(ConstraintViolation, match="must offer a string"):
        _occurrence("std_msgs/msg/Float64")
