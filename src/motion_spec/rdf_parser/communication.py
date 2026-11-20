# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What crosses the program's boundary.

In order: the rows that say how to read a recorded run; the frame-log samples; the ROS interface
-- what the model publishes, the goals it sends and the one it answers, and the goal status a sent
goal writes back; and `build_telemetry`, which taps the data access analysis for what the run
records, in the one order the frame layout depends on.

No other module appends to the telemetry artifact or to the frame log.
"""

from __future__ import annotations

from motion_spec_dsl.vocab import CSTR, CSTR_EXT, CSTR_HDL, MOT, SENSORS
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdflib.namespace import PROV, RDF, RDFS, SOSA
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.base import unique_by_id
from motion_spec.classes.geometry import Direction, Orientation, Pose, Position, SpatialCoordinate
from motion_spec.classes.handlers import (
    EdgeMonitor,
    FeedForwardController,
    LevelMonitor,
    PIDController,
)
from motion_spec.classes.motion import DataValue
from motion_spec.classes.qudt import FreeVector, Quantity, SetpointQuantity
from motion_spec.rdf_parser import constraint_handler, quantities, ros_messages
from motion_spec.rdf_parser.data_access import PORT_WRITERS, bound_id
from motion_spec.rdf_parser.views import views_by_subobject

_SPATIAL_SLOT_KINDS = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}


_RUNTIME_ROW_FIELDS = {
    "controller_internal_state": ("controller", "role", "state"),
    "control_parameter": ("value", "owner", "role", "parameter"),
    "joint_space": ("quantity_kind", "unit", "runtime", "role", "channel", "joint", "writer"),
    None: ("value",),
}


def runtime_row(member) -> dict:
    fields = _RUNTIME_ROW_FIELDS.get(member.role if isinstance(member, DataValue) else None, ())
    return {
        "id": member.id,
        "type": member.type,
        **{name: getattr(member, name) for name in fields if getattr(member, name) is not None},
    }


def member_desc(member: str):
    return lambda quantity_id, axis: {
        "kind": "member",
        "id": quantity_id,
        "member": member,
        "axis": axis,
    }


# key order inside each descriptor is load-bearing for the frame layout
_POSE_AXIS_SAMPLES = (
    ("position", lambda quantity_id, axis: {"kind": "pose_pos", "id": quantity_id, "axis": axis}),
    (
        "orientation",
        lambda quantity_id, axis: {"kind": "pose_orient", "id": quantity_id, "axis": axis},
    ),
)
_TWIST_AXIS_SAMPLES = (("angular", member_desc("rot")), ("linear", member_desc("vel")))
_VECTOR_AXIS_SAMPLES = (
    ("", lambda quantity_id, axis: {"kind": "vec", "id": quantity_id, "axis": axis}),
)
_AXIS_SAMPLES = {
    "Position": _VECTOR_AXIS_SAMPLES,
    "Direction": _VECTOR_AXIS_SAMPLES,
    "FreeVector": _VECTOR_AXIS_SAMPLES,
    "Orientation": (
        ("", lambda quantity_id, axis: {"kind": "orientation", "id": quantity_id, "axis": axis}),
    ),
    "Pose": _POSE_AXIS_SAMPLES,
    "SetpointQuantity": _POSE_AXIS_SAMPLES,
    "VelocityTwist": _TWIST_AXIS_SAMPLES,
    "AccelerationTwist": _TWIST_AXIS_SAMPLES,
    "PoseDifference": _TWIST_AXIS_SAMPLES,
    "Wrench": (("torque", member_desc("torque")), ("force", member_desc("force"))),
}
_COMPOSITE_SUPEROBJECTS = {
    "Pose",
    "Wrench",
    "VelocityTwist",
    "AccelerationTwist",
    "FreeVector",
    "Direction",
}


def scalar_descriptor(quantity: dict, quantity_id: str, data_ids, spatial_ids, indexed_views):
    """How a scalar quantity is sampled: as a literal, through a view, or off its own field."""
    if quantity_id not in indexed_views:
        if quantity.get("value") is not None and quantity_id not in data_ids:
            return {"kind": "literal", "value": str(quantity["value"])}

        return {"kind": "data", "id": quantity_id} if quantity_id in data_ids else None
    views = indexed_views[quantity_id]
    if not all(view.axis is not None for view in views):
        # a directionless view would repeat its logged superobject slot
        if quantity_id in data_ids and not any(
            view.direction is None and view.superobject.id in spatial_ids for view in views
        ):
            return {"kind": "data", "id": quantity_id}
        return None
    types = {view.superobject.type for view in views}
    if len(types) != 1:
        raise ConstraintViolation(
            "telemetry", f"quantity sampling: '{quantity_id}' belongs to incompatible MAP views"
        )
    if next(iter(types)) in _COMPOSITE_SUPEROBJECTS:
        return {"kind": "access", "ref": quantity_id}

    return {"kind": "data", "id": quantity_id}


def publisher_member(by_channel: dict, entry: dict) -> str:
    """The publisher member a channel's writers share, added on first use.

    Raises:
        ConstraintViolation: one channel is written as two message types, so the one member they
            share could only be typed as one of them.
    """
    publisher = by_channel.setdefault(entry["channel"], entry)
    if publisher["cpp_type"] != entry["cpp_type"]:
        raise ConstraintViolation(
            "communication",
            f"channel '{entry['channel']}' is published as both '{publisher['cpp_type']}' and "
            f"'{entry['cpp_type']}'; one channel carries one message type",
        )

    return publisher["pub_id"]


def ros_publishers(motions, standing=()) -> list:
    """The ROS publishers the model asks for, one per distinct topic.

    Codegen links rclcpp and sets up publishers only when a model publishes at all, so a model
    with none contributes nothing rather than an empty section.
    """
    by_channel: dict = {}
    for ros in ros_publications(motions):
        # writers sharing a topic share the publisher member
        ros.pub_id = publisher_member(
            by_channel, {key: getattr(ros, key) for key in _PUBLISHER_KEYS}
        )
    for publish in standing:
        publish["pub_id"] = publisher_member(
            by_channel, {key: publish.get(key) for key in _PUBLISHER_KEYS}
        )

    return list(by_channel.values())


_PUBLISHER_KEYS = (
    "pub_id",
    "channel",
    "cpp_type",
    "include",
    "pkg",
    "auto_time",
    "auto_context_id",
)


def ros_standing(model, data_structures, control_period_ns: int, segment_by_iri: dict) -> list:
    """The topics published for the whole run, one per standing publish.

    A standing publish reports readings rather than a verdict, so it belongs to the run and keeps
    publishing between motions. Its rate is the model's, stated on the topic: it becomes the
    number of control cycles between two messages, since the loop is the fastest it can go.

    Raises:
        ConstraintViolation: the publish names no quantity, states no positive rate, reports a
            quantity that never reaches the algorithm data, or maps a field the message does not
            offer.
    """
    graph = model.graph
    by_id = {item.id: item for item in data_structures}
    period_s = control_period_ns * 1e-9
    standing = []
    for node in graph.subjects(RDF["type"], NS_MM_ROS["Topic"]):
        rate_node = graph.value(node, SENSORS["update-rate"])
        if rate_node is None or CSTR_HDL["Monitor"] in get_node_types(graph, node):
            continue
        reported = [
            row
            for row in graph.objects(node, RDFS.member)
            if graph.value(row, NS_MM_ROS["field-path"]) is None
        ]
        if not reported:
            raise ConstraintViolation(
                "communication",
                f"'{model.id(node)}' publishes at a rate but names no quantity to report",
            )
        records = []
        for row in reported:
            value_node = graph.value(row, RDF.value)
            record = by_id.get(model.id(value_node))
            if record is None:
                raise ConstraintViolation(
                    "communication",
                    f"'{model.id(node)}' publishes '{model.id(value_node)}', which is no data "
                    "structure the run computes",
                )
            records.append((row, record))
        kinds = {record.type for _row, record in records}
        if len(kinds) > 1:
            raise ConstraintViolation(
                "communication",
                f"'{model.id(node)}' reports {', '.join(kinds)} on one message; a "
                "message carries one kind of quantity",
            )
        rate_hz = quantities.quantity(model, rate_node).value
        if not rate_hz or rate_hz <= 0.0:
            raise ConstraintViolation(
                "communication",
                f"'{model.id(node)}' publishes at {rate_hz} Hz; a rate says how often, so it is "
                "positive",
            )
        pub_id = f"{model.id(node)}_pub".replace("-", "_")
        type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
        shape = ros_messages.standing_shape(type_name, records[0][1].type)
        entry_shape = shape["entry"]
        if entry_shape is None and len(records) > 1:
            raise ConstraintViolation(
                "communication",
                f"'{model.id(node)}' reports {len(records)} quantities on '{type_name}', which "
                "carries one; a message reporting many holds an array of them",
            )
        entries = []
        for index, (member, record) in enumerate(records):
            entry = {
                "pub_id": pub_id,
                "value_id": record.id,
                "value_type": record.type,
                "carrier": shape["carrier"],
            }
            if entry_shape is None:
                entry["payload_path"] = shape["payload_path"]
                entries.append(entry)
                continue
            at = f"{entry_shape['path']}[{index}]."
            stated_in = entry_shape["frame_path"]
            frame_id = stated_against(record, segment_by_iri, pub_id) if stated_in else None
            if shape["carrier"] == "Transform":
                subject_id = segment_of(segment_by_iri, record.of.uri, pub_id)
            else:
                subject = graph.value(member, SOSA.hasFeatureOfInterest)
                if subject is None:
                    raise ConstraintViolation(
                        "communication",
                        f"'{record.id}' is one entry of an array the message carries, but the "
                        "publish names no entity it reports it of, so the entry could not say "
                        "what it is an entry for",
                    )
                subject_id = str(subject)
            entry.update(
                payload_path=f"{at}{entry_shape['payload_path']}",
                frame_path=f"{at}{stated_in}" if stated_in else None,
                frame_id=frame_id,
                id_path=f"{at}{entry_shape['id_path']}",
                id_value=subject_id,
                auto_time=[f"{at}{path}" for path in entry_shape["auto_time"]],
            )
            entries.append(entry)
        frames = (
            {
                frame
                for _row, record in records
                if (frame := stated_against(record, segment_by_iri, pub_id))
            }
            if shape["frame_path"]
            else set()
        )
        fields = []
        for member in graph.objects(node, RDFS.member):
            authored = graph.value(member, NS_MM_ROS["field-path"])
            if authored is None:
                continue
            path = str(authored)
            if path not in shape["leaves"]:
                raise ConstraintViolation(
                    "communication",
                    f"'{path}' is not a payload field of '{shape['type_name']}'; it offers "
                    f"{', '.join(shape['leaves']) or 'none'}",
                )
            fields.append({"path": path, "ref": model.id(graph.value(member, RDF.value))})
        publish = {
            "pub_id": pub_id,
            "channel": str(graph.value(node, NS_MM_ROS["channel-name"]) or ""),
            # at or above the loop rate, every cycle
            "divider": max(1, round(1.0 / (rate_hz * period_s))),
            "entries": [
                {key: value for key, value in entry.items() if value is not None and value != []}
                for entry in entries
            ],
            # a frame only when all entries agree on one
            "frame_id": frames.pop() if len(frames) == 1 else None,
            "resize": [{"path": entry_shape["path"], "size": len(entries)}] if entry_shape else [],
            "fields": fields,
            "pkg": shape["package"],
            **{
                key: shape[key]
                for key in (
                    "type_name",
                    "cpp_type",
                    "include",
                    "frame_path",
                    "auto_time",
                    "auto_context_id",
                )
            },
        }
        standing.append(
            {key: value for key, value in publish.items() if value is not None and value != []}
        )

    return standing


def stated_against(record, segment_by_iri: dict, where: str) -> str | None:
    """The segment a reported quantity is stated against, when it is stated against one."""
    if (
        not isinstance(record, (Direction, Position, Orientation, Pose, SpatialCoordinate))
        or record.as_seen_by is None
    ):
        return None

    return segment_of(segment_by_iri, record.as_seen_by.uri, where)


def ros_action_clients(model) -> list:
    """The ROS action clients the detect acts ask for, one per act.

    An act is its own client: it names the channel the goal goes out on, the scene objects the
    goal asks for, and -- per object -- the world pose the result writes and the frame a
    detection has to arrive in to be that pose.
    """
    graph = model.graph
    written = quantities.perceived_written_poses(model)
    clients = []
    for act in graph.subjects(RDF["type"], NS_MM_ROS["Action"]):
        if (act, RDFS.member, None) in graph:
            continue
        type_name = str(graph.value(act, NS_MM_ROS["type-name"]) or "")
        shape = ros_messages.action_shape(type_name)
        detect = ros_messages.detect_shape(
            type_name, str(graph.value(act, NS_MM_ROS["field-path"]) or "")
        )
        status_slot = graph.value(predicate=PROV.wasDerivedFrom, object=act)
        if status_slot is None:
            raise ConstraintViolation(
                "communication", f"action '{model.id(act)}' declares no goal-status slot"
            )
        rows = written[str(act)]
        watching = set(graph.subjects(CSTR["quantity"], status_slot))
        for motion in graph.subjects(RDF["type"], MOT["GuardedMotion"]):
            members = set(graph.objects(motion, MOT["until"]))
            members |= {
                member
                for item in members
                for member in graph.objects(item, CSTR_EXT["has-constraint"])
            }
            if members & watching:
                motion_id = model.id(motion)
                break
        else:
            raise ConstraintViolation(
                "communication",
                f"no motion's 'until' reads '{model.id(status_slot)}', so nothing sends the goal "
                "or ends the act; compare the act's status in the motion that detects",
            )
        clients.append(
            {
                "act_id": model.id(act),
                "client_id": f"{model.id(act)}_client",
                "channel": str(graph.value(act, NS_MM_ROS["channel-name"]) or ""),
                "type_name": type_name,
                "cpp_type": shape["cpp_type"],
                "include": shape["include"],
                "pkg": shape["package"],
                "status_id": model.id(status_slot),
                "motion": motion_id,
                "target_iris": list({row["target_iri"] for row in rows}),
                "written_poses": rows,
                **detect,
            }
        )

    return clients


def ros_subscriptions(model, segment_by_iri: dict) -> list:
    """The topics the model reads object poses off, one per channel.

    A topic the model subscribes to is one it states features of interest for: it says which
    objects the channel informs it about and -- per object -- the world pose a detection writes.
    A topic the model publishes carries field rows and no feature of interest, so it is not one
    of these.

    A detection arrives stated in whatever frame its sender put in the header, which is a
    camera's and not the frame the world pose is stated against. Only the pose's own frame is
    the model's to state, so only that one is resolved here to the world-model segment the
    runtime reads it off; the sender's is resolved by name as each message arrives, since one
    channel may carry two cameras and only the header tells them apart.

    Parameters:
        segment_by_iri: every scene element the built tree carries, by IRI, as `robot_setups`
            resolved it

    Raises:
        ConstraintViolation: a pose a detection writes is stated against a frame the built tree
            has no segment for, so the runtime could not ask where it is.
    """
    graph = model.graph
    written = quantities.perceived_written_poses(model)
    subscriptions = []
    for node in graph.subjects(RDF["type"], NS_MM_ROS["Topic"]):
        rows = written.get(str(node))
        if not rows:
            continue
        sub_id = model.id(node)
        type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
        shape = ros_messages.observation_shape(
            type_name, str(graph.value(node, NS_MM_ROS["field-path"]) or "")
        )
        subscriptions.append(
            {
                "sub_id": sub_id,
                "channel": str(graph.value(node, NS_MM_ROS["channel-name"]) or ""),
                "type_name": type_name,
                "cpp_type": shape["cpp_type"],
                "include": shape["include"],
                "pkg": shape["package"],
                "written_poses": [
                    {
                        **row,
                        "frame_segment": segment_of(segment_by_iri, row["frame_iri"], sub_id),
                        "observed_wrt_segment": segment_of(
                            segment_by_iri, row.get("observed_wrt_iri") or row["frame_iri"], sub_id
                        ),
                        "observed_of_segment": segment_of(
                            segment_by_iri,
                            row.get("observed_of_iri") or row["target_of_iri"],
                            sub_id,
                        ),
                        "target_of_segment": segment_of(
                            segment_by_iri, row["target_of_iri"], sub_id
                        ),
                        "observed_body_segment": segment_of(
                            segment_by_iri,
                            row.get("observed_body_iri") or row["target_of_iri"],
                            sub_id,
                        ),
                        "reframed": bool(row.get("observed_of_iri")),
                        "match_iri": row.get("observed_of_iri") or row["target_iri"],
                    }
                    for row in rows
                ],
                **{
                    key: shape[key]
                    for key in (
                        "detections_path",
                        "id_path",
                        "frame_path",
                        "pose_path",
                        "pose_container",
                    )
                },
            }
        )

    return subscriptions


def segment_of(segment_by_iri: dict, iri, where: str) -> str:
    """The world-model segment standing for one frame, as the built tree names it.

    Resolved here rather than at run time: the pose's frame is the model's own statement, so a
    frame the tree does not carry is a broken model, not a message to drop.
    """
    segment = segment_by_iri.get(str(iri))
    if segment is None:
        raise ConstraintViolation(
            "communication",
            f"topic '{where}' reads a pose against '{iri}', but the built tree carries no "
            "segment standing for it, so the world model cannot be asked where it is.",
        )

    return segment


def add_goal_status_values(model, algorithm_data: list, action_clients) -> None:
    """Give each act's goal status a D-block the client writes and the until reads.

    Without one the status is nowhere: the monitor has nothing to compare and the run nothing to
    record.
    """
    present = {item.id for item in algorithm_data}
    for client in action_clients:
        status_id = client["status_id"]
        parent = model.iri_of(status_id)
        if parent is None:
            raise RuntimeError(f"goal status: '{status_id}' has no IRI to derive from")
        if status_id not in present:
            present.add(status_id)
            algorithm_data.append(DataValue(id=status_id, type="IntCounter"))
        model.register_derived(status_id, parent, "status", PROV.wasDerivedFrom)


def action_server(model, fsm) -> dict | None:
    """The action server the model declares, or None when it declares none.

    Everything the generated server needs comes off the action the model named: the C++ type it
    instantiates, its header, the goal field carrying the scenario a run belongs to, and the
    result fields the node owns rather than the model. What the goal is answered with comes off
    the monitor that answers it, which is where the run reaches that point.

    Raises:
        ConstraintViolation: the model serves goals without importing an FSM, or names an event
            that FSM does not declare or react to -- either way nothing could start
    """
    graph = model.graph
    served = [
        node
        for node in graph.subjects(RDF["type"], NS_MM_ROS["Action"])
        if any(
            graph.value(member, RDF.value) is None for member in graph.objects(node, RDFS.member)
        )
    ]
    if len(served) > 1:
        raise ConstraintViolation(
            "communication",
            f"the model serves {len(served)} actions ({', '.join(model.id(n) for n in served)}); "
            "a runtime answers one",
        )
    if not served:
        return None
    node = served[0]
    if not fsm:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' serves goals, but the model imports no FSM, so an "
            "accepted goal has nothing to start",
        )

    goal_events = [
        row for row in graph.objects(node, RDFS.member) if graph.value(row, RDF.value) is None
    ]
    if len(goal_events) != 1:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' names {len(goal_events)} events for an accepted goal to "
            "produce; it needs exactly one",
        )
    (goal_event,) = goal_events
    # tokens, never indices: coord-dsl numbers the events
    token_by_uri = {uri: token for token, uri in fsm["event_uris"].items()}
    goal_token = token_by_uri.get(str(goal_event))
    if goal_token is None:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' names event '{goal_event}', which FSM '{fsm['name']}' does "
            "not declare",
        )
    # an FSM event lives one tick: fire only in a state that reacts to it
    transitions = {row["id"]: row for row in fsm["transitions_table"]}
    armed_states = [
        transitions[row["do_transition"]]["from_state"]
        for row in fsm["reactions_table"]
        if row["when_event"] == goal_token
    ]
    if not armed_states:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' produces '{goal_token}' on a goal, but no FSM reaction "
            "consumes it, so an accepted goal could never start anything",
        )

    type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
    shape = ros_messages.action_shape(type_name)
    goal, result = shape["goal"], shape["result"]

    return {
        "action_name": str(graph.value(node, NS_MM_ROS["channel-name"])),
        "type_name": type_name,
        "cpp_type": shape["cpp_type"],
        "result_cpp_type": result["cpp_type"],
        "include": shape["include"],
        "pkg": shape["package"],
        "goal_event": goal_token,
        "goal_states": armed_states,
        "goal_context_id": [path for path, kind in goal["auto"].items() if kind == "context_id"],
        "ignored_goal_fields": goal["repeated"],
        "result_auto_time": [path for path, kind in result["auto"].items() if kind == "time"],
        "result_auto_context_id": [
            path for path, kind in result["auto"].items() if kind == "context_id"
        ],
    }


def ros_publications(motions):
    """Every monitor publish in the model, in motion order."""
    return [
        monitor.ros
        for motion in motions
        for monitor in [*motion.when_monitors, *motion.while_monitors, *motion.until_monitors]
        if monitor.ros is not None
    ]


def annotate_publish_rates(motions, control_period_ns: int) -> None:
    """Turn each monitor publish's authored rate into cycles between two messages.

    Done here rather than where the publish is read, because a rate says how often in seconds and
    only the loop knows how many cycles that is. A publish stating no rate goes out every cycle
    its motion is active, and carries no divider to say so.
    """
    period_s = control_period_ns * 1e-9
    for publication in ros_publications(motions):
        if publication.rate_hz:
            publication.divider = max(1, round(1.0 / (publication.rate_hz * period_s)))


def build_telemetry(
    model, motions, computation, algorithm_data, data_access, analysis, control_period_ns
):
    """What the run records, and how to read it back.

    A logging stream tapping the algorithm data, adding nothing to it (Bruyninckx 2026, §2.5.15:
    a data stream with its metadata stream): a slot per tick for each D-block the analysis
    finds written per tick, gated by the motions that write it; a header slot for each value
    written once; and the rows saying which motion, constraint and signal each slot belongs to.

    The order here is the frame layout: list order becomes the positional indices the frame log and
    the generated struct are built from.

    Raises:
        RuntimeError: a slot samples a value the analysis has no answer for.
    """
    uri_by_id = {row["id"]: row["uri"] for row in model.uri_rows()}
    # by IRI: until terms may share a short id
    errors = {
        function["constraint_uri"]: function.get("error")
        for function in computation.functions.values()
        if function.get("type") == "ErrorEvaluator" and function.get("constraint_uri")
    }
    # unset keys are dropped: absent, not null
    motion_rows, controller_rows, monitor_rows = [], [], []
    for motion in motions:
        motion_row = {
            "id": motion.id,
            "uri": uri_by_id.get(motion.id),
            "motion": motion.motion_id,
            "motion_uri": uri_by_id.get(motion.motion_id),
            "controllers": [controller.id for controller in motion.controllers],
            "monitors": [
                monitor.id
                for monitor in (
                    *motion.when_monitors,
                    *motion.while_monitors,
                    *motion.until_monitors,
                )
            ],
        }
        motion_rows.append(
            {key: value for key, value in motion_row.items() if value is not None and value != []}
        )
        for controller in motion.controllers:
            controller_row = {
                "id": controller.id,
                "uri": uri_by_id.get(controller.id),
                "motion": motion.id,
                "type": controller.type,
                "constraint": controller.constraint,
                "constraint_uri": controller.constraint_uri,
                **{
                    source: getattr(controller, source)
                    for _name, source, _required in constraint_handler.CONTROLLER_GAIN_FIELDS.get(
                        controller.type, ()
                    )
                },
                "error_signal": bound_id(controller.error_signal),
                "tolerance_signal": controller.tolerance_id or None,
                "reference_signal": (
                    bound_id(controller.reference_signal)
                    if isinstance(controller, FeedForwardController)
                    else None
                ),
                "measured_derivative": (
                    bound_id(controller.measured_derivative)
                    if isinstance(controller, PIDController)
                    else None
                ),
                "output_signal": bound_id(controller.control_signal),
                "measured_signal": controller.measured_signal,
                "setpoint_signal": controller.setpoint_signal,
            }
            controller_rows.append(
                {
                    key: value
                    for key, value in controller_row.items()
                    if value is not None and value != []
                }
            )
        for phase, monitors in (
            ("when", motion.when_monitors),
            ("while", motion.while_monitors),
            ("until", motion.until_monitors),
        ):
            for monitor in monitors:
                edge = monitor if isinstance(monitor, EdgeMonitor) else None
                monitor_row = {
                    "id": monitor.id,
                    "uri": uri_by_id.get(monitor.id),
                    "motion": motion.id,
                    "phase": phase,
                    "type": monitor.monitor_type,
                    "trigger": "edge" if monitor.is_edge_triggered else "level",
                    "event": edge.event if edge else None,
                    "event_uri": edge.event_uri if edge else None,
                    "event_name": edge.event_name if edge else None,
                    "flag": monitor.flag if isinstance(monitor, LevelMonitor) else None,
                    "error_signal": bound_id(monitor.error),
                    "tolerance_signal": bound_id(monitor.tolerance),
                    "constraint_ids": monitor.constraint_ids,
                    "constraint_uris": monitor.constraint_uris,
                    "watched": [
                        {
                            key: value
                            for key, value in {
                                "id": member,
                                "uri": uri,
                                "error_signal": errors.get(uri),
                                "tolerance_signal": band or None,
                            }.items()
                            if value is not None and value != []
                        }
                        for member, uri, band in zip(
                            monitor.group_constraint_ids,
                            monitor.group_constraint_uris,
                            monitor.group_constraint_tolerances,
                            strict=True,
                        )
                    ],
                    "fallback_motion": edge.fallback_motion if edge else None,
                    "debounce_signal": edge.debounce_id if edge else None,
                }
                monitor_rows.append(
                    {
                        key: value
                        for key, value in monitor_row.items()
                        if value is not None and value != []
                    }
                )
    snapshot_ids = quantities.snapshot_target_ids(model)
    structure_ids = {item.id for item in computation.data_structures}
    quantity_rows = []
    for item in algorithm_data:
        if item.id in structure_ids:
            quantity_row = {
                "id": item.id,
                "uri": uri_by_id.get(item.id),
                "type": item.type,
                "reference_value": item.reference_value if isinstance(item, Quantity) else None,
                "value": item.value if isinstance(item, (Quantity, DataValue)) else None,
                "authored": isinstance(
                    item,
                    (Quantity, FreeVector, SetpointQuantity, Orientation, Pose, SpatialCoordinate),
                )
                and item.provenance.authored,
                "snapshot": item.id in snapshot_ids,
            }
            quantity_rows.append(
                {
                    key: value
                    for key, value in quantity_row.items()
                    if value is not None and value != []
                }
            )
        elif isinstance(item, DataValue) and item.role and item.type == "Quantity":
            quantity_rows.append({**runtime_row(item), "uri": uri_by_id.get(item.id)})

    telemetry = {
        "contract_version": 2,
        "control_period_ns": control_period_ns,
        "motions": motion_rows,
        "controllers": unique_by_id(controller_rows),
        "monitors": unique_by_id(monitor_rows),
        "quantities": unique_by_id(quantity_rows),
    }

    data_ids = {item.id for item in algorithm_data if item.id}
    spatial_ids = {
        item.id for item in algorithm_data if item.type in _SPATIAL_SLOT_KINDS and item.id
    }
    indexed_views = views_by_subobject(computation.views)
    sampled_parts = []  # (source row, component, sample descriptor)
    for quantity in telemetry["quantities"]:
        quantity_id = quantity.get("id")
        if not quantity_id or quantity_id in spatial_ids:
            continue
        if quantity.get("type") != "Quantity":
            if quantity_id not in data_ids:
                continue
            for prefix, make_desc in _AXIS_SAMPLES.get(quantity.get("type"), ()):
                for index, name in enumerate("xyz"):
                    component = f"{prefix}.{name}" if prefix else name
                    sampled_parts.append((quantity, component, make_desc(quantity_id, index)))
            continue
        desc = scalar_descriptor(quantity, quantity_id, data_ids, spatial_ids, indexed_views)
        if desc is not None:
            sampled_parts.append((quantity, "", desc))

    sampled = {source.get("id") for source, _component, _desc in sampled_parts}
    for item in algorithm_data:
        if not item.id or item.id in sampled:
            continue
        if item.type == "Bool":
            sampled_parts.append((runtime_row(item), "", {"kind": "bool", "id": item.id}))
        elif item.type == "IntCounter":
            sampled_parts.append((runtime_row(item), "", {"kind": "int", "id": item.id}))
        elif item.id in PORT_WRITERS:
            sampled_parts.append((runtime_row(item), "", {"kind": "data", "id": item.id}))

    logged, recorded = [], []
    for source, component, desc in sampled_parts:
        sample = {
            **{key: value for key, value in source.items() if key != "index"},
            "id": source["id"] if not component else f"{source['id']}.{component}",
            "source_id": source["id"],
            "component": component or None,
            "type": "Scalar",
            "source_type": source.get("type"),
            "sample_desc": desc,
        }
        found = analysis.get(sample["source_id"])
        if found is None:
            raise RuntimeError(f"telemetry: '{sample['id']}' samples a value with no analysis")
        if found["storage"] == "log":
            logged.append({**sample, "cadence": found["cadence"]})
        # a draw is recorded by the run, not the header
        elif data_access[sample["source_id"]]["write"]["kind"] != "sampled":
            constant = {
                "id": sample["id"],
                "source_id": sample["source_id"],
                "uri": sample.get("uri"),
                "sample_desc": sample["sample_desc"],
            }
            recorded.append(
                {key: value for key, value in constant.items() if value is not None and value != []}
            )
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in algorithm_data:
        pool = _SPATIAL_SLOT_KINDS.get(item.type)
        if item.id and pool is not None and analysis[item.id]["storage"] == "log":
            spatial[pool].append(
                {
                    "id": item.id,
                    "index": len(spatial[pool]),
                    "uri": uri_by_id.get(item.id),
                    "cadence": analysis[item.id]["cadence"],
                }
            )
    telemetry.update(quantity_samples=logged, spatial_samples=spatial, constants=recorded)

    return telemetry
