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

from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, CSTR_HDL, MOT, SENSORS
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

# Types that get a whole-object frame-log slot rather than per-axis scalar rows.
_SPATIAL_SLOT_KINDS = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}


def _prune(row: dict) -> dict:
    """Drop the keys a row leaves unset, so an absent value is absent rather than null."""
    return {key: value for key, value in row.items() if value is not None and value != []}


def _controller_row(controller, motion_id: str, uri_by_id: dict) -> dict:
    """One controller's telemetry row."""
    return _prune(
        {
            "id": controller.id,
            "uri": uri_by_id.get(controller.id),
            "motion": motion_id,
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
            # Folded onto the record by `constraint_handler.annotate_controller_signals`, which reads the
            # error-evaluator function this controller consumes.
            "measured_signal": controller.measured_signal,
            "setpoint_signal": controller.setpoint_signal,
        }
    )


def _quantity_row(item, uri_by_id: dict, snapshot_ids: set) -> dict:
    """One data structure's telemetry row: what it is, and where its value came from.

    `snapshot` is a graph fact, not `Provenance`'s -- looked up by id against the snapshot
    targets `quantities.snapshot_target_ids` collects, since this reader holds a record, not a
    graph node.
    """
    return _prune(
        {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "reference_value": item.reference_value if isinstance(item, Quantity) else None,
            "value": item.value if isinstance(item, (Quantity, DataValue)) else None,
            "authored": isinstance(
                item, (Quantity, FreeVector, SetpointQuantity, Orientation, Pose, SpatialCoordinate)
            )
            and item.provenance.authored,
            "snapshot": item.id in snapshot_ids,
        }
    )


def _watched_constraints(monitor, functions: dict, uri_by_id: dict) -> list:
    """What an aggregate monitor watches: each member constraint and the error it is evaluated on.

    The scalar a conjunction writes says only whether every member holds. A reader asking why
    it does not needs the members: each one's error, as its evaluator writes it, and the
    tolerance that error is compared with.
    """
    # By IRI: two motions' until terms may share a short id.
    errors = {
        function["constraint_uri"]: function.get("error")
        for function in functions.values()
        if function.get("type") == "ErrorEvaluator" and function.get("constraint_uri")
    }
    return [
        _prune(
            {
                "id": member,
                "uri": uri,
                "error_signal": errors.get(uri),
                "tolerance_signal": band or None,
            }
        )
        for member, uri, band in zip(
            monitor.group_constraint_ids or (),
            monitor.group_constraint_uris or (),
            monitor.group_constraint_tolerances or [""] * len(monitor.group_constraint_ids or ()),
            strict=False,
        )
    ]


def _motion_rows(motions, uri_by_id: dict, functions: dict):
    """The motion, controller and monitor rows, walked in one pass over the motions."""
    motion_rows, controller_rows, monitor_rows = [], [], []
    for motion in motions:
        motion_rows.append(
            _prune(
                {
                    "id": motion.id,
                    "uri": uri_by_id.get(motion.id),
                    "motion": motion.motion_id,
                    "motion_uri": uri_by_id.get(motion.motion_id),
                    "controllers": [controller.id for controller in motion.controllers],
                    "monitors": [
                        monitor.id
                        for group in (
                            motion.when_monitors,
                            motion.while_monitors,
                            motion.until_monitors,
                        )
                        for monitor in group
                    ],
                }
            )
        )
        controller_rows += [
            _controller_row(controller, motion.id, uri_by_id) for controller in motion.controllers
        ]
        for phase, monitors in (
            ("when", motion.when_monitors),
            ("while", motion.while_monitors),
            ("until", motion.until_monitors),
        ):
            for monitor in monitors:
                edge = monitor if isinstance(monitor, EdgeMonitor) else None
                monitor_rows.append(
                    _prune(
                        {
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
                            "watched": _watched_constraints(monitor, functions, uri_by_id),
                            "fallback_motion": edge.fallback_motion if edge else None,
                            "debounce_signal": edge.debounce_id if edge else None,
                        }
                    )
                )

    return motion_rows, controller_rows, monitor_rows


# Per role, the descriptive fields a runtime value reports beyond its id and type, in the order
# the artifact emits them. A value with no role -- the clock, the tare state -- reports only what
# it holds.
_RUNTIME_ROW_FIELDS = {
    "controller_internal_state": ("controller", "role", "state"),
    "control_parameter": ("value", "owner", "role", "parameter"),
    "joint_space": ("quantity_kind", "unit", "runtime", "role", "channel", "joint", "writer"),
    None: ("value",),
}


def _runtime_row(member) -> dict:
    fields = _RUNTIME_ROW_FIELDS.get(member.role if isinstance(member, DataValue) else None, ())
    return {
        "id": member.id,
        "type": member.type,
        **{name: getattr(member, name) for name in fields if getattr(member, name) is not None},
    }


def _vec_desc(kind: str):
    return lambda quantity_id, axis: {"kind": kind, "id": quantity_id, "axis": axis}


def _member_desc(member: str):
    return lambda quantity_id, axis: {
        "kind": "member",
        "id": quantity_id,
        "member": member,
        "axis": axis,
    }


# Per quantity type, the `(component prefix, descriptor)` pairs its per-axis rows carry, in
# emission order. Key order inside each descriptor is load-bearing for the frame layout.
_POSE_AXIS_SAMPLES = (
    ("position", _vec_desc("pose_pos")),
    ("orientation", _vec_desc("pose_orient")),
)
_TWIST_AXIS_SAMPLES = (("angular", _member_desc("rot")), ("linear", _member_desc("vel")))
_AXIS_SAMPLES = {
    "Position": (("", _vec_desc("vec")),),
    "Direction": (("", _vec_desc("vec")),),
    "FreeVector": (("", _vec_desc("vec")),),
    "Orientation": (("", _vec_desc("orientation")),),
    "Pose": _POSE_AXIS_SAMPLES,
    "SetpointQuantity": _POSE_AXIS_SAMPLES,
    "VelocityTwist": _TWIST_AXIS_SAMPLES,
    "AccelerationTwist": _TWIST_AXIS_SAMPLES,
    "PoseDifference": _TWIST_AXIS_SAMPLES,
    "Wrench": (("torque", _member_desc("torque")), ("force", _member_desc("force"))),
}
# The superobject types whose scalar view resolves to a composite-member access; a view onto any
# other superobject samples the quantity's own shared field instead.
_COMPOSITE_SUPEROBJECTS = {
    "Pose",
    "Wrench",
    "VelocityTwist",
    "AccelerationTwist",
    "FreeVector",
    "Direction",
}


def add_quantity_samples(telemetry: dict, algorithm_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample rows.

    Each row carries a backend-agnostic descriptor -- a kind plus ids and an axis -- and the view
    renders the sampling expression from it.

    Raises:
        ConstraintViolation: a quantity belongs to MAP views whose superobjects disagree on how to
            reach it.
    """
    data_ids = {item.id for item in algorithm_data if item.id}
    spatial_ids = {
        item.id for item in algorithm_data if item.type in _SPATIAL_SLOT_KINDS and item.id
    }
    indexed_views = views_by_subobject(views)
    sampled_parts = []  # (source row, component, sample descriptor)
    for quantity in telemetry["quantities"]:
        quantity_id = quantity.get("id")
        if not quantity_id or quantity_id in spatial_ids:
            continue
        if quantity.get("type") != "Quantity":
            for prefix, make_desc in _AXIS_SAMPLES.get(quantity.get("type"), ()):
                if quantity_id not in data_ids:
                    continue
                for index, name in enumerate(("x", "y", "z")):
                    component = f"{prefix}.{name}" if prefix else name
                    sampled_parts.append((quantity, component, make_desc(quantity_id, index)))
            continue
        desc = _scalar_descriptor(quantity, quantity_id, data_ids, spatial_ids, indexed_views)
        if desc is not None:
            sampled_parts.append((quantity, "", desc))

    sampled = {source.get("id") for source, _component, _desc in sampled_parts}
    for item in algorithm_data:
        if not item.id or item.id in sampled:
            continue
        # A runtime value has no `quantities` row of its own to be sampled from, so its row is
        # built here from what it carries.
        if item.type == "Bool":
            sampled_parts.append((_runtime_row(item), "", {"kind": "bool", "id": item.id}))
        elif item.type == "IntCounter":
            sampled_parts.append((_runtime_row(item), "", {"kind": "int", "id": item.id}))
        elif item.id in PORT_WRITERS:
            sampled_parts.append((_runtime_row(item), "", {"kind": "data", "id": item.id}))

    samples = []
    for source, component, desc in sampled_parts:
        source_id = source.get("id")
        samples.append(
            {
                **{key: value for key, value in source.items() if key != "index"},
                "id": source_id if not component else f"{source_id}.{component}",
                "source_id": source_id,
                "component": component or None,
                "type": "Scalar",
                "source_type": source.get("type"),
                "sample_desc": desc,
            }
        )
    telemetry["quantity_samples"] = samples


def _scalar_descriptor(quantity: dict, quantity_id: str, data_ids, spatial_ids, indexed_views):
    """How a scalar quantity is sampled: as a literal, through a view, or off its own field."""
    viewed = quantity_id in indexed_views
    if quantity.get("value") is not None and quantity_id not in data_ids and not viewed:
        return {"kind": "literal", "value": str(quantity["value"])}
    if viewed:
        views = indexed_views[quantity_id]
        if not all(view.axis is not None for view in views):
            # An axis-less view names no field, but a D-block is still sampled off its
            # own. A directionless one is a whole-subspace alias: when a logged spatial slot
            # carries its superobject, sampling it would put the same value on the wire twice.
            # A directed projection is new information the superobject slot cannot restate.
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
    if quantity_id in data_ids:
        return {"kind": "data", "id": quantity_id}

    return None


def add_spatial_samples(telemetry: dict, algorithm_data: list) -> None:
    """Build the whole-object pose, twist and wrench frame-log slots."""
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in algorithm_data:
        pool = _SPATIAL_SLOT_KINDS.get(item.type)
        if item.id and pool is not None:
            spatial[pool].append({"id": item.id, "index": len(spatial[pool])})
    telemetry["spatial_samples"] = spatial


def _publisher_member(by_channel: dict, entry: dict) -> str:
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
        # One channel is one publisher: writers that share a topic share the member too.
        ros.pub_id = _publisher_member(
            by_channel,
            {
                "pub_id": ros.pub_id,
                "channel": ros.channel,
                "cpp_type": ros.cpp_type,
                "include": ros.include,
                "pkg": ros.pkg,
                "auto_time": ros.auto_time,
                "auto_context_id": ros.auto_context_id,
            },
        )
    for publish in standing:
        publish["pub_id"] = _publisher_member(
            by_channel, {key: publish.get(key) for key in _PUBLISHER_KEYS}
        )

    return list(by_channel.values())


# What a standing publish contributes to the publisher member it writes through.
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
        # A monitor states a rate too, but what it publishes belongs to its motion rather than
        # to the run, so it is published where the motion is and not from here.
        if rate_node is None or CSTR_HDL["Monitor"] in get_node_types(graph, node):
            continue
        # A member stating a field path maps one field; one stating none is an entry the message
        # carries whole.
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
        if shape["entry"] is None and len(records) > 1:
            raise ConstraintViolation(
                "communication",
                f"'{model.id(node)}' reports {len(records)} quantities on '{type_name}', which "
                "carries one; a message reporting many holds an array of them",
            )
        entries = _standing_entries(model, pub_id, shape, records, segment_by_iri)
        frames = (
            {
                frame
                for _row, record in records
                if (frame := _stated_against(record, segment_by_iri, pub_id))
            }
            if shape["frame_path"]
            else set()
        )
        standing.append(
            _prune(
                {
                    "pub_id": pub_id,
                    "channel": str(graph.value(node, NS_MM_ROS["channel-name"]) or ""),
                    # Every Nth cycle: a rate at or above the loop rate publishes each one.
                    "divider": max(1, round(1.0 / (rate_hz * period_s))),
                    "entries": entries,
                    # The array's own header states a frame only when its entries agree on one:
                    # two cameras on one topic leave the message with no single frame to name.
                    "frame_id": frames.pop() if len(frames) == 1 else None,
                    "resize": (
                        [{"path": shape["entry"]["path"], "size": len(entries)}]
                        if shape["entry"]
                        else []
                    ),
                    "fields": _standing_fields(model, node, shape),
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
            )
        )

    return standing


def _standing_entries(model, pub_id: str, shape: dict, records: list, segment_by_iri: dict) -> list:
    """Where in the message each reported quantity is written.

    A message carrying one quantity has one entry writing straight into it. A message carrying an
    array has one entry per quantity, each stating which entity it is about and the frame it says
    that in -- the two things a reader needs to tell one entry from another.
    """
    entry = shape["entry"]
    rows = []
    for index, (node, record) in enumerate(records):
        row = {
            "pub_id": pub_id,
            "value_id": record.id,
            "value_type": record.type,
            "carrier": shape["carrier"],
        }
        if entry is None:
            # The message is the quantity: its frame and its stamp are the message's own, and
            # the run writes them where the message states them.
            rows.append(_prune({**row, "payload_path": shape["payload_path"]}))
            continue
        at = f"{entry['path']}[{index}]."
        # An entry states a frame only where it carries a header to state it in.
        stated_in = entry["frame_path"]
        rows.append(
            _prune(
                {
                    **row,
                    "payload_path": f"{at}{entry['payload_path']}",
                    "frame_path": f"{at}{stated_in}" if stated_in else None,
                    "frame_id": _stated_against(record, segment_by_iri, pub_id)
                    if stated_in
                    else None,
                    "id_path": f"{at}{entry['id_path']}",
                    "id_value": _entry_subject(
                        model, node, record, shape["carrier"], segment_by_iri, pub_id
                    ),
                    "auto_time": [f"{at}{path}" for path in entry["auto_time"]],
                }
            )
        )

    return rows


def _entry_subject(model, node, record, carrier: str, segment_by_iri: dict, where: str) -> str:
    """What an entry says it is about: a transform names the segment its pose is of, anything
    else the entity the model states."""
    if carrier == "Transform":
        return _segment_of(segment_by_iri, record.of.uri, where)

    return _reported_subject(model, node, record)


def _stated_against(record, segment_by_iri: dict, where: str) -> str | None:
    """The segment a reported quantity is stated against, when it is stated against one."""
    if (
        not isinstance(record, (Direction, Position, Orientation, Pose, SpatialCoordinate))
        or record.as_seen_by is None
    ):
        return None

    return _segment_of(segment_by_iri, record.as_seen_by.uri, where)


def _reported_subject(model, node, record) -> str:
    """The entity an entry says it is about, as the model names it.

    Stated rather than derived: a reader of the message matches the entity it models, and a pose
    is a pose of a frame on that entity rather than of the entity itself.

    Raises:
        ConstraintViolation: the entry names no entity, so it could not say which of the several
            the message carries it is.
    """
    subject = model.graph.value(node, SOSA.hasFeatureOfInterest)
    if subject is None:
        raise ConstraintViolation(
            "communication",
            f"'{record.id}' is one entry of an array the message carries, but the publish names "
            "no entity it reports it of, so the entry could not say what it is an entry for",
        )

    return str(subject)


def _standing_fields(model, node, shape: dict) -> list:
    """The fields a standing publish maps itself, each naming the value it reports instead of the
    component of the quantity that field would carry.
    """
    graph = model.graph
    rows = []
    for row in graph.objects(node, RDFS.member):
        authored = graph.value(row, NS_MM_ROS["field-path"])
        if authored is None:
            continue
        path = str(authored)
        if path not in shape["leaves"]:
            raise ConstraintViolation(
                "communication",
                f"'{path}' is not a payload field of '{shape['type_name']}'; it offers "
                f"{', '.join(shape['leaves']) or 'none'}",
            )
        rows.append({"path": path, "ref": model.id(graph.value(row, RDF.value))})

    return rows


def _act_status_slot(model, act):
    """The slot an act's terminal goal status lands in.

    Raises:
        ConstraintViolation: the act declares no status slot, so nothing can read its outcome.
    """
    slot = next(iter(model.graph.subjects(PROV.wasDerivedFrom, act)), None)
    if slot is None:
        raise ConstraintViolation(
            "communication", f"action '{model.id(act)}' declares no goal-status slot"
        )
    return slot


def _act_motion(model, status_slot) -> str:
    """The motion that owns an act: the one whose `until` reads the act's status.

    Raises:
        ConstraintViolation: no motion reads the status, so nothing decides when the goal is
            sent or when it stops mattering.
    """
    graph = model.graph
    watching = set(graph.subjects(CSTR["quantity"], status_slot))
    for node in graph.subjects(RDF["type"], MOT["GuardedMotion"]):
        members = set(graph.objects(node, MOT["until"]))
        members |= {
            member for item in members for member in graph.objects(item, CSTR_EXT["has-constraint"])
        }
        if members & watching:
            return model.id(node)
    raise ConstraintViolation(
        "communication",
        f"no motion's 'until' reads '{model.id(status_slot)}', so nothing sends the goal or "
        "ends the act; compare the act's status in the motion that detects",
    )


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
        # A member is a goal arriving: that action is served, not performed.
        if next(iter(graph.objects(act, RDFS.member)), None) is not None:
            continue
        type_name = str(graph.value(act, NS_MM_ROS["type-name"]) or "")
        shape = ros_messages.action_shape(type_name)
        detect = ros_messages.detect_shape(
            type_name, str(graph.value(act, NS_MM_ROS["field-path"]) or "")
        )
        status_slot = _act_status_slot(model, act)
        rows = written[str(act)]
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
                "motion": _act_motion(model, status_slot),
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
        rows = written.get(str(node)) or []
        if not rows:
            continue
        type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
        shape = ros_messages.observation_shape(
            type_name, str(graph.value(node, NS_MM_ROS["field-path"]) or "")
        )
        subscriptions.append(
            {
                "sub_id": model.id(node),
                "channel": str(graph.value(node, NS_MM_ROS["channel-name"]) or ""),
                "type_name": type_name,
                "cpp_type": shape["cpp_type"],
                "include": shape["include"],
                "pkg": shape["package"],
                "written_poses": [
                    {
                        **row,
                        "frame_segment": _segment_of(
                            segment_by_iri, row["frame_iri"], model.id(node)
                        ),
                        # Absent an authored of/wrt the reading is the quantity itself, so both
                        # ends collapse onto its own frames and the composition is the identity.
                        "observed_wrt_segment": _segment_of(
                            segment_by_iri,
                            row.get("observed_wrt_iri") or row["frame_iri"],
                            model.id(node),
                        ),
                        "observed_of_segment": _segment_of(
                            segment_by_iri,
                            row.get("observed_of_iri") or row["target_of_iri"],
                            model.id(node),
                        ),
                        "target_of_segment": _segment_of(
                            segment_by_iri, row["target_of_iri"], model.id(node)
                        ),
                        "observed_body_segment": _segment_of(
                            segment_by_iri,
                            row.get("observed_body_iri") or row["target_of_iri"],
                            model.id(node),
                        ),
                        "reframed": bool(row.get("observed_of_iri")),
                        # A detection is identified by what it is a reading of, which is the
                        # observed frame when the channel states one and the target otherwise.
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


def _segment_of(segment_by_iri: dict, iri, where: str) -> str:
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


def _served_action(model):
    """The action the model serves, or None when it serves none.

    A goal arrives on a served action and starts something: its valueless member is the event an
    accepted goal produces. An act the model performs has no member -- nothing arrives on it --
    and a monitor answering a goal states every member with a value, since each is one field of
    the result it answers with.

    Raises:
        ConstraintViolation: the model serves more than one action, which one runtime cannot do.
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
    return served[0] if served else None


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
    node = _served_action(model)
    if node is None:
        return None
    if not fsm:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' serves goals, but the model imports no FSM, so an "
            "accepted goal has nothing to start",
        )

    # A member with no authored value is not a result row: it is the event a goal produces.
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
    # Tokens, never indices: coord-dsl's enum alone numbers the events.
    token_by_uri = {uri: token for token, uri in fsm["event_uris"].items()}
    goal_token = token_by_uri.get(str(goal_event))
    if goal_token is None:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' names event '{goal_event}', which FSM '{fsm['name']}' does "
            "not declare",
        )
    # An FSM event lives one tick, so the goal event is produced only when the FSM sits in a
    # state that reacts to it -- a goal accepted during startup must not fire into S_START.
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
        # The scenario a run belongs to arrives on the goal; the run stamps it on everything it
        # publishes afterwards.
        "goal_context_id": [path for path, kind in goal["auto"].items() if kind == "context_id"],
        # A goal may carry more than the run reads. Repeated fields are the ones it can report
        # having ignored, since only they can be counted.
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
    motion_rows, controller_rows, monitor_rows = _motion_rows(
        motions, uri_by_id, computation.functions
    )
    snapshot_ids = quantities.snapshot_target_ids(model)
    structure_ids = {item.id for item in computation.data_structures}
    quantity_rows = []
    for item in algorithm_data:
        if item.id in structure_ids:
            quantity_rows.append(_quantity_row(item, uri_by_id, snapshot_ids))
        # A boolean flag has no quantity row: it is sampled straight off the algorithm data.
        elif isinstance(item, DataValue) and item.role and item.type == "Quantity":
            quantity_rows.append({**_runtime_row(item), "uri": uri_by_id.get(item.id)})

    telemetry = {
        "contract_version": 2,
        "control_period_ns": control_period_ns,
        "motions": motion_rows,
        "controllers": unique_by_id(controller_rows),
        "monitors": unique_by_id(monitor_rows),
        "quantities": unique_by_id(quantity_rows),
    }
    add_quantity_samples(telemetry, algorithm_data, computation.views)
    add_spatial_samples(telemetry, algorithm_data)

    logged, recorded = [], []
    for sample in telemetry["quantity_samples"]:
        found = analysis.get(sample["source_id"])
        if found is None:
            raise RuntimeError(f"telemetry: '{sample['id']}' samples a value with no analysis")
        if found["storage"] == "log":
            logged.append({**sample, "cadence": found["cadence"]})
        # A draw is recorded by the run that made it, not by the generation's header.
        elif data_access[sample["source_id"]]["write"]["kind"] != "sampled":
            recorded.append(
                _prune(
                    {
                        "id": sample["id"],
                        "source_id": sample["source_id"],
                        "uri": sample.get("uri"),
                        "sample_desc": sample["sample_desc"],
                    }
                )
            )
    telemetry["quantity_samples"] = logged
    telemetry["constants"] = recorded
    for pool, rows in telemetry["spatial_samples"].items():
        kept = [row for row in rows if analysis[row["id"]]["storage"] == "log"]
        telemetry["spatial_samples"][pool] = [
            {
                **row,
                "index": index,
                "uri": uri_by_id.get(row["id"]),
                "cadence": analysis[row["id"]]["cadence"],
            }
            for index, row in enumerate(kept)
        ]

    return telemetry
