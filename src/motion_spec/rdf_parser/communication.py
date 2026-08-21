# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What leaves the loop.

In order: the rows the introspection artifact is made of; the members each concern's table adds
to the blackboard, and the rows that report them; the frame-log samples; the ROS interface -- what
the model publishes, the goals it sends and the one it answers; the provenance document; and
`build_introspection`, which runs all of it in the one order the frame layout depends on.

No other module appends to the introspection artifact or to the frame log. A concern publishes
what it knows as a table -- the internal state a PID keeps, the gains a controller carries, the
joint-space channels a chain mirrors -- and this module reads the table and builds every member
and every row itself.
"""

from __future__ import annotations

from enum import Enum

from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, CSTR_HDL, EXEC, MOT, SENSORS
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.naming import get_valid_var_name
from rdflib.namespace import PROV, RDF, RDFS, SOSA
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.motion import BlackboardValue
from motion_spec.classes.qudt import QuantityKind, Unit
from motion_spec.rdf_parser import constraint_handler, coordination, quantities, resources
from motion_spec.rdf_parser.model import identifier

# Types that get a whole-object frame-log slot rather than per-axis scalar rows.
_SPATIAL_SLOT_KINDS = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}
_MONITOR_PHASES = ("when", "while", "until")
# The gain fields a controller row reports, in the order they are emitted.
_GAIN_ROW_FIELDS = (
    "proportional_gain",
    "integral_gain",
    "derivative_gain",
    "decay_rate",
    "stiffness",
    "damping",
)


def _prune(row: dict) -> dict:
    """Drop the keys a row leaves unset, so an absent value is absent rather than null."""
    return {key: value for key, value in row.items() if value is not None and value != []}


def _dedupe_by_id(rows: list) -> list:
    """Rows deduplicated by id, keeping the first occurrence."""
    result, seen = [], set()
    for row in rows:
        if row.get("id") in seen:
            continue
        seen.add(row.get("id"))
        result.append(row)
    return result


def _id_of(value):
    """The id a row field names, whether it holds the id itself, a record or an enum."""
    if value is None or isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    return getattr(value, "id", None)


def _controller_rows(controller, motion_id: str, uri_by_id: dict):
    """One controller's introspection row, and one row per signal it binds."""
    entry = _prune(
        {
            "id": controller.id,
            "uri": uri_by_id.get(controller.id),
            "motion": motion_id,
            "type": controller.type,
            "constraint": controller.constraint,
            "constraint_uri": controller.constraint_uri,
            **{name: getattr(controller, name, None) for name in _GAIN_ROW_FIELDS},
            "error_signal": _id_of(getattr(controller, "error_signal", None)),
            "tolerance_signal": controller.tolerance_id or None,
            "reference_signal": _id_of(getattr(controller, "reference_signal", None)),
            "measured_derivative": _id_of(getattr(controller, "measured_derivative", None)),
            "output_signal": _id_of(controller.control_signal),
            # Folded onto the record by `constraint_handler.annotate_controller_signals`, which reads the
            # error-evaluator closure this controller consumes.
            "measured_signal": controller.measured_signal,
            "setpoint_signal": controller.setpoint_signal,
        }
    )
    signals = [
        _prune(
            {
                "id": f"{controller.id}.{role}",
                "uri": uri_by_id.get(quantity_id),
                "quantity": quantity_id,
                "role": role,
                "owner": controller.id,
            }
        )
        for role in constraint_handler.CONTROLLER_SIGNAL_ROLES
        if (quantity_id := _id_of(getattr(controller, role, None)))
    ]
    # Named `tolerance_id` on the record, so the role loop cannot pick it up.
    if controller.tolerance_id:
        signals.append(
            _prune(
                {
                    "id": f"{controller.id}.tolerance_signal",
                    "uri": uri_by_id.get(controller.tolerance_id),
                    "quantity": controller.tolerance_id,
                    "role": "tolerance_signal",
                    "owner": controller.id,
                }
            )
        )

    return entry, signals


def _quantity_row(item, uri_by_id: dict, snapshot_ids: frozenset) -> dict:
    """One data structure's introspection row: what it is, and where its value came from.

    `snapshot` is a graph fact, not `Provenance`'s -- looked up by id against the snapshot
    targets `quantities.snapshot_target_ids` collects, since this reader holds a record, not a
    graph node.
    """
    provenance = getattr(item, "provenance", None)

    return _prune(
        {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "reference_value": getattr(item, "reference_value", None),
            "value": getattr(item, "value", None),
            "authored": getattr(provenance, "authored", False),
            "snapshot": item.id in snapshot_ids,
        }
    )


def _motion_rows(motions, uri_by_id: dict):
    """The motion, controller, monitor and signal rows, walked in one pass over the motions."""
    motion_rows, controller_rows, monitor_rows, signals = [], [], [], []
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
        for controller in motion.controllers:
            entry, controller_signals = _controller_rows(controller, motion.id, uri_by_id)
            controller_rows.append(entry)
            signals.extend(controller_signals)
        for phase in _MONITOR_PHASES:
            for monitor in getattr(motion, f"{phase}_monitors"):
                monitor_rows.append(
                    _prune(
                        {
                            "id": monitor.id,
                            "uri": uri_by_id.get(monitor.id),
                            "motion": motion.id,
                            "phase": phase,
                            "type": monitor.monitor_type,
                            "trigger": "edge" if monitor.is_edge_triggered else "level",
                            "event": getattr(monitor, "event", None),
                            "event_uri": getattr(monitor, "event_uri", None),
                            "event_name": getattr(monitor, "event_name", None),
                            "flag": getattr(monitor, "flag", None),
                            "error_signal": _id_of(monitor.error),
                            "tolerance_signal": _id_of(monitor.tolerance),
                            "constraint_ids": monitor.constraint_ids,
                            "constraint_uris": monitor.constraint_uris,
                            "fallback_motion": getattr(monitor, "fallback_motion", None),
                            "debounce_duration_s": getattr(monitor, "debounce_duration_s", None),
                        }
                    )
                )
                if monitor.error is None:
                    continue
                signals.append(
                    _prune(
                        {
                            "id": f"{monitor.id}.error",
                            "uri": uri_by_id.get(monitor.error.id),
                            "quantity": monitor.error.id,
                            "role": "monitor_error",
                            "owner": monitor.id,
                        }
                    )
                )

    return motion_rows, controller_rows, monitor_rows, signals


# Per role, the descriptive fields a runtime value reports beyond its id and type, in the order
# the artifact emits them. A value with no role -- the clock, the tare state -- reports only what
# it holds.
_RUNTIME_ROW_FIELDS = {
    "controller_internal_state": ("controller", "role", "state"),
    "control_parameter": ("value", "owner", "role", "parameter"),
    "joint_space": ("quantity_kind", "unit", "runtime", "role", "channel", "joint", "producer"),
    None: ("value",),
}


def _runtime_row(member) -> dict:
    fields = _RUNTIME_ROW_FIELDS.get(getattr(member, "role", None), ())
    return {
        "id": member.id,
        "type": member.type,
        **{name: getattr(member, name) for name in fields if getattr(member, name) is not None},
    }


def _add_member(model, shared_data, rows, seen, member: BlackboardValue, row, parent, suffix):
    """Add one runtime value to the blackboard, its row to the quantities, and mint its IRI.

    A member with no row is one the artifact reports through another family -- a boolean flag is
    sampled straight off the blackboard rather than declared as a quantity.
    """
    if member.id not in seen["shared"]:
        shared_data.append(member)
        seen["shared"].add(member.id)
    if row is not None and member.id not in seen["rows"]:
        rows.append(row)
        seen["rows"].add(member.id)
    model.register_derived(member.id, parent, suffix, PROV.wasDerivedFrom)


def _add_controller_state(model, closures, shared_data, rows, seen, motions) -> None:
    """Publish the internal state a stateful controller keeps between ticks.

    A value the controller integrates is a runtime value like any other: without a slot it cannot
    be reported, and its step call has nowhere to keep it.
    """
    state_by_controller = {
        controller.id: constraint_handler.CONTROLLER_STATE_FIELDS[controller.type]
        for motion in motions
        for controller in motion.controllers
        if controller.type in constraint_handler.CONTROLLER_STATE_FIELDS
    }
    for closure in closures.values():
        fields = (
            state_by_controller.get(closure["id"]) if closure.get("type") == "Controller" else None
        )
        if not fields:
            continue
        # Through the registry, not the graph: a per-axis controller is itself derived.
        parent = model.iri_of(closure["id"])
        if parent is None:
            raise RuntimeError(
                f"controller internal state: '{closure['id']}' has no IRI to derive from"
            )
        samples = []
        for state in fields:
            member_id = f"{closure['id']}_{state.name}"
            member = BlackboardValue(
                id=member_id,
                type=state.type,
                role="controller_internal_state",
                controller=closure["id"],
                state=state.name,
            )
            # A boolean flag has no quantity row: it is sampled straight off the blackboard.
            row = _runtime_row(member) if state.type == "Quantity" else None
            _add_member(model, shared_data, rows, seen, member, row, parent, state.name)
            samples.append({"id": member_id, "getter": state.getter})
        closure["internal_state_samples"] = samples


def _add_control_parameters(model, closures, shared_data, rows, seen, motions) -> None:
    """Publish every authored control parameter as a shared value the step call reads.

    A gain baked into a constructor can neither be reported nor vary; as a shared value it carries
    a producer -- authored, so written once -- and lands in the run's header record.
    """
    controller_by_id = {
        controller.id: controller for motion in motions for controller in motion.controllers
    }

    def publish(owner_id: str, name: str, value, required: bool = True) -> str:
        parent = model.iri_of(owner_id)
        if parent is None:
            raise RuntimeError(f"control parameter: '{owner_id}' has no IRI to derive from")
        if value is None and required:
            raise RuntimeError(f"control parameter: '{owner_id}' authors no '{name}'")
        member = BlackboardValue(
            id=f"{owner_id}_{name}",
            type="Quantity",
            value=float(value) if value is not None else 0.0,
            role="control_parameter",
            owner=owner_id,
            parameter=name,
        )
        _add_member(model, shared_data, rows, seen, member, _runtime_row(member), parent, name)

        return member.id

    for closure in closures.values():
        if closure.get("type") == "Admittance":
            for name in constraint_handler.ADMITTANCE_PARAMETERS:
                closure[name] = publish(closure["id"], name, closure[name])
            continue
        if closure.get("type") != "Controller":
            continue
        controller = controller_by_id.get(closure["id"])
        gains = constraint_handler.CONTROLLER_GAIN_FIELDS.get(getattr(controller, "type", None))
        if not gains:
            continue
        closure["controller_type"] = controller.type
        closure["gains"] = {
            name: publish(closure["id"], name, getattr(controller, source), required)
            for name, source, required in gains
        }
        # The bounds are authored shared quantities already; the call site reads them by id. Both
        # cross here: the reader binds them to the same signal, and a bound that stops at the
        # controller record is a limit the model authored and the robot never sees.
        closure["integral_saturation"] = getattr(controller, "integral_saturation", None)
        closure["output_saturation"] = controller.output_saturation


def _add_joint_space_mirrors(model, robots, motions, shared_data, rows, seen, backend) -> None:
    """Mirror each runtime's joint-space signals onto the blackboard so the frame log carries them.

    Keyed by runtime, not by solver: these are the arm's ports, and the command port is
    last-writer-wins, so a runtime-keyed slot records what the port received even when several
    solvers on one runtime run in the same tick. Keying by solver would multiply the field count
    by the number of motions to record the same ports.
    """
    copies_by_id: dict[str, list] = {}
    for motion in motions:
        for solver in motion.serial_chain_solvers:
            copies_by_id.setdefault(solver.id, []).append(solver)

    by_runtime: dict[str, list] = {}
    for solver in robots.serial_chains:
        by_runtime.setdefault(solver.runtime.id or solver.id, []).append(solver)

    for runtime_id, solvers in by_runtime.items():
        # Channel ids name the runtime's own joints, so the chain's joints get its prefix here.
        joints = [f"{solvers[0].runtime.prefix}{joint}" for joint in solvers[0].chain.joints]
        if not joints:
            raise RuntimeError(
                f"joint-space logging: solver '{solvers[0].id}' has no chain joints; the shared "
                "ids are compile-time names, so a wrong joint count mislabels every channel"
            )
        # tau_cmd differs from tau_ctrl only where a limit clamps it, so that saturation is its
        # producer -- named only when the runtime carries exactly one.
        saturations = {
            solver.torque_saturation.id for solver in solvers if solver.torque_saturation
        }
        available = tuple(
            channel
            for channel in resources.JOINT_SPACE_CHANNELS
            if channel.backends is None or backend in channel.backends
        )
        channels = available + ((resources.JOINT_SPACE_COMMAND_CHANNEL,) if saturations else ())
        producer_id = {
            "port": runtime_id,
            "sensor": runtime_id,
            # None where several instances write the value: no one of them is its producer.
            "solver": next(iter({s.id for s in solvers}), None) if len(solvers) == 1 else None,
            "saturation": next(iter(saturations)) if len(saturations) == 1 else None,
        }
        # Mirrors are keyed by runtime, so they derive from the runtime's own solver node.
        parent = model.iri_of(runtime_id) or model.iri_of(solvers[0].id)
        if parent is None:
            raise RuntimeError(
                f"joint-space logging: runtime '{runtime_id}' has no IRI to derive from"
            )

        ids_by_channel: dict[str, list] = {channel.name: [] for channel in channels}
        for index, joint in enumerate(joints):
            for channel in channels:
                member_id = f"{runtime_id}_{channel.name}_{identifier(joint)}"
                if member_id in seen["shared"]:
                    raise RuntimeError(
                        f"joint-space logging: id '{member_id}' collides with an existing "
                        "shared value"
                    )
                member = BlackboardValue(
                    id=member_id,
                    type="Quantity",
                    role="joint_space",
                    producer={"kind": channel.producer, "id": producer_id[channel.producer]},
                    quantity_kind=QuantityKind(channel.quantity_kind),
                    unit=Unit(channel.unit),
                    runtime=runtime_id,
                    channel=channel.name,
                    joint=joint,
                )
                _add_member(
                    model,
                    shared_data,
                    rows,
                    seen,
                    member,
                    _runtime_row(member),
                    parent,
                    f"{channel.name}-{identifier(joint)}",
                )
                ids_by_channel[channel.name].append({"id": member_id, "index": index})

        # Every solver on the runtime mirrors the same ids: whichever motion is active writes them.
        samples = [
            {"id": entry["id"], "channel": channel.name, "index": entry["index"]}
            for channel in available
            for entry in ids_by_channel[channel.name]
        ]
        command_ids = ids_by_channel.get(resources.JOINT_SPACE_COMMAND_CHANNEL.name, [])
        for solver in solvers:
            for target in (solver, *copies_by_id.get(solver.id, ())):
                target.joint_space_samples = samples
                target.joint_space_cmd_samples = command_ids if solver.torque_saturation else []


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
_COMPOSITE_SUPEROBJECTS = {"Pose", "Wrench", "VelocityTwist", "AccelerationTwist", "FreeVector"}


def add_quantity_samples(introspection: dict, shared_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample rows.

    Each row carries a backend-agnostic descriptor -- a kind plus ids and an axis -- and the view
    renders the sampling expression from it.

    Raises:
        ConstraintViolation: a quantity belongs to MAP views whose superobjects disagree on how to
            reach it.
    """
    shared_ids = {item.id for item in shared_data if item.id}
    spatial_ids = {item.id for item in shared_data if item.type in _SPATIAL_SLOT_KINDS and item.id}
    indexed_views = quantities.views_by_subobject(views)
    samples = []

    def add(source, component: str, desc: dict) -> None:
        row = {key: value for key, value in source.items() if key != "index"}
        source_id = source.get("id")
        row.update(
            {
                "id": source_id if not component else f"{source_id}.{component}",
                "source_id": source_id,
                "component": component or None,
                "type": "Scalar",
                "source_type": source.get("type"),
                "sample_desc": desc,
            }
        )
        samples.append(row)

    for quantity in introspection["quantities"]:
        quantity_id = quantity.get("id")
        if not quantity_id or quantity_id in spatial_ids:
            continue
        if quantity.get("type") != "Quantity":
            for prefix, make_desc in _AXIS_SAMPLES.get(quantity.get("type"), ()):
                if quantity_id not in shared_ids:
                    continue
                for index, name in enumerate(("x", "y", "z")):
                    add(
                        quantity,
                        f"{prefix}.{name}" if prefix else name,
                        make_desc(quantity_id, index),
                    )
            continue
        desc = _scalar_descriptor(quantity, quantity_id, shared_ids, indexed_views)
        if desc is not None:
            add(quantity, "", desc)

    sampled = {sample["source_id"] for sample in samples}
    for item in shared_data:
        if not item.id or item.id in sampled:
            continue
        # A runtime value has no `quantities` row of its own to be sampled from, so its row is
        # built here from what it carries.
        if item.type == "Bool":
            add(_runtime_row(item), "", {"kind": "bool", "id": item.id})
        elif item.type == "IntCounter":
            add(_runtime_row(item), "", {"kind": "int", "id": item.id})
        elif item.id in quantities.PORT_PRODUCERS:
            add(_runtime_row(item), "", {"kind": "shared", "id": item.id})

    introspection["quantity_samples"] = samples


def _scalar_descriptor(quantity: dict, quantity_id: str, shared_ids, indexed_views):
    """How a scalar quantity is sampled: as a literal, through a view, or off its own field."""
    viewed = quantity_id in indexed_views
    if quantity.get("value") is not None and quantity_id not in shared_ids and not viewed:
        return {"kind": "literal", "value": str(quantity["value"])}
    if viewed:
        views = indexed_views[quantity_id]
        if not all(view.axis is not None for view in views):
            # An axis-less view names no field, but a shared value is still sampled off its own.
            return {"kind": "shared", "id": quantity_id} if quantity_id in shared_ids else None
        types = {view.superobject.type for view in views}
        if len(types) != 1:
            raise ConstraintViolation(
                "introspection",
                f"quantity sampling: '{quantity_id}' belongs to incompatible MAP views",
            )
        if next(iter(types)) in _COMPOSITE_SUPEROBJECTS:
            return {"kind": "access", "ref": quantity_id}

        return {"kind": "shared", "id": quantity_id}
    if quantity_id in shared_ids:
        return {"kind": "shared", "id": quantity_id}

    return None


def add_spatial_samples(introspection: dict, shared_data: list) -> None:
    """Build the whole-object pose, twist and wrench frame-log slots."""
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in shared_data:
        pool = _SPATIAL_SLOT_KINDS.get(item.type)
        if item.id and pool is not None:
            spatial[pool].append({"id": item.id, "index": len(spatial[pool])})
    introspection["spatial_samples"] = spatial


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


def ros_standing(model, data_structures, control_period_ns: int) -> list:
    """The topics published for the whole run, one per standing publish.

    A standing publish reports readings rather than a verdict, so it belongs to the run and keeps
    publishing between motions. Its rate is the model's, stated on the topic: it becomes the
    number of control cycles between two messages, since the loop is the fastest it can go.

    Raises:
        ConstraintViolation: the publish names no quantity, states no positive rate, reports a
            quantity that never reaches the blackboard, or maps a field the message does not
            offer.
    """
    graph = model.graph
    by_id = {item.id: item for item in data_structures}
    period_s = control_period_ns * 1e-9
    standing = []
    for node in sorted(graph.subjects(RDF["type"], NS_MM_ROS["Topic"]), key=str):
        rate_node = graph.value(node, SENSORS["update-rate"])
        # A monitor states a rate too, but what it publishes belongs to its motion rather than
        # to the run, so it is published where the motion is and not from here.
        if rate_node is None or CSTR_HDL["Monitor"] in get_node_types(graph, node):
            continue
        # A member stating a field path maps one field; one stating none is an entry the message
        # carries whole.
        reported = sorted(
            (
                row
                for row in graph.objects(node, RDFS.member)
                if graph.value(row, NS_MM_ROS["field-path"]) is None
            ),
            key=str,
        )
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
                f"'{model.id(node)}' reports {', '.join(sorted(kinds))} on one message; a "
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
        shape = coordination.standing_shape(type_name, records[0][1].type)
        if shape["entry"] is None and len(records) > 1:
            raise ConstraintViolation(
                "communication",
                f"'{model.id(node)}' reports {len(records)} quantities on '{type_name}', which "
                "carries one; a message reporting many holds an array of them",
            )
        entries = _standing_entries(model, pub_id, shape, records)
        frames = (
            {frame for _row, record in records if (frame := _stated_against(record))}
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
                            "packages",
                            "frame_path",
                            "auto_time",
                            "auto_context_id",
                        )
                    },
                }
            )
        )

    return standing


def _standing_entries(model, pub_id: str, shape: dict, records: list) -> list:
    """Where in the message each reported quantity is written.

    A message carrying one quantity has one entry writing straight into it. A message carrying an
    array has one entry per quantity, each stating which entity it is about and the frame it says
    that in -- the two things a reader needs to tell one entry from another.
    """
    entry = shape["entry"]
    rows = []
    for index, (node, record) in enumerate(records):
        row = {"pub_id": pub_id, "value_id": record.id, "value_type": record.type}
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
                    "frame_id": _stated_against(record) if stated_in else None,
                    "id_path": f"{at}{entry['id_path']}",
                    "id_value": _reported_subject(model, node, record),
                    "auto_time": [f"{at}{path}" for path in entry["auto_time"]],
                }
            )
        )

    return rows


def _stated_against(record) -> str | None:
    """The frame a reported quantity is stated against, when it is stated against one."""
    return getattr(getattr(record, "as_seen_by", None), "id", None)


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
    for row in sorted(graph.objects(node, RDFS.member)):
        authored = graph.value(row, NS_MM_ROS["field-path"])
        if authored is None:
            continue
        path = str(authored)
        if path not in shape["leaves"]:
            raise ConstraintViolation(
                "communication",
                f"'{path}' is not a payload field of '{shape['type_name']}'; it offers "
                f"{', '.join(sorted(shape['leaves'])) or 'none'}",
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
    for node in sorted(graph.subjects(RDF["type"], MOT["GuardedMotion"]), key=str):
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
    for act in sorted(graph.subjects(RDF["type"], NS_MM_ROS["Action"]), key=str):
        # A member is a goal arriving: that action is served, not performed.
        if next(iter(graph.objects(act, RDFS.member)), None) is not None:
            continue
        type_name = str(graph.value(act, NS_MM_ROS["type-name"]) or "")
        shape = coordination.action_shape(type_name)
        detect = coordination.detect_shape(
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
                "packages": shape["packages"],
                "status_id": model.id(status_slot),
                "motion": _act_motion(model, status_slot),
                "target_iris": sorted({row["target_iri"] for row in rows}),
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
    for node in sorted(graph.subjects(RDF["type"], NS_MM_ROS["Topic"]), key=str):
        rows = written.get(str(node)) or []
        if not rows:
            continue
        type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
        shape = coordination.observation_shape(
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
                "packages": shape["packages"],
                "written_poses": [
                    {
                        **row,
                        "frame_segment": _segment_of(
                            segment_by_iri, row["frame_iri"], model.id(node)
                        ),
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


def _add_goal_status_slots(model, shared_data, rows, seen, action_clients) -> None:
    """Publish each act's goal status as a runtime value the client writes and the until reads.

    Without a slot the status is nowhere: the monitor has nothing to compare and the frame log
    has nothing to report.
    """
    for client in action_clients:
        member = BlackboardValue(id=client["status_id"], type="IntCounter")
        parent = model.iri_of(client["status_id"])
        if parent is None:
            raise RuntimeError(f"goal status: '{client['status_id']}' has no IRI to derive from")
        _add_member(model, shared_data, rows, seen, member, None, parent, "status")


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
        for node in sorted(graph.subjects(RDF["type"], NS_MM_ROS["Action"]), key=str)
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
    goal_event = next(
        (
            row
            for row in sorted(graph.objects(node, RDFS.member))
            if graph.value(row, RDF.value) is None
        ),
        None,
    )
    if goal_event is None:
        raise ConstraintViolation(
            "communication", f"'{model.id(node)}' names no event for an accepted goal to produce"
        )
    # Tokens, never raw indices: the generated FSM enum is declaration-ordered while this
    # reader's tables are sorted, so only the enum symbol is stable across the two.
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
    armed_states = sorted(
        transitions[row["do_transition"]]["from_state"]
        for row in fsm["reactions_table"]
        if row["when_event"] == goal_token
    )
    if not armed_states:
        raise ConstraintViolation(
            "communication",
            f"'{model.id(node)}' produces '{goal_token}' on a goal, but no FSM reaction "
            "consumes it, so an accepted goal could never start anything",
        )

    type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
    shape = coordination.action_shape(type_name)
    goal, result = shape["goal"], shape["result"]

    return {
        "action_name": str(graph.value(node, NS_MM_ROS["channel-name"])),
        "type_name": type_name,
        "cpp_type": shape["cpp_type"],
        "result_cpp_type": result["cpp_type"],
        "include": shape["include"],
        "pkg": shape["package"],
        "packages": shape["packages"],
        "goal_event": goal_token,
        "goal_states": armed_states,
        # The scenario a run belongs to arrives on the goal; the run stamps it on everything it
        # publishes afterwards.
        "goal_context_id": sorted(
            path for path, kind in goal["auto"].items() if kind == "context_id"
        ),
        # A goal may carry more than the run reads. Repeated fields are the ones it can report
        # having ignored, since only they can be counted.
        "ignored_goal_fields": sorted(goal["repeated"]),
        "result_auto_time": sorted(path for path, kind in result["auto"].items() if kind == "time"),
        "result_auto_context_id": sorted(
            path for path, kind in result["auto"].items() if kind == "context_id"
        ),
    }


def ros_publications(motions):
    """Every monitor publish in the model, in motion order."""
    return [
        monitor.ros
        for motion in motions
        for phase in _MONITOR_PHASES
        for monitor in getattr(motion, f"{phase}_monitors")
        if getattr(monitor, "ros", None) is not None
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


def _provenance(model, scene, platform: dict) -> dict:
    """The PROV document: what generated this IR, from what, and who will run it."""
    # One authored fact -- the exec context's platform -- decides all three; never re-derived from
    # the backend token or from substrings of the agent id.
    simulated = platform["simulated"]
    runtime_id = (
        f"agent:runtime:{get_valid_var_name(platform['name']).casefold()}"
        if simulated
        else "agent:runtime:real_robot"
    )
    entities = [
        {
            "id": "entity:app_manifest",
            "types": ["prov:Entity"],
            "role": "app_manifest",
            "path": str(model.app_path),
        },
        {
            "id": "entity:motion_spec_ir",
            "types": ["prov:Entity"],
            "role": "motion_spec_ir",
            "wasGeneratedBy": "activity:motion_spec_ir_generation",
            "wasDerivedFrom": "entity:app_manifest",
        },
    ]
    for role, sources in (
        ("imported_model_graph", model.imported_models),
        ("imported_provenance", model.imported_provenance),
    ):
        key = "imported_graph" if role == "imported_model_graph" else "imported_provenance"
        entities.extend(
            {
                "id": f"entity:{key}:{index}",
                "types": ["prov:Entity"],
                "role": role,
                "source": source,
            }
            for index, source in enumerate(sources)
        )

    agents = [
        {
            "id": "agent:motion_spec_ir_gen",
            "types": ["prov:SoftwareAgent", "obs:ObservationProvider"],
            "role": "ir_generator",
        },
        {
            "id": runtime_id,
            "types": ["prov:SoftwareAgent", "exec:Simulation" if simulated else "exec:RealWorld"],
            "role": "runtime_runner",
        },
        {
            "id": "agent:controller_process",
            "types": ["prov:SoftwareAgent"],
            "role": "controller_process",
            "actedOnBehalfOf": runtime_id,
        },
    ]
    agents.extend(
        {
            "id": f"agent:modelled:{robot.id}",
            "types": ["prov:Agent", "agn:ModelledAgent"],
            "role": "robot",
            "model": robot.path,
        }
        for robot in scene.robots
    )

    return {
        # Only id/uri are consumed; the uri is the published IRI, which the rdf-utils resolver
        # maps to a local checkout. No local paths are baked in.
        "contexts": [
            {"id": "prov", "uri": str(PROV)},
            {"id": "bdd", "uri": "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#"},
            {"id": "agent", "uri": "https://secorolab.github.io/metamodels/agent#"},
            {"id": "observation", "uri": "https://secorolab.github.io/metamodels/observation#"},
            {"id": "execution-context", "uri": str(EXEC.ExecutionContext)},
        ],
        "entities": entities,
        "activities": [
            {
                "id": "activity:motion_spec_ir_generation",
                "types": ["prov:Activity"],
                "used": [entity["id"] for entity in entities if entity["role"] != "motion_spec_ir"],
                "wasAssociatedWith": "agent:motion_spec_ir_gen",
                "role": "motion_spec_ir_generation",
            },
            {
                "id": "activity:controller_execution",
                "types": [
                    "prov:Activity",
                    "bdd:SimulatedExecution" if simulated else "bdd:ScenarioExecution",
                ],
                "used": ["entity:motion_spec_ir"],
                "wasAssociatedWith": "agent:controller_process",
                "role": "controller_execution",
            },
        ],
        "agents": agents,
    }


def build_introspection(
    model,
    motions,
    computation,
    shared_data,
    robots,
    scene,
    platform,
    control_period_ns,
    backend,
    action_clients=(),
    subscriptions=(),
    config_poses=(),
):
    """Build the introspection artifact.

    The order here is the frame layout: the members each concern contributes are added before the
    two sorts, and the two sorts turn list order into the positional indices the frame log and the
    generated struct are built from. Moving either moves the frame layout.

    Raises:
        RuntimeError: a published id has no IRI, or a member has no resolvable dataflow contract.
    """
    # Appended, never substituted: authored nodes keep their IRIs, derived ones extend them.
    uri_by_id = {row["id"]: row["uri"] for row in model.uri_rows()}
    motion_rows, controller_rows, monitor_rows, signals = _motion_rows(motions, uri_by_id)
    snapshot_ids = quantities.snapshot_target_ids(model)
    quantity_rows = [
        _quantity_row(item, uri_by_id, snapshot_ids) for item in computation.data_structures
    ]

    introspection = {
        "contract_version": 1,
        "control_period_ns": control_period_ns,
        "uris": model.uri_rows(),
        "motions": motion_rows,
        "controllers": _dedupe_by_id(controller_rows),
        "monitors": _dedupe_by_id(monitor_rows),
        "quantities": _dedupe_by_id(quantity_rows),
        "signals": _dedupe_by_id(signals),
        "provenance": _provenance(model, scene, platform),
    }

    rows = introspection["quantities"]
    seen = {
        "shared": {item.id for item in shared_data if item.id},
        "rows": {row["id"] for row in rows if row.get("id")},
    }
    _add_controller_state(model, computation.closures, shared_data, rows, seen, motions)
    _add_control_parameters(model, computation.closures, shared_data, rows, seen, motions)
    _add_joint_space_mirrors(model, robots, motions, shared_data, rows, seen, backend)
    _add_goal_status_slots(model, shared_data, rows, seen, action_clients)

    shared_data.sort(key=lambda item: item.id or "")
    rows.sort(key=lambda row: row.get("id") or "")
    add_quantity_samples(introspection, shared_data, computation.views)
    add_spatial_samples(introspection, shared_data)
    quantities.annotate_dataflow(
        introspection,
        shared_data,
        computation.closures,
        motions,
        robots.serial_chains,
        computation.views,
        subscriptions,
        config_poses,
    )

    # The registry grew while folding the samples in: rebuild the table and backfill every row
    # minted before it was complete.
    introspection["uris"] = model.uri_rows()
    introspection["derivations"] = model.derivation_nodes()
    complete = {row["id"]: row["uri"] for row in introspection["uris"] if row.get("uri")}
    for key in _ROW_FAMILIES:
        for row in introspection.get(key) or ():
            if not row.get("uri"):
                row["uri"] = complete.get(row.get("source_id") or row.get("id")) or row.get("uri")
    for rows in (introspection.get("spatial_samples") or {}).values():
        for row in rows:
            if not row.get("uri"):
                row["uri"] = complete.get(row.get("id")) or row.get("uri")
    _check_every_id_resolves(introspection)

    return introspection


# The row families that name a slot in the frame log or an entity in the artifact.
_ROW_FAMILIES = ("controllers", "monitors", "motions", "quantities", "quantity_samples")


def _check_every_id_resolves(introspection: dict) -> None:
    """Every published id must resolve to an IRI, or a run graph cannot state what was recorded.

    Raises:
        RuntimeError: an id has no IRI -- a derived entity was minted without registering where it
            came from.
    """
    uri_by_id = {row["id"]: row["uri"] for row in introspection["uris"] if row.get("uri")}
    unresolved: dict[str, set] = {}

    def check(id_, origin: str) -> None:
        if isinstance(id_, str) and id_ and id_ not in uri_by_id:
            unresolved.setdefault(id_, set()).add(origin)

    for key in (*_ROW_FAMILIES, "signals"):
        for row in introspection.get(key) or ():
            if row.get("uri"):
                continue
            # A signal row is a binding, not an entity: its id is a synthetic `<owner>.<role>` and
            # its uri is the quantity's, so the quantity is what has to resolve.
            if key == "signals":
                check(row.get("quantity"), key)
            else:
                check(row.get("source_id") or row.get("id"), key)
    for pool, rows in (introspection.get("spatial_samples") or {}).items():
        for row in rows:
            check(row.get("id"), f"spatial_samples.{pool}")
    for member_id, entry in (introspection.get("dataflow") or {}).items():
        check(member_id, "dataflow")
        check((entry.get("producer") or {}).get("id"), "dataflow.producer")

    if unresolved:
        details = "\n".join(
            f"  '{id_}' (from {', '.join(sorted(origins))})"
            for id_, origins in sorted(unresolved.items())
        )
        raise RuntimeError(
            "introspection: ids with no IRI -- a derived entity was minted without registering "
            f"its IRI against the node it came from:\n{details}"
        )
