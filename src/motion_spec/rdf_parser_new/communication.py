# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What leaves the loop.

In order: the rows the introspection artifact is made of; the members each concern's table adds
to the blackboard, and the rows that report them; the frame-log samples; the provenance document;
and `build_introspection`, which runs all of it in the one order the frame layout depends on.

No other module appends to the introspection artifact or to the frame log. A concern publishes
what it knows as a table -- the internal state a PID keeps, the gains a controller carries, the
joint-space channels a chain mirrors -- and this module reads the table and builds every member
and every row itself.
"""

from __future__ import annotations

from enum import Enum
from functools import partial

from motion_spec_dsl.rdf_parser.vocab import EXEC
from rdf_utils.naming import get_valid_var_name
from rdflib.namespace import PROV

from motion_spec.classes.entities import QuantityKind, RuntimeValue, Unit
from motion_spec.rdf_parser_new import constraint_handler, quantities, resources
from motion_spec.rdf_parser_new.model import identifier

__all__ = ["add_quantity_samples", "add_spatial_samples", "build_introspection", "ros_publishers"]

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

    return entry, signals


def _monitor_row(monitor, motion_id: str, phase: str, uri_by_id: dict) -> dict:
    """One monitor's introspection row: what it watches, and what it does when it fires."""
    return _prune(
        {
            "id": monitor.id,
            "uri": uri_by_id.get(monitor.id),
            "motion": motion_id,
            "phase": phase,
            "type": monitor.monitor_type,
            "trigger": "edge" if monitor.is_edge_triggered else "level",
            "event": getattr(monitor, "event", None),
            "event_uri": getattr(monitor, "event_uri", None),
            "event_name": getattr(monitor, "event_name", None),
            "flag": getattr(monitor, "flag", None),
            "error_signal": _id_of(monitor.error),
            "tolerance_signal": _id_of(monitor.tolerance),
            "fallback_motion": getattr(monitor, "fallback_motion", None),
            "debounce_steps": monitor.debounce_steps,
        }
    )


def _quantity_row(item, uri_by_id: dict) -> dict:
    """One data structure's introspection row: what it is, and where its value came from."""
    provenance = getattr(item, "provenance", None)

    return _prune(
        {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "reference_value": getattr(item, "reference_value", None),
            "value": getattr(item, "value", None),
            "authored": getattr(provenance, "authored", False),
            "snapshot": getattr(provenance, "snapshot", False),
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
                    "handler": motion.handler,
                    "handler_uri": uri_by_id.get(motion.handler),
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
                monitor_rows.append(_monitor_row(monitor, motion.id, phase, uri_by_id))
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
    """A runtime value's introspection row: what it is, and what it belongs to."""
    fields = _RUNTIME_ROW_FIELDS.get(getattr(member, "role", None), ())

    return {
        "id": member.id,
        "type": member.type,
        **{name: getattr(member, name) for name in fields if getattr(member, name) is not None},
    }


def _add_member(model, shared_data, rows, seen, member: RuntimeValue, row, parent, suffix):
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
            member = RuntimeValue(
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
        member = RuntimeValue(
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
        by_runtime.setdefault(solver.runtime_id or solver.id, []).append(solver)

    for runtime_id, solvers in by_runtime.items():
        joints = solvers[0].kdl_joints
        if not joints:
            raise RuntimeError(
                f"joint-space logging: solver '{solvers[0].id}' has no kdl_joints; the shared ids "
                "are compile-time names, so a wrong joint count mislabels every channel"
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
            "solver": _sole({solver.id for solver in solvers}),
            "saturation": _sole(saturations),
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
                member = RuntimeValue(
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


def _sole(ids) -> str | None:
    """The one id in a set, or None when several instances write the same value."""
    return next(iter(ids)) if len(ids) == 1 else None


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
    "Setpoint": _POSE_AXIS_SAMPLES,
    "VelocityTwist": _TWIST_AXIS_SAMPLES,
    "AccelerationTwist": _TWIST_AXIS_SAMPLES,
    "PoseDifference": _TWIST_AXIS_SAMPLES,
    "Wrench": (("torque", _member_desc("torque")), ("force", _member_desc("force"))),
}
# The superobject types whose scalar view resolves to a composite-member access; a view onto any
# other superobject samples the quantity's own shared field instead.
_COMPOSITE_SUPEROBJECTS = {"Pose", "Wrench", "VelocityTwist", "AccelerationTwist"}


def add_quantity_samples(introspection: dict, shared_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample rows.

    Each row carries a backend-agnostic descriptor -- a kind plus ids and an axis -- and the view
    renders the sampling expression from it.

    Raises:
        ValueError: a quantity belongs to MAP views whose superobjects disagree on how to reach it.
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
                if quantity_id in shared_ids:
                    _add_axes(add, quantity, prefix, partial(make_desc, quantity_id))
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


def _add_axes(add, source, prefix: str, make_desc) -> None:
    """Append the x/y/z rows of one vector component group."""
    for index, name in enumerate(("x", "y", "z")):
        add(source, f"{prefix}.{name}" if prefix else name, make_desc(index))


def _scalar_descriptor(quantity: dict, quantity_id: str, shared_ids, indexed_views):
    """How a scalar quantity is sampled: as a literal, through a view, or off its own field."""
    viewed = quantity_id in indexed_views
    if quantity.get("value") is not None and quantity_id not in shared_ids and not viewed:
        return {"kind": "literal", "value": str(quantity["value"])}
    if viewed:
        views = indexed_views[quantity_id]
        if not all(view.axis is not None for view in views):
            return None
        types = {view.superobject.type for view in views}
        if len(types) != 1:
            raise ValueError(
                f"quantity sampling: '{quantity_id}' belongs to incompatible MAP views"
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


def ros_publishers(motions) -> list:
    """The ROS publishers the monitors ask for, one per distinct topic.

    Codegen links rclcpp and sets up publishers only when a model publishes at all, so a model
    with none contributes nothing rather than an empty section.
    """
    by_pub_id = {}
    for motion in motions:
        for phase in _MONITOR_PHASES:
            for monitor in getattr(motion, f"{phase}_monitors"):
                channel = getattr(monitor, "ros_channel", None)
                if channel is None:
                    continue
                by_pub_id.setdefault(
                    monitor.ros_pub_id,
                    {
                        "pub_id": monitor.ros_pub_id,
                        "channel": channel,
                        "cpp_type": monitor.ros_cpp_type,
                        "include": monitor.ros_include,
                        "pkg": monitor.ros_pkg,
                    },
                )

    return list(by_pub_id.values())


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
    model, motions, computation, shared_data, robots, scene, platform, control_period_ns, backend
):
    """Build the introspection artifact, and the value projections that ride beside it.

    The order here is the frame layout: the members each concern contributes are added before the
    two sorts, and the two sorts turn list order into the positional indices the frame log and the
    generated struct are built from. Moving either moves the frame layout.

    Returns:
        `(introspection, values)`: the artifact, and the layer-B role projections
        `quantities.annotate_dataflow` answers off the same contract

    Raises:
        RuntimeError: a published id has no IRI, or a member has no resolvable dataflow contract.
    """
    # Appended, never substituted: authored nodes keep their IRIs, derived ones extend them.
    uri_by_id = {row["id"]: row["uri"] for row in model.uri_rows()}
    motion_rows, controller_rows, monitor_rows, signals = _motion_rows(motions, uri_by_id)
    quantity_rows = [_quantity_row(item, uri_by_id) for item in computation.data_structures]

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

    shared_data.sort(key=lambda item: item.id or "")
    rows.sort(key=lambda row: row.get("id") or "")
    add_quantity_samples(introspection, shared_data, computation.views)
    add_spatial_samples(introspection, shared_data)
    values = quantities.annotate_dataflow(
        introspection,
        shared_data,
        computation.closures,
        motions,
        robots.serial_chains,
        computation.views,
    )

    # The registry grew while folding the samples in: rebuild the table and backfill earlier rows.
    introspection["uris"] = model.uri_rows()
    introspection["derivations"] = model.derivation_nodes()
    _backfill_uris(introspection)
    _check_every_id_resolves(introspection)

    return introspection, values


# The row families that name a slot in the frame log or an entity in the artifact.
_ROW_FAMILIES = ("controllers", "monitors", "motions", "quantities", "quantity_samples")


def _backfill_uris(introspection: dict) -> None:
    """Attach the IRI to every row minted before the derivation registry was complete."""
    uri_by_id = {row["id"]: row["uri"] for row in introspection["uris"] if row.get("uri")}
    for key in _ROW_FAMILIES:
        for row in introspection.get(key) or ():
            if not row.get("uri"):
                row["uri"] = uri_by_id.get(row.get("source_id") or row.get("id")) or row.get("uri")
    for rows in (introspection.get("spatial_samples") or {}).values():
        for row in rows:
            if not row.get("uri"):
                row["uri"] = uri_by_id.get(row.get("id")) or row.get("uri")


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
