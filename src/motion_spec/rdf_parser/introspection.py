# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The introspection artifact: uris, rows, frame-log samples and the check that every published
id resolves to an IRI."""

from __future__ import annotations

from functools import partial
from rdf_utils.naming import get_valid_var_name
from motion_spec_dsl.rdf_parser.vocab import EXEC

from motion_spec.rdf_parser.records import _as_dict, _dedupe_dicts, _field, _prune
from motion_spec.rdf_parser.graph import _id_ref, _uri_table
from motion_spec.rdf_parser.controllers import (
    controller_rows,
    _annotate_controller_signals,
    add_control_parameters,
    add_controller_internal_state_logging,
)
from motion_spec.rdf_parser.solvers import add_joint_space_logging
from motion_spec.rdf_parser.computation import _views_by_subobject
from motion_spec.rdf_parser.dataflow import _PORT_PRODUCERS, annotate_dataflow


def _build_introspection(
    *,
    app_model_path,
    imported_models,
    imported_provenance,
    id_nodes,
    node_by_id,
    motions,
    data_structures,
    control_period_ns,
    backend,
    scene,
    closures,
    views,
    shared_data,
    serial_chain_solvers,
    platform,
    iris,
):
    """Build the introspection artifact (uris, motions, controllers, monitors, quantities,
    provenance) and fold in the controller-state and frame-log samples.
    """
    # Appended, never substituted: authored nodes keep their IRIs, derived ones extend them.
    # Last-wins on a repeated id is deliberate -- constraint names, metamodel predicates and
    # aliases legitimately share a bare id (Parser.assert_no_id_collisions polices the rest).
    uri_rows = _uri_table(id_nodes) + iris.rows()
    uri_by_id = {row["id"]: row["uri"] for row in uri_rows}

    controllers = []
    monitors = []
    signals = []
    for motion in motions:
        for controller in motion.controllers:
            entry, controller_signals = controller_rows(controller, motion.id, uri_by_id)
            controllers.append(entry)
            signals.extend(controller_signals)
        for phase in ("when", "while", "until"):
            for monitor in getattr(motion, f"{phase}_monitors"):
                monitor_entry = {
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
                    "error_signal": _id_ref(monitor.error),
                    "tolerance_signal": _id_ref(getattr(monitor, "tolerance", None)),
                    "fallback_motion": getattr(monitor, "fallback_motion", None),
                    "debounce_duration_s": getattr(monitor, "debounce_duration_s", None),
                    "debounce_steps": getattr(monitor, "debounce_steps", None),
                }
                monitors.append(_prune(monitor_entry))
                if monitor.error is not None:
                    signal_entry = {
                        "id": f"{monitor.id}.error",
                        "uri": uri_by_id.get(monitor.error.id),
                        "quantity": monitor.error.id,
                        "role": "monitor_error",
                        "owner": monitor.id,
                    }
                    signals.append(_prune(signal_entry))

    quantities = []
    for item in data_structures:
        quantity_entry = {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "reference_value": getattr(item, "reference_value", None),
            "value": getattr(item, "value", None),
            "authored": getattr(getattr(item, "provenance", None), "authored", False),
            "snapshot": getattr(getattr(item, "provenance", None), "snapshot", False),
        }
        quantities.append(_prune(quantity_entry))

    # One authored fact -- the exec-context's platform -- decides all three; never re-derived
    # from the backend token or from substrings of the agent id.
    runtime_type = "exec:Simulation" if platform["simulated"] else "exec:RealWorld"
    runtime_id = (
        f"agent:runtime:{get_valid_var_name(platform['name']).casefold()}"
        if platform["simulated"]
        else "agent:runtime:real_robot"
    )
    runtime_activity_type = (
        "bdd:SimulatedExecution" if platform["simulated"] else "bdd:ScenarioExecution"
    )

    entities = [
        {
            "id": "entity:app_manifest",
            "types": ["prov:Entity"],
            "role": "app_manifest",
            "path": str(app_model_path),
        },
        {
            "id": "entity:motion_spec_ir",
            "types": ["prov:Entity"],
            "role": "motion_spec_ir",
            "wasGeneratedBy": "activity:motion_spec_ir_generation",
            "wasDerivedFrom": "entity:app_manifest",
        },
    ]
    entities.extend(
        {
            "id": f"entity:imported_graph:{idx}",
            "types": ["prov:Entity"],
            "role": "imported_model_graph",
            "source": source,
        }
        for idx, source in enumerate(imported_models)
    )
    entities.extend(
        {
            "id": f"entity:imported_provenance:{idx}",
            "types": ["prov:Entity"],
            "role": "imported_provenance",
            "source": source,
        }
        for idx, source in enumerate(imported_provenance)
    )

    agents = [
        {
            "id": "agent:motion_spec_ir_gen",
            "types": ["prov:SoftwareAgent", "obs:ObservationProvider"],
            "role": "ir_generator",
        },
        {"id": runtime_id, "types": ["prov:SoftwareAgent", runtime_type], "role": "runtime_runner"},
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

    introspection = {
        "contract_version": 1,
        "control_period_ns": control_period_ns,
        "uris": uri_rows,
        "motions": [
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
            for motion in motions
        ],
        "states": [],
        "controllers": _dedupe_dicts(controllers),
        "monitors": _dedupe_dicts(monitors),
        "quantities": _dedupe_dicts(quantities),
        "signals": _dedupe_dicts(signals),
        "provenance": {
            # Only id/uri are consumed; the uri is the published IRI, which the rdf-utils
            # resolver maps to a local checkout. No local paths are baked in.
            "contexts": [
                {"id": "prov", "uri": "http://www.w3.org/ns/prov#"},
                {
                    "id": "bdd",
                    "uri": "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#",
                },
                {"id": "agent", "uri": "https://secorolab.github.io/metamodels/agent#"},
                {"id": "observation", "uri": "https://secorolab.github.io/metamodels/observation#"},
                {"id": "execution-context", "uri": str(EXEC.ExecutionContext)},
            ],
            "entities": entities,
            "activities": [
                {
                    "id": "activity:motion_spec_ir_generation",
                    "types": ["prov:Activity"],
                    "used": [
                        entity["id"] for entity in entities if entity["role"] != "motion_spec_ir"
                    ],
                    "wasAssociatedWith": "agent:motion_spec_ir_gen",
                    "role": "motion_spec_ir_generation",
                },
                {
                    "id": "activity:controller_execution",
                    "types": ["prov:Activity", runtime_activity_type],
                    "used": ["entity:motion_spec_ir"],
                    "wasAssociatedWith": "agent:controller_process",
                    "role": "controller_execution",
                },
            ],
            "agents": agents,
        },
    }

    # Ordered: controller signal ids, then internal-state logging (which grows shared_data),
    # then the frame-log samples that read them.
    _annotate_controller_signals(introspection["controllers"], closures)
    add_controller_internal_state_logging(closures, shared_data, introspection, motions, iris)
    add_control_parameters(closures, shared_data, introspection, motions, iris)
    add_joint_space_logging(
        serial_chain_solvers, motions, shared_data, introspection, backend, iris
    )
    # The sample passes below turn list order into the indices the frame layout and the
    # generated struct are built from, so order both lists once, here.
    shared_data.sort(key=lambda item: _field(item, "id") or "")
    introspection["quantities"].sort(key=lambda row: row.get("id") or "")
    add_quantity_samples(introspection, shared_data, views)
    add_spatial_samples(introspection, shared_data)
    values = annotate_dataflow(
        introspection, shared_data, closures, motions, serial_chain_solvers, views
    )
    # The registry grew while folding the samples in: rebuild the table and backfill earlier rows.
    introspection["uris"] = _uri_table(id_nodes) + iris.rows()
    introspection["derivations"] = iris.nodes()
    _backfill_uris(introspection)
    _assert_every_id_resolves(introspection)
    return introspection, values


def _backfill_uris(introspection: dict) -> None:
    """Attach the IRI to every row minted before the derivation registry was complete."""
    uri_by_id = {row["id"]: row["uri"] for row in introspection.get("uris", []) if row.get("uri")}
    for key in ("controllers", "monitors", "motions", "quantities", "quantity_samples"):
        for row in introspection.get(key) or ():
            if isinstance(row, dict) and not row.get("uri"):
                uri = uri_by_id.get(row.get("source_id") or row.get("id"))
                if uri:
                    row["uri"] = uri
    for rows in (introspection.get("spatial_samples") or {}).values():
        for row in rows:
            if isinstance(row, dict) and not row.get("uri"):
                uri = uri_by_id.get(row.get("id"))
                if uri:
                    row["uri"] = uri


# Ids that name a slot in the frame log or a row in the introspection artifact. Every one of them
# has to resolve to an IRI, or a run graph cannot make a statement about what the log recorded.
def _assert_every_id_resolves(introspection: dict) -> None:
    """Fail loudly when an introspection id has no IRI, listing every one rather than the first."""
    uri_by_id = {row["id"]: row["uri"] for row in introspection.get("uris", []) if row.get("uri")}
    unresolved: dict[str, set] = {}

    def check(id_, origin: str) -> None:
        if isinstance(id_, str) and id_ and id_ not in uri_by_id:
            unresolved.setdefault(id_, set()).add(origin)

    def check_row(row, key: str) -> None:
        """A row is resolved if it carries a uri; otherwise its id must be in the table."""
        if _field(row, "uri"):
            return
        # A signal row is a binding, not an entity: its id is a synthetic `<owner>.<role>` and
        # its uri is the quantity's, so the quantity is what has to resolve.
        if key == "signals":
            check(_field(row, "quantity"), key)
            return
        check(_field(row, "source_id") or _field(row, "id"), key)

    for key in ("controllers", "monitors", "motions", "quantities", "signals", "quantity_samples"):
        for row in introspection.get(key) or ():
            check_row(row, key)
    for pool, rows in (introspection.get("spatial_samples") or {}).items():
        for row in rows:
            check(_field(row, "id"), f"spatial_samples.{pool}")
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


# Types that get a whole-object frame-log slot (PoseSlot/TwistSlot/WrenchSlot).
_SPATIAL_SLOT_KINDS = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}


def _spatial_slot_ids(shared_data: list) -> set:
    """Ids carried by a whole-object spatial slot, so no per-axis scalar rows are emitted too.
    AccelerationTwist and PoseDifference have no slot, so their scalar rows must survive.
    """
    return {
        _field(item, "id")
        for item in shared_data
        if _field(item, "type") in _SPATIAL_SLOT_KINDS and _field(item, "id")
    }


def _vec_desc(kind: str):
    return lambda q, i: {"kind": kind, "id": q, "axis": i}


def _member_desc(member: str):
    return lambda q, i: {"kind": "member", "id": q, "member": member, "axis": i}


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


def add_quantity_samples(introspection: dict, shared_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample descriptors from the introspection quantities and shared data."""
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    spatial_ids = _spatial_slot_ids(shared_data)
    indexed_views = _views_by_subobject(views)
    samples = []

    # Each sample carries a backend-agnostic descriptor (kind + ids/axis); the C++
    # sample expression is rendered by the sample-expr template (shared_data.stg).
    def add(source, component: str, desc: dict) -> None:
        """Append one scalar frame-log sample row for a source and component."""
        src = _as_dict(source)
        row = {key: value for key, value in src.items() if key != "index"}
        source_id = src.get("id")
        row.update(
            {
                "id": source_id if not component else f"{source_id}.{component}",
                "source_id": source_id,
                "component": component or None,
                "type": "Scalar",
                "source_type": src.get("type"),
                "sample_desc": desc,
            }
        )
        samples.append(row)

    def add_axes(source, prefix: str, make_desc) -> None:
        """Append per-axis (x/y/z) sample rows for a vector quantity."""
        for idx, axis in enumerate(("x", "y", "z")):
            add(source, f"{prefix}.{axis}" if prefix else axis, make_desc(idx))

    def scalar_view(data_id: str) -> bool:
        """True when a data id has no view or its view selects a single axis."""
        matches = indexed_views.get(data_id, ())
        return not matches or all(_field(view, "axis") is not None for view in matches)

    for quantity in introspection.get("quantities", []):
        qid = quantity.get("id")
        if not qid or qid in spatial_ids:
            continue
        qtype = quantity.get("type")
        if qtype == "Quantity":
            if (
                quantity.get("value") is not None
                and qid not in shared_ids
                and qid not in indexed_views
            ):
                add(quantity, "", {"kind": "literal", "value": str(quantity["value"])})
            elif qid in indexed_views and scalar_view(qid):
                # A scalar view resolves to a composite-member access only for these
                # superobject types; other superobjects (e.g. PoseDifference) sample the
                # quantity's own shared field instead.
                qviews = indexed_views[qid]
                so_types = {_field(_field(view, "superobject"), "type") for view in qviews}
                if len(so_types) != 1:
                    raise ValueError(
                        f"quantity sampling: '{qid}' belongs to incompatible MAP views"
                    )
                so_type = next(iter(so_types))
                if so_type in {"Pose", "Wrench", "VelocityTwist", "AccelerationTwist"}:
                    add(quantity, "", {"kind": "access", "ref": qid})
                else:
                    add(quantity, "", {"kind": "shared", "id": qid})
            elif qid in shared_ids and qid not in indexed_views:
                add(quantity, "", {"kind": "shared", "id": qid})
        elif qid in shared_ids:
            for prefix, make_desc in _AXIS_SAMPLES.get(qtype, ()):
                add_axes(quantity, prefix, partial(make_desc, qid))

    sampled_ids = {sample.get("source_id") for sample in samples}
    for item in shared_data:
        item_id = _field(item, "id")
        if not item_id or item_id in sampled_ids:
            continue
        if _field(item, "type") == "Bool":
            add(item, "", {"kind": "bool", "id": item_id})
        elif _field(item, "type") == "IntCounter":
            add(item, "", {"kind": "int", "id": item_id})
        elif item_id in _PORT_PRODUCERS:
            # No model entity declares it, so it has no `quantities` row to be sampled from.
            add(item, "", {"kind": "shared", "id": item_id})

    introspection["quantity_samples"] = samples


def add_spatial_samples(introspection: dict, shared_data: list) -> None:
    """Add per-object pose, velocity-twist and wrench frame-log samples."""
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in shared_data:
        iid = _field(item, "id")
        pool = _SPATIAL_SLOT_KINDS.get(_field(item, "type"))
        if not iid or pool is None:
            continue
        spatial[pool].append({"id": iid, "index": len(spatial[pool])})
    introspection["spatial_samples"] = spatial
