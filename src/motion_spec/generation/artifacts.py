# SPDX-License-Identifier: MPL-2.0
"""Static codegen contract artifacts generated from the enriched motion-spec IR."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from motion_spec.introspection.provenance import (
    build_derivation_document,
    build_provenance_document,
)

SCHEMA_VERSION = 1
# 3: the log carries its own decode contract in its header record -- the message descriptor,
# every slot's id and IRI, the per-motion gate and the FSM tables. A v2 log has none of that,
# so read_contract rejects it rather than guessing.
FRAME_LAYOUT_VERSION = 4
RUNTIME_RDF_CONTRACT_VERSION = 1
FIELD_BYTES = 8
TRIGGER_POOL_SIZE = 32
HEADER = [
    ("seq", "Q"),
    ("t", "d"),
    ("step", "Q"),
    ("fsm_state", "q"),
    ("active_motion", "q"),
    ("last_event", "q"),
    ("state_since_t", "d"),
    ("state_since_wall_ns", "q"),
    ("event_t", "d"),
    ("event_wall_ns", "q"),
    ("wall_ns", "q"),
    ("period_ns", "q"),
    ("compute_ns", "q"),
]
CSLOT = [
    ("active", "q"),
    ("error", "d"),
    ("output", "d"),
    ("satisfied", "q"),
    ("sat_t", "d"),
    ("measured", "d"),
    ("setpoint", "d"),
]
MSLOT = [("active", "q"), ("value", "d"), ("satisfied", "q"), ("sat_t", "d")]
TSLOT = [("kind", "q"), ("idx", "q"), ("fsm_state", "q"), ("t", "d"), ("wall_ns", "q")]
PSLOT = [("active", "q"), ("px", "d"), ("py", "d"), ("pz", "d"), ("qx", "d"), ("qy", "d"), ("qz", "d"), ("qw", "d")]
VSLOT = [("active", "q"), ("lx", "d"), ("ly", "d"), ("lz", "d"), ("ax", "d"), ("ay", "d"), ("az", "d")]
KSLOT = [("active", "q"), ("fx", "d"), ("fy", "d"), ("fz", "d"), ("tx", "d"), ("ty", "d"), ("tz", "d")]


def field_names_and_format(pools: dict) -> tuple[str, list[str]]:
    """Struct format string and flat field names for a runtime frame, given the per-category pool sizes."""
    fmt = "<" + "".join(code for _, code in HEADER)
    names = [name for name, _ in HEADER]
    for idx in range(pools["constraints"]):
        fmt += "".join(code for _, code in CSLOT)
        names.extend(f"c{idx}.{name}" for name, _ in CSLOT)
    for idx in range(pools["monitors"]):
        fmt += "".join(code for _, code in MSLOT)
        names.extend(f"m{idx}.{name}" for name, _ in MSLOT)
    fmt += "d" * pools["quantities"]
    names.extend(f"q{idx}" for idx in range(pools["quantities"]))
    for idx in range(pools["triggers"]):
        fmt += "".join(code for _, code in TSLOT)
        names.extend(f"tr{idx}.{name}" for name, _ in TSLOT)
    fmt += "q"
    names.append("trigger_count")
    for prefix, slot, key in (
        ("pose", PSLOT, "poses"),
        ("twist", VSLOT, "twists"),
        ("wrench", KSLOT, "wrenches"),
    ):
        for idx in range(pools.get(key, 0)):
            fmt += "".join(code for _, code in slot)
            names.extend(f"{prefix}{idx}.{name}" for name, _ in slot)
    return fmt, names


def fields_with_offsets(pools: dict) -> tuple[list[dict], int]:
    """Per-field {name, fmt, offset, size} list and the total frame size in bytes."""
    fmt, names = field_names_and_format(pools)
    fields = [
        {"name": name, "fmt": code, "offset": idx * FIELD_BYTES, "size": FIELD_BYTES}
        for idx, (name, code) in enumerate(zip(names, fmt[1:]))
    ]
    return fields, len(fields) * FIELD_BYTES


def _uri_by_id(ir: dict) -> dict:
    """Map every introspection id to its canonical URI."""
    return {
        row["id"]: row["uri"]
        for row in ir.get("introspection", {}).get("uris", ir.get("uris", []))
        if isinstance(row, dict) and row.get("id") and row.get("uri")
    }


def _signal_id(value) -> str | None:
    """Id of a signal value (a dict with 'id', a str, or None)."""
    if isinstance(value, dict):
        return value.get("id")
    if isinstance(value, str):
        return value
    return None


def _controller_slot(controller: dict, index: int, motion: dict, uri_by_id: dict) -> dict:
    """Introspection slot for a controller: gains and resolved signal ids/URIs."""
    error_id = _signal_id(controller.get("error_signal"))
    output_id = _signal_id(controller.get("control_signal")) or controller.get("output_signal")
    reference_id = _signal_id(controller.get("reference_signal"))
    measured_id = _signal_id(controller.get("measured_signal")) or controller.get("quantity")
    setpoint_id = (
        reference_id
        or _signal_id(controller.get("setpoint_signal"))
        or controller.get("reference_value")
    )
    measured_derivative_id = _signal_id(controller.get("measured_derivative"))
    tolerance_id = _signal_id(controller.get("tolerance_signal")) or controller.get("tolerance_id")
    return {
        "index": index,
        "id": controller.get("id"),
        "uri": uri_by_id.get(controller.get("id")),
        "motion": motion.get("id"),
        "type": controller.get("type"),
        "gains": {
            key: controller[key]
            for key in (
                "proportional_gain",
                "integral_gain",
                "derivative_gain",
                "decay_rate",
                "stiffness",
                "damping",
            )
            if controller.get(key) is not None
        },
        "error_signal": error_id,
        "error_signal_uri": uri_by_id.get(error_id),
        **({"tolerance_signal": tolerance_id} if tolerance_id else {}),
        "reference_signal": reference_id,
        "reference_signal_uri": uri_by_id.get(reference_id),
        "measured_signal": measured_id,
        "measured_signal_uri": uri_by_id.get(measured_id),
        "measured_derivative_signal": measured_derivative_id,
        "measured_derivative_signal_uri": uri_by_id.get(measured_derivative_id),
        "setpoint_signal": setpoint_id,
        "setpoint_signal_uri": uri_by_id.get(setpoint_id),
        "output_signal": output_id,
        "output_signal_uri": uri_by_id.get(output_id),
    }


def _monitor_slot(monitor: dict, index: int, motion: dict, uri_by_id: dict, phase: str) -> dict:
    """Introspection slot for a monitor: trigger, event/flag and active-condition terms."""
    error = monitor.get("error")
    error_id = _signal_id(error) or monitor.get("error_signal")
    tolerance_id = _signal_id(monitor.get("tolerance")) or monitor.get("tolerance_signal")
    event_id = monitor.get("event")
    return {
        "index": index,
        "id": monitor.get("id"),
        "uri": uri_by_id.get(monitor.get("id")),
        "motion": motion.get("id"),
        "phase": phase,
        "type": monitor.get("monitor_type") or monitor.get("type"),
        "trigger": "edge" if monitor.get("is_edge_triggered") else "level",
        "event": event_id,
        "event_index": monitor.get("fsm_event_idx", monitor.get("event_idx")),
        "event_uri": monitor.get("event_uri") or uri_by_id.get(event_id),
        "event_name": monitor.get("event_name"),
        "flag": monitor.get("flag"),
        "error_signal": error_id,
        "error_signal_uri": uri_by_id.get(error_id),
        # Only when authored: an always-present key would move every schema hash.
        **({"tolerance_signal": tolerance_id} if tolerance_id else {}),
        "composite_error": isinstance(error, dict)
        and error.get("type") in {"Pose", "VelocityTwist"},
        "has_active": monitor.get("has_active", False),
        "active_terms": monitor.get("active_terms"),
        "active_terms_present": monitor.get("active_terms_present", False),
        "active_any": monitor.get("active_any", False),
        "fallback_motion": monitor.get("fallback_motion"),
    }


def _fsm_meta(fsm_ir: dict | None) -> dict:
    """Flatten the framed FSM into indexed states/events/transitions (empty when there is no FSM)."""
    if not fsm_ir:
        return {
            "namespace": None,
            "start": None,
            "end": None,
            "states": [],
            "events": [],
            "transitions": [],
        }
    state_index = {state: idx for idx, state in enumerate(fsm_ir.get("states", []))}
    event_index = {event: idx for idx, event in enumerate(fsm_ir.get("events", []))}
    # A transition can be driven by more than one event, so collect them all rather than let the
    # last reaction win. `event`/`event_index` stay singular and are only filled when the answer is
    # unambiguous; a reader that needs the full picture uses `events`/`event_indices`.
    transition_events: dict[str, list] = {}
    for reaction in fsm_ir.get("reactions_table", []):
        transition_events.setdefault(reaction.get("do_transition"), []).append(
            reaction.get("when_event")
        )

    def _sole_event(transition_id):
        events = transition_events.get(transition_id) or []
        return events[0] if len(events) == 1 else None
    return {
        "namespace": fsm_ir.get("namespace_uri"),
        "start": state_index.get(fsm_ir.get("start_state")),
        "end": state_index.get(fsm_ir.get("end_state")),
        "states": [
            {
                "index": idx,
                "id": state,
                "uri": (fsm_ir.get("state_uris") or {}).get(state),
                "motion": None,
            }
            for state, idx in state_index.items()
        ],
        "events": [
            {"index": idx, "id": event, "uri": (fsm_ir.get("event_uris") or {}).get(event)}
            for event, idx in event_index.items()
        ],
        "transitions": [
            {
                "id": transition.get("id"),
                "uri": transition.get("uri"),
                "from": state_index.get(transition.get("from_state")),
                "to": state_index.get(transition.get("to_state")),
                "event": _sole_event(transition.get("id")),
                "event_index": event_index.get(_sole_event(transition.get("id"))),
                "events": transition_events.get(transition.get("id")) or [],
                "event_indices": [
                    event_index[event]
                    for event in transition_events.get(transition.get("id")) or []
                    if event in event_index
                ],
            }
            for transition in fsm_ir.get("transitions_table", [])
        ],
    }


def build_schema(ir: dict, *, ir_path: Path, output_dir: Path, fsm_ir: dict | None) -> dict:
    """Build the run's introspection schema (pools, per-state slots, quantities, provenance) and its schema_hash."""
    introspection = ir.get("introspection") or {}
    uri_by_id = _uri_by_id(ir)
    fsm = _fsm_meta(fsm_ir)
    motions = ir.get("unique_motions") or ir.get("motions", [])
    motion_by_id = {motion.get("id"): motion for motion in motions}
    states = fsm["states"]
    state_by_id = {state["id"]: state for state in states}
    # Slots are keyed by the motion that computes them, not by the coordinator state that happens
    # to select it: a motion owns the closures and solvers that write its values under an FSM, a
    # behaviour tree or a plain sequencer alike. The index space is ir_gen's motion order,
    # so nothing downstream has to agree with a second generator about what index 3 means.
    by_motion = {}
    # ir_gen owns this index (add_motion_function_interfaces); read it, never re-derive it.
    motion_index = {motion.get("id"): motion.get("index", -1) for motion in motions}
    if -1 in motion_index.values():
        raise RuntimeError(f"motions without an introspection index: {sorted(motion_index)}")

    for motion in motions:
        state_id = motion.get("fsm_state") or motion.get("id")
        if state_id in state_by_id:
            state_by_id[state_id]["motion"] = motion.get("id")
        elif not fsm_ir:
            states.append(
                {
                    "index": len(states),
                    "id": state_id,
                    "uri": uri_by_id.get(motion.get("id")),
                    "motion": motion.get("id"),
                }
            )

        controller_slots = [
            _controller_slot(controller, idx, motion, uri_by_id)
            for idx, controller in enumerate(motion.get("controllers", []))
        ]
        monitor_sources = []
        for phase in ("while", "until"):
            monitor_sources.extend(
                (phase, monitor, motion) for monitor in motion.get(f"{phase}_monitors", [])
            )
        for gate_motion_id in motion.get("fsm_when_gate_motions", []):
            gate_motion = motion_by_id.get(gate_motion_id)
            if gate_motion is None:
                continue
            monitor_sources.extend(
                ("when", monitor, gate_motion) for monitor in gate_motion.get("when_monitors", [])
            )
        monitor_slots = [
            _monitor_slot(monitor, idx, owner, uri_by_id, phase)
            for idx, (phase, monitor, owner) in enumerate(monitor_sources)
        ]
        by_motion[motion["id"]] = {
            "index": motion_index[motion["id"]],
            "uri": uri_by_id.get(motion.get("id")),
            "fsm_state": state_id if state_id in state_by_id else None,
            "controllers": controller_slots,
            "monitors": monitor_slots,
        }

    quantities = [
        {"index": idx, **quantity}
        for idx, quantity in enumerate(
            introspection.get("quantity_samples") or introspection.get("quantities", [])
        )
    ]
    # A gated slot keeps its global index for the whole run -- only the set_ call is gated -- so a
    # decoder resolves "unset" against the writing motions here rather than guessing from absence.
    spatial = introspection.get("spatial_samples") or {"poses": [], "twists": [], "wrenches": []}
    dataflow = introspection.get("dataflow") or {}

    def gate(category: str, slot_index: int, cadence) -> None:
        # cadence is already expressed in motions (plan 011 §2b) -- no coordinator in between.
        for motion_id in cadence["motions"] if isinstance(cadence, dict) else ():
            entry = by_motion.get(motion_id)
            if entry is not None:
                entry.setdefault(category, []).append(slot_index)

    for quantity in quantities:
        gate("quantities", quantity["index"], quantity.get("cadence"))
    for category, rows in spatial.items():
        for row in rows:
            gate(category, row["index"], (dataflow.get(row["id"]) or {}).get("cadence"))
    max_controllers = max((len(entry["controllers"]) for entry in by_motion.values()), default=0)
    max_monitors = max((len(entry["monitors"]) for entry in by_motion.values()), default=0)
    pools = {
        "constraints": max_controllers,
        "monitors": max_monitors,
        "quantities": len(quantities),
        # Sized for the events one tick can produce; the heartbeat is not recorded, so it
        # needs no slot.
        "triggers": max(TRIGGER_POOL_SIZE, len(fsm.get("events", [])), max_monitors),
        "poses": len(spatial["poses"]),
        "twists": len(spatial["twists"]),
        "wrenches": len(spatial["wrenches"]),
    }
    provenance = introspection.get("provenance", {})
    contexts = {
        item["id"]: item["uri"]
        for item in provenance.get("contexts", [])
        if item.get("id") and item.get("uri")
    }
    runtime_provenance = _runtime_provenance(provenance)
    schema = {
        "schema_version": SCHEMA_VERSION,
        "frame_layout_version": FRAME_LAYOUT_VERSION,
        "runtime_rdf_contract_version": RUNTIME_RDF_CONTRACT_VERSION,
        "generated_by": "motion_spec.generation.codegen",
        # Portable basenames only — absolute build-tree paths here would leak machine
        # paths into the archive AND make schema_hash (carried in the frame-log header)
        # depend on where the build ran. The archive resolves these against its own dirs.
        "ir_path": Path(ir_path).name,
        "graph": next(
            (
                Path(entity["path"]).name
                for entity in provenance.get("entities", [])
                if entity.get("role") == "app_manifest" and entity.get("path")
            ),
            Path(ir_path).name,
        ),
        # The authored execution platform travels with the run so the runtime graph and the
        # archive validator read one fact rather than sniffing the runtime agent id.
        "platform": ir.get("platform") or {},
        "context": contexts,
        "pools": pools,
        "timing": {"nominal_period_ns": introspection.get("control_period_ns")},
        "control_period_ns": introspection.get("control_period_ns"),
        "fsm": fsm,
        "by_motion": by_motion,
        "motions": introspection.get("motions", []),
        "controllers": introspection.get("controllers", []),
        "monitors": introspection.get("monitors", []),
        "quantities": quantities,
        # Written once at init: one copy in the header says everything repeating it per tick would.
        "constants": introspection.get("constants", []),
        # The dataflow contract for everything that survives into the layout, so a reader can see
        # who writes each value and when without re-deriving it from the model graph.
        "catalogue": [
            {"id": member_id, **entry}
            for member_id, entry in sorted((introspection.get("dataflow") or {}).items())
            if entry["storage"] != "absent"
        ],
        "spatial": spatial,
        "signals": introspection.get("signals", []),
        "provenance_contexts": provenance.get("contexts", []),
        "runtime_provenance": runtime_provenance,
    }
    # The wire field mapping is part of the run contract: fold it into schema_hash so the
    # frame-log header hash changes whenever a slot's protobuf field name/number changes.
    schema["protobuf"] = build_frame_log_proto_fields(schema)
    schema["schema_hash"] = hashlib.sha256(json.dumps(schema, sort_keys=True).encode()).hexdigest()[
        :16
    ]
    return schema


def _runtime_provenance(provenance: dict) -> dict:
    """Resolve the controller-execution activity and its producer/runtime agents from provenance."""
    activity = next(
        (
            item
            for item in provenance.get("activities", [])
            if item.get("role") == "controller_execution"
            or item.get("id") == "activity:controller_execution"
        ),
        None,
    )
    if activity is None or not activity.get("wasAssociatedWith"):
        raise RuntimeError(
            "Introspection provenance must identify the controller execution activity "
            "and associated producer agent."
        )
    producer_id = activity["wasAssociatedWith"]
    producer = next(
        (item for item in provenance.get("agents", []) if item.get("id") == producer_id), None
    )
    if producer is None:
        raise RuntimeError(
            f"Introspection provenance activity {activity['id']} references missing agent {producer_id}."
        )
    return {
        "activity_id": activity["id"],
        "producer_agent_id": producer_id,
        "runtime_agent_id": producer.get("actedOnBehalfOf"),
    }


def build_frame_layout(schema: dict) -> dict:
    """Compute the binary frame layout (field offsets, frame size) and its frame_layout_hash from the schema."""
    fields, frame_size = fields_with_offsets(schema["pools"])
    layout = {
        "frame_layout_version": FRAME_LAYOUT_VERSION,
        "schema_version": schema["schema_version"],
        "runtime_rdf_contract_version": schema["runtime_rdf_contract_version"],
        "pools": schema["pools"],
        "field_bytes": FIELD_BYTES,
        "frame_size_bytes": frame_size,
        "schema_hash": schema["schema_hash"],
        "runtime_provenance": schema["runtime_provenance"],
        # The runner records the run before any log exists, so this one fact cannot
        # come from the log's own header.
        "platform": schema.get("platform") or {},
        "fields": fields,
    }
    layout["frame_layout_hash"] = hashlib.sha256(
        json.dumps(layout, sort_keys=True).encode()
    ).hexdigest()[:16]
    return layout


# Deterministic per-category field-number ranges. Each runtime-frame slot maps to one
# singular protobuf field, so `<base> + slot_index` is stable across runs of the same model.
PROTO_FIELD_BASES = {
    "constraints": 1000,
    "monitors": 2000,
    "quantities": 3000,
    "triggers": 4000,
    "poses": 5000,
    "twists": 6000,
    "wrenches": 7000,
}
PROTO_MAX_FIELD_NUMBER = 536870911
RUNTIME_FRAME_MESSAGE = "RuntimeFrame"


def _proto_field_name(value: str | None, used: set[str], fallback: str) -> str:
    """Sanitize a schema id to a unique, valid protobuf3 field identifier."""
    name = re.sub(r"[^a-z0-9]+", "_", (value or "").lower()).strip("_")
    if not name:
        name = fallback
    if name[0].isdigit():
        name = f"field_{name}"
    base = name
    suffix = 2
    while name in used:
        name = f"{base}_{suffix}"
        suffix += 1
    used.add(name)
    return name


def build_frame_log_proto_fields(schema: dict) -> dict:
    """Per-category runtime-frame slot -> protobuf field {index, id, name, number}.

    Constraint/monitor/trigger slots are reused per FSM state, so their names are slot-stable
    (`constraint_0`), never state-specific. Quantities and spatial slots take semantic names
    sanitized from their schema ids. Names are unique across the whole RuntimeFrame message.
    """
    pools = schema.get("pools", {})
    spatial = schema.get("spatial") or {"poses": [], "twists": [], "wrenches": []}
    used: set[str] = set()
    fields: dict[str, list] = {}
    next_number = 1

    def _pool_slots(category: str) -> None:
        """Emit slot-stable proto fields (e.g. constraint_0) for a fixed-size pool category."""
        nonlocal next_number
        base = max(PROTO_FIELD_BASES[category], next_number)
        singular = category[:-1]
        fields[category] = []
        for idx in range(pools.get(category, 0)):
            slot_id = f"{singular}_{idx}"
            fields[category].append(
                {
                    "index": idx,
                    "id": slot_id,
                    "name": _proto_field_name(slot_id, used, slot_id),
                    "number": base + idx,
                }
            )
        next_number = base + len(fields[category])

    def _semantic_slots(category: str, entries: list) -> None:
        """Emit proto fields named from each entry's schema id (quantities and spatial slots)."""
        nonlocal next_number
        base = max(PROTO_FIELD_BASES[category], next_number)
        singular = category[:-1]
        fields[category] = []
        for entry in sorted(entries, key=lambda e: e.get("index", 0)):
            idx = entry.get("index", len(fields[category]))
            name = _proto_field_name(entry.get("id"), used, f"{singular}_{idx}")
            # A value the model declares as a flag is a flag on the wire too: a proto bool costs
            # one byte where a double costs eight, and proto3 drops it entirely when false.
            kind = (entry.get("sample_desc") or {}).get("kind")
            fields[category].append(
                {
                    "index": idx,
                    "id": entry.get("id"),
                    # The slot's model identity travels with the wire field that carries it.
                    "iri": entry.get("uri"),
                    "name": name,
                    "number": base + idx,
                    "proto_type": "bool" if kind == "bool" else "double",
                }
            )
        next_number = max((entry["number"] for entry in fields[category]), default=base - 1) + 1

    _pool_slots("constraints")
    _pool_slots("monitors")
    _semantic_slots("quantities", schema.get("quantities", []))
    _pool_slots("triggers")
    _semantic_slots("poses", spatial.get("poses", []))
    _semantic_slots("twists", spatial.get("twists", []))
    _semantic_slots("wrenches", spatial.get("wrenches", []))

    for category, entries in fields.items():
        for entry in entries:
            if entry["number"] > PROTO_MAX_FIELD_NUMBER:
                raise RuntimeError(
                    f"protobuf field number {entry['number']} for {category}[{entry['index']}] "
                    f"exceeds maximum {PROTO_MAX_FIELD_NUMBER}"
                )
    return {"runtime_frame": RUNTIME_FRAME_MESSAGE, "fields": fields}


def _uri_comment(uri: str | None) -> str:
    """A model URI safe to drop into a C++ line comment (no newlines, no comment terminator)."""
    return re.sub(r"[\r\n]|\*/", " ", uri or "")


def _shared_signal(signal_id: str | None, shared_ids) -> str | None:
    """The signal id when it names a shared field, else None so the template samples a constant."""
    return signal_id if signal_id and signal_id in shared_ids else None


def build_introspection_model(schema: dict, ir: dict) -> dict:
    """Per-FSM-state sample model (controller/monitor exprs, quantity/spatial ids) that the introspect_model template renders."""
    shared_ids = {
        item.get("id")
        for item in ir.get("shared_data", [])
        if isinstance(item, dict) and item.get("id")
    }
    # A value only its own motion recomputes is stale whenever another motion is active, so its
    # sample call moves into that motion's case; one written by the global schedule stays
    # unconditional.
    spatial = schema.get("spatial", {"poses": [], "twists": [], "wrenches": []})
    slots = {
        "quantities": {
            q["index"]: {"index": q["index"], "desc": q.get("sample_desc")}
            for q in schema.get("quantities", [])
        },
        **{
            category: {row["index"]: {"index": row["index"], "id": row["id"]} for row in rows}
            for category, rows in spatial.items()
        },
    }
    gated = {
        category: {
            index
            for entry in schema.get("by_motion", {}).values()
            for index in entry.get(category, [])
        }
        for category in slots
    }
    ungated = {
        category: [slot for index, slot in by_index.items() if index not in gated[category]]
        for category, by_index in slots.items()
    }
    cases = []
    for entry in schema.get("by_motion", {}).values():
        controllers = []
        for slot in entry.get("controllers", []):
            # error/output are the controller's own dedicated shared fields (never views), so the
            # template reads them with shared-sig. measured/setpoint may reference a view, so they
            # go through access-expr(id, views) instead.
            controllers.append(
                {
                    "index": slot.get("index", 0),
                    "uri_comment": _uri_comment(slot.get("uri")),
                    "error_signal": _shared_signal(slot.get("error_signal"), shared_ids),
                    "tolerance_signal": _shared_signal(slot.get("tolerance_signal"), shared_ids),
                    "output_signal": _shared_signal(slot.get("output_signal"), shared_ids),
                    "measured_signal": slot.get("measured_signal"),
                    "setpoint_signal": slot.get("setpoint_signal"),
                }
            )
        monitors = []
        for slot in entry.get("monitors", []):
            # Active (aggregate/elapsed) monitors: the boolean value/satisfied condition is
            # rendered from the structured terms by the bool-condition template. Plain error
            # monitors: sample the error value + constraint_satisfied (a plain shared field).
            if slot.get("has_active"):
                monitors.append(
                    {
                        "index": slot.get("index", 0),
                        "uri_comment": _uri_comment(slot.get("uri")),
                        "has_active": True,
                        "active_terms": slot.get("active_terms"),
                        "active_terms_present": slot.get("active_terms_present", False),
                        "active_any": slot.get("active_any", False),
                    }
                )
            else:
                monitors.append(
                    {
                        "index": slot.get("index", 0),
                        "uri_comment": _uri_comment(slot.get("uri")),
                        "has_active": False,
                        "value_signal": _shared_signal(slot.get("error_signal"), shared_ids),
                        "composite_error": slot.get("composite_error", False),
                        "tolerance_signal": _shared_signal(
                            slot.get("tolerance_signal"), shared_ids
                        ),
                    }
                )
        cases.append(
            {
                "index": entry.get("index", -1),
                "controllers": controllers,
                "monitors": monitors,
                **{
                    category: [slots[category][index] for index in entry.get(category, [])]
                    for category in slots
                },
            }
        )
    return {"motions": cases, **ungated}


def write_introspection_artifacts(ir: dict, *, ir_path: Path, output_dir: Path) -> dict:
    """Write frame_layout.json, provenance.ld.json and the derivation graph, and return the
    frame-log header + sample model that codegen folds into the IR.

    The decode contract is not written here: it is serialized into the frame log's own header
    record, so a log needs no companion artifact to be read. The framed FSM lives in ir["fsm"].
    """
    schema = build_schema(ir, ir_path=ir_path, output_dir=output_dir, fsm_ir=ir.get("fsm"))
    layout = build_frame_layout(schema)
    end_state = schema.get("fsm", {}).get("end")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "frame_layout.json").write_text(json.dumps(layout, indent=4) + "\n")
    (output_dir / "provenance.ld.json").write_text(
        json.dumps(build_provenance_document(ir, output_dir), indent=4) + "\n"
    )
    # Declares the IRIs the frame log's derived slots carry; moved beside the model graphs it
    # extends by _organize_generation.
    (output_dir / "derived.ld.json").write_text(
        json.dumps(build_derivation_document(ir), indent=4) + "\n"
    )
    return {
        "schema_hash": schema["schema_hash"],
        "frame_layout_hash": layout["frame_layout_hash"],
        "frame_layout": {
            "schema_version": schema["schema_version"],
            "frame_layout_version": layout["frame_layout_version"],
            "runtime_rdf_contract_version": schema["runtime_rdf_contract_version"],
            "pools": schema["pools"],
            "frame_size_bytes": layout["frame_size_bytes"],
            "schema_hash": schema["schema_hash"],
            "frame_layout_hash": layout["frame_layout_hash"],
            "runtime_activity_id": schema["runtime_provenance"]["activity_id"],
            "runtime_producer_agent_id": schema["runtime_provenance"]["producer_agent_id"],
            "runtime_agent_id": schema["runtime_provenance"].get("runtime_agent_id") or "",
            "end_state": end_state if end_state is not None else -1,
            "nominal_period_ns": schema.get("control_period_ns") or 0,
            "protobuf": schema["protobuf"],
            "header_record_rows": _hex_rows(build_frame_log_header_record(schema)),
        },
        "model": build_introspection_model(schema, ir),
    }


def _hex_rows(blob: bytes, per_row: int = 16) -> list[list[str]]:
    """Byte literals grouped into rows, so the emitted array is not one enormous line."""
    values = [f"0x{byte:02x}" for byte in blob]
    return [values[i : i + per_row] for i in range(0, len(values), per_row)]


def build_frame_log_header_record(schema: dict) -> bytes:
    """Serialize the run's FrameLogRecord header: everything a decoder needs and the wire cannot say.

    Built here, once, rather than assembled by generated C++: every field is known at generation
    time, so the runtime only has to write these bytes out verbatim.
    """
    from google.protobuf import descriptor_pb2

    from motion_spec.introspection import frame_log_pb

    record_cls, fields = frame_log_pb._record_class(schema)
    descriptor_set = descriptor_pb2.FileDescriptorSet()
    descriptor_set.file.add().CopyFrom(frame_log_pb._build_file_descriptor(fields))

    rec = record_cls()
    header = rec.header
    header.SetInParent()
    header.schema_hash = schema["schema_hash"]
    meta = schema.get("runtime_provenance") or {}
    header.producer_agent_id = meta.get("producer_agent_id", "")
    header.activity_id = meta.get("activity_id", "")
    header.runtime_agent_id = meta.get("runtime_agent_id") or ""
    header.descriptor_set = descriptor_set.SerializeToString()
    header.trigger_pool = schema["pools"].get("triggers", 0)
    platform = schema.get("platform") or {}
    header.platform_name = platform.get("name") or ""
    header.simulated = bool(platform.get("simulated"))
    fsm = schema.get("fsm") or {}
    end_state = fsm.get("end")
    header.end_state = end_state if end_state is not None else -1
    header.nominal_period_ns = schema.get("control_period_ns") or 0
    header.fsm_namespace = fsm.get("namespace") or ""

    # Slot identity, keyed by the field number that carries it on the wire.
    for category in ("quantities", "poses", "twists", "wrenches"):
        for entry in fields.get(category, ()):
            slot = header.slots.add()
            slot.number, slot.id = entry["number"], entry["id"]
            slot.iri = entry.get("iri") or ""

    state_index = {
        row["id"]: row["index"] for row in (fsm.get("states") or ()) if isinstance(row, dict)
    }
    for motion_id, entry in (schema.get("by_motion") or {}).items():
        gate = header.motions.add()
        gate.index, gate.id = entry["index"], motion_id
        gate.iri = entry.get("uri") or ""
        gate.fsm_state = state_index.get(entry.get("fsm_state"), -1)
        for category in ("quantities", "poses", "twists", "wrenches"):
            getattr(gate, category).extend(entry.get(category, ()))
        for category in ("controllers", "monitors"):
            for slot_entry in entry.get(category, ()):
                slot = getattr(gate, category).add()
                slot.number, slot.id = slot_entry["index"], slot_entry.get("id") or ""
                slot.iri = slot_entry.get("uri") or ""
                # A controller slot names the constraint it serves, a monitor slot the event it
                # fires -- runtime.ttl attributes an occurrence to those, not to the slot.
                slot.constraint_iri = slot_entry.get("constraint_uri") or ""
                slot.event_iri = slot_entry.get("event_uri") or ""

    for key, target in (("states", header.fsm_states), ("events", header.fsm_events)):
        for index, row in enumerate(fsm.get(key) or ()):
            row = row if isinstance(row, dict) else {"id": row}
            named = target.add()
            named.number = row.get("index", index)
            named.id = str(row.get("id") or "")
            named.iri = row.get("uri") or ""

    for index, row in enumerate(fsm.get("transitions") or ()):
        transition = header.fsm_transitions.add()
        transition.index, transition.id = index, str(row.get("id") or "")
        transition.iri = row.get("uri") or ""
        # -1, not 0: 0 is a real state index, so a missing endpoint must not read as one.
        transition.from_state = row["from"] if row.get("from") is not None else -1
        transition.to_state = row["to"] if row.get("to") is not None else -1
        event_index = row.get("event_index")
        transition.event_index = event_index if event_index is not None else -1
        transition.event_indices.extend(row.get("event_indices") or ())

    for entry in schema.get("constants") or ():
        constant = header.constants.add()
        constant.id = entry["id"]
        constant.source_id = entry.get("source_id") or ""
        constant.value = float(entry.get("value") or 0.0)

    return rec.SerializeToString()
