# SPDX-License-Identifier: MPL-2.0
"""Static codegen contract artifacts generated from the enriched motion-spec IR."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from motion_spec.provenance import build_provenance_document

SCHEMA_VERSION = 1
FRAME_LAYOUT_VERSION = 1
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
PSLOT = [("px", "d"), ("py", "d"), ("pz", "d"), ("qx", "d"), ("qy", "d"), ("qz", "d"), ("qw", "d")]
VSLOT = [("lx", "d"), ("ly", "d"), ("lz", "d"), ("ax", "d"), ("ay", "d"), ("az", "d")]
KSLOT = [("fx", "d"), ("fy", "d"), ("fz", "d"), ("tx", "d"), ("ty", "d"), ("tz", "d")]


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
    transition_event = {
        reaction.get("do_transition"): reaction.get("when_event")
        for reaction in fsm_ir.get("reactions_table", [])
    }
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
                "event": transition_event.get(transition.get("id")),
                "event_index": event_index.get(transition_event.get(transition.get("id"))),
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
    by_state = {}

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
        else:
            continue

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
        by_state[state_id] = {"controllers": controller_slots, "monitors": monitor_slots}

    quantities = [
        {"index": idx, **quantity}
        for idx, quantity in enumerate(
            introspection.get("quantity_samples") or introspection.get("quantities", [])
        )
    ]
    max_controllers = max((len(entry["controllers"]) for entry in by_state.values()), default=0)
    max_monitors = max((len(entry["monitors"]) for entry in by_state.values()), default=0)
    heartbeat_events = (
        1 if any(event.get("id") == "E_STEP" for event in fsm.get("events", [])) else 0
    )
    spatial = introspection.get("spatial_samples") or {"poses": [], "twists": [], "wrenches": []}
    pools = {
        "constraints": max_controllers,
        "monitors": max_monitors,
        "quantities": len(quantities),
        "triggers": max(
            TRIGGER_POOL_SIZE, len(fsm.get("events", [])), max_monitors + heartbeat_events
        ),
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
        "generated_by": "motion_spec.codegen",
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
        "context": contexts,
        "pools": pools,
        "timing": {"nominal_period_ns": introspection.get("control_period_ns")},
        "control_period_ns": introspection.get("control_period_ns"),
        "fsm": fsm,
        "by_state": by_state,
        "motions": introspection.get("motions", []),
        "controllers": introspection.get("controllers", []),
        "monitors": introspection.get("monitors", []),
        "quantities": quantities,
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

    def _pool_slots(category: str) -> None:
        """Emit slot-stable proto fields (e.g. constraint_0) for a fixed-size pool category."""
        base = PROTO_FIELD_BASES[category]
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

    def _semantic_slots(category: str, entries: list) -> None:
        """Emit proto fields named from each entry's schema id (quantities and spatial slots)."""
        base = PROTO_FIELD_BASES[category]
        singular = category[:-1]
        fields[category] = []
        for entry in sorted(entries, key=lambda e: e.get("index", 0)):
            idx = entry.get("index", len(fields[category]))
            name = _proto_field_name(entry.get("id"), used, f"{singular}_{idx}")
            fields[category].append(
                {"index": idx, "id": entry.get("id"), "name": name, "number": base + idx}
            )

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


def _shared_expr(signal_id: str | None, shared_ids: set[str]) -> str:
    """C++ access for a shared signal ('shared.<id>'), or '0.0' when it is not a shared field."""
    if signal_id and signal_id in shared_ids:
        return f"shared.{signal_id}"
    return "0.0"


def build_introspection_model(schema: dict, ir: dict) -> dict:
    """Per-FSM-state sample model (controller/monitor exprs, quantity/spatial ids) that the introspect_model template renders."""
    shared_ids = {
        item.get("id")
        for item in ir.get("shared_data", [])
        if isinstance(item, dict) and item.get("id")
    }
    quantities = [
        {"index": quantity["index"], "desc": quantity.get("sample_desc")}
        for quantity in schema.get("quantities", [])
    ]
    states = []
    for state in schema.get("fsm", {}).get("states", []):
        state_id = state.get("id")
        if state_id not in schema.get("by_state", {}):
            continue
        entry = schema["by_state"][state_id]
        controllers = []
        for slot in entry.get("controllers", []):
            # error/output are the controller's own dedicated shared fields (never views).
            # measured/setpoint may reference a view, so pass their ids and let the
            # template render them via access-expr(id, views).
            controllers.append(
                {
                    "uri": json.dumps(slot.get("uri") or ""),
                    "error_expr": _shared_expr(slot.get("error_signal"), shared_ids),
                    "output_expr": _shared_expr(slot.get("output_signal"), shared_ids),
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
                        "uri": json.dumps(slot.get("uri") or ""),
                        "has_active": True,
                        "active_terms": slot.get("active_terms"),
                        "active_terms_present": slot.get("active_terms_present", False),
                        "active_any": slot.get("active_any", False),
                    }
                )
            else:
                value_expr = _shared_expr(slot.get("error_signal"), shared_ids)
                monitors.append(
                    {
                        "uri": json.dumps(slot.get("uri") or ""),
                        "has_active": False,
                        "value_expr": value_expr,
                        "composite_error": slot.get("composite_error", False),
                    }
                )
        states.append(
            {"index": state.get("index", -1), "controllers": controllers, "monitors": monitors}
        )
    spatial = schema.get("spatial", {"poses": [], "twists": [], "wrenches": []})
    return {
        "states": states,
        "quantities": quantities,
        "poses": [{"index": p["index"], "id": p["id"]} for p in spatial["poses"]],
        "twists": [{"index": t["index"], "id": t["id"]} for t in spatial["twists"]],
        "wrenches": [{"index": w["index"], "id": w["id"]} for w in spatial["wrenches"]],
    }


def write_introspection_artifacts(ir: dict, *, ir_path: Path, output_dir: Path) -> dict:
    """Write schema.json, frame_layout.json and provenance.jsonld, and return the frame-log
    header + sample model that codegen folds into the IR. The framed FSM lives in ir["fsm"].
    """
    schema = build_schema(ir, ir_path=ir_path, output_dir=output_dir, fsm_ir=ir.get("fsm"))
    layout = build_frame_layout(schema)
    end_state = schema.get("fsm", {}).get("end")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "schema.json").write_text(json.dumps(schema, indent=4) + "\n")
    (output_dir / "frame_layout.json").write_text(json.dumps(layout, indent=4) + "\n")
    (output_dir / "provenance.jsonld").write_text(
        json.dumps(build_provenance_document(ir, output_dir), indent=4) + "\n"
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
        },
        "model": build_introspection_model(schema, ir),
    }
