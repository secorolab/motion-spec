# SPDX-License-Identifier: MPL-2.0
"""Static codegen contract artifacts generated from the enriched motion-spec IR."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

# readers refuse any other version
FRAME_LAYOUT_VERSION = 5
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
DSLOT = [("seq", "Q"), ("success", "q")]
TSLOT = [("kind", "q"), ("idx", "q"), ("fsm_state", "q"), ("t", "d"), ("wall_ns", "q")]
PSLOT = [
    ("active", "q"),
    ("px", "d"),
    ("py", "d"),
    ("pz", "d"),
    ("qx", "d"),
    ("qy", "d"),
    ("qz", "d"),
    ("qw", "d"),
]
VSLOT = [
    ("active", "q"),
    ("lx", "d"),
    ("ly", "d"),
    ("lz", "d"),
    ("ax", "d"),
    ("ay", "d"),
    ("az", "d"),
]
KSLOT = [
    ("active", "q"),
    ("fx", "d"),
    ("fy", "d"),
    ("fz", "d"),
    ("tx", "d"),
    ("ty", "d"),
    ("tz", "d"),
]


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
    for idx in range(pools["devices"]):
        fmt += "".join(code for _, code in DSLOT)
        names.extend(f"device{idx}.{name}" for name, _ in DSLOT)
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
        for idx in range(pools[key]):
            fmt += "".join(code for _, code in slot)
            names.extend(f"{prefix}{idx}.{name}" for name, _ in slot)
    return fmt, names


_VECTOR_FIELDS = {
    "Position": "position",
    "Pose": "position",
    "Direction": "direction",
    "FreeVector": "vector",
}


def signal_id(value) -> str | None:
    """Id of a signal value (a dict with 'id', a str, or None)."""
    if isinstance(value, dict):
        return value["id"]
    if isinstance(value, str):
        return value
    return None


# operand keys in read order; a numeric literal here is a value, not an id
_EVALUATOR_OPERANDS = {
    "PoseDiffEvaluator": ("in1", "in2"),
    "ErrorEvaluator": (
        "quantity",
        "reference_value",
        "threshold",
        "lower_threshold",
        "upper_threshold",
    ),
}


def evaluator_terms(evaluators: dict, error_id: str | None) -> dict:
    """The slot's evaluator, the quantities it compares and the difference it writes.

    Empty when no function produces this error: a slot says what the run computes, never a guess.
    """
    function = evaluators.get(error_id) if error_id else None
    if function is None:
        return {}
    operands = [
        value
        for key in _EVALUATOR_OPERANDS.get(function.get("type"), ())
        if isinstance(value := function.get(key), str)
    ]
    difference_id = signal_id(function.get("out"))
    return {
        "evaluator_id": function.get("id"),
        **({"operand_ids": operands} if operands else {}),
        **({"difference_id": difference_id} if difference_id else {}),
    }


def build_schema(ir: dict, *, ir_path: Path, fsm_ir: dict | None) -> dict:
    """Build the run's telemetry schema (pools, per-state slots, quantities, provenance) and its schema_hash."""
    telemetry = ir["communication"]["telemetry"]
    uri_by_id = {row["id"]: row["uri"] for row in ir["provenance"]["uris"] if row["uri"]}
    evaluators: dict = {}
    for function in ir["computation"]["functions"].values():
        if not isinstance(function, dict):
            continue
        for error in (function.get("error"), *(function.get("errors") or ())):
            error_id = signal_id(error)
            if error_id and evaluators.setdefault(error_id, function) != function:
                raise ValueError(
                    f"error '{error_id}' is computed by both '{evaluators[error_id].get('id')}' "
                    f"and '{function.get('id')}' -- one evaluator writes an error"
                )
    if not fsm_ir:
        fsm = {
            "namespace": None,
            "start": None,
            "end": None,
            "states": [],
            "events": [],
            "transitions": [],
        }
    else:
        state_index = {state: idx for idx, state in enumerate(fsm_ir["states"])}
        event_index = {event: idx for idx, event in enumerate(fsm_ir["events"])}
        # several events can drive one transition; `event` is set only when unambiguous
        transition_events: dict[str, list] = {}
        for reaction in fsm_ir["reactions_table"]:
            transition_events.setdefault(reaction["do_transition"], []).append(
                reaction["when_event"]
            )
        transitions = []
        for transition in fsm_ir["transitions_table"]:
            events = transition_events.get(transition["id"]) or []
            sole_event = events[0] if len(events) == 1 else None
            transitions.append(
                {
                    "id": transition["id"],
                    "uri": transition["uri"],
                    "from": state_index.get(transition["from_state"]),
                    "to": state_index.get(transition["to_state"]),
                    "event": sole_event,
                    "event_index": event_index.get(sole_event),
                    "events": events,
                    "event_indices": [
                        event_index[event] for event in events if event in event_index
                    ],
                }
            )
        fsm = {
            "namespace": fsm_ir["namespace_uri"],
            "start": state_index.get(fsm_ir["start_state"]),
            "end": state_index.get(fsm_ir["end_state"]),
            "states": [
                {"index": idx, "id": state, "uri": fsm_ir["state_uris"].get(state), "motion": None}
                for state, idx in state_index.items()
            ],
            "events": [
                {"index": idx, "id": event, "uri": fsm_ir["event_uris"].get(event)}
                for event, idx in event_index.items()
            ],
            "transitions": transitions,
        }
    motions = ir["coordination"]["motions"]
    motion_by_id = {motion["id"]: motion for motion in motions}
    monitor_rows = {row["id"]: row for row in telemetry["monitors"]}
    controller_rows = {row["id"]: row for row in telemetry["controllers"]}
    states = fsm["states"]
    state_by_id = {state["id"]: state for state in states}
    # keyed by motion, not coordinator state, so any coordinator decodes alike
    by_motion = {}
    # owned by ir_gen; never re-derive
    motion_index = {motion["id"]: motion["index"] for motion in motions}
    if -1 in motion_index.values():
        raise RuntimeError(f"motions without an telemetry index: {list(motion_index)}")

    for motion in motions:
        state_id = motion["fsm_state"] or motion["id"]
        if state_id in state_by_id:
            state_by_id[state_id]["motion"] = motion["id"]
        elif not fsm_ir:
            states.append(
                {
                    "index": len(states),
                    "id": state_id,
                    "uri": uri_by_id.get(motion["id"]),
                    "motion": motion["id"],
                }
            )

        controller_slots = []
        for idx, controller in enumerate(motion["controllers"]):
            row = controller_rows[controller["id"]]
            error_id = row.get("error_signal")
            output_id = row.get("output_signal")
            reference_id = row.get("reference_signal")
            measured_id = row.get("measured_signal")
            setpoint_id = reference_id or row.get("setpoint_signal")
            measured_derivative_id = row.get("measured_derivative")
            tolerance_id = row.get("tolerance_signal")
            controller_slots.append(
                {
                    "index": idx,
                    "id": row["id"],
                    "uri": row.get("uri"),
                    "motion": row.get("motion"),
                    "type": row.get("type"),
                    "constraint": row.get("constraint"),
                    "constraint_uri": row.get("constraint_uri"),
                    "gains": {
                        key: row[key]
                        for key in (
                            "proportional_gain",
                            "integral_gain",
                            "derivative_gain",
                            "decay_rate",
                            "stiffness",
                            "damping",
                        )
                        if row.get(key) is not None
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
                    **evaluator_terms(evaluators, error_id),
                }
            )
        monitor_sources = []
        for phase in ("while", "until"):
            monitor_sources.extend(
                (phase, monitor, motion) for monitor in motion[f"{phase}_monitors"]
            )
        for gate_motion_id in motion["fsm_when_gate_motions"]:
            gate_motion = motion_by_id.get(gate_motion_id)
            if gate_motion is None:
                continue
            monitor_sources.extend(
                ("when", monitor, gate_motion) for monitor in gate_motion["when_monitors"]
            )
        monitor_slots = []
        for idx, (phase, monitor, owner) in enumerate(monitor_sources):
            # watched constraints are only on the published row
            row = monitor_rows[monitor["id"]]
            error = monitor["error"]
            error_id = signal_id(error)
            tolerance_id = signal_id(monitor["tolerance"])
            event_id = monitor.get("event")
            monitor_slots.append(
                {
                    "index": idx,
                    "id": monitor["id"],
                    "uri": uri_by_id.get(monitor["id"]),
                    "motion": owner["id"],
                    "phase": phase,
                    "constraint_ids": row.get("constraint_ids") or [],
                    "constraint_uris": row.get("constraint_uris") or [],
                    "watched": row.get("watched") or [],
                    "type": monitor["monitor_type"],
                    "trigger": "edge" if monitor["is_edge_triggered"] else "level",
                    "event": event_id,
                    "event_uri": monitor.get("event_uri") or uri_by_id.get(event_id),
                    "event_name": monitor.get("event_name"),
                    "flag": monitor.get("flag"),
                    "error_signal": error_id,
                    "error_signal_uri": uri_by_id.get(error_id),
                    # an always-present key would move every schema hash
                    **({"tolerance_signal": tolerance_id} if tolerance_id else {}),
                    "composite_error": isinstance(error, dict)
                    and error["type"] in {"Pose", "VelocityTwist"},
                    "has_active": monitor["has_active"],
                    "active_terms": monitor["active_terms"],
                    "active_any": monitor["active_any"],
                    "fallback_motion": monitor.get("fallback_motion"),
                    **evaluator_terms(evaluators, error_id),
                }
            )
        by_motion[motion["id"]] = {
            "index": motion_index[motion["id"]],
            "uri": uri_by_id.get(motion["id"]),
            "fsm_state": state_id if state_id in state_by_id else None,
            "controllers": controller_slots,
            "monitors": monitor_slots,
        }

    quantities = [
        {"index": idx, **quantity} for idx, quantity in enumerate(telemetry["quantity_samples"])
    ]
    spatial = telemetry["spatial_samples"]
    data_access = ir["computation"]["data_access"]
    data_by_id = {item["id"]: item for item in ir["computation"]["data"]}
    constants = []
    for row in telemetry["constants"]:
        item = data_by_id[row["source_id"]]
        desc = row["sample_desc"]
        kind = desc["kind"]
        if kind == "literal":
            value = float(desc["value"])
        elif kind in {"data", "access", "bool", "int"}:
            value = item.get("value")
        elif kind == "vec":
            values = item.get(_VECTOR_FIELDS.get(item["type"], ""))
            value = values[desc["axis"]] if values is not None else None
        else:
            value = None
        if value is None:
            raise ValueError(f"header slot '{row['id']}' records a value with no number to record")
        constants.append(
            {
                "id": row["id"],
                "source_id": row["source_id"],
                "value": value,
                **({"uri": row["uri"]} if row.get("uri") else {}),
                **(
                    {"readers": readers}
                    if (readers := data_access[row["source_id"]].get("read"))
                    else {}
                ),
            }
        )
    devices = [
        {
            "index": device["health_index"],
            "id": device["config_key"],
            "required_by_motion": device["required_by_motion"],
        }
        for solver in ir["resources"]["by_kind"].get("serial_chain", [])
        if solver["runtime"]["owner"]
        for device in solver["devices"]
        if device["health_index"] is not None
    ]
    cameras = [
        {
            key: camera[key]
            for key in ("id", "width", "height", "rate_hz", "uri", "topic", "message")
        }
        for camera in ir["composition"]["scene"]["cameras"]
    ]

    gated_slots = [
        ("quantities", quantity["index"], quantity.get("cadence")) for quantity in quantities
    ] + [
        (category, row["index"], row["cadence"])
        for category, rows in spatial.items()
        for row in rows
    ]
    for category, slot_index, cadence in gated_slots:
        for motion_id in cadence["motions"] if isinstance(cadence, dict) else ():
            entry = by_motion.get(motion_id)
            if entry is not None:
                entry.setdefault(category, []).append(slot_index)
    max_controllers = max((len(entry["controllers"]) for entry in by_motion.values()), default=0)
    max_monitors = max((len(entry["monitors"]) for entry in by_motion.values()), default=0)
    pools = {
        "constraints": max_controllers,
        "monitors": max_monitors,
        "quantities": len(quantities),
        "devices": len(devices),
        # events one tick can produce; the heartbeat is not recorded
        "triggers": max(TRIGGER_POOL_SIZE, len(fsm["events"]), max_monitors),
        "poses": len(spatial["poses"]),
        "twists": len(spatial["twists"]),
        "wrenches": len(spatial["wrenches"]),
    }
    schema = {
        "generated_by": "motion_spec.generation.codegen",
        # basename: schema_hash must not depend on where the build ran
        "ir_path": Path(ir_path).name,
        "platform": ir["configuration"]["platform"],
        "pools": pools,
        "timing": {"nominal_period_ns": telemetry["control_period_ns"]},
        "control_period_ns": telemetry["control_period_ns"],
        "fsm": fsm,
        "by_motion": by_motion,
        "motions": telemetry["motions"],
        "controllers": telemetry["controllers"],
        "monitors": telemetry["monitors"],
        "quantities": quantities,
        "devices": devices,
        "cameras": cameras,
        "constants": constants,
        "catalogue": [{"id": value_id, **entry} for value_id, entry in data_access.items()],
        "spatial": spatial,
    }
    schema["schema_hash"] = hashlib.sha256(json.dumps(schema, sort_keys=True).encode()).hexdigest()[
        :16
    ]
    return schema


def build_frame_layout(schema: dict) -> dict:
    """Compute the binary frame layout (field offsets, frame size) and its frame_layout_hash from the schema."""
    fmt, names = field_names_and_format(schema["pools"])
    fields = [
        {"name": name, "fmt": code, "offset": idx * FIELD_BYTES, "size": FIELD_BYTES}
        for idx, (name, code) in enumerate(zip(names, fmt[1:]))
    ]
    layout = {
        "frame_layout_version": FRAME_LAYOUT_VERSION,
        "pools": schema["pools"],
        "field_bytes": FIELD_BYTES,
        "frame_size_bytes": len(fields) * FIELD_BYTES,
        "schema_hash": schema["schema_hash"],
        "fields": fields,
    }
    layout["frame_layout_hash"] = hashlib.sha256(
        json.dumps(layout, sort_keys=True).encode()
    ).hexdigest()[:16]
    return layout


def build_telemetry_model(schema: dict, ir: dict) -> dict:
    """Per-FSM-state sample model (controller/monitor exprs, quantity/spatial ids) that the telemetry_model template renders."""
    data_ids = {item["id"] for item in ir["computation"]["data"]}
    # a motion-only value is stale under other motions: sample it in that motion's case
    spatial = schema["spatial"]
    slots = {
        "quantities": {
            q["index"]: {"index": q["index"], "desc": q.get("sample_desc")}
            for q in schema["quantities"]
        },
        **{
            category: {row["index"]: {"index": row["index"], "id": row["id"]} for row in rows}
            for category, rows in spatial.items()
        },
    }
    gated = {
        category: {
            index for entry in schema["by_motion"].values() for index in entry.get(category, [])
        }
        for category in slots
    }
    ungated = {
        category: [slot for index, slot in by_index.items() if index not in gated[category]]
        for category, by_index in slots.items()
    }
    cases = []
    for entry in schema["by_motion"].values():
        controllers = []
        for slot in entry["controllers"]:
            # error/output are data fields, never views; measured/setpoint may be views
            for role in ("error_signal", "tolerance_signal", "output_signal"):
                if slot.get(role) and slot[role] not in data_ids:
                    raise ValueError(f"telemetry {role} '{slot[role]}' names no shared field")
            controllers.append(
                {
                    "index": slot["index"],
                    # safe inside a C++ comment
                    "uri_comment": re.sub(r"[\r\n]|\*/", " ", slot["uri"] or ""),
                    "error_signal": slot["error_signal"] or None,
                    "tolerance_signal": slot.get("tolerance_signal") or None,
                    "output_signal": slot["output_signal"] or None,
                    "measured_signal": slot["measured_signal"],
                    "setpoint_signal": slot["setpoint_signal"],
                }
            )
        monitors = []
        for slot in entry["monitors"]:
            if slot["has_active"]:
                monitors.append(
                    {
                        "index": slot["index"],
                        "uri_comment": re.sub(r"[\r\n]|\*/", " ", slot["uri"] or ""),
                        "has_active": True,
                        "active_terms": slot["active_terms"],
                        "active_any": slot["active_any"],
                    }
                )
            else:
                for role in ("error_signal", "tolerance_signal"):
                    if slot.get(role) and slot[role] not in data_ids:
                        raise ValueError(f"telemetry {role} '{slot[role]}' names no shared field")
                monitors.append(
                    {
                        "index": slot["index"],
                        "uri_comment": re.sub(r"[\r\n]|\*/", " ", slot["uri"] or ""),
                        "has_active": False,
                        "value_signal": slot["error_signal"] or None,
                        "composite_error": slot["composite_error"],
                        "tolerance_signal": slot.get("tolerance_signal") or None,
                    }
                )
        cases.append(
            {
                "index": entry["index"],
                "controllers": controllers,
                "monitors": monitors,
                **{
                    category: [slots[category][index] for index in entry.get(category, [])]
                    for category in slots
                },
            }
        )
    return {"motions": cases, **ungated}


def write_telemetry_artifacts(ir: dict, *, ir_path: Path, output_dir: Path, fsm_ir: dict) -> dict:
    """Write frame_layout.json and the frame-log header record, and return the frame-log header
    + sample model that codegen folds into the IR.

    The decode contract is not written here: it is serialized into the frame log's own header
    record, so a log needs no companion artifact to be read. `fsm_ir` is coord-dsl's framed FSM.
    """
    schema = build_schema(ir, ir_path=ir_path, fsm_ir=fsm_ir)
    layout = build_frame_layout(schema)
    end_state = schema["fsm"]["end"]
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "frame_layout.json").write_text(json.dumps(layout, indent=4) + "\n")
    header_record = build_frame_log_header_record(schema)
    # the record the runtime writes first, readable before any run
    from motion_spec.telemetry import frame_log_pb

    with (output_dir / "frame_log_header.pb").open("wb") as fh:
        frame_log_pb.write_delimited(fh, header_record)
    header_bytes = [f"0x{byte:02x}" for byte in header_record]
    return {
        "schema_hash": schema["schema_hash"],
        "frame_layout_hash": layout["frame_layout_hash"],
        "frame_layout": {
            "pools": schema["pools"],
            "frame_size_bytes": layout["frame_size_bytes"],
            "schema_hash": schema["schema_hash"],
            "frame_layout_hash": layout["frame_layout_hash"],
            "end_state": end_state if end_state is not None else -1,
            "nominal_period_ns": schema["control_period_ns"] or 0,
            "header_record_rows": [
                header_bytes[i : i + 16] for i in range(0, len(header_bytes), 16)
            ],
        },
        "model": build_telemetry_model(schema, ir),
    }


_SLOT_SIGNAL_FIELDS = (
    ("error_id", "error_signal"),
    ("output_id", "output_signal"),
    ("measured_id", "measured_signal"),
    ("setpoint_id", "setpoint_signal"),
    ("tolerance_id", "tolerance_signal"),
    ("difference_id", "difference_id"),
    ("evaluator_id", "evaluator_id"),
)


def fill_slot(slot, slot_entry: dict) -> None:
    """The fields a controller and a monitor share, and the signal ids this message carries."""
    slot.number, slot.id = slot_entry["index"], slot_entry["id"]
    slot.iri = slot_entry["uri"] or ""
    # the scalar only when unambiguous: an aggregate monitor watches several
    iris = slot_entry.get("constraint_uris") or (
        [uri] if (uri := slot_entry.get("constraint_uri")) else []
    )
    if len(iris) == 1:
        slot.constraint_iri = iris[0]
    ids = slot_entry.get("constraint_ids") or (
        [constraint] if (constraint := slot_entry.get("constraint")) else []
    )
    if len(ids) == 1:
        slot.constraint_id = ids[0]
    carried = slot.DESCRIPTOR.fields_by_name
    for field, key in _SLOT_SIGNAL_FIELDS:
        if field in carried:
            setattr(slot, field, slot_entry.get(key) or "")
    slot.operand_ids.extend(slot_entry.get("operand_ids") or ())


def build_frame_log_header_record(schema: dict) -> bytes:
    """Serialize the run's FrameLogRecord header: everything a decoder needs and the wire cannot say.

    Built here, once, rather than assembled by generated C++: every field is known at generation
    time, so the runtime only has to write these bytes out verbatim.
    """
    from motion_spec.telemetry import frame_log_pb

    rec = frame_log_pb.record_class()()
    header = rec.header
    header.format_version = frame_log_pb.FORMAT_VERSION
    header.schema_hash = schema["schema_hash"]
    header.descriptor_set = frame_log_pb.descriptor_set()
    header.trigger_pool = schema["pools"]["triggers"]
    platform = schema["platform"]
    header.platform_name = platform["name"] or ""
    header.simulated = bool(platform["simulated"])
    fsm = schema["fsm"]
    end_state = fsm["end"]
    header.end_state = end_state if end_state is not None else -1
    header.nominal_period_ns = schema["control_period_ns"] or 0
    header.fsm_namespace = fsm["namespace"] or ""

    spatial = schema["spatial"]
    for category, entries in (
        ("quantities", schema["quantities"]),
        ("poses", spatial["poses"]),
        ("twists", spatial["twists"]),
        ("wrenches", spatial["wrenches"]),
        ("devices", schema["devices"]),
    ):
        for entry in entries:
            slot = getattr(header, category).add()
            slot.number, slot.id = entry["index"], entry["id"]
            slot.iri = entry.get("uri") or ""

    state_index = {row["id"]: row["index"] for row in fsm["states"]}
    for motion_id, entry in schema["by_motion"].items():
        gate = header.motions.add()
        gate.index, gate.id = entry["index"], motion_id
        gate.iri = entry["uri"] or ""
        gate.fsm_state = state_index.get(entry["fsm_state"], -1)
        for category in ("quantities", "poses", "twists", "wrenches"):
            getattr(gate, category).extend(entry.get(category, ()))
        for slot_entry in entry["controllers"]:
            slot = gate.controllers.add()
            fill_slot(slot, slot_entry)
            for role, value in slot_entry["gains"].items():
                gain = slot.gains.add()
                gain.role, gain.value = role, float(value)
        for slot_entry in entry["monitors"]:
            slot = gate.monitors.add()
            fill_slot(slot, slot_entry)
            slot.event_iri = slot_entry["event_uri"] or ""
            slot.phase = slot_entry["phase"]
            slot.constraint_iris.extend(
                slot_entry.get("constraint_uris")
                or ([uri] if (uri := slot_entry.get("constraint_uri")) else [])
            )
            for member in slot_entry["watched"]:
                watched = slot.watched.add()
                watched.id = member["id"]
                watched.iri = member["uri"] or ""
                watched.error_id = member.get("error_signal") or ""
                watched.tolerance_id = member.get("tolerance_signal") or ""

    for key, target in (("states", header.fsm_states), ("events", header.fsm_events)):
        for row in fsm[key]:
            named = target.add()
            named.number = row["index"]
            named.id = str(row["id"])
            named.iri = row["uri"] or ""

    for index, row in enumerate(fsm["transitions"]):
        transition = header.fsm_transitions.add()
        transition.index, transition.id = index, str(row["id"])
        transition.iri = row["uri"] or ""
        # -1: 0 is a real state index
        transition.from_state = row["from"] if row["from"] is not None else -1
        transition.to_state = row["to"] if row["to"] is not None else -1
        event_index = row["event_index"]
        transition.event_index = event_index if event_index is not None else -1
        transition.event_indices.extend(row["event_indices"])

    for entry in schema["constants"]:
        constant = header.constants.add()
        constant.id = entry["id"]
        constant.source_id = entry["source_id"]
        constant.value = float(entry["value"])
        constant.uri = entry.get("uri") or ""
        # empty means no reader found, e.g. scene geometry baked into poses
        for reader in entry.get("readers") or ():
            added = constant.readers.add()
            added.id = reader["id"]
            added.kind = reader["kind"]
            added.port = reader["port"]

    return rec.SerializeToString()
