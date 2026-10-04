# SPDX-License-Identifier: MPL-2.0
"""Static codegen contract artifacts generated from the enriched motion-spec IR."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

# The shared-memory Frame struct frame_layout.json describes; a reader refuses any other.
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


def fields_with_offsets(pools: dict) -> tuple[list[dict], int]:
    """Per-field {name, fmt, offset, size} list and the total frame size in bytes."""
    fmt, names = field_names_and_format(pools)
    fields = [
        {"name": name, "fmt": code, "offset": idx * FIELD_BYTES, "size": FIELD_BYTES}
        for idx, (name, code) in enumerate(zip(names, fmt[1:]))
    ]
    return fields, len(fields) * FIELD_BYTES


def _uri_by_id(ir: dict) -> dict:
    """Map every telemetry id to its canonical URI."""
    return {row["id"]: row["uri"] for row in ir["communication"]["telemetry"]["uris"] if row["uri"]}


def _signal_id(value) -> str | None:
    """Id of a signal value (a dict with 'id', a str, or None)."""
    if isinstance(value, dict):
        return value["id"]
    if isinstance(value, str):
        return value
    return None


# What an evaluator compares, by closure type: the operand keys, in the order the closure reads
# them. A numeric literal in one of these slots is a value, not an id, so only strings travel.
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


def _evaluator_by_error(ir: dict) -> dict:
    """Evaluator closure keyed by every error signal it produces: what computes that error."""
    index: dict = {}
    # Closure records differ by type, so their keys stay optional.
    for closure in ir["computation"]["closures"].values():
        if not isinstance(closure, dict) or not str(closure.get("type", "")).endswith("Evaluator"):
            continue
        for error in (closure.get("error"), *(closure.get("errors") or ())):
            error_id = _signal_id(error)
            if error_id and index.setdefault(error_id, closure) != closure:
                raise ValueError(
                    f"error '{error_id}' is computed by both '{index[error_id].get('id')}' and "
                    f"'{closure.get('id')}' -- one evaluator writes an error"
                )
    return index


def _evaluator_terms(evaluators: dict, error_id: str | None) -> dict:
    """The slot's evaluator, the quantities it compares and the difference it writes.

    Empty when no closure produces this error: a slot says what the run computes, never a guess.
    """
    closure = evaluators.get(error_id) if error_id else None
    if closure is None:
        return {}
    operands = [
        value
        for key in _EVALUATOR_OPERANDS.get(closure.get("type"), ())
        if isinstance(value := closure.get(key), str)
    ]
    difference_id = _signal_id(closure.get("out"))
    return {
        "evaluator_id": closure.get("id"),
        **({"operand_ids": operands} if operands else {}),
        **({"difference_id": difference_id} if difference_id else {}),
    }


def _controller_slot(row: dict, index: int, uri_by_id: dict, evaluators: dict) -> dict:
    """Telemetry slot for a controller: its published row, indexed, with signal URIs.

    A row drops every key it leaves unset, so its signals are read as optional.
    """
    error_id = row.get("error_signal")
    output_id = row.get("output_signal")
    reference_id = row.get("reference_signal")
    measured_id = row.get("measured_signal")
    setpoint_id = reference_id or row.get("setpoint_signal")
    measured_derivative_id = row.get("measured_derivative")
    tolerance_id = row.get("tolerance_signal")
    return {
        "index": index,
        "id": row["id"],
        "uri": row.get("uri"),
        "motion": row.get("motion"),
        "type": row.get("type"),
        # The constraint this controller serves: what a plot of its error is about.
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
        **_evaluator_terms(evaluators, error_id),
    }


def _monitor_slot(
    monitor: dict,
    index: int,
    motion: dict,
    uri_by_id: dict,
    phase: str,
    row: dict,
    evaluators: dict,
) -> dict:
    """Telemetry slot for a monitor: trigger, event/flag and active-condition terms.

    `row` is the monitor's published telemetry row: it carries the watched constraints, which
    are construction-only on the coordination record. A row drops the keys it leaves unset; on
    the record, the event fields exist only on an edge monitor and `flag` only on a level one.
    """
    error = monitor["error"]
    error_id = _signal_id(error)
    tolerance_id = _signal_id(monitor["tolerance"])
    event_id = monitor.get("event")
    return {
        "index": index,
        "id": monitor["id"],
        "uri": uri_by_id.get(monitor["id"]),
        "motion": motion["id"],
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
        # Only when authored: an always-present key would move every schema hash.
        **({"tolerance_signal": tolerance_id} if tolerance_id else {}),
        "composite_error": isinstance(error, dict) and error["type"] in {"Pose", "VelocityTwist"},
        "has_active": monitor["has_active"],
        "active_terms": monitor["active_terms"],
        "active_any": monitor["active_any"],
        "fallback_motion": monitor.get("fallback_motion"),
        **_evaluator_terms(evaluators, error_id),
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
    state_index = {state: idx for idx, state in enumerate(fsm_ir["states"])}
    event_index = {event: idx for idx, event in enumerate(fsm_ir["events"])}
    # A transition can be driven by more than one event, so collect them all rather than let the
    # last reaction win. `event`/`event_index` stay singular and are only filled when the answer is
    # unambiguous; a reader that needs the full picture uses `events`/`event_indices`.
    transition_events: dict[str, list] = {}
    for reaction in fsm_ir["reactions_table"]:
        transition_events.setdefault(reaction["do_transition"], []).append(reaction["when_event"])
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
                "event_indices": [event_index[event] for event in events if event in event_index],
            }
        )

    return {
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


def build_schema(ir: dict, *, ir_path: Path, output_dir: Path, fsm_ir: dict | None) -> dict:
    """Build the run's telemetry schema (pools, per-state slots, quantities, provenance) and its schema_hash."""
    telemetry = ir["communication"]["telemetry"]
    uri_by_id = _uri_by_id(ir)
    evaluators = _evaluator_by_error(ir)
    fsm = _fsm_meta(fsm_ir)
    motions = ir["coordination"]["motions"]
    motion_by_id = {motion["id"]: motion for motion in motions}
    monitor_rows = {row["id"]: row for row in telemetry["monitors"]}
    controller_rows = {row["id"]: row for row in telemetry["controllers"]}
    states = fsm["states"]
    state_by_id = {state["id"]: state for state in states}
    # Slots are keyed by the motion that computes them, not by the coordinator state that happens
    # to select it: a motion owns the closures and solvers that write its values under an FSM, a
    # behaviour tree or a plain sequencer alike. The index space is ir_gen's motion order,
    # so nothing downstream has to agree with a second generator about what index 3 means.
    by_motion = {}
    # ir_gen owns this index (add_motion_function_interfaces); read it, never re-derive it.
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

        controller_slots = [
            _controller_slot(controller_rows[controller["id"]], idx, uri_by_id, evaluators)
            for idx, controller in enumerate(motion["controllers"])
        ]
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
        monitor_slots = [
            _monitor_slot(
                monitor, idx, owner, uri_by_id, phase, monitor_rows[monitor["id"]], evaluators
            )
            for idx, (phase, monitor, owner) in enumerate(monitor_sources)
        ]
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
    # A gated slot keeps its global index for the whole run -- only the set_ call is gated -- so a
    # decoder resolves "unset" against the writing motions here rather than guessing from absence.
    spatial = telemetry["spatial_samples"]
    dataflow = telemetry["dataflow"]
    # A model with no arm has no serial chain.
    devices = sorted(
        (
            {
                "index": device["health_index"],
                "id": device["config_key"],
                "required_by_motion": device["required_by_motion"],
            }
            for solver in ir["resources"]["by_kind"].get("serial_chain", [])
            if solver["runtime"]["owner"]
            for device in solver["devices"]
            if device["health_index"] is not None
        ),
        key=lambda device: device["index"],
    )
    # What this program can record, named as the runtime names it. A reader of the run -- the
    # dashboard, a script -- asks the contract what the cameras are; the model behind them is
    # the generator's to read, not theirs.
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
        (category, row["index"], (dataflow.get(row["id"]) or {}).get("cadence"))
        for category, rows in spatial.items()
        for row in rows
    ]
    # A cadence is already expressed in motions -- no coordinator in between.
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
        # Sized for the events one tick can produce; the heartbeat is not recorded, so it
        # needs no slot.
        "triggers": max(TRIGGER_POOL_SIZE, len(fsm["events"]), max_monitors),
        "poses": len(spatial["poses"]),
        "twists": len(spatial["twists"]),
        "wrenches": len(spatial["wrenches"]),
    }
    schema = {
        "generated_by": "motion_spec.generation.codegen",
        # Portable basenames only — absolute build-tree paths here would leak machine
        # paths into the archive AND make schema_hash (carried in the frame-log header)
        # depend on where the build ran. The archive resolves these against its own dirs.
        "ir_path": Path(ir_path).name,
        # The authored execution platform travels with the run so the runner and the archive
        # validator read one fact rather than sniffing a derived agent id.
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
        # Written once at init: one copy in the header says everything repeating it per tick would.
        "constants": telemetry["constants"],
        # The dataflow contract for everything that survives into the layout, so a reader can see
        # who writes each value and when without re-deriving it from the model graph.
        "catalogue": [
            {"id": member_id, **entry}
            for member_id, entry in dataflow.items()
            if entry["storage"] != "absent"
        ],
        "spatial": spatial,
        "signals": telemetry["signals"],
    }
    schema["schema_hash"] = hashlib.sha256(json.dumps(schema, sort_keys=True).encode()).hexdigest()[
        :16
    ]
    return schema


def build_frame_layout(schema: dict) -> dict:
    """Compute the binary frame layout (field offsets, frame size) and its frame_layout_hash from the schema."""
    fields, frame_size = fields_with_offsets(schema["pools"])
    layout = {
        "frame_layout_version": FRAME_LAYOUT_VERSION,
        "pools": schema["pools"],
        "field_bytes": FIELD_BYTES,
        "frame_size_bytes": frame_size,
        "schema_hash": schema["schema_hash"],
        # The runner records the run before any log exists, so this one fact cannot
        # come from the log's own header.
        "platform": schema["platform"],
        # Recording is chosen before a run starts, so the choice is offered from here.
        "cameras": schema["cameras"],
        "fields": fields,
    }
    layout["frame_layout_hash"] = hashlib.sha256(
        json.dumps(layout, sort_keys=True).encode()
    ).hexdigest()[:16]
    return layout


def _uri_comment(uri: str | None) -> str:
    """A model URI safe to drop into a C++ line comment (no newlines, no comment terminator)."""
    return re.sub(r"[\r\n]|\*/", " ", uri or "")


def build_telemetry_model(schema: dict, ir: dict) -> dict:
    """Per-FSM-state sample model (controller/monitor exprs, quantity/spatial ids) that the telemetry_model template renders."""
    shared_ids = {item["id"] for item in ir["computation"]["shared_data"]}
    # A value only its own motion recomputes is stale whenever another motion is active, so its
    # sample call moves into that motion's case; one written by the global schedule stays
    # unconditional.
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
    # A motion entry carries a category only when one of its slots is gated to it.
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
            # error/output are the controller's own dedicated shared fields (never views), so the
            # template reads them with shared-sig. measured/setpoint may reference a view, so they
            # go through access-expr(id, views) instead. A slot names a tolerance only when authored.
            for role in ("error_signal", "tolerance_signal", "output_signal"):
                if slot.get(role) and slot[role] not in shared_ids:
                    raise ValueError(f"telemetry {role} '{slot[role]}' names no shared field")
            controllers.append(
                {
                    "index": slot["index"],
                    "uri_comment": _uri_comment(slot["uri"]),
                    "error_signal": slot["error_signal"] or None,
                    "tolerance_signal": slot.get("tolerance_signal") or None,
                    "output_signal": slot["output_signal"] or None,
                    "measured_signal": slot["measured_signal"],
                    "setpoint_signal": slot["setpoint_signal"],
                }
            )
        monitors = []
        for slot in entry["monitors"]:
            # Active (aggregate/elapsed) monitors: the boolean value/satisfied condition is
            # rendered from the structured terms by the bool-condition template. Plain error
            # monitors: sample the error value + constraint_satisfied (a plain shared field).
            if slot["has_active"]:
                monitors.append(
                    {
                        "index": slot["index"],
                        "uri_comment": _uri_comment(slot["uri"]),
                        "has_active": True,
                        "active_terms": slot["active_terms"],
                        "active_any": slot["active_any"],
                    }
                )
            else:
                for role in ("error_signal", "tolerance_signal"):
                    if slot.get(role) and slot[role] not in shared_ids:
                        raise ValueError(f"telemetry {role} '{slot[role]}' names no shared field")
                monitors.append(
                    {
                        "index": slot["index"],
                        "uri_comment": _uri_comment(slot["uri"]),
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
    """Write frame_layout.json, provenance.ld.json and the derivation graph, and return the
    frame-log header + sample model that codegen folds into the IR.

    The decode contract is not written here: it is serialized into the frame log's own header
    record, so a log needs no companion artifact to be read. `fsm_ir` is coord-dsl's framed FSM.
    """
    schema = build_schema(ir, ir_path=ir_path, output_dir=output_dir, fsm_ir=fsm_ir)
    layout = build_frame_layout(schema)
    end_state = schema["fsm"]["end"]
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "frame_layout.json").write_text(json.dumps(layout, indent=4) + "\n")
    header_record = build_frame_log_header_record(schema)
    # The same delimited record the runtime writes as the log's first bytes: a reader gets the
    # decode contract from the generation before any run has written a log.
    from motion_spec.telemetry import frame_log_pb

    with (output_dir / "frame_log_header.pb").open("wb") as fh:
        frame_log_pb.write_delimited(fh, header_record)
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
            "header_record_rows": _hex_rows(header_record),
        },
        "model": build_telemetry_model(schema, ir),
    }


def _hex_rows(blob: bytes, per_row: int = 16) -> list[list[str]]:
    """Byte literals grouped into rows, so the emitted array is not one enormous line."""
    values = [f"0x{byte:02x}" for byte in blob]
    return [values[i : i + per_row] for i in range(0, len(values), per_row)]


# Header field <- slot-entry key: the quantity ids that join a slot to the quantity pool.
_SLOT_SIGNAL_FIELDS = (
    ("error_id", "error_signal"),
    ("output_id", "output_signal"),
    ("measured_id", "measured_signal"),
    ("setpoint_id", "setpoint_signal"),
    ("tolerance_id", "tolerance_signal"),
    # The closure that evaluates the constraint, and the difference it writes.
    ("difference_id", "difference_id"),
    ("evaluator_id", "evaluator_id"),
)


def _as_list(value) -> list:
    """A scalar as the one-element list the repeated field wants, or empty when unset."""
    return [value] if value else []


def _constraint_iris(slot_entry: dict) -> list:
    return slot_entry.get("constraint_uris") or _as_list(slot_entry.get("constraint_uri"))


def _fill_slot(slot, slot_entry: dict) -> None:
    """The fields a controller and a monitor share, and the signal ids this message carries."""
    slot.number, slot.id = slot_entry["index"], slot_entry["id"]
    slot.iri = slot_entry["uri"] or ""
    # A controller slot names one constraint, a monitor slot a list; the signal ids and the
    # evaluator terms are there only when the slot has them.
    # The scalar only when it is unambiguous: an aggregate monitor watches several.
    iris = _constraint_iris(slot_entry)
    if len(iris) == 1:
        slot.constraint_iri = iris[0]
    ids = slot_entry.get("constraint_ids") or _as_list(slot_entry.get("constraint"))
    if len(ids) == 1:
        slot.constraint_id = ids[0]
    carried = slot.DESCRIPTOR.fields_by_name
    for field, key in _SLOT_SIGNAL_FIELDS:
        if field in carried:
            setattr(slot, field, slot_entry.get(key) or "")
    # The two quantities the evaluator compares: what "between" is drawn from.
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
    # A real platform has no name; a simulator does.
    header.platform_name = platform["name"] or ""
    header.simulated = bool(platform["simulated"])
    fsm = schema["fsm"]
    end_state = fsm["end"]
    header.end_state = end_state if end_state is not None else -1
    header.nominal_period_ns = schema["control_period_ns"] or 0
    header.fsm_namespace = fsm["namespace"] or ""

    spatial = schema["spatial"]
    # A quantity row carries the IRI it was registered under; a spatial or device row may not.
    for category, entries in (
        ("quantities", schema["quantities"]),
        ("poses", spatial["poses"]),
        ("twists", spatial["twists"]),
        ("wrenches", spatial["wrenches"]),
        ("devices", schema["devices"]),
    ):
        for entry in sorted(entries, key=lambda e: e["index"]):
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
            _fill_slot(slot, slot_entry)
            for role, value in slot_entry["gains"].items():
                gain = slot.gains.add()
                gain.role, gain.value = role, float(value)
        for slot_entry in entry["monitors"]:
            slot = gate.monitors.add()
            _fill_slot(slot, slot_entry)
            # The event it fires: runtime.ttl attributes an occurrence to it, not to the slot.
            slot.event_iri = slot_entry["event_uri"] or ""
            slot.phase = slot_entry["phase"]
            slot.constraint_iris.extend(_constraint_iris(slot_entry))
            # An aggregate monitor's members, each with the error it is judged by; a member's
            # signals are dropped when unset.
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
        # -1, not 0: 0 is a real state index, so a missing endpoint must not read as one.
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
        # A constant drops the IRI and the readers it has none of.
        constant.uri = entry.get("uri") or ""
        # Empty means the deriver looked and found nobody, not that it did not look: scene
        # geometry is baked into poses at generation time and no reader binds it.
        for reader in entry.get("consumers") or ():
            consumer = constant.consumers.add()
            consumer.id = reader["id"]
            consumer.kind = reader["kind"]
            consumer.role = reader["role"]

    return rec.SerializeToString()
