# SPDX-License-Identifier: MPL-2.0
"""Binary frame layout used by generated introspection logs."""

from __future__ import annotations

import struct

FIELD_BYTES = 8
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


def field_names_and_format(pools: dict) -> tuple[str, list[str]]:
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
    return fmt, names


def fields_with_offsets(pools: dict) -> tuple[list[dict], int]:
    fmt, names = field_names_and_format(pools)
    fields = [
        {"name": name, "fmt": code, "offset": idx * FIELD_BYTES, "size": FIELD_BYTES}
        for idx, (name, code) in enumerate(zip(names, fmt[1:]))
    ]
    return fields, len(fields) * FIELD_BYTES


def frame_struct(pools: dict) -> tuple[struct.Struct, list[str]]:
    fmt, names = field_names_and_format(pools)
    return struct.Struct(fmt), names


def _json_type(code: str):
    # Doubles may be non-finite; the writer emits null for those, so allow it.
    return ["number", "null"] if code == "d" else "integer"


def _slot_schema(fields: list[tuple[str, str]]) -> dict:
    return {
        "type": "array",
        "items": {
            "type": "object",
            "properties": {name: {"type": _json_type(code)} for name, code in fields},
            "required": [name for name, _ in fields],
        },
    }


def frame_json_schema() -> dict:
    """JSON Schema for the nested per-frame record emitted to the .mcap log.

    Mirrors ``replay.to_record()`` so a decoded mcap message equals the decoded
    ``.bin`` frame. Pool-size independent (repeated arrays, not fixed slots)."""
    top = {
        name: {"type": _json_type(code)}
        for name, code in HEADER
        if name not in ("seq", "wall_ns", "period_ns", "compute_ns")
    }
    top["timing"] = {
        "type": "object",
        "properties": {k: {"type": "integer"} for k in ("wall_ns", "period_ns", "compute_ns")},
        "required": ["wall_ns", "period_ns", "compute_ns"],
    }
    top["constraints"] = _slot_schema(CSLOT)
    top["monitors"] = _slot_schema(MSLOT)
    top["quantities"] = {"type": "array", "items": {"type": ["number", "null"]}}
    top["triggers"] = _slot_schema(TSLOT)
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": "motion_spec.introspection.Frame",
        "type": "object",
        "properties": top,
    }

