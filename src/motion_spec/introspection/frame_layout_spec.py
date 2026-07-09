# SPDX-License-Identifier: MPL-2.0
"""Binary frame layout used by generated introspection logs.

The layout sizes the in-memory ``Frame`` struct (and its ``frame_size_bytes`` hash); the
log stores protobuf-delimited frame payloads with this binary struct inside. Spatial slots
(pose/twist/wrench) ride in the struct for offline export/visualization."""

from __future__ import annotations

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
# Spatial slots — feed the pose/twist/wrench channels, not the frame record.
PSLOT = [("px", "d"), ("py", "d"), ("pz", "d"), ("qx", "d"), ("qy", "d"), ("qz", "d"), ("qw", "d")]
VSLOT = [("lx", "d"), ("ly", "d"), ("lz", "d"), ("ax", "d"), ("ay", "d"), ("az", "d")]
KSLOT = [("fx", "d"), ("fy", "d"), ("fz", "d"), ("tx", "d"), ("ty", "d"), ("tz", "d")]


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
    for prefix, slot, key in (("pose", PSLOT, "poses"), ("twist", VSLOT, "twists"), ("wrench", KSLOT, "wrenches")):
        for idx in range(pools.get(key, 0)):
            fmt += "".join(code for _, code in slot)
            names.extend(f"{prefix}{idx}.{name}" for name, _ in slot)
    return fmt, names


def fields_with_offsets(pools: dict) -> tuple[list[dict], int]:
    fmt, names = field_names_and_format(pools)
    fields = [
        {"name": name, "fmt": code, "offset": idx * FIELD_BYTES, "size": FIELD_BYTES}
        for idx, (name, code) in enumerate(zip(names, fmt[1:]))
    ]
    return fields, len(fields) * FIELD_BYTES


def quantity_ids(quantities: list[dict]) -> list[str]:
    """Ordered quantity ids (by pool index) — the keys of the frame ``quantities`` object
    and the C++ ``kQuantityIds[]`` table."""
    return [q["id"] for q in sorted(quantities, key=lambda q: q.get("index", 0))]


def _slot_schema(fields: list[tuple[str, str]]) -> dict:
    return {
        "type": "array",
        "items": {
            "type": "object",
            "properties": {name: {"type": _json_type(code)} for name, code in fields},
            "required": [name for name, _ in fields],
        },
    }


def _json_type(code: str):
    # Doubles may be non-finite; the writer emits null for those, so allow it.
    return ["number", "null"] if code == "d" else "integer"


def _quantity_property(quantity: dict) -> dict:
    prop: dict = {"type": ["number", "null"], "title": quantity["id"]}
    bits = []
    if quantity.get("quantity_kind"):
        bits.append("/".join(quantity["quantity_kind"]))
    if quantity.get("unit"):
        bits.append("[" + "/".join(quantity["unit"]) + "]")
    if quantity.get("uri"):
        bits.append(quantity["uri"])
    if bits:
        prop["description"] = " ".join(bits)
    return prop


def frame_json_schema(quantities: list[dict]) -> dict:
    """JSON Schema for the nested per-frame record on ``/motion_spec/frame``.

    ``quantities`` is a **named object** keyed by quantity id (each signal a typed,
    plottable Foxglove path) — mirrors ``replay.to_record()``. Pool-size independent for
    the repeated slot arrays; the quantity object is model-specific."""
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
    top["quantities"] = {
        "type": "object",
        "properties": {q["id"]: _quantity_property(q) for q in sorted(quantities, key=lambda q: q.get("index", 0))},
    }
    top["triggers"] = _slot_schema(TSLOT)
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": "motion_spec.introspection.Frame",
        "type": "object",
        "properties": top,
    }
