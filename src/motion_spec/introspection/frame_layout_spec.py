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
