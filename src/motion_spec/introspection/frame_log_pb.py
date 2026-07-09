# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Length-delimited protobuf frame-log helpers."""

from __future__ import annotations

import struct
from collections.abc import Iterator
from pathlib import Path

from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.frame_layout_spec import quantity_ids

WIRE_VARINT = 0
WIRE_64BIT = 1
WIRE_LEN = 2

REC_HEADER = 1
REC_FRAME = 2


def _key(field: int, wire_type: int) -> int:
    return (field << 3) | wire_type


def _varint(value: int) -> bytes:
    out = bytearray()
    while value >= 0x80:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def _read_varint(data: bytes, pos: int) -> tuple[int, int]:
    shift = 0
    value = 0
    while pos < len(data):
        byte = data[pos]
        pos += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, pos
        shift += 7
        if shift > 63:
            break
    raise ArchiveError("invalid protobuf varint in frame log")


def _bytes_field(field: int, value: bytes) -> bytes:
    return _varint(_key(field, WIRE_LEN)) + _varint(len(value)) + value


def _string_field(field: int, value: str) -> bytes:
    return _bytes_field(field, value.encode())


def _uint_field(field: int, value: int) -> bytes:
    return _varint(_key(field, WIRE_VARINT)) + _varint(value)


def _sfixed64_field(field: int, value: int) -> bytes:
    return _varint(_key(field, WIRE_64BIT)) + struct.pack("<q", value)


def _double_field(field: int, value: float) -> bytes:
    return _varint(_key(field, WIRE_64BIT)) + struct.pack("<d", value)


def header_record(schema: dict, layout: dict | None = None) -> bytes:
    meta = schema.get("runtime_provenance", {})
    header = b"".join(
        (
            _string_field(1, schema["schema_hash"]),
            _string_field(2, meta.get("producer_agent_id", "")),
            _string_field(3, meta.get("activity_id", "")),
        )
    )
    return _bytes_field(REC_HEADER, header)


def _slot(fields: list[tuple[int, str, object]]) -> bytes:
    out = []
    for number, kind, value in fields:
        if kind == "i":
            out.append(_sfixed64_field(number, int(value)))
        elif kind == "u":
            out.append(_uint_field(number, int(value)))
        elif kind == "d":
            out.append(_double_field(number, float(value)))
    return b"".join(out)


def frame_record(flat: dict, schema: dict) -> bytes:
    pools = schema["pools"]
    payload = [
        _double_field(1, flat["t"]),
        _uint_field(2, flat["step"]),
        _sfixed64_field(3, flat["fsm_state"]),
        _sfixed64_field(4, flat["active_motion"]),
        _sfixed64_field(5, flat["last_event"]),
        _double_field(6, flat["state_since_t"]),
        _sfixed64_field(7, flat["state_since_wall_ns"]),
        _double_field(8, flat["event_t"]),
        _sfixed64_field(9, flat["event_wall_ns"]),
        _sfixed64_field(10, flat["wall_ns"]),
        _sfixed64_field(11, flat["period_ns"]),
        _sfixed64_field(12, flat["compute_ns"]),
    ]
    for idx in range(pools["constraints"]):
        payload.append(
            _bytes_field(
                13,
                _slot(
                    [
                        (1, "i", flat[f"c{idx}.active"]),
                        (2, "d", flat[f"c{idx}.error"]),
                        (3, "d", flat[f"c{idx}.output"]),
                        (4, "i", flat[f"c{idx}.satisfied"]),
                        (5, "d", flat[f"c{idx}.sat_t"]),
                        (6, "d", flat[f"c{idx}.measured"]),
                        (7, "d", flat[f"c{idx}.setpoint"]),
                    ]
                ),
            )
        )
    for idx in range(pools["monitors"]):
        payload.append(
            _bytes_field(
                14,
                _slot(
                    [
                        (1, "i", flat[f"m{idx}.active"]),
                        (2, "d", flat[f"m{idx}.value"]),
                        (3, "i", flat[f"m{idx}.satisfied"]),
                        (4, "d", flat[f"m{idx}.sat_t"]),
                    ]
                ),
            )
        )
    for idx in range(pools["quantities"]):
        payload.append(_double_field(15, flat[f"q{idx}"]))
    for idx in range(pools["triggers"]):
        payload.append(
            _bytes_field(
                16,
                _slot(
                    [
                        (1, "i", flat[f"tr{idx}.kind"]),
                        (2, "i", flat[f"tr{idx}.idx"]),
                        (3, "i", flat[f"tr{idx}.fsm_state"]),
                        (4, "d", flat[f"tr{idx}.t"]),
                        (5, "i", flat[f"tr{idx}.wall_ns"]),
                    ]
                ),
            )
        )
    payload.append(_sfixed64_field(17, flat["trigger_count"]))
    for idx in range(pools.get("poses", 0)):
        payload.append(
            _bytes_field(
                18,
                _slot([(n, "d", flat[f"pose{idx}.{name}"]) for n, name in enumerate(("px", "py", "pz", "qx", "qy", "qz", "qw"), 1)]),
            )
        )
    for idx in range(pools.get("twists", 0)):
        payload.append(
            _bytes_field(
                19,
                _slot([(n, "d", flat[f"twist{idx}.{name}"]) for n, name in enumerate(("lx", "ly", "lz", "ax", "ay", "az"), 1)]),
            )
        )
    for idx in range(pools.get("wrenches", 0)):
        payload.append(
            _bytes_field(
                20,
                _slot([(n, "d", flat[f"wrench{idx}.{name}"]) for n, name in enumerate(("fx", "fy", "fz", "tx", "ty", "tz"), 1)]),
            )
        )
    return _bytes_field(REC_FRAME, b"".join(payload))


def write_delimited(fh, message: bytes) -> None:
    fh.write(_varint(len(message)))
    fh.write(message)


def _read_delimited(fh) -> bytes | None:
    first = fh.read(1)
    if not first:
        return None
    prefix = bytearray(first)
    while prefix[-1] & 0x80:
        nxt = fh.read(1)
        if not nxt:
            raise ArchiveError("truncated protobuf frame log length")
        prefix.extend(nxt)
    size, _pos = _read_varint(bytes(prefix), 0)
    data = fh.read(size)
    if len(data) != size:
        raise ArchiveError("truncated protobuf frame log")
    return data


def _fields(data: bytes) -> Iterator[tuple[int, int, object]]:
    pos = 0
    while pos < len(data):
        key, pos = _read_varint(data, pos)
        field = key >> 3
        wire_type = key & 7
        if wire_type == WIRE_VARINT:
            value, pos = _read_varint(data, pos)
            yield field, wire_type, value
        elif wire_type == WIRE_64BIT:
            if pos + 8 > len(data):
                raise ArchiveError("truncated protobuf 64-bit field")
            yield field, wire_type, data[pos : pos + 8]
            pos += 8
        elif wire_type == WIRE_LEN:
            size, pos = _read_varint(data, pos)
            if pos + size > len(data):
                raise ArchiveError("truncated protobuf length field")
            yield field, wire_type, data[pos : pos + size]
            pos += size
        else:
            raise ArchiveError(f"unsupported protobuf wire type {wire_type}")


def _parse_header(data: bytes) -> dict:
    out = {}
    names = {
        1: "schema_hash",
        2: "producer_agent_id",
        3: "activity_id",
    }
    for field, _wire, value in _fields(data):
        name = names.get(field)
        if not name:
            continue
        if isinstance(value, int):
            out[name] = value
        else:
            out[name] = bytes(value).decode()
    return out


def _sfixed64(value: object) -> int:
    return struct.unpack("<q", bytes(value))[0]


def _double(value: object) -> float:
    return struct.unpack("<d", bytes(value))[0]


def _parse_constraint(data: bytes) -> dict:
    out = {"active": 0, "error": 0.0, "output": 0.0, "satisfied": 0, "sat_t": 0.0, "measured": 0.0, "setpoint": 0.0}
    for field, _wire, value in _fields(data):
        if field == 1:
            out["active"] = _sfixed64(value)
        elif field == 2:
            out["error"] = _double(value)
        elif field == 3:
            out["output"] = _double(value)
        elif field == 4:
            out["satisfied"] = _sfixed64(value)
        elif field == 5:
            out["sat_t"] = _double(value)
        elif field == 6:
            out["measured"] = _double(value)
        elif field == 7:
            out["setpoint"] = _double(value)
    return out


def _parse_monitor(data: bytes) -> dict:
    out = {"active": 0, "value": 0.0, "satisfied": 0, "sat_t": 0.0}
    for field, _wire, value in _fields(data):
        if field == 1:
            out["active"] = _sfixed64(value)
        elif field == 2:
            out["value"] = _double(value)
        elif field == 3:
            out["satisfied"] = _sfixed64(value)
        elif field == 4:
            out["sat_t"] = _double(value)
    return out


def _parse_trigger(data: bytes) -> dict:
    out = {"kind": 0, "idx": 0, "fsm_state": 0, "t": 0.0, "wall_ns": 0}
    for field, _wire, value in _fields(data):
        if field == 1:
            out["kind"] = _sfixed64(value)
        elif field == 2:
            out["idx"] = _sfixed64(value)
        elif field == 3:
            out["fsm_state"] = _sfixed64(value)
        elif field == 4:
            out["t"] = _double(value)
        elif field == 5:
            out["wall_ns"] = _sfixed64(value)
    return out


def _parse_doubles(data: bytes, names: tuple[str, ...]) -> dict:
    out = {name: 0.0 for name in names}
    for field, _wire, value in _fields(data):
        if 1 <= field <= len(names):
            out[names[field - 1]] = _double(value)
    return out


def _parse_frame(data: bytes, schema: dict) -> dict:
    pools = schema["pools"]
    constraints = []
    monitors = []
    quantities = []
    triggers = []
    poses = []
    twists = []
    wrenches = []
    record = {
        "t": 0.0,
        "step": 0,
        "fsm_state": 0,
        "active_motion": 0,
        "last_event": 0,
        "state_since_t": 0.0,
        "state_since_wall_ns": 0,
        "event_t": 0.0,
        "event_wall_ns": 0,
        "timing": {"wall_ns": 0, "period_ns": 0, "compute_ns": 0},
    }
    trigger_count = 0
    for field, wire, value in _fields(data):
        if field == 1 and wire == WIRE_64BIT:
            record["t"] = _double(value)
        elif field == 2 and isinstance(value, int):
            record["step"] = value
        elif field == 3:
            record["fsm_state"] = _sfixed64(value)
        elif field == 4:
            record["active_motion"] = _sfixed64(value)
        elif field == 5:
            record["last_event"] = _sfixed64(value)
        elif field == 6:
            record["state_since_t"] = _double(value)
        elif field == 7:
            record["state_since_wall_ns"] = _sfixed64(value)
        elif field == 8:
            record["event_t"] = _double(value)
        elif field == 9:
            record["event_wall_ns"] = _sfixed64(value)
        elif field == 10:
            record["timing"]["wall_ns"] = _sfixed64(value)
        elif field == 11:
            record["timing"]["period_ns"] = _sfixed64(value)
        elif field == 12:
            record["timing"]["compute_ns"] = _sfixed64(value)
        elif field == 13:
            constraints.append(_parse_constraint(bytes(value)))
        elif field == 14:
            monitors.append(_parse_monitor(bytes(value)))
        elif field == 15:
            quantities.append(_double(value))
        elif field == 16:
            triggers.append(_parse_trigger(bytes(value)))
        elif field == 17:
            trigger_count = _sfixed64(value)
        elif field == 18:
            poses.append(_parse_doubles(bytes(value), ("px", "py", "pz", "qx", "qy", "qz", "qw")))
        elif field == 19:
            twists.append(_parse_doubles(bytes(value), ("lx", "ly", "lz", "ax", "ay", "az")))
        elif field == 20:
            wrenches.append(_parse_doubles(bytes(value), ("fx", "fy", "fz", "tx", "ty", "tz")))
    qids = quantity_ids(schema["quantities"])
    record["constraints"] = constraints[: pools["constraints"]]
    record["monitors"] = monitors[: pools["monitors"]]
    record["quantities"] = {qids[idx]: quantities[idx] for idx in range(min(len(qids), len(quantities)))}
    start = max(0, trigger_count - pools["triggers"])
    record["triggers"] = (
        [triggers[idx % pools["triggers"]] for idx in range(start, trigger_count)]
        if triggers and pools["triggers"]
        else []
    )
    record["poses"] = poses[: pools.get("poses", 0)]
    record["twists"] = twists[: pools.get("twists", 0)]
    record["wrenches"] = wrenches[: pools.get("wrenches", 0)]
    return record


def iter_messages(path: Path | str, schema: dict | None = None) -> Iterator[tuple[str, object]]:
    with Path(path).open("rb") as fh:
        while True:
            message = _read_delimited(fh)
            if message is None:
                return
            for field, _wire, value in _fields(message):
                if field == REC_HEADER:
                    yield "header", _parse_header(bytes(value))
                elif field == REC_FRAME:
                    if schema is None:
                        yield "frame", bytes(value)
                    else:
                        yield "frame", _parse_frame(bytes(value), schema)


def read_header(path: Path | str) -> dict:
    for kind, value in iter_messages(path):
        if kind == "header":
            return dict(value)
    raise ArchiveError(f"{path}: protobuf header not found")


def frame_records(path: Path | str, schema: dict, layout: dict | None = None) -> Iterator[dict]:
    for kind, value in iter_messages(path, schema):
        if kind != "frame":
            continue
        yield value
