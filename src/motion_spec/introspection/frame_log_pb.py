# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Length-delimited protobuf frame-log helpers."""

from __future__ import annotations

import struct
from collections.abc import Iterator
from pathlib import Path

from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.frame_layout_spec import CSLOT, MSLOT, TSLOT, field_names_and_format, quantity_ids

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


def _int64_field(field: int, value: int) -> bytes:
    return _varint(_key(field, WIRE_64BIT)) + struct.pack("<q", value)


def header_record(schema: dict, layout: dict) -> bytes:
    meta = schema.get("runtime_provenance", {})
    header = b"".join(
        (
            _string_field(1, schema["schema_hash"]),
            _string_field(2, layout["frame_layout_hash"]),
            _string_field(3, meta.get("producer_agent_id", "")),
            _string_field(4, meta.get("activity_id", "")),
            _uint_field(5, int(layout["frame_size_bytes"])),
        )
    )
    return _bytes_field(REC_HEADER, header)


def frame_record(step: int, wall_ns: int, frame: bytes) -> bytes:
    payload = b"".join((_uint_field(1, step), _int64_field(2, wall_ns), _bytes_field(3, frame)))
    return _bytes_field(REC_FRAME, payload)


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
        2: "frame_layout_hash",
        3: "producer_agent_id",
        4: "activity_id",
        5: "frame_size_bytes",
    }
    for field, _wire, value in _fields(data):
        name = names.get(field)
        if not name:
            continue
        out[name] = value if isinstance(value, int) else bytes(value).decode()
    return out


def _parse_frame(data: bytes) -> tuple[int, int, bytes]:
    step = 0
    wall_ns = 0
    frame = b""
    for field, wire, value in _fields(data):
        if field == 1 and isinstance(value, int):
            step = value
        elif field == 2 and wire == WIRE_64BIT:
            wall_ns = struct.unpack("<q", bytes(value))[0]
        elif field == 3:
            frame = bytes(value)
    return step, wall_ns, frame


def iter_messages(path: Path | str) -> Iterator[tuple[str, object]]:
    with Path(path).open("rb") as fh:
        while True:
            message = _read_delimited(fh)
            if message is None:
                return
            for field, _wire, value in _fields(message):
                if field == REC_HEADER:
                    yield "header", _parse_header(bytes(value))
                elif field == REC_FRAME:
                    yield "frame", _parse_frame(bytes(value))


def read_header(path: Path | str) -> dict:
    for kind, value in iter_messages(path):
        if kind == "header":
            return dict(value)
    raise ArchiveError(f"{path}: protobuf header not found")


def frame_records(path: Path | str, schema: dict, layout: dict) -> Iterator[dict]:
    fmt, names = field_names_and_format(layout["pools"])
    expected = struct.calcsize(fmt)
    qids = quantity_ids(schema["quantities"])
    pools = schema["pools"]
    for kind, value in iter_messages(path):
        if kind != "frame":
            continue
        _step, _wall_ns, payload = value
        if len(payload) != expected:
            raise ArchiveError(f"{path}: frame payload size {len(payload)} != layout {expected}")
        flat = dict(zip(names, struct.unpack(fmt, payload), strict=True))
        yield _to_record(flat, pools["constraints"], pools["monitors"], pools["quantities"], pools["triggers"], qids)


def _to_record(flat: dict, n_c: int, n_m: int, n_q: int, n_t: int, qids: list[str]) -> dict:
    record = {
        key: flat[key]
        for key in (
            "t",
            "step",
            "fsm_state",
            "active_motion",
            "last_event",
            "state_since_t",
            "state_since_wall_ns",
            "event_t",
            "event_wall_ns",
        )
    }
    record["timing"] = {key: flat[key] for key in ("wall_ns", "period_ns", "compute_ns")}
    record["constraints"] = [{name: flat[f"c{idx}.{name}"] for name, _ in CSLOT} for idx in range(n_c)]
    record["monitors"] = [{name: flat[f"m{idx}.{name}"] for name, _ in MSLOT} for idx in range(n_m)]
    record["quantities"] = {qids[idx]: flat[f"q{idx}"] for idx in range(n_q)}
    start = max(0, flat["trigger_count"] - n_t)
    record["triggers"] = [
        {name: flat[f"tr{idx % n_t}.{name}"] for name, _ in TSLOT}
        for idx in range(start, flat["trigger_count"])
    ]
    return record
