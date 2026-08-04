# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Length-delimited protobuf frame-log helpers.

The wire format is standard protobuf; protoc owns the C++ side. Here the message
classes are built at runtime from the run's schema (no protoc, no generated _pb2)
so replay stays a pure-Python, dependency-light reader."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

from motion_spec.introspection.archive import ArchiveError
from motion_spec.generation.artifacts import build_frame_log_proto_fields

PROTO_PACKAGE = "motion_spec.introspection.log"
POSE_NAMES = ("px", "py", "pz", "qx", "qy", "qz", "qw")
TWIST_NAMES = ("lx", "ly", "lz", "ax", "ay", "az")
WRENCH_NAMES = ("fx", "fy", "fz", "tx", "ty", "tz")
_SLOT_MESSAGE = {
    "constraints": "ConstraintSlot",
    "monitors": "MonitorSlot",
    "triggers": "Trigger",
    "poses": "PoseSlot",
    "twists": "TwistSlot",
    "wrenches": "WrenchSlot",
}
_CONSTRAINT_KEYS = ("active", "error", "output", "satisfied", "sat_t", "measured", "setpoint")
_MONITOR_KEYS = ("active", "value", "satisfied", "sat_t")
_TRIGGER_KEYS = ("kind", "idx", "fsm_state", "t", "wall_ns")
# Header-only schema: enough to decode the FrameLogHeader without a model's field map.
_HEADER_SCHEMA = {
    "protobuf": {"fields": {cat: [] for cat in ("constraints", "monitors", "quantities", "triggers", "poses", "twists", "wrenches")}},
    "pools": {},
    "quantities": [],
}
_CLASS_CACHE: dict = {}


def _proto_fields(schema: dict) -> dict:
    protobuf = schema.get("protobuf")
    fields = protobuf.get("fields") if protobuf else None
    return fields if fields is not None else build_frame_log_proto_fields(schema)["fields"]


def _build_file_descriptor(fields: dict) -> descriptor_pb2.FileDescriptorProto:
    """FileDescriptorProto mirroring the generated frame_log.proto, from the schema field map."""
    D = descriptor_pb2.FieldDescriptorProto
    fdp = descriptor_pb2.FileDescriptorProto(name="frame_log.proto", package=PROTO_PACKAGE, syntax="proto3")

    def message(name: str, entries: list) -> None:
        m = fdp.message_type.add(name=name)
        for fname, ftype, number in entries:
            m.field.add(name=fname, number=number, label=D.LABEL_OPTIONAL, type=ftype)

    message("FrameLogHeader", [("schema_hash", D.TYPE_STRING, 1), ("producer_agent_id", D.TYPE_STRING, 2), ("activity_id", D.TYPE_STRING, 3)])
    message("ConstraintSlot", [("active", D.TYPE_SFIXED64, 1), ("error", D.TYPE_DOUBLE, 2), ("output", D.TYPE_DOUBLE, 3), ("satisfied", D.TYPE_SFIXED64, 4), ("sat_t", D.TYPE_DOUBLE, 5), ("measured", D.TYPE_DOUBLE, 6), ("setpoint", D.TYPE_DOUBLE, 7)])
    message("MonitorSlot", [("active", D.TYPE_SFIXED64, 1), ("value", D.TYPE_DOUBLE, 2), ("satisfied", D.TYPE_SFIXED64, 3), ("sat_t", D.TYPE_DOUBLE, 4)])
    message("Trigger", [("kind", D.TYPE_SFIXED64, 1), ("idx", D.TYPE_SFIXED64, 2), ("fsm_state", D.TYPE_SFIXED64, 3), ("t", D.TYPE_DOUBLE, 4), ("wall_ns", D.TYPE_SFIXED64, 5)])
    message("PoseSlot", [(n, D.TYPE_DOUBLE, i) for i, n in enumerate(POSE_NAMES, 1)])
    message("TwistSlot", [(n, D.TYPE_DOUBLE, i) for i, n in enumerate(TWIST_NAMES, 1)])
    message("WrenchSlot", [(n, D.TYPE_DOUBLE, i) for i, n in enumerate(WRENCH_NAMES, 1)])

    rf = fdp.message_type.add(name="RuntimeFrame")
    core = [("t", D.TYPE_DOUBLE, 1), ("step", D.TYPE_UINT64, 2), ("fsm_state", D.TYPE_SFIXED64, 3), ("active_motion", D.TYPE_SFIXED64, 4), ("last_event", D.TYPE_SFIXED64, 5), ("state_since_t", D.TYPE_DOUBLE, 6), ("state_since_wall_ns", D.TYPE_SFIXED64, 7), ("event_t", D.TYPE_DOUBLE, 8), ("event_wall_ns", D.TYPE_SFIXED64, 9), ("wall_ns", D.TYPE_SFIXED64, 10), ("period_ns", D.TYPE_SFIXED64, 11), ("compute_ns", D.TYPE_SFIXED64, 12), ("trigger_count", D.TYPE_SFIXED64, 17)]
    for fname, ftype, number in core:
        rf.field.add(name=fname, number=number, label=D.LABEL_OPTIONAL, type=ftype)
    for category, entries in fields.items():
        for entry in entries:
            if category == "quantities":
                rf.field.add(
                    name=entry["name"], number=entry["number"], label=D.LABEL_OPTIONAL,
                    type=D.TYPE_BOOL if entry.get("proto_type") == "bool" else D.TYPE_DOUBLE,
                )
            else:
                rf.field.add(
                    name=entry["name"], number=entry["number"], label=D.LABEL_OPTIONAL,
                    type=D.TYPE_MESSAGE, type_name=f".{PROTO_PACKAGE}.{_SLOT_MESSAGE[category]}",
                )

    rec = fdp.message_type.add(name="FrameLogRecord")
    rec.oneof_decl.add(name="record")
    rec.field.add(name="header", number=1, label=D.LABEL_OPTIONAL, type=D.TYPE_MESSAGE, type_name=f".{PROTO_PACKAGE}.FrameLogHeader", oneof_index=0)
    rec.field.add(name="frame", number=2, label=D.LABEL_OPTIONAL, type=D.TYPE_MESSAGE, type_name=f".{PROTO_PACKAGE}.RuntimeFrame", oneof_index=0)
    return fdp


def _record_class(schema: dict):
    """(FrameLogRecord class, field map) for a schema, built once per schema and cached."""
    key = schema.get("schema_hash") or id(schema)
    cached = _CLASS_CACHE.get(key)
    if cached is not None:
        return cached
    fields = _proto_fields(schema)
    pool = descriptor_pool.DescriptorPool()
    pool.Add(_build_file_descriptor(fields))
    record_cls = message_factory.GetMessageClass(pool.FindMessageTypeByName(f"{PROTO_PACKAGE}.FrameLogRecord"))
    _CLASS_CACHE[key] = (record_cls, fields)
    return _CLASS_CACHE[key]


# --- length-delimited framing (a varint size prefix per protobuf record) ---
def _varint(value: int) -> bytes:
    out = bytearray()
    while value >= 0x80:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def write_delimited(fh, message: bytes) -> None:
    fh.write(_varint(len(message)))
    fh.write(message)


def _read_delimited(fh) -> bytes | None:
    prefix = bytearray()
    while True:
        byte = fh.read(1)
        if not byte:
            if not prefix:
                return None
            raise ArchiveError("truncated protobuf frame log length")
        prefix.extend(byte)
        if not byte[0] & 0x80:
            break
    size = 0
    for shift, b in enumerate(prefix):
        size |= (b & 0x7F) << (7 * shift)
    data = fh.read(size)
    if len(data) != size:
        raise ArchiveError("truncated protobuf frame log")
    return data


# --- encode (fixtures/tests) ---
def header_record(schema: dict) -> bytes:
    record_cls, _ = _record_class(schema)
    meta = schema.get("runtime_provenance", {})
    rec = record_cls()
    rec.header.schema_hash = schema["schema_hash"]
    rec.header.producer_agent_id = meta.get("producer_agent_id", "")
    rec.header.activity_id = meta.get("activity_id", "")
    return rec.SerializeToString()


def frame_record(flat: dict, schema: dict) -> bytes:
    record_cls, fields = _record_class(schema)
    rec = record_cls()
    m = rec.frame
    m.SetInParent()
    m.t = flat["t"]
    m.step = flat["step"]
    m.fsm_state = flat["fsm_state"]
    m.active_motion = flat["active_motion"]
    m.last_event = flat["last_event"]
    m.state_since_t = flat["state_since_t"]
    m.state_since_wall_ns = flat["state_since_wall_ns"]
    m.event_t = flat["event_t"]
    m.event_wall_ns = flat["event_wall_ns"]
    m.wall_ns = flat["wall_ns"]
    m.period_ns = flat["period_ns"]
    m.compute_ns = flat["compute_ns"]
    m.trigger_count = flat["trigger_count"]
    for e in fields["constraints"]:
        s, i = getattr(m, e["name"]), e["index"]
        s.active, s.error, s.output = flat[f"c{i}.active"], flat[f"c{i}.error"], flat[f"c{i}.output"]
        s.satisfied, s.sat_t, s.measured, s.setpoint = flat[f"c{i}.satisfied"], flat[f"c{i}.sat_t"], flat[f"c{i}.measured"], flat[f"c{i}.setpoint"]
    for e in fields["monitors"]:
        s, i = getattr(m, e["name"]), e["index"]
        s.active, s.value, s.satisfied, s.sat_t = flat[f"m{i}.active"], flat[f"m{i}.value"], flat[f"m{i}.satisfied"], flat[f"m{i}.sat_t"]
    for e in fields["quantities"]:
        value = flat[f"q{e['index']}"]
        setattr(m, e["name"], value != 0 if e.get("proto_type") == "bool" else value)
    for e in fields["triggers"]:
        s, i = getattr(m, e["name"]), e["index"]
        s.kind, s.idx, s.fsm_state, s.t, s.wall_ns = flat[f"tr{i}.kind"], flat[f"tr{i}.idx"], flat[f"tr{i}.fsm_state"], flat[f"tr{i}.t"], flat[f"tr{i}.wall_ns"]
    for prefix, names, category in (("pose", POSE_NAMES, "poses"), ("twist", TWIST_NAMES, "twists"), ("wrench", WRENCH_NAMES, "wrenches")):
        for e in fields[category]:
            s, i = getattr(m, e["name"]), e["index"]
            for name in names:
                setattr(s, name, flat[f"{prefix}{i}.{name}"])
    return rec.SerializeToString()


# --- decode ---
_GATE_CACHE: dict = {}
_COUNT_CACHE: dict = {}


def _slot_gate(schema: dict) -> dict:
    """Per category, motion index to the slot indices that motion writes (absent when none gated).

    Proto3 elides a genuine 0.0 exactly as it elides "never written", so absence on the wire
    cannot tell an inactive slot from a zero one. The frame carries active_motion and the schema
    says which slots that motion writes; activity is resolved from that, never from a missing
    field. Keyed on the motion rather than the coordinator's state, so the same decoder reads a
    log produced under an FSM, a behaviour tree or the plain app_main loop.
    """
    key = schema.get("schema_hash")
    if key in _GATE_CACHE:
        return _GATE_CACHE[key]
    by_motion = schema.get("by_motion") or {}
    spatial = schema.get("spatial") or {}
    gate = {}
    for category, rows in (("quantities", schema.get("quantities", ())), *spatial.items()):
        gated = {index for entry in by_motion.values() for index in entry.get(category, ())}
        if not gated:
            continue
        ungated = {row.get("index", 0) for row in rows} - gated
        gate[category] = {
            entry["index"]: ungated | set(entry.get(category, ()))
            for entry in by_motion.values()
        }
    _GATE_CACHE[key] = gate
    return gate


def _slot_counts(schema: dict) -> dict:
    """Motion index -> how many constraint/monitor slots that motion drives."""
    key = schema.get("schema_hash")
    cached = _COUNT_CACHE.get(key)
    if cached is None:
        cached = _COUNT_CACHE[key] = {
            entry["index"]: {
                "controllers": len(entry.get("controllers", ())),
                "monitors": len(entry.get("monitors", ())),
            }
            for entry in (schema.get("by_motion") or {}).values()
        }
    return cached


def _parse_frame(msg, schema: dict) -> dict:
    fields = _proto_fields(schema)
    pools = schema["pools"]
    record = {
        "t": msg.t,
        "step": msg.step,
        "fsm_state": msg.fsm_state,
        "active_motion": msg.active_motion,
        "last_event": msg.last_event,
        "state_since_t": msg.state_since_t,
        "state_since_wall_ns": msg.state_since_wall_ns,
        "event_t": msg.event_t,
        "event_wall_ns": msg.event_wall_ns,
        "timing": {"wall_ns": msg.wall_ns, "period_ns": msg.period_ns, "compute_ns": msg.compute_ns},
    }
    # `active` is derived, not carried: schema["by_motion"] already says how many constraint and
    # monitor slots the active motion drives, so writing a constant 1 per slot per tick would only
    # restate it. Slots beyond that count belong to some other motion and were not written.
    counts = _slot_counts(schema).get(msg.active_motion, {})
    record["constraints"] = [
        {**{k: getattr(getattr(msg, e["name"]), k) for k in _CONSTRAINT_KEYS},
         "active": 1 if e["index"] < counts.get("controllers", 0) else 0}
        for e in fields["constraints"]
    ]
    record["monitors"] = [
        {**{k: getattr(getattr(msg, e["name"]), k) for k in _MONITOR_KEYS},
         "active": 1 if e["index"] < counts.get("monitors", 0) else 0}
        for e in fields["monitors"]
    ]
    # Flags come back as bool; keep the decoded record numeric so readers see one value type.
    quantities = [float(getattr(msg, e["name"])) for e in fields["quantities"]]
    triggers = [{k: getattr(getattr(msg, e["name"]), k) for k in _TRIGGER_KEYS} for e in fields["triggers"]]
    qids = [q["id"] for q in sorted(schema["quantities"], key=lambda q: q.get("index", 0))]
    gate = _slot_gate(schema)

    def written(category: str, index: int) -> bool:
        by_index = gate.get(category)
        return by_index is None or index in by_index.get(msg.active_motion, ())

    record["quantities"] = {
        qids[idx]: quantities[idx]
        for idx in range(min(len(qids), len(quantities)))
        if written("quantities", idx)
    }
    trigger_count = msg.trigger_count
    start = max(0, trigger_count - pools["triggers"])
    record["triggers"] = (
        [triggers[idx % pools["triggers"]] for idx in range(start, trigger_count)]
        if triggers and pools["triggers"]
        else []
    )
    # A slot the active state does not write stays a hole in the list rather than a decoded zero;
    # the list stays positional so a slot keeps one index for the whole run.
    for names, category in ((POSE_NAMES, "poses"), (TWIST_NAMES, "twists"), (WRENCH_NAMES, "wrenches")):
        record[category] = [
            {n: getattr(getattr(msg, e["name"]), n) for n in names}
            if written(category, e["index"])
            else None
            for e in fields[category]
        ]
    return record


def iter_messages(path: Path | str, schema: dict | None = None) -> Iterator[tuple[str, object]]:
    record_cls, _ = _record_class(schema if schema is not None else _HEADER_SCHEMA)
    with Path(path).open("rb") as fh:
        while True:
            data = _read_delimited(fh)
            if data is None:
                return
            rec = record_cls()
            rec.ParseFromString(data)
            which = rec.WhichOneof("record")
            if which == "header":
                yield "header", {
                    "schema_hash": rec.header.schema_hash,
                    "producer_agent_id": rec.header.producer_agent_id,
                    "activity_id": rec.header.activity_id,
                }
            elif which == "frame" and schema is not None:
                yield "frame", _parse_frame(rec.frame, schema)


def read_header(path: Path | str) -> dict:
    for kind, value in iter_messages(path):
        if kind == "header":
            return dict(value)
    raise ArchiveError(f"{path}: protobuf header not found")


def frame_records(path: Path | str, schema: dict) -> Iterator[dict]:
    for kind, value in iter_messages(path, schema):
        if kind == "frame":
            yield value
