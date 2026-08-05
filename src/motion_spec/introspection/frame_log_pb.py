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

    message("SlotIri", [("number", D.TYPE_UINT32, 1), ("id", D.TYPE_STRING, 2), ("iri", D.TYPE_STRING, 3),
                        ("constraint_iri", D.TYPE_STRING, 4), ("event_iri", D.TYPE_STRING, 5)])
    transition = fdp.message_type.add(name="Transition")
    for fname, ftype, number in (
        ("index", D.TYPE_UINT32, 1), ("id", D.TYPE_STRING, 2), ("iri", D.TYPE_STRING, 3),
        ("from_state", D.TYPE_INT32, 4), ("to_state", D.TYPE_INT32, 5), ("event_index", D.TYPE_INT32, 6),
    ):
        transition.field.add(name=fname, number=number, label=D.LABEL_OPTIONAL, type=ftype)
    transition.field.add(name="event_indices", number=7, label=D.LABEL_REPEATED, type=D.TYPE_UINT32)
    message("Constant", [("id", D.TYPE_STRING, 1), ("source_id", D.TYPE_STRING, 2), ("value", D.TYPE_DOUBLE, 3)])

    gate = fdp.message_type.add(name="MotionGate")
    for fname, ftype, number in (
        ("index", D.TYPE_UINT32, 1), ("id", D.TYPE_STRING, 2), ("iri", D.TYPE_STRING, 3),
        ("fsm_state", D.TYPE_INT32, 4),
    ):
        gate.field.add(name=fname, number=number, label=D.LABEL_OPTIONAL, type=ftype)
    for fname, number in (("quantities", 5), ("poses", 6), ("twists", 7), ("wrenches", 8)):
        gate.field.add(name=fname, number=number, label=D.LABEL_REPEATED, type=D.TYPE_UINT32)
    for fname, number in (("controllers", 9), ("monitors", 10)):
        gate.field.add(name=fname, number=number, label=D.LABEL_REPEATED,
                       type=D.TYPE_MESSAGE, type_name=f".{PROTO_PACKAGE}.SlotIri")

    hdr = fdp.message_type.add(name="FrameLogHeader")
    for fname, ftype, number in (
        ("schema_hash", D.TYPE_STRING, 1), ("producer_agent_id", D.TYPE_STRING, 2),
        ("activity_id", D.TYPE_STRING, 3), ("descriptor_set", D.TYPE_BYTES, 4),
        ("trigger_pool", D.TYPE_UINT32, 7), ("runtime_agent_id", D.TYPE_STRING, 12),
        ("platform_name", D.TYPE_STRING, 13), ("simulated", D.TYPE_BOOL, 14),
        ("end_state", D.TYPE_INT32, 15), ("nominal_period_ns", D.TYPE_INT64, 16), ("fsm_namespace", D.TYPE_STRING, 17),
    ):
        hdr.field.add(name=fname, number=number, label=D.LABEL_OPTIONAL, type=ftype)
    for fname, number, type_name in (
        ("slots", 5, "SlotIri"), ("motions", 6, "MotionGate"), ("fsm_states", 8, "SlotIri"),
        ("fsm_events", 9, "SlotIri"), ("fsm_transitions", 10, "Transition"),
        ("constants", 11, "Constant"),
    ):
        hdr.field.add(name=fname, number=number, label=D.LABEL_REPEATED,
                      type=D.TYPE_MESSAGE, type_name=f".{PROTO_PACKAGE}.{type_name}")
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
# Core RuntimeFrame fields carry the tick itself, not a slot; everything else is a slot whose
# category follows from its wire type, so the descriptor alone says what the frame contains.
_CORE_FIELDS = frozenset(
    (
        "t", "step", "fsm_state", "active_motion", "last_event", "state_since_t",
        "state_since_wall_ns", "event_t", "event_wall_ns", "wall_ns", "period_ns",
        "compute_ns", "trigger_count",
    )
)
_CATEGORY_BY_MESSAGE = {
    "ConstraintSlot": "constraints",
    "MonitorSlot": "monitors",
    "Trigger": "triggers",
    "PoseSlot": "poses",
    "TwistSlot": "twists",
    "WrenchSlot": "wrenches",
}
_SPATIAL = (("poses", POSE_NAMES), ("twists", TWIST_NAMES), ("wrenches", WRENCH_NAMES))
_BOOTSTRAP_FIELDS = {category: [] for category in (*_SLOT_MESSAGE, "quantities")}


class LogContract:
    """Everything needed to decode a frame log, read from the log's own header record.

    The header carries the message descriptor, each slot's id and IRI, and the per-motion gate --
    the three things the wire cannot say about itself. Nothing here comes from a companion file.
    """

    def __init__(self, header, record_cls, fields):
        self.header = header
        self.record_cls = record_cls
        self.fields = fields
        self.trigger_pool = header.trigger_pool
        self.gate = _slot_gate(header, fields)
        self.counts = {
            motion.index: {
                "controllers": len(motion.controllers),
                "monitors": len(motion.monitors),
            }
            for motion in header.motions
        }
        self.quantity_ids = [entry["id"] for entry in fields["quantities"]]
        self.iri_by_id = {
            entry["id"]: entry["iri"]
            for category in ("quantities", *(name for name, _ in _SPATIAL))
            for entry in fields[category]
            if entry["iri"]
        }

    def summary(self) -> dict:
        """The run facts the archive and provenance record, read off the log's own header."""
        return {
            "schema_hash": self.header.schema_hash,
            "runtime_provenance": {
                "activity_id": self.header.activity_id,
                "producer_agent_id": self.header.producer_agent_id,
                "runtime_agent_id": self.header.runtime_agent_id,
            },
            "platform": {"name": self.header.platform_name, "simulated": self.header.simulated},
        }


def _fields_from_descriptor(frame_descriptor, header) -> dict:
    """Per-category slot list, derived from the embedded descriptor and the header's slot table."""
    slot_by_number = {slot.number: slot for slot in header.slots}
    fields: dict[str, list] = {category: [] for category in (*_SLOT_MESSAGE, "quantities")}
    for field in sorted(frame_descriptor.fields, key=lambda f: f.number):
        if field.name in _CORE_FIELDS:
            continue
        category = (
            _CATEGORY_BY_MESSAGE[field.message_type.name]
            if field.message_type is not None
            else "quantities"
        )
        slot = slot_by_number.get(field.number)
        fields[category].append(
            {
                "index": len(fields[category]),
                "id": slot.id if slot is not None else field.name,
                "iri": slot.iri if slot is not None else "",
                "name": field.name,
                "number": field.number,
                "proto_type": "bool" if field.type == field.TYPE_BOOL else "double",
            }
        )
    return fields


def _slot_gate(header, fields: dict) -> dict:
    """Per category, motion index to the slot indices that motion writes (absent when none gated).

    Proto3 elides a genuine 0.0 exactly as it elides "never written", so absence on the wire
    cannot tell an inactive slot from a zero one. The frame carries active_motion and the header
    says which slots that motion writes; activity is resolved from that, never from a missing
    field. Keyed on the motion rather than the coordinator's state, so the same decoder reads a
    log produced under an FSM, a behaviour tree or the plain app_main loop.
    """
    gate = {}
    for category in ("quantities", *(name for name, _ in _SPATIAL)):
        claimed = {
            index for motion in header.motions for index in getattr(motion, category)
        }
        if not claimed:
            continue
        # A slot no motion claims is written unconditionally, so it stays visible everywhere.
        unclaimed = {entry["index"] for entry in fields[category]} - claimed
        gate[category] = {
            motion.index: unclaimed | set(getattr(motion, category))
            for motion in header.motions
        }
    return gate


def _bootstrap_class():
    """FrameLogRecord class carrying only the header messages, to read record 1 of any log."""
    cached = _CLASS_CACHE.get("__bootstrap__")
    if cached is None:
        pool = descriptor_pool.DescriptorPool()
        pool.Add(_build_file_descriptor(_BOOTSTRAP_FIELDS))
        cached = _CLASS_CACHE["__bootstrap__"] = message_factory.GetMessageClass(
            pool.FindMessageTypeByName(f"{PROTO_PACKAGE}.FrameLogRecord")
        )
    return cached


def read_contract(path: Path | str) -> LogContract:
    """Read a log's header record and build its decode contract from that alone."""
    with Path(path).open("rb") as fh:
        data = _read_delimited(fh)
    if data is None:
        raise ArchiveError(f"{path}: empty frame log")
    record = _bootstrap_class()()
    record.ParseFromString(data)
    if record.WhichOneof("record") != "header":
        raise ArchiveError(f"{path}: first record is not a frame-log header")
    header = record.header
    if not header.descriptor_set:
        raise ArchiveError(
            f"{path}: header carries no descriptor set -- written by a pre-v3 runtime"
        )
    descriptor_set = descriptor_pb2.FileDescriptorSet()
    descriptor_set.ParseFromString(header.descriptor_set)
    pool = descriptor_pool.DescriptorPool()
    for file_proto in descriptor_set.file:
        pool.Add(file_proto)
    record_cls = message_factory.GetMessageClass(
        pool.FindMessageTypeByName(f"{PROTO_PACKAGE}.FrameLogRecord")
    )
    frame_descriptor = pool.FindMessageTypeByName(f"{PROTO_PACKAGE}.RuntimeFrame")
    return LogContract(header, record_cls, _fields_from_descriptor(frame_descriptor, header))


def _parse_frame(msg, contract: LogContract) -> dict:
    fields = contract.fields
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
    # `active` is derived, not carried: the header already says how many constraint and monitor
    # slots the active motion drives, so writing a constant 1 per slot per tick would only
    # restate it. Slots beyond that count belong to some other motion and were not written.
    counts = contract.counts.get(msg.active_motion, {})
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
    qids = contract.quantity_ids

    def written(category: str, index: int) -> bool:
        by_index = contract.gate.get(category)
        return by_index is None or index in by_index.get(msg.active_motion, ())

    record["quantities"] = {
        qids[idx]: quantities[idx]
        for idx in range(min(len(qids), len(quantities)))
        if written("quantities", idx)
    }
    trigger_count = msg.trigger_count
    pool_size = contract.trigger_pool
    start = max(0, trigger_count - pool_size)
    record["triggers"] = (
        [triggers[idx % pool_size] for idx in range(start, trigger_count)]
        if triggers and pool_size
        else []
    )
    # A slot the active state does not write stays a hole in the list rather than a decoded zero;
    # the list stays positional so a slot keeps one index for the whole run.
    for category, names in _SPATIAL:
        record[category] = [
            {n: getattr(getattr(msg, e["name"]), n) for n in names}
            if written(category, e["index"])
            else None
            for e in fields[category]
        ]
    return record


def iter_messages(path: Path | str, contract: LogContract | None = None) -> Iterator[tuple[str, object]]:
    """Yield ('header', dict) then ('frame', decoded) for each record in a log."""
    if contract is None:
        contract = read_contract(path)
    with Path(path).open("rb") as fh:
        while True:
            data = _read_delimited(fh)
            if data is None:
                return
            rec = contract.record_cls()
            rec.ParseFromString(data)
            which = rec.WhichOneof("record")
            if which == "header":
                yield "header", {
                    "schema_hash": rec.header.schema_hash,
                    "producer_agent_id": rec.header.producer_agent_id,
                    "activity_id": rec.header.activity_id,
                }
            elif which == "frame":
                yield "frame", _parse_frame(rec.frame, contract)


def read_header(path: Path | str) -> dict:
    """The log's identity record: schema hash and the agents that produced it."""
    contract = read_contract(path)
    return {
        "schema_hash": contract.header.schema_hash,
        "producer_agent_id": contract.header.producer_agent_id,
        "activity_id": contract.header.activity_id,
    }


def frame_records(path: Path | str, contract: LogContract | None = None) -> Iterator[dict]:
    """Decoded frames, in order. Needs no companion file -- the log describes itself."""
    if contract is None:
        contract = read_contract(path)
    for kind, value in iter_messages(path, contract):
        if kind == "frame":
            yield value
