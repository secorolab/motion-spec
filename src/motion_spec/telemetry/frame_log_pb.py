# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Length-delimited protobuf frame-log helpers.

frame_log.proto beside this module is the one schema: protoc compiles it to C++ at generation,
and a log embeds it, so a reader builds its classes from the log alone."""

from __future__ import annotations

import io
import subprocess
import tempfile
from collections.abc import Iterator
from pathlib import Path

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

from motion_spec.runs.archive import ArchiveError

PROTO_PACKAGE = "motion_spec.telemetry.log"

# A run writes its log uncompressed -- the writer is on the control loop's heels and the
# dashboard tails the file while it grows -- and archiving compresses it once, afterwards.
# So a log on disk is one of two files, and everything that reads one has to accept either.
LOG_SUFFIX = ".zst"


def shm_name_for(schema_hash: str, run_id: str | None = None) -> str:
    """The frame block the generated runtime publishes under, absent a MOTION_SPEC_SHM_NAME override.

    Named per run when the run is known, so two runs of one generation never share a block.
    """
    return f"/motion_spec_{schema_hash[:16]}" + (f"_{run_id}" if run_id else "")


def ctrl_shm_name(schema_hash: str, run_id: str | None = None) -> str:
    """The control block of a simulated run, absent a MOTION_SPEC_CTRL_SHM_NAME override."""
    return f"/motion_spec_ctrl_{schema_hash[:16]}" + (f"_{run_id}" if run_id else "")


def log_path(path: Path | str) -> Path:
    """The frame log that is actually there, compressed or not, given either name."""
    path = Path(path)
    if path.suffix == LOG_SUFFIX:
        return path if path.exists() or not path.with_suffix("").exists() else path.with_suffix("")
    packed = path.with_name(path.name + LOG_SUFFIX)
    return packed if packed.exists() and not path.exists() else path


def open_log(path: Path | str):
    """Open a frame log for reading, unpacking it first if it was archived compressed.

    Into memory rather than through a streaming decompressor: the readers seek -- to the end
    to see whether the writer stopped mid-record, and by offset to walk records -- and a
    zstd stream only goes forwards. A decoded log costs more than this anyway.
    """
    path = log_path(path)
    if path.suffix != LOG_SUFFIX:
        return path.open("rb")
    import zstandard

    with path.open("rb") as packed:
        return io.BytesIO(zstandard.ZstdDecompressor().stream_reader(packed).read())


POSE_NAMES = ("px", "py", "pz", "qx", "qy", "qz", "qw")
TWIST_NAMES = ("lx", "ly", "lz", "ax", "ay", "az")
WRENCH_NAMES = ("fx", "fy", "fz", "tx", "ty", "tz")
PROTO = Path(__file__).with_name("frame_log.proto")
FORMAT_VERSION = 2
_HEADER_SLOTS = ("quantities", "poses", "twists", "wrenches", "devices")
_CONSTRAINT_KEYS = ("active", "error", "output", "satisfied", "sat_t", "measured", "setpoint")
_MONITOR_KEYS = ("active", "value", "satisfied", "sat_t")
_TRIGGER_KEYS = ("kind", "idx", "fsm_state", "t", "wall_ns")
_CLASS_CACHE: dict = {}


def slot(frame, category: str, index: int):
    """Slot `index` of a frame's category; None where the frame carries none."""
    items = getattr(frame, category)
    return items[index] if index < len(items) else None


def quantities(frame, size: int) -> list[float]:
    """Every quantity slot's value; one the frame leaves out is zero."""
    values = [0.0] * size
    for index, value in zip(frame.quantity_slots, frame.quantities):
        values[index] = value
    return values


def run_protoc(*arguments: str) -> None:
    """Run protoc over the shipped frame_log.proto."""
    try:
        subprocess.run(
            ["protoc", f"--proto_path={PROTO.parent}", *arguments, PROTO.name],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "protoc was not found. Install the protobuf compiler (apt: protobuf-compiler) "
            "to generate the frame-log codec."
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(exc.stderr.strip() or exc.stdout.strip()) from exc


def descriptor_set() -> bytes:
    """frame_log.proto compiled by protoc to a FileDescriptorSet, built once."""
    if "descriptor_set" not in _CLASS_CACHE:
        with tempfile.TemporaryDirectory() as scratch:
            out = Path(scratch) / "frame_log.desc"
            run_protoc(f"--descriptor_set_out={out}")
            _CLASS_CACHE["descriptor_set"] = out.read_bytes()
    return _CLASS_CACHE["descriptor_set"]


def _record_class_of(serialized_set: bytes):
    pool = descriptor_pool.DescriptorPool()
    for file_proto in descriptor_pb2.FileDescriptorSet.FromString(serialized_set).file:
        pool.Add(file_proto)
    return message_factory.GetMessageClass(
        pool.FindMessageTypeByName(f"{PROTO_PACKAGE}.FrameLogRecord")
    )


def record_class():
    """The FrameLogRecord class of the shipped frame_log.proto, built once."""
    if "record" not in _CLASS_CACHE:
        _CLASS_CACHE["record"] = _record_class_of(descriptor_set())
    return _CLASS_CACHE["record"]


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


def _varint_at(data: bytes, offset: int) -> tuple[int, int]:
    value = shift = 0
    while True:
        if offset >= len(data) or shift > 63:
            raise ArchiveError("frame log record is not protobuf")
        byte = data[offset]
        value |= (byte & 0x7F) << shift
        offset, shift = offset + 1, shift + 7
        if not byte & 0x80:
            return value, offset


def _length_delimited(data: bytes, number: int) -> bytes | None:
    """Field `number` of a serialized message, read off the wire without its schema."""
    offset = 0
    while offset < len(data):
        key, offset = _varint_at(data, offset)
        wire = key & 7
        if wire == 2:
            size, offset = _varint_at(data, offset)
            if key >> 3 == number:
                return data[offset : offset + size]
            offset += size
        elif wire == 0:
            _, offset = _varint_at(data, offset)
        elif wire in (1, 5):
            offset += 8 if wire == 1 else 4
        else:
            raise ArchiveError("frame log record is not protobuf")
    return None


def _read_delimited_at(fh, offset: int) -> tuple[bytes | None, int]:
    """One message starting at `offset`, plus the offset past it.

    A record still being appended to (short prefix or short payload) reads as (None, offset) --
    the caller keeps its offset and retries once the writer has finished the record.
    """
    fh.seek(offset)
    prefix = bytearray()
    while True:
        byte = fh.read(1)
        if not byte:
            return None, offset
        prefix.extend(byte)
        if not byte[0] & 0x80:
            break
        if len(prefix) > 10:
            raise ArchiveError("frame log length prefix is not a varint")
    size = 0
    for shift, b in enumerate(prefix):
        size |= (b & 0x7F) << (7 * shift)
    data = fh.read(size)
    if len(data) != size:
        return None, offset
    return data, offset + len(prefix) + size


def _read_delimited(fh, *, partial_ok: bool = False) -> bytes | None:
    """One message, or None at the end of the stream.

    A complete file ends on a record boundary. A run killed mid-write leaves a short final
    record: `partial_ok` readers stop there and keep every whole record before it, which is
    all a crashed run has to say. The header is read strictly -- a log whose contract is
    half-written describes nothing.
    """
    start = fh.tell()
    data, end = _read_delimited_at(fh, start)
    if data is None:
        if partial_ok or fh.seek(0, 2) == start:
            return None
        raise ArchiveError("truncated protobuf frame log")
    fh.seek(end)
    return data


def tail_is_partial(path: Path | str) -> bool:
    """Whether the log ends mid-record, i.e. the writer never closed it."""
    with open_log(path) as fh:
        offset = 0
        while True:
            data, next_offset = _read_delimited_at(fh, offset)
            if data is None:
                return fh.seek(0, 2) != offset
            offset = next_offset


# --- encode (fixtures/tests) ---
def frame_record(flat: dict, schema: dict) -> bytes:
    pools = schema["pools"]
    rec = record_class()()
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
    for i in range(pools["constraints"]):
        if flat[f"c{i}.active"]:
            s = m.constraints.add()
            for key in _CONSTRAINT_KEYS[1:]:
                setattr(s, key, flat[f"c{i}.{key}"])
    for i in range(pools["monitors"]):
        if flat[f"m{i}.active"]:
            s = m.monitors.add()
            for key in _MONITOR_KEYS[1:]:
                setattr(s, key, flat[f"m{i}.{key}"])
    for i in range(pools["quantities"]):
        if flat[f"q{i}"]:
            m.quantity_slots.append(i)
            m.quantities.append(flat[f"q{i}"])
    for i in range(pools.get("devices", 0)):
        s = m.devices.add()
        s.seq = flat.get(f"device{i}.seq", 0)
        s.success = bool(flat.get(f"device{i}.success", 0))
    for i in range(min(flat["trigger_count"], pools["triggers"])):
        s = m.triggers.add()
        for key in _TRIGGER_KEYS:
            setattr(s, key, flat[f"tr{i}.{key}"])
    for prefix, names, category in (
        ("pose", POSE_NAMES, "poses"),
        ("twist", TWIST_NAMES, "twists"),
        ("wrench", WRENCH_NAMES, "wrenches"),
    ):
        for i in range(pools.get(category, 0)):
            s = getattr(m, category).add()
            if not flat.get(f"{prefix}{i}.active", 1):
                continue
            for name in names:
                setattr(s, name, flat[f"{prefix}{i}.{name}"])
    return rec.SerializeToString()


# --- decode ---
_SPATIAL = (("poses", POSE_NAMES), ("twists", TWIST_NAMES), ("wrenches", WRENCH_NAMES))


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
            motion.index: {"controllers": len(motion.controllers), "monitors": len(motion.monitors)}
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
            "platform": {"name": self.header.platform_name, "simulated": self.header.simulated},
        }


def _fields_from_header(header) -> dict:
    """Per-category slot list: pooled slots by index, model slots as the header names them."""
    pools = {
        "constraints": max((len(motion.controllers) for motion in header.motions), default=0),
        "monitors": max((len(motion.monitors) for motion in header.motions), default=0),
        "triggers": header.trigger_pool,
    }
    fields = {
        category: [{"index": i, "id": f"{category[:-1]}_{i}", "iri": ""} for i in range(size)]
        for category, size in pools.items()
    }
    for category in _HEADER_SLOTS:
        fields[category] = [
            {"index": slot.number, "id": slot.id, "iri": slot.iri}
            for slot in getattr(header, category)
        ]
    return fields


def _slot_gate(header, fields: dict) -> dict:
    """Per category, motion index to the slot indices that motion writes (absent when none gated).

    Proto3 elides a genuine 0.0 exactly as it elides "never written", so absence on the wire
    cannot tell an inactive slot from a zero one. The frame carries active_motion and the header
    says which slots that motion writes; activity is resolved from that, never from a missing
    field. Keyed on the motion rather than the coordinator's state, so the same decoder reads a
    log produced under an FSM, a behaviour tree or a plain sequencer.
    """
    gate = {}
    for category in ("quantities", *(name for name, _ in _SPATIAL)):
        claimed = {index for motion in header.motions for index in getattr(motion, category)}
        if not claimed:
            continue
        # A slot no motion claims is written unconditionally, so it stays visible everywhere.
        unclaimed = {entry["index"] for entry in fields[category]} - claimed
        gate[category] = {
            motion.index: unclaimed | set(getattr(motion, category)) for motion in header.motions
        }
    return gate


def read_contract(path: Path | str) -> LogContract:
    """Read a log's header record and build its decode contract from that alone."""
    with open_log(path) as fh:
        data = _read_delimited(fh)
    if data is None:
        raise ArchiveError(f"{path}: empty frame log")
    header = _length_delimited(data, 1)
    embedded = _length_delimited(header, 4) if header is not None else None
    if not embedded:
        raise ArchiveError(f"{path}: first record is not a frame-log header carrying its schema")
    record_cls = _record_class_of(embedded)
    record = record_cls.FromString(data)
    version = getattr(record.header, "format_version", 0)
    if version != FORMAT_VERSION:
        raise ArchiveError(
            f"{path}: frame-log format {version}; this motion-spec reads format {FORMAT_VERSION}"
        )
    return LogContract(record.header, record_cls, _fields_from_header(record.header))


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
        "timing": {
            "wall_ns": msg.wall_ns,
            "period_ns": msg.period_ns,
            "compute_ns": msg.compute_ns,
        },
    }
    # `active` is derived, not carried: the header already says how many constraint and monitor
    # slots the active motion drives. Slots beyond that count belong to some other motion.
    counts = contract.counts.get(msg.active_motion, {})

    def slots(category: str, keys: tuple, count: int) -> list:
        return [
            {
                "active": 1 if i < count else 0,
                **{k: getattr(s, k) if (s := slot(msg, category, i)) else 0 for k in keys[1:]},
            }
            for i in range(len(fields[category]))
        ]

    record["constraints"] = slots("constraints", _CONSTRAINT_KEYS, counts.get("controllers", 0))
    record["monitors"] = slots("monitors", _MONITOR_KEYS, counts.get("monitors", 0))
    qids = contract.quantity_ids
    values = quantities(msg, len(qids))
    triggers = [{k: getattr(entry, k) for k in _TRIGGER_KEYS} for entry in msg.triggers]

    def written(category: str, index: int) -> bool:
        by_index = contract.gate.get(category)
        return by_index is None or index in by_index.get(msg.active_motion, ())

    record["quantities"] = {
        qid: values[idx] for idx, qid in enumerate(qids) if written("quantities", idx)
    }
    record["devices"] = {
        entry["id"]: {"seq": device.seq, "success": bool(device.success)}
        for entry, device in zip(fields["devices"], msg.devices)
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
            {n: getattr(s, n) for n in names}
            if written(category, e["index"]) and (s := slot(msg, category, e["index"]))
            else None
            for e in fields[category]
        ]
    return record


def raw_frames(fh, contract: LogContract, offset: int = 0) -> Iterator[tuple[object, int]]:
    """Each whole RuntimeFrame from OFFSET on, undecoded, with the offset just past it.

    A record still being written ends the walk, so a crashed run's log reads up to its last
    complete frame and a growing one resumes from the last offset yielded.
    """
    while True:
        data, offset = _read_delimited_at(fh, offset)
        if data is None:
            return
        record = contract.record_cls.FromString(data)
        if record.WhichOneof("record") == "frame":
            yield record.frame, offset


def read_sampling(path: Path | str) -> dict | None:
    """The run's draw, written right after the header; None for a run that sampled nothing."""
    contract = read_contract(path)
    with open_log(path) as fh:
        _read_delimited(fh)
        data = _read_delimited(fh, partial_ok=True)
    if data is None:
        return None
    record = contract.record_cls()
    record.ParseFromString(data)
    if record.WhichOneof("record") != "sampling":
        return None
    return {
        "seed": record.sampling.seed,
        "drawn_at_ns": record.sampling.drawn_at_ns,
        "draws": {draw.quantity: list(draw.values) for draw in record.sampling.draws},
    }


def frame_records(path: Path | str, contract: LogContract | None = None) -> Iterator[dict]:
    """Decoded frames, in order. Needs no companion file -- the log describes itself."""
    if contract is None:
        contract = read_contract(path)
    with open_log(path) as fh:
        for frame, _ in raw_frames(fh, contract):
            yield _parse_frame(frame, contract)


def stream_records(
    fh, contract: LogContract, offset: int, stride: int = 1
) -> tuple[list[dict], int]:
    """Frames appended since `offset`, plus the offset to resume from.

    Tailing a log the runtime is still writing: a partial trailing record leaves the offset
    where it was, so the next call re-reads it once the writer has completed it. Header
    records are skipped, exactly as `frame_records` skips them.

    A stride above one is for a reader that wants the shape of a run rather than its every
    tick: it keeps one frame in n by the frame's own step, plus every frame that enters a
    state or fires an event, so no transition falls between two samples. Shaping a frame into
    a dict costs some thirty times what parsing the record costs, so the saving is in what is
    not shaped.
    """
    records = []
    state = event = None
    resume = offset
    for frame, resume in raw_frames(fh, contract, offset):
        moved = frame.fsm_state != state or frame.last_event != event
        state, event = frame.fsm_state, frame.last_event
        if moved or frame.step % stride == 0:
            records.append(_parse_frame(frame, contract))
    return records, resume
