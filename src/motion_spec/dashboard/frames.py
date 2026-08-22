# SPDX-License-Identifier: MPL-2.0
"""Read the generated runtime's latest-frame shared-memory block.

The publisher overwrites one Frame slot per tick behind a seqlock, so this is the cheapest
view of "right now" -- lossy by design. The frame log (see tail.py) stays the lossless record.
"""

from __future__ import annotations

import json
import mmap
import struct
from pathlib import Path

from motion_spec.introspection.frame_log_pb import (
    _CONSTRAINT_KEYS,
    _MONITOR_KEYS,
    _SPATIAL,
    _TRIGGER_KEYS,
)

_CORE_KEYS = (
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
_TIMING_KEYS = ("wall_ns", "period_ns", "compute_ns")
_SPATIAL_PREFIX = {"poses": "pose", "twists": "twist", "wrenches": "wrench"}
# The words a live sampler steers by: where the run is, and what put it there.
CORE_SIGNALS = ("step", "t", "fsm_state", "active_motion", "last_event")
# Timing signals are nanoseconds in the frame and milliseconds on a plot.
_TIMING_MS = {"timing.compute_ms": ("compute_ns", 1e6), "timing.period_ms": ("period_ns", 1e6)}

# How a spatial slot's proto fields read as authored components.
SLOT_PARTS = {
    "poses": {
        "position.x": "px",
        "position.y": "py",
        "position.z": "pz",
        "orientation.x": "qx",
        "orientation.y": "qy",
        "orientation.z": "qz",
        "orientation.w": "qw",
    },
    "twists": {
        "linear.x": "lx",
        "linear.y": "ly",
        "linear.z": "lz",
        "angular.x": "ax",
        "angular.y": "ay",
        "angular.z": "az",
    },
    "wrenches": {
        "force.x": "fx",
        "force.y": "fy",
        "force.z": "fz",
        "torque.x": "tx",
        "torque.y": "ty",
        "torque.z": "tz",
    },
}


def slot_signals(contract) -> dict:
    """Every spatial slot component as signal name -> (kind, field, component)."""
    return {
        f"{field['id']}.{part}": (kind, field, attribute)
        for kind, parts in SLOT_PARTS.items()
        for field in contract.fields.get(kind, ())
        for part, attribute in parts.items()
        if hasattr(contract.record_cls().frame, field["name"])
    }


def shm_name_for(schema_hash: str) -> str:
    """The name the generated runtime publishes under, absent a MOTION_SPEC_SHM_NAME override."""
    return f"/motion_spec_{schema_hash[:16]}"


def shm_path(name: str) -> Path:
    """A POSIX shm name resolves under /dev/shm; anything else is taken as a file path."""
    if name.startswith("/") and "/" not in name[1:]:
        return Path("/dev/shm") / name[1:]
    return Path(name)


class FrameLayout:
    """A generation's frame_layout.json as the struct that decodes its shm Frame."""

    def __init__(self, layout: dict):
        self.layout = layout
        self.fields = layout["fields"]
        self.pools = layout["pools"]
        self.schema_hash = layout["schema_hash"]
        self.platform = layout.get("platform") or {}
        self.names = [field["name"] for field in self.fields]
        self.struct = struct.Struct("<" + "".join(field["fmt"] for field in self.fields))
        if self.struct.size != layout["frame_size_bytes"]:
            raise ValueError(
                f"frame_layout.json is inconsistent: {len(self.fields)} fields pack to "
                f"{self.struct.size} bytes but frame_size_bytes says {layout['frame_size_bytes']}"
            )

    @classmethod
    def load(cls, path: Path | str) -> FrameLayout:
        return cls(json.loads(Path(path).read_text()))

    @classmethod
    def for_generation(cls, generation_dir: Path | str) -> FrameLayout:
        return cls.load(Path(generation_dir) / "generated" / "contract" / "frame_layout.json")

    @property
    def shm_name(self) -> str:
        return shm_name_for(self.schema_hash)


class ShmFrameReader:
    """The latest published frame, read through the writer's seqlock.

    `contract` is the frame log's own header contract. With it the quantities come back under
    their model ids and the motion gate applies, so a shm frame and a logged frame are the same
    dict; without it quantities fall back to their positional layout names.
    """

    def __init__(self, shm_name: str, layout: FrameLayout, contract=None):
        self.name = shm_name
        self.layout = layout
        self.contract = contract
        self._mm: mmap.mmap | None = None

    def open(self) -> bool:
        """True once the block exists and is fully sized; the runtime creates it at startup."""
        if self._mm is not None:
            return True
        path, size = shm_path(self.name), self.layout.struct.size
        if not path.exists() or path.stat().st_size < size:
            return False
        with path.open("rb") as fh:
            self._mm = mmap.mmap(fh.fileno(), size, prot=mmap.PROT_READ)
        return True

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def latest_raw(self, retries: int = 8) -> bytes | None:
        """The newest whole frame as the bytes it was published as, or None while the writer
        keeps tearing it (or has yet to publish one). A torn read is never returned: seq
        brackets every publish."""
        if not self.open():
            return None
        mm, size = self._mm, self.layout.struct.size
        for _ in range(retries):
            seq = struct.unpack_from("<Q", mm, 0)[0]
            if seq == 0 or seq & 1:  # nothing published yet / mid-write
                continue
            raw = mm[:size]
            if struct.unpack_from("<Q", mm, 0)[0] != seq:
                continue
            return raw
        return None

    def latest(self, retries: int = 8) -> dict | None:
        """The newest whole frame, shaped as the log's records are. Shaping costs some thirty
        times the read, so a sampler reading every tick takes latest_raw instead."""
        raw = self.latest_raw(retries)
        if raw is None:
            return None
        return self.shape(dict(zip(self.layout.names, self.layout.struct.unpack(raw))))

    def shape(self, flat: dict) -> dict:
        """A flat struct read as the record shape frame_log_pb.frame_records yields."""
        pools, contract = self.layout.pools, self.contract
        record = {key: flat[key] for key in _CORE_KEYS}
        record["timing"] = {key: flat[key] for key in _TIMING_KEYS}
        record["constraints"] = [
            {key: flat[f"c{idx}.{key}"] for key in _CONSTRAINT_KEYS}
            for idx in range(pools["constraints"])
        ]
        record["monitors"] = [
            {key: flat[f"m{idx}.{key}"] for key in _MONITOR_KEYS}
            for idx in range(pools["monitors"])
        ]
        # Quantities carry no per-slot active word, so which ones the active motion writes comes
        # from the log header's gate -- the same source the protobuf decoder uses.
        qids = (
            contract.quantity_ids
            if contract is not None
            else [f"q{idx}" for idx in range(pools["quantities"])]
        )
        gate = contract.gate.get("quantities") if contract is not None else None
        written = gate.get(flat["active_motion"], ()) if gate is not None else None
        record["quantities"] = {
            qids[idx]: flat[f"q{idx}"]
            for idx in range(min(len(qids), pools["quantities"]))
            if written is None or idx in written
        }
        devices = (
            contract.fields.get("devices", [])
            if contract is not None
            else [{"index": idx, "id": f"device{idx}"} for idx in range(pools.get("devices", 0))]
        )
        record["devices"] = {
            entry["id"]: {
                "seq": flat[f"device{entry['index']}.seq"],
                "success": bool(flat[f"device{entry['index']}.success"]),
            }
            for entry in devices
        }
        pool_size, count = pools["triggers"], flat["trigger_count"]
        record["triggers"] = (
            [
                {key: flat[f"tr{idx % pool_size}.{key}"] for key in _TRIGGER_KEYS}
                for idx in range(max(0, count - pool_size), count)
            ]
            if pool_size
            else []
        )
        # A slot the active motion does not write stays a hole, so a slot keeps one index for
        # the whole run. Here the slot's own active word says so; the log infers it from the gate.
        for category, names in _SPATIAL:
            prefix = _SPATIAL_PREFIX[category]
            record[category] = [
                {name: flat[f"{prefix}{idx}.{name}"] for name in names}
                if flat[f"{prefix}{idx}.active"]
                else None
                for idx in range(pools.get(category, 0))
            ]
        return record


class SignalFields:
    """Signal names resolved to their (offset, struct) in a raw frame, plus the motion gate.

    server.signal_reader reads the same names off a protobuf frame; the gates and the units here
    are that reader's, so a live point means what the same point means in history. A flat layout
    name is positional (c0.error, m0.value, q0, wrench0.fx) while the contract names the same
    slot semantically -- the slot index is what joins the two.
    """

    def __init__(self, layout: FrameLayout, contract):
        self.layout, self.contract = layout, contract
        self.offsets: dict[str, tuple[int, struct.Struct]] = {}
        offset = 0
        for field in layout.fields:
            item = struct.Struct("<" + field["fmt"])
            self.offsets[field["name"]] = (offset, item)
            offset += item.size
        if offset != layout.struct.size:
            raise ValueError(
                f"frame_layout.json field formats pack to {offset} bytes, but the frame "
                f"struct is {layout.struct.size}"
            )
        self._constraints = {
            field["id"]: field["index"] for field in contract.fields["constraints"]
        }
        self._monitors = {
            slot.id: (contract.fields["monitors"][slot.number]["index"], motion.index)
            for motion in contract.header.motions
            for slot in motion.monitors
            if slot.number < len(contract.fields["monitors"])
        }
        self._quantities = {field["id"]: field["index"] for field in contract.fields["quantities"]}
        self._slots = slot_signals(contract)
        self._resolved: dict[str, tuple] = {}

    def core(self, raw: bytes) -> dict:
        """The words a sampler steers by, straight out of the frame."""
        return {key: _read(self.offsets[key], raw) for key in CORE_SIGNALS}

    def extract(self, raw: bytes, active_motion: int, names) -> list:
        """Each name's value in this frame; None where the active motion does not write it."""
        values = []
        for name in names:
            where, divisor, gate = self._resolve(name)
            if gate is not None and not self._written(gate, active_motion):
                values.append(None)
                continue
            value = _read(where, raw)
            values.append(value / divisor if divisor else value)
        return values

    def _written(self, gate: tuple, motion: int) -> bool:
        """Whether the active motion writes this slot -- signal_reader's gate, same source."""
        kind, index = gate
        if kind == "owner":
            return motion == index
        table = self.contract.gate.get(kind)
        return table is None or index in table.get(motion, ())

    def _resolve(self, name: str) -> tuple:
        if name not in self._resolved:
            self._resolved[name] = self._locate(name)
        return self._resolved[name]

    def _locate(self, name: str) -> tuple:
        if name in _TIMING_MS:
            flat, divisor = _TIMING_MS[name]
            return self._field(name, flat), divisor, None
        if name in self._quantities:
            index = self._quantities[name]
            return self._field(name, f"q{index}"), None, ("quantities", index)
        if name in self._slots:
            kind, field, attribute = self._slots[name]
            flat = f"{_SPATIAL_PREFIX[kind]}{field['index']}.{attribute}"
            return self._field(name, flat), None, (kind, field["index"])
        prefix, _, key = name.rpartition(".")
        if prefix in self._constraints:
            return self._field(name, f"c{self._constraints[prefix]}.{key}"), None, None
        if prefix in self._monitors:
            index, owner = self._monitors[prefix]
            return self._field(name, f"m{index}.{key}"), None, ("owner", owner)
        raise ValueError(f"unknown signal: {name}")

    def _field(self, name: str, flat: str) -> tuple:
        if flat not in self.offsets:
            raise ValueError(f"signal {name} wants frame field {flat}, which this layout has not")
        return self.offsets[flat]


def _read(where: tuple, raw: bytes):
    offset, item = where
    return item.unpack_from(raw, offset)[0]
