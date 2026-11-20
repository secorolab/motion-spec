# SPDX-License-Identifier: MPL-2.0
"""Read the generated runtime's latest-frame shared-memory block.

The publisher overwrites one Frame slot per tick behind a seqlock, so this is the cheapest
view of "right now" -- lossy by design. The frame log (see tail.py) stays the lossless record.
"""

from __future__ import annotations

import mmap
import struct

from motion_spec.telemetry.frame_log_pb import shm_path

READ_ATTEMPTS = 8
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
    }


class FrameLayout:
    """The struct that decodes a program's shm Frame, laid out from its frame-log header the way
    the generator laid out the Frame it compiled in."""

    def __init__(self, contract):
        from motion_spec.generation.artifacts import field_names_and_format

        self.pools = {category: len(entries) for category, entries in contract.fields.items()}
        self.schema_hash = contract.header.schema_hash
        fmt, self.names = field_names_and_format(self.pools)
        self.struct = struct.Struct(fmt)
        self.formats = list(fmt[1:])


class ShmFrameReader:
    """The latest published frame, read through the writer's seqlock."""

    def __init__(self, shm_name: str, layout: FrameLayout):
        self.name = shm_name
        self.layout = layout
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

    def latest_raw(self) -> bytes | None:
        """The newest whole frame as the bytes it was published as, or None while the writer
        keeps tearing it (or has yet to publish one). A torn read is never returned: seq
        brackets every publish."""
        if not self.open():
            return None
        mm, size = self._mm, self.layout.struct.size
        for _ in range(READ_ATTEMPTS):
            seq = struct.unpack_from("<Q", mm, 0)[0]
            if seq == 0 or seq & 1:  # nothing published yet / mid-write
                continue
            raw = mm[:size]
            if struct.unpack_from("<Q", mm, 0)[0] != seq:
                continue
            return raw
        return None


class SignalFields:
    """Signal names resolved to their (offset, struct) in a raw frame, plus the motion gate.

    replay.signal_reader reads the same names off a protobuf frame; the gates and the units here
    are that reader's, so a live point means what the same point means in history. A flat layout
    name is positional (c0.error, m0.value, q0, wrench0.fx) while the contract names the same
    slot semantically -- the slot index is what joins the two.
    """

    def __init__(self, layout: FrameLayout, contract):
        self.contract = contract
        self.offsets: dict[str, tuple[int, struct.Struct]] = {}
        offset = 0
        for name, code in zip(layout.names, layout.formats, strict=True):
            item = struct.Struct("<" + code)
            self.offsets[name] = (offset, item)
            offset += item.size
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

    def extract(self, raw: bytes, active_motion: int, names) -> list:
        """Each name's value in this frame; None where the active motion does not write it."""
        values = []
        for name in names:
            if name not in self._resolved:
                self._resolved[name] = self.locate(name)
            where, divisor, writers = self._resolved[name]
            if writers is not None and active_motion not in writers:
                values.append(None)
                continue
            value = read(where, raw)
            values.append(value / divisor if divisor else value)
        return values

    def locate(self, name: str) -> tuple:
        """(offset and struct, divisor, the active motions that write it or None for all)."""
        if name in _TIMING_MS:
            flat, divisor = _TIMING_MS[name]
            return self.field_of(name, flat), divisor, None
        if name in self._quantities:
            index = self._quantities[name]
            return self.field_of(name, f"q{index}"), None, self.writers("quantities", index)
        if name in self._slots:
            kind, field, attribute = self._slots[name]
            flat = f"{_SPATIAL_PREFIX[kind]}{field['index']}.{attribute}"
            return self.field_of(name, flat), None, self.writers(kind, field["index"])
        prefix, _, key = name.rpartition(".")
        if prefix in self._constraints:
            return self.field_of(name, f"c{self._constraints[prefix]}.{key}"), None, None
        if prefix in self._monitors:
            index, owner = self._monitors[prefix]
            return self.field_of(name, f"m{index}.{key}"), None, {owner}
        raise ValueError(f"unknown signal: {name}")

    def writers(self, kind: str, index: int) -> set | None:
        """The active motions whose gate writes slot INDEX of KIND; None when KIND is ungated.

        signal_reader's gate, same source, resolved once per signal rather than per frame.
        """
        table = self.contract.gate.get(kind)
        if table is None:
            return None
        return {motion for motion, written in table.items() if index in written}

    def field_of(self, name: str, flat: str) -> tuple:
        if flat not in self.offsets:
            raise ValueError(f"signal {name} wants frame field {flat}, which this layout has not")
        return self.offsets[flat]


def read(where: tuple, raw: bytes):
    offset, item = where
    return item.unpack_from(raw, offset)[0]
