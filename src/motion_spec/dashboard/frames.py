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

    def latest(self, retries: int = 8) -> dict | None:
        """The newest whole frame, or None while the writer keeps tearing it (or has yet to
        publish one). A torn read is never returned: seq brackets every publish."""
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
            return self.shape(dict(zip(self.layout.names, self.layout.struct.unpack(raw))))
        return None

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
            else [
                {"index": idx, "id": f"device{idx}"}
                for idx in range(pools.get("devices", 0))
            ]
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
