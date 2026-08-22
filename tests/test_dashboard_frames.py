# SPDX-License-Identifier: MPL-2.0
"""The shm reader must yield the record the frame log yields, and never a torn one."""

from __future__ import annotations

import json
import struct

import pytest

from motion_spec.dashboard import replay
from motion_spec.dashboard.frames import (
    FrameLayout,
    ShmFrameReader,
    SignalFields,
    shm_name_for,
    shm_path,
)
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.introspection import frame_log_pb

from dashboard_fixture import schema
from frame_log_fixture import flat_frame, write_frame_log_pb
from test_dashboard_runs import _contract_schema

FRAME = {
    "t": 2.5,
    "step": 12,
    "fsm_state": 0,
    "active_motion": 0,
    "last_event": -1,
    "state_since_t": 1.0,
    "state_since_wall_ns": 200,
    "event_t": 0.0,
    "event_wall_ns": 0,
    "wall_ns": 900,
    "period_ns": 1_000_000,
    "compute_ns": 25_000,
    "c0.active": 1,
    "c0.error": 0.125,
    "c0.output": -3.5,
    "c0.satisfied": 1,
    "c0.sat_t": 2.0,
    "c0.measured": 0.5,
    "c0.setpoint": 0.375,
    "m0.active": 1,
    "m0.value": 0.004,
    "m0.satisfied": 1,
    "m0.sat_t": 2.4,
    "q0": 42.5,
    "trigger_count": 1,
    "tr0.kind": 1,
    "tr0.idx": 3,
    "tr0.fsm_state": 0,
    "tr0.t": 2.4,
    "tr0.wall_ns": 800,
    "pose0.active": 1,
    "pose0.px": 1.0,
    "pose0.py": 2.0,
    "pose0.pz": 3.0,
    "pose0.qw": 1.0,
}


def _layout(tmp_path, doc):
    path = tmp_path / "frame_layout.json"
    path.write_text(json.dumps(build_frame_layout(doc), indent=4))
    return FrameLayout.load(path)


def _publish(tmp_path, layout, flat, seq):
    """A shm block as the C++ publisher leaves it: the frame, bracketed by an even seq."""
    block = tmp_path / "shm_block"
    values = dict(flat, seq=seq)
    block.write_bytes(layout.struct.pack(*(values[name] for name in layout.names)))
    return block


def test_shm_frame_decodes_to_the_same_record_the_log_does(tmp_path):
    doc = schema()
    layout = _layout(tmp_path, doc)
    flat = flat_frame(doc, **FRAME)

    log = tmp_path / "frame_log.pb"
    write_frame_log_pb(log, doc, [flat])
    contract = frame_log_pb.read_contract(log)
    (logged,) = list(frame_log_pb.frame_records(log, contract))

    block = _publish(tmp_path, layout, flat, seq=2)
    live = ShmFrameReader(str(block), layout, contract).latest()

    # One shape, so every consumer downstream is source-agnostic.
    assert live == logged
    assert live["step"] == 12 and live["t"] == 2.5
    assert live["constraints"][0]["error"] == 0.125
    assert live["quantities"] == {"dist": 42.5}
    assert live["poses"][0]["px"] == 1.0
    assert [t["idx"] for t in live["triggers"]] == [3]


def test_an_inactive_spatial_slot_stays_a_hole(tmp_path):
    doc = schema()
    layout = _layout(tmp_path, doc)
    flat = flat_frame(doc, **{**FRAME, "pose0.active": 0})
    block = _publish(tmp_path, layout, flat, seq=4)

    live = ShmFrameReader(str(block), layout).latest()
    assert live["poses"] == [None]


def test_a_reader_never_returns_a_torn_frame(tmp_path):
    doc = schema()
    layout = _layout(tmp_path, doc)
    block = _publish(tmp_path, layout, flat_frame(doc, **FRAME), seq=3)  # odd: mid-write

    assert ShmFrameReader(str(block), layout).latest() is None

    # An unpublished block (seq 0) is not a frame of zeros either.
    empty = _publish(tmp_path, layout, flat_frame(doc), seq=0)
    assert ShmFrameReader(str(empty), layout).latest() is None


def test_a_missing_block_reads_as_no_frame(tmp_path):
    layout = _layout(tmp_path, schema())
    reader = ShmFrameReader(str(tmp_path / "absent"), layout)
    assert reader.open() is False
    assert reader.latest() is None


def test_the_shm_name_follows_the_generated_runtime(tmp_path):
    doc = schema()
    layout = _layout(tmp_path, doc)
    assert layout.shm_name == f"/motion_spec_{doc['schema_hash'][:16]}"
    assert layout.shm_name == shm_name_for(doc["schema_hash"])
    assert shm_path(layout.shm_name).parent.name == "shm"
    assert shm_path("/tmp/some/file") == shm_path("/tmp/some/file")


def test_the_layout_struct_matches_the_declared_frame_size(tmp_path):
    doc = schema()
    layout = _layout(tmp_path, doc)
    assert layout.struct.size == layout.layout["frame_size_bytes"]
    assert struct.calcsize(layout.struct.format) == len(layout.fields) * 8


# One name per class signal_reader knows: constraint, monitor, quantity, spatial slot, timing.
SIGNALS = (
    "constraint_0.error",
    "constraint_0.satisfied",
    "done_mon.value",
    "dist",
    "err_x",
    "cmd_wrench.force.x",
    "timing.compute_ms",
    "timing.period_ms",
)
LIVE_FRAME = {
    "t": 0.5,
    "step": 7,
    "period_ns": 1_000_000,
    "compute_ns": 25_000,
    "c0.error": 0.125,
    "c0.satisfied": 1,
    "m0.value": 0.004,
    "q0": 42.5,
    "q1": 0.02,
    "wrench0.active": 1,
    "wrench0.fx": -3.5,
}


def _both_readers(tmp_path, active_motion):
    """The same frame read off the packed block and off the log, by name."""
    doc = _contract_schema()
    layout = _layout(tmp_path, doc)
    flat = flat_frame(doc, **LIVE_FRAME, active_motion=active_motion, last_event=-1)
    log = tmp_path / "frame_log.pb"
    write_frame_log_pb(log, doc, [flat])
    contract = frame_log_pb.read_contract(log)
    with log.open("rb") as fh:
        frame_log_pb._read_delimited(fh)
        record = contract.record_cls()
        record.ParseFromString(frame_log_pb._read_delimited(fh))
    read = replay.signal_reader(contract)
    fields = SignalFields(layout, contract)
    raw = layout.struct.pack(*(dict(flat, seq=2)[name] for name in layout.names))
    return layout, fields, [read(record.frame, name) for name in SIGNALS], raw


def test_a_signal_reads_the_same_off_the_frame_as_off_the_log(tmp_path):
    """One lookup for both transports: a live point must mean what the same point means later."""
    _layouted, fields, logged, raw = _both_readers(tmp_path, active_motion=0)
    assert fields.extract(raw, 0, SIGNALS) == logged
    assert logged[SIGNALS.index("constraint_0.error")] == 0.125
    assert logged[SIGNALS.index("timing.compute_ms")] == 0.025


def test_a_signal_the_active_motion_does_not_write_reads_as_nothing(tmp_path):
    """The gate is signal_reader's own, so a gated-out signal is a hole in both readers."""
    _layouted, fields, logged, raw = _both_readers(tmp_path, active_motion=1)
    assert fields.extract(raw, 1, SIGNALS) == logged
    assert None in logged


def test_every_frame_field_sits_where_the_layout_says_it_does(tmp_path):
    doc = _contract_schema()
    layout = _layout(tmp_path, doc)
    log = tmp_path / "frame_log.pb"
    write_frame_log_pb(log, doc, [flat_frame(doc)])
    fields = SignalFields(layout, frame_log_pb.read_contract(log))
    assert [fields.offsets[field["name"]][0] for field in layout.fields] == [
        field["offset"] for field in layout.fields
    ]
    assert sum(item.size for _offset, item in fields.offsets.values()) == layout.struct.size


def test_a_name_no_signal_carries_is_refused(tmp_path):
    doc = _contract_schema()
    layout = _layout(tmp_path, doc)
    log = tmp_path / "frame_log.pb"
    write_frame_log_pb(log, doc, [flat_frame(doc)])
    fields = SignalFields(layout, frame_log_pb.read_contract(log))
    with pytest.raises(ValueError, match="unknown signal"):
        fields.extract(b"\0" * layout.struct.size, 0, ["no.such.signal"])
