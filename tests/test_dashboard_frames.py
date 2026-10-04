# SPDX-License-Identifier: MPL-2.0
"""The shm reader must yield the record the frame log yields, and never a torn one."""

from __future__ import annotations

import json

import pytest

from support import DASHBOARD_SCHEMA

from motion_spec.dashboard.frames import FrameLayout, ShmFrameReader
from motion_spec.generation.artifacts import (
    build_frame_layout,
    build_frame_log_header_record,
    field_names_and_format,
)
from motion_spec.telemetry import frame_log_pb

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


ZERO_FRAME = {name: 0 for name in field_names_and_format(DASHBOARD_SCHEMA["pools"])[1]}


@pytest.fixture
def layout(tmp_path) -> FrameLayout:
    path = tmp_path / "frame_layout.json"
    path.write_text(json.dumps(build_frame_layout(DASHBOARD_SCHEMA), indent=4))
    return FrameLayout.load(path)


def test_shm_frame_decodes_to_the_same_record_the_log_does(tmp_path, layout):
    flat = ZERO_FRAME | FRAME
    log = tmp_path / "frame_log.pb"
    with log.open("wb") as fh:
        frame_log_pb.write_delimited(fh, build_frame_log_header_record(DASHBOARD_SCHEMA))
        frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(flat, DASHBOARD_SCHEMA))
    contract = frame_log_pb.read_contract(log)
    (logged,) = list(frame_log_pb.frame_records(log, contract))

    # A shm block as the C++ publisher leaves it: the frame, bracketed by an even seq.
    block = tmp_path / "shm_block"
    block.write_bytes(layout.struct.pack(*((flat | {"seq": 2})[name] for name in layout.names)))
    live = ShmFrameReader(str(block), layout, contract).latest()
    assert live == logged
    assert live["constraints"][0]["error"] == 0.125
    assert live["quantities"] == {"dist": 42.5}
    assert [t["idx"] for t in live["triggers"]] == [3]


# An odd seq is a block mid-write; an unpublished block (seq 0) is not a frame of zeros either.
@pytest.mark.parametrize(("flat", "seq"), [(ZERO_FRAME | FRAME, 3), (ZERO_FRAME, 0)])
def test_a_reader_never_returns_a_torn_frame(tmp_path, layout, flat, seq):
    block = tmp_path / "shm_block"
    block.write_bytes(layout.struct.pack(*((flat | {"seq": seq})[name] for name in layout.names)))
    assert ShmFrameReader(str(block), layout).latest() is None
