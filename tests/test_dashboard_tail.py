# SPDX-License-Identifier: MPL-2.0
"""Tailing a log the runtime is still writing: nothing duplicated, nothing lost, nothing torn."""

from __future__ import annotations

from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.generation.artifacts import build_frame_log_header_record
from motion_spec.introspection import frame_log_pb

from dashboard_fixture import schema
from frame_log_fixture import flat_frame


def _flat(doc, step):
    return flat_frame(
        doc,
        t=step * 0.1,
        step=step,
        fsm_state=0,
        active_motion=0,
        last_event=-1,
        wall_ns=1000 + step,
        period_ns=1_000_000,
        **{"c0.active": 1, "c0.error": step * 0.5, "m0.active": 1},
    )


def _start_log(path, doc):
    fh = path.open("wb")
    frame_log_pb.write_delimited(fh, build_frame_log_header_record(doc))
    fh.flush()
    return fh


def _append(fh, doc, step):
    frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(_flat(doc, step), doc))
    fh.flush()


def test_polling_between_appends_loses_and_repeats_nothing(tmp_path):
    doc = schema()
    log = tmp_path / "frame_log.pb"
    fh = _start_log(log, doc)
    tail = FrameLogTail(log)

    seen = []
    for batch in ([0, 1], [2], [], [3, 4, 5]):
        for step in batch:
            _append(fh, doc, step)
        seen.extend(frame["step"] for frame in tail.poll())

    assert seen == [0, 1, 2, 3, 4, 5]
    assert tail.poll() == []  # nothing new, nothing repeated
    fh.close()
    tail.close()


def test_a_partial_trailing_record_waits_for_the_writer(tmp_path):
    doc = schema()
    log = tmp_path / "frame_log.pb"
    fh = _start_log(log, doc)
    tail = FrameLogTail(log)
    assert tail.poll() == []

    blob = frame_log_pb.frame_record(_flat(doc, 5), doc)
    prefix = frame_log_pb._varint(len(blob))
    if len(prefix) > 1:  # a length prefix cut mid-varint
        fh.write(prefix[:1])
        fh.flush()
        assert tail.poll() == []
        fh.write(prefix[1:])
    else:
        fh.write(prefix)
    fh.write(blob[:-4])  # a payload cut short
    fh.flush()
    assert tail.poll() == []

    fh.write(blob[-4:])
    fh.flush()
    assert [frame["step"] for frame in tail.poll()] == [5]
    fh.close()
    tail.close()


def test_the_tail_stays_shut_until_the_header_is_complete(tmp_path):
    doc = schema()
    log = tmp_path / "frame_log.pb"
    tail = FrameLogTail(log)
    assert tail.open() is False  # no file yet
    assert tail.poll() == []

    header = build_frame_log_header_record(doc)
    fh = log.open("wb")
    fh.write(frame_log_pb._varint(len(header)))
    fh.write(header[:20])
    fh.flush()
    assert tail.open() is False  # the contract is not readable yet
    assert tail.contract is None

    fh.write(header[20:])
    fh.flush()
    assert tail.open() is True
    assert tail.contract.header.schema_hash == doc["schema_hash"]
    fh.close()
    tail.close()


def test_the_tail_yields_the_same_records_a_finished_log_does(tmp_path):
    doc = schema()
    log = tmp_path / "frame_log.pb"
    fh = _start_log(log, doc)
    for step in range(4):
        _append(fh, doc, step)
    fh.close()

    tail = FrameLogTail(log)
    tailed = tail.poll()
    tail.close()
    assert tailed == list(frame_log_pb.frame_records(log))
