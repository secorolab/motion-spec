# SPDX-License-Identifier: MPL-2.0
"""Live constraint plots: signals streamed out of the block the runtime republishes each tick.

The history a chart opens with comes from /api/plot; what is tested here is the increment --
that a poll carries the samples taken since the last one, and no sample twice.
"""

from __future__ import annotations

import json
import struct
import time

import pytest

from motion_spec.dashboard import jobs, live, replay, roots
from motion_spec.dashboard.frames import FrameLayout
from motion_spec.generation.artifacts import build_frame_layout, build_frame_log_header_record
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.replay import resolve_archive

from frame_log_fixture import flat_frame
from support import _hash_doc
from test_dashboard_runs import _contract_schema

SIGNALS = ["constraint_0.error", "timing.compute_ms"]


def _flat(doc, step, **values):
    """One tick, with the constraint error carrying its own frame index."""
    return flat_frame(
        doc,
        **{
            "t": step * 0.001,
            "step": step,
            "fsm_state": 0,
            "active_motion": 0,
            "last_event": -1,
            "period_ns": 1_000_000,
            "compute_ns": step * 1000,
            "c0.active": 1,
            "c0.error": float(step),
            "m0.active": 1,
            "q0": 0.5,
            "q1": 0.02,
            **values,
        },
    )


def _until(check, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return True
        time.sleep(0.005)
    return False


@pytest.fixture
def live_run(tmp_path, monkeypatch):
    """A run whose frame block is being republished, with the dashboard rooted above it."""
    doc = _contract_schema()
    # A run with something to fire: the sampler must mark events, not only state entries.
    doc["fsm"]["events"] = [{"index": 0, "id": "E_SETTLED", "uri": "https://example.test/settled"}]
    doc["schema_hash"] = _hash_doc(doc)
    run = tmp_path / "demo" / "20260822T000000Z" / "runs" / "run-1"
    (run / "logs").mkdir(parents=True)
    # live_state asks the run's control block, and the layout is what names it
    path = run.parent.parent / "generated" / "contract" / "frame_layout.json"
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(build_frame_layout(doc)))
    layout = FrameLayout.load(path)
    fh = (run / "logs" / "frame_log.pb").open("wb")
    frame_log_pb.write_delimited(fh, build_frame_log_header_record(doc))
    fh.flush()
    block = tmp_path / "shm_block"
    block.write_bytes(bytes(layout.struct.size))
    handle = block.open("r+b")
    monkeypatch.setenv("MOTION_SPEC_SHM_NAME", str(block))
    monkeypatch.setattr(roots, "GENERATIONS", tmp_path)
    live._LIVE.clear()

    def append(steps):
        """Frames into the log, which only the no-block fallback and /api/plot ever read."""
        for step in steps:
            frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(_flat(doc, step), doc))
        fh.flush()

    def pack(step, **values):
        flat = _flat(doc, step, **values)
        return layout.struct.pack(*(dict(flat, seq=2)[name] for name in layout.names))

    def sampler():
        session = live._LIVE.get(str(run))
        return session["sampler"] if session else None

    seq = [0]

    def publish(step, **values):
        """The seqlock protocol the C++ publisher writes: odd, body, even."""
        raw, seq[0] = pack(step, **values), seq[0] + 2
        for at, chunk in (
            (0, struct.pack("<Q", seq[0] - 1)),
            (8, raw[8:]),
            (0, struct.pack("<Q", seq[0])),
        ):
            handle.seek(at)
            handle.write(chunk)
            handle.flush()

    def feed(steps, **values):
        """Hand the sampler what the block would have carried, without racing its own thread."""
        for step in steps:
            sampler().absorb(pack(step, **values))

    live.live_state(run)  # the first poll is what opens the sampler
    yield type(
        "LiveRun",
        (),
        {
            "path": run,
            "doc": doc,
            "append": staticmethod(append),
            "publish": staticmethod(publish),
            "feed": staticmethod(feed),
            "sampler": staticmethod(sampler),
        },
    )
    fh.close()
    handle.close()
    for session in list(live._LIVE.values()):
        live._close_live(session)
    live._LIVE.clear()


def _raw_frames(log, contract):
    """The log's frames unshaped, as the replay reader is handed them."""
    frames = []
    with log.open("rb") as fh:
        while data := frame_log_pb._read_delimited(fh, partial_ok=True):
            record = contract.record_cls()
            record.ParseFromString(data)
            if record.WhichOneof("record") == "frame":
                frames.append(record.frame)
    return frames


def test_the_reader_reads_a_raw_frame_the_way_the_plots_do(live_run):
    """One lookup for both paths: a live point must mean what the same point means in history."""
    live_run.append(range(4))
    _run, log, _manifest, contract = resolve_archive(live_run.path)
    read = replay.signal_reader(contract)

    recorded = replay.plot_data(live_run.path, SIGNALS, (0, 3))["signals"]
    for name in SIGNALS:
        assert [read(frame, name) for frame in _raw_frames(log, contract)] == recorded[name]
    assert recorded["constraint_0.error"] == [0.0, 1.0, 2.0, 3.0]
    assert recorded["timing.compute_ms"] == [0.0, 0.001, 0.002, 0.003]


def test_the_sampler_reads_the_block_the_runtime_republishes(live_run):
    """The live transport end to end: what the publisher wrote is what the poll answers with."""
    sampler = live_run.sampler()
    live_run.publish(0)  # the block's standing frame, which is not an advance
    assert _until(lambda: sampler.seen == 0)
    assert sampler.taken == 0

    live_run.publish(1)
    assert _until(lambda: sampler.taken == 1)
    assert live.live_state(live_run.path)["frames"] == 2


def test_a_block_that_republishes_the_same_step_is_not_a_new_sample(live_run):
    """A paused run keeps publishing; nothing has happened until the step moves."""
    live_run.feed([0, 1])
    taken = live_run.sampler().taken
    live_run.feed([1, 1, 1])
    live_run.feed([2])
    assert live_run.sampler().taken == taken + 1


def test_the_sampler_marks_a_state_and_an_event_where_they_change(live_run):
    """Events and states come off the same samples the values do -- one transport, not two."""
    live_run.feed([0])
    live_run.feed([1])
    live_run.feed([2], fsm_state=1, last_event=0)
    events = live.live_state(live_run.path)["events"]
    assert [(entry["frame"], entry["kind"]) for entry in events] == [
        (1, "state"),
        (2, "event"),
        (2, "state"),
    ]
    assert [entry["label"] for entry in events] == ["S_MOVE", "E_SETTLED", "1"]


def test_the_first_poll_with_signals_starts_the_stream_at_the_ring_s_end(live_run):
    """History is the chart's /api/plot backfill; the stream must not replay it."""
    live_run.feed(range(5))
    first = live.live_state(live_run.path, SIGNALS)
    assert first["plot"] == {"frames": [], "series": {name: [] for name in SIGNALS}}

    live_run.feed(range(5, 8))
    plot = live.live_state(live_run.path, SIGNALS)["plot"]
    assert plot["frames"] == [5, 6, 7]
    assert plot["series"]["constraint_0.error"] == [5.0, 6.0, 7.0]

    assert live.live_state(live_run.path, SIGNALS)["plot"]["frames"] == []  # nothing repeated


def test_a_poll_returns_at_most_the_point_cap_per_signal(live_run):
    live_run.feed(range(2))
    live.live_state(live_run.path, SIGNALS)
    live_run.feed(range(2, 2 + 4 * live.LIVE_PLOT_POINTS))

    plot = live.live_state(live_run.path, SIGNALS)["plot"]
    assert 0 < len(plot["frames"]) <= live.LIVE_PLOT_POINTS
    assert plot["frames"] == sorted(set(plot["frames"]))
    assert len(plot["series"]["constraint_0.error"]) == len(plot["frames"])
    # decimated, not truncated: the sample spans the whole increment
    assert plot["frames"][-1] > 3 * live.LIVE_PLOT_POINTS


def test_changing_the_signal_set_keeps_every_sample_since_the_last_poll(live_run):
    """A motion entering opens cards mid-run: the samples it opened over are the run's own.

    The cursor counts samples, not names, so the charts already on screen carry on across the
    change -- a motion that satisfies in milliseconds still gets its points.
    """
    live_run.feed(range(5))
    live.live_state(live_run.path, ["constraint_0.error"])
    live_run.feed(range(5, 10))

    plot = live.live_state(live_run.path, SIGNALS)["plot"]
    assert plot["frames"] == [5, 6, 7, 8, 9]
    assert plot["series"]["constraint_0.error"] == [5.0, 6.0, 7.0, 8.0, 9.0]
    live_run.feed(range(10, 12))
    assert live.live_state(live_run.path, SIGNALS)["plot"]["frames"] == [10, 11]


def test_a_signal_added_mid_stream_carries_its_own_values_from_then_on(live_run):
    """The new name reads from the same samples as the old one; its history is /api/plot's."""
    live_run.feed(range(4))
    live.live_state(live_run.path, ["constraint_0.error"])
    live_run.feed(range(4, 7))

    plot = live.live_state(live_run.path, SIGNALS)["plot"]
    assert plot["frames"] == [4, 5, 6]
    assert plot["series"]["timing.compute_ms"] == [0.004, 0.005, 0.006]


def test_a_run_with_no_block_to_read_still_follows_its_log(live_run):
    """Hardware and builds older than the block: the log is all there is, and it still says."""
    live_run.append(range(3))
    answer = live.live_state(live_run.path)
    assert "plot" not in answer
    assert answer["frames"] == 1 and answer["events"][0]["kind"] == "state"

    live_run.append(range(3, 200))
    later = live.live_state(live_run.path)
    assert "plot" not in later and later["frames"] > answer["frames"]


def test_only_a_headless_simulation_asks_the_runtime_to_run_in_realtime():
    """A headless loop is uncapped, which is nothing to watch: realtime is what a plot needs."""
    assert jobs.run_arguments({"headless": True}, True) == [
        "--headless",
        "--rtf",
        "1",
        "--start-paused",
    ]
    assert jobs.run_arguments({"headless": True, "realtime": False}, True) == [
        "--headless",
        "--start-paused",
    ]
    assert jobs.run_arguments({"headless": False, "realtime": True}, True) == ["--start-paused"]
    assert jobs.run_arguments({"headless": True, "realtime": True}, False) == []


def test_logs_off_asks_the_cli_for_a_run_that_writes_none():
    """The frame log is the runtime's cost on any platform, so the choice is offered on all."""
    assert jobs.run_arguments({"headless": True, "log": False}, True) == [
        "--headless",
        "--rtf",
        "1",
        "--no-log",
        "--start-paused",
    ]
    assert jobs.run_arguments({"headless": False, "log": False}, True) == [
        "--no-log",
        "--start-paused",
    ]
    assert jobs.run_arguments({"log": False}, False) == ["--no-log"]
    assert jobs.run_arguments({"headless": False, "log": True}, True) == ["--start-paused"]
    assert jobs.run_arguments({"headless": False}, True) == ["--start-paused"]


def test_a_run_started_from_the_page_arms_paused_unless_it_is_hardware():
    """The page has the transport: a simulation waits for play; a robot has no pause to seed."""
    for options in ({}, {"headless": True}, {"headless": False, "log": False}):
        assert "--start-paused" in jobs.run_arguments(options, True)
        assert "--start-paused" not in jobs.run_arguments(options, False)


def test_run_status_says_whether_the_run_kept_a_frame_log(live_run, monkeypatch):
    """A page that finds no log must know it was a choice, not a run that never started."""
    generation = live_run.path.parent.parent
    finished = type("Finished", (), {"poll": lambda self: 0, "returncode": 0, "pid": 1})()
    monkeypatch.setitem(
        jobs.RUNNING, str(generation), {"process": finished, "run_id": live_run.path.name}
    )
    manifest = live_run.path / "manifest.json"

    assert jobs.run_status(generation)["recorded"] is None  # nothing has said yet
    manifest.write_text(json.dumps({"run_id": "run-1", "recorded": False, "files": {}}))
    assert jobs.run_status(generation)["recorded"] is False
    manifest.write_text(json.dumps({"run_id": "run-1", "files": {}}))
    assert jobs.run_status(generation)["recorded"] is True


def test_the_response_names_the_motion_of_the_newest_sample(live_run):
    live_run.feed(range(4))
    assert live.live_state(live_run.path, SIGNALS)["active_motion"] == "move"
