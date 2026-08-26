# SPDX-License-Identifier: MPL-2.0
"""The temporal and causal views: timeline, gates, compare.

All three read the archived runtime graph joined to the design graph. What is tested here is
what a reader is told -- including what the views refuse to make up when the run has not
archived a graph yet.
"""

from __future__ import annotations

import json
import threading
import urllib.parse
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from motion_spec.dashboard import roots, server, tail
from motion_spec.dashboard.queries import compare, gates, timeline
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.runtime_graph import write_runtime_ttl

from dashboard_fixture import schema
from frame_log_fixture import flat_frame, write_frame_log_pb
from support import _hash_doc

MODEL = "https://example.test/"
S_START, S_MOVE, S_HOLD, S_DONE = (f"{MODEL}S_{name}" for name in ("START", "MOVE", "HOLD", "DONE"))
E_GO, E_HOLD, E_RESUME, E_DONE = (f"{MODEL}E_{name}" for name in ("GO", "HOLD", "RESUME", "DONE"))
CONSTRAINT = f"{MODEL}hold_height"
MON_DWELL = f"{MODEL}handler_move/mon_settled"
MON_NODWELL = f"{MODEL}handler_move/mon_arrived"
MON_QUIET = f"{MODEL}handler_recover/mon_slipped"
PERIOD_NS = 10_000_000  # 100 Hz, so a step is a hundredth of a second

FSM = "https://secorolab.github.io/metamodels/behaviour/fsm#"
CSTR = "https://comp-rob2b.github.io/metamodels/task/constraint#"
CSTR_HDL = "https://comp-rob2b.github.io/metamodels/task/constraint-handler#"
CH = "https://secorolab.github.io/metamodels/task/constraint-handler#"
QUDT = "http://qudt.org/schema/qudt/"


def model_jsonld() -> dict:
    """What the design graph declares: the elements a run can occupy, and the gates it arms.

    The runtime graph names only these IRIs, so it is this file that says which of them is a
    coordination element and which a monitor -- and which monitors declare a dwell.
    """
    dwell = {"@id": f"{MON_DWELL}.debounce", f"{QUDT}value": 0.05}
    quiet_dwell = {"@id": f"{MON_QUIET}.debounce", f"{QUDT}value": 0.2}
    return {
        "@graph": [
            *(
                {"@id": state, "@type": f"{FSM}State"}
                for state in (S_START, S_MOVE, S_HOLD, S_DONE)
            ),
            {"@id": CONSTRAINT, "@type": f"{CSTR}Constraint"},
            {
                "@id": MON_DWELL,
                "@type": f"{CSTR_HDL}Monitor",
                f"{CSTR_HDL}event": {"@id": E_HOLD},
                f"{CH}debounce-duration": dwell,
            },
            dwell,
            # A gate the model declares no dwell for: the view must say so, never read it as 0.
            {
                "@id": MON_NODWELL,
                "@type": f"{CSTR_HDL}Monitor",
                f"{CSTR_HDL}event": {"@id": E_DONE},
            },
            # Armed by a handler this run never reached, so it has no occurrence at all.
            {
                "@id": MON_QUIET,
                "@type": f"{CSTR_HDL}Monitor",
                f"{CH}debounce-duration": quiet_dwell,
            },
            quiet_dwell,
        ]
    }


def views_schema() -> dict:
    """A contract with four states, four events, and two gates that fire on two of them."""
    doc = schema()
    doc["control_period_ns"] = PERIOD_NS
    doc["pools"] = {"constraints": 1, "monitors": 2, "quantities": 1, "triggers": 1, "poses": 1}
    doc["fsm"] = {
        "namespace": MODEL,
        "states": [
            {"index": index, "id": f"S{index}", "uri": uri}
            for index, uri in enumerate((S_START, S_MOVE, S_HOLD, S_DONE))
        ],
        "events": [
            {"index": index, "id": f"E{index}", "uri": uri}
            for index, uri in enumerate((E_GO, E_HOLD, E_RESUME, E_DONE))
        ],
        "transitions": [
            {"id": "T_GO", "uri": f"{MODEL}T_GO", "from": 0, "to": 1, "event_index": 0},
            {"id": "T_HOLD", "uri": f"{MODEL}T_HOLD", "from": 1, "to": 2, "event_index": 1},
            {"id": "T_RESUME", "uri": f"{MODEL}T_RESUME", "from": 2, "to": 1, "event_index": 2},
            {"id": "T_DONE", "uri": f"{MODEL}T_DONE", "from": 1, "to": 3, "event_index": 3},
        ],
        "end": 3,
    }
    doc["by_motion"]["move"]["controllers"] = [
        {"index": 0, "id": "ctrl_x", "uri": f"{MODEL}ctrl_x", "constraint_uri": CONSTRAINT}
    ]
    doc["by_motion"]["move"]["monitors"] = [
        {"index": 0, "id": "mon_settled", "uri": MON_DWELL, "event_uri": E_HOLD},
        {"index": 1, "id": "mon_arrived", "uri": MON_NODWELL, "event_uri": E_DONE},
    ]
    # Re-hash: the frame-record class is cached on schema_hash, so borrowing the unmodified
    # schema's hash hands this fixture whichever field map another test cached first.
    doc["schema_hash"] = _hash_doc(doc)
    return doc


def _frame(doc, step, *, state, entered, event=None, settled=0, arrived=0, held=0):
    wall = 1_000_000_000 + step * PERIOD_NS
    trigger = (
        {"trigger_count": 1, "tr0.kind": 1, "tr0.idx": event, "tr0.wall_ns": wall}
        if event is not None
        else {"trigger_count": 0}
    )
    return flat_frame(
        doc,
        t=step * PERIOD_NS / 1e9,
        step=step,
        fsm_state=state,
        active_motion=0,
        last_event=-1,
        wall_ns=wall,
        period_ns=PERIOD_NS,
        state_since_wall_ns=1_000_000_000 + entered * PERIOD_NS,
        **{
            "c0.active": 1,
            "c0.satisfied": held,
            "c0.error": 0.01,
            "m0.active": 1,
            "m0.satisfied": settled,
            "m0.value": 0.02,
            "m1.active": 1,
            "m1.satisfied": arrived,
            "m1.value": 0.03,
        },
        **trigger,
    )


def full_run_frames(doc) -> list[dict]:
    """A run that enters S_MOVE, holds, comes back to S_MOVE, and finishes.

    `mon_settled` first holds at step 14 after breaking once, and fires with E_HOLD at 20:
    six steps, 0.06 s, against a declared 0.05 s. `mon_arrived` first holds at 38 and fires
    with E_DONE at 40, and its model declares no dwell at all.
    """
    frames = []
    for step in range(46):
        state = 0 if step < 2 else 1 if step < 21 else 2 if step < 31 else 1 if step < 41 else 3
        entered = (
            0 if step < 2 else 2 if step < 21 else 21 if step < 31 else 31 if step < 41 else 41
        )
        event = {1: 0, 20: 1, 30: 2, 40: 3}.get(step)
        settled = int(10 <= step < 12 or 14 <= step <= 20)
        arrived = int(38 <= step <= 40)
        # The goal constraint holds through the first S_MOVE occupancy and nothing else.
        held = int(4 <= step <= 18)
        frames.append(
            _frame(
                doc,
                step,
                state=state,
                entered=entered,
                event=event,
                settled=settled,
                arrived=arrived,
                held=held,
            )
        )
    return frames


def short_run_frames(doc) -> list[dict]:
    """The same model, stopped in S_HOLD: no second S_MOVE occupancy and no S_DONE."""
    return [frame for frame in full_run_frames(doc) if frame["step"] <= 25]


def _run_dir(tmp_path, name, doc, frames, *, archive=True) -> Path:
    """A run archive with its own model graph, and its runtime graph unless it is still going."""
    run = tmp_path / "demo" / "20260825T000000Z" / "runs" / name
    (run / "logs").mkdir(parents=True)
    (run / "model").mkdir()
    (run / "model" / "test-app.ld.json").write_text(json.dumps(model_jsonld()))
    layout = run.parent.parent / "generated" / "contract"
    layout.mkdir(parents=True, exist_ok=True)
    (layout / "frame_layout.json").write_text(json.dumps(build_frame_layout(doc)))
    log = run / "logs" / "frame_log.pb"
    write_frame_log_pb(log, doc, frames)
    files = {"frame_log": "logs/frame_log.pb", "model": "model/test-app.ld.json"}
    (run / "manifest.json").write_text(json.dumps({"run_id": name, "files": files}))
    if archive:
        contract = frame_log_pb.read_contract(log)
        write_runtime_ttl(run, list(frame_log_pb.frame_records(log, contract)))
    return run


@pytest.fixture
def run(tmp_path):
    return _run_dir(tmp_path, "run-1", views_schema(), full_run_frames(views_schema()))


def _names(payload):
    return [span["name"] for span in payload["spans"]]


def test_the_timeline_is_every_occupancy_with_its_duration_and_entry_event(run):
    payload = timeline(run)

    assert payload["runtime_source"] == "archive"
    assert payload["period_s"] == pytest.approx(0.01)
    assert _names(payload) == ["S_START", "S_MOVE", "S_HOLD", "S_MOVE", "S_DONE"]
    first_move = payload["spans"][1]
    assert (first_move["entered_s"], first_move["duration_s"]) == (0.02, 0.19)
    assert (first_move["transition"], first_move["event"]) == ("T_GO", "E_GO")
    assert payload["spans"][2]["event"] == "E_HOLD"


def test_a_state_entered_twice_keeps_both_occupancies(run):
    """A re-entry is usually the thing being investigated; collapsing it hides the question."""
    moves = [span for span in timeline(run)["spans"] if span["name"] == "S_MOVE"]

    assert len(moves) == 2
    assert [span["begin_step"] for span in moves] == [2, 31]


def test_neither_route_opens_the_frame_log(run, monkeypatch):
    """The boundary that keeps these views cheap: 900 triples answer, 160 MB is never read.

    Every frame reader raises -- the tail, the record decoder, and the contract read the log's
    own header. An archived run answers from its runtime graph with all three disarmed.
    """

    def refuse(*_args, **_kwargs):
        raise AssertionError("the views opened the frame log")

    monkeypatch.setattr(tail.FrameLogTail, "open", refuse)
    monkeypatch.setattr(frame_log_pb, "read_contract", refuse)
    monkeypatch.setattr(frame_log_pb, "frame_records", refuse)
    monkeypatch.setattr(frame_log_pb, "stream_records", refuse)

    assert timeline(run)["spans"]
    assert gates(run)["gates"]


def test_a_gate_reports_its_observed_wait_against_the_declared_dwell(run):
    settled = next(
        gate for gate in gates(run)["gates"] if gate["monitor_name"].endswith("mon_settled")
    )

    assert (settled["first_held_step"], settled["fired_step"]) == (14, 20)
    assert settled["waited_s"] == pytest.approx(0.06)
    assert settled["declared_dwell_s"] == pytest.approx(0.05)
    assert settled["rearm_count"] == 1
    assert settled["event_name"] == "E_HOLD"


def test_a_gate_whose_model_declares_no_dwell_reports_none_not_zero(run):
    arrived = next(
        gate for gate in gates(run)["gates"] if gate["monitor_name"].endswith("mon_arrived")
    )

    assert arrived["declared_dwell_s"] is None
    assert arrived["fired_step"] == 40


def test_a_gate_that_never_fired_keeps_its_row_with_nothing_invented(run):
    quiet = next(
        gate for gate in gates(run)["gates"] if gate["monitor_name"].endswith("mon_slipped")
    )

    assert quiet["fired_step"] is None
    assert quiet["first_held_step"] is None
    assert quiet["waited_s"] is None
    assert quiet["declared_dwell_s"] == pytest.approx(0.2)


def test_comparing_a_run_against_itself_is_all_zero_deltas(run):
    payload = compare(run, run)

    assert payload["same_model"] is True
    assert [row["delta_s"] for row in payload["activities"]] == [0.0] * 5
    assert [row["name"] for row in payload["activities"]] == _names(timeline(run))


def test_an_activity_entered_in_only_one_run_keeps_its_row(tmp_path):
    """A state that stopped being entered is the regression this view exists to show."""
    doc = views_schema()
    full = _run_dir(tmp_path, "run-full", doc, full_run_frames(doc))
    short = _run_dir(tmp_path, "run-short", doc, short_run_frames(doc))

    rows = {row["name"]: row for row in compare(full, short)["activities"]}
    assert rows["S_DONE"]["left_s"] is not None and rows["S_DONE"]["right_s"] is None
    assert rows["S_DONE"]["delta_s"] is None
    # The other direction keeps them too, appended after what did align.
    mirrored = compare(short, full)["activities"]
    assert [row["name"] for row in mirrored if row["left_s"] is None] == ["S_MOVE", "S_DONE"]


def test_a_run_with_no_archived_graph_says_so_and_invents_no_waits(tmp_path):
    """A live run degrades: 015 projects from strided frames, which cannot time an arming.

    Reporting 0 re-arms there would be worse than reporting nothing -- it reads as a fact.
    """
    doc = views_schema()
    live = _run_dir(tmp_path, "run-live", doc, full_run_frames(doc), archive=False)

    spans = timeline(live)
    assert spans["runtime_source"] == "projected"
    assert spans["spans"]

    payload = gates(live)
    assert payload["runtime_source"] == "projected"
    assert payload["gates"], "the gates the model declares are listed whatever the run knows"
    assert all(gate["waited_s"] is None for gate in payload["gates"])
    assert all(gate["rearm_count"] is None for gate in payload["gates"])


def test_the_timeline_narrows_to_one_design_iri_with_what_held_throughout(run):
    """The shape the Explore page's "Occurrences" action links to."""
    payload = timeline(run, S_MOVE)

    assert _names(payload) == ["S_MOVE", "S_MOVE"]
    assert payload["spans"][0]["satisfied"] == ["hold_height"]
    assert payload["spans"][1]["satisfied"] == []


@pytest.fixture
def dashboard(run, monkeypatch):
    """A server on an ephemeral port, rooted at the tree holding the archived run."""
    root = run.parents[3]
    monkeypatch.setattr(roots, "GENERATIONS", root)
    monkeypatch.setattr(roots, "WORKSPACE", root)
    monkeypatch.setattr(server, "LIFECYCLE", None)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.DashboardHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    host = f"http://127.0.0.1:{httpd.server_address[1]}"
    path = urllib.parse.quote(str(run.relative_to(root)))

    def get(query):
        with urllib.request.urlopen(f"{host}{query}") as response:
            return json.load(response)

    yield type("Dashboard", (), {"get": staticmethod(get), "path": path})
    httpd.shutdown()


def test_the_three_routes_answer_over_http(dashboard):
    assert len(dashboard.get(f"/api/run/timeline?path={dashboard.path}")["spans"]) == 5
    narrowed = dashboard.get(
        f"/api/run/timeline?path={dashboard.path}&iri={urllib.parse.quote(S_MOVE)}"
    )
    assert len(narrowed["spans"]) == 2
    assert len(dashboard.get(f"/api/run/gates?path={dashboard.path}")["gates"]) == 3
    both = f"left={dashboard.path}&right={dashboard.path}"
    assert dashboard.get(f"/api/run/compare?{both}")["same_model"] is True
