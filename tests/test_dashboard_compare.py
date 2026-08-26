# SPDX-License-Identifier: MPL-2.0
"""Cross-generation comparison: did this change alter behaviour, and how?

The first case is the one everything else rests on -- a generation against itself has to
diff to nothing. Generation provenance moves on every regenerate, so a diff that leaves it
in reports a change for every run, and every figure below it becomes untrustworthy.

The design fixtures here therefore carry that provenance deliberately: an activity, a
generatedAtTime, and the file:// location of the output, each different per generation,
exactly as a real generated graph carries them.
"""

from __future__ import annotations

import json
import math
import threading
import urllib.parse
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
from frame_log_fixture import write_frame_log_pb
from support import _hash_doc
from test_dashboard_views import (
    MODEL,
    S_DONE,
    full_run_frames,
    model_jsonld,
    short_run_frames,
    views_schema,
)

from motion_spec.dashboard import roots, server
from motion_spec.dashboard.compare import compare_generations, model_diff
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.runtime_graph import write_runtime_ttl

DSLPROV = "https://secorolab.github.io/motion-spec-dsl/provenance/"
PROV = "http://www.w3.org/ns/prov#"
XSD = "http://www.w3.org/2001/XMLSchema#"
CSTR_HDL = "https://comp-rob2b.github.io/metamodels/task/constraint-handler#"
FSM = "https://secorolab.github.io/metamodels/behaviour/fsm#"
CTRL_Z = f"{MODEL}ctrl_z"
BAND = "settle_band"
MANIFEST = Path("generated/model/test-app.ld.json")


def compare_schema() -> dict:
    """The views fixture with a gate that judges the one quantity the run records.

    Without a band on a signal there is nothing for the signal axis to be restricted *to*,
    and a self-comparison would pass by having compared nothing.
    """
    doc = views_schema()
    doc["constants"] = [{"id": BAND, "value": 0.02, "uri": f"{MODEL}settle-band"}]
    doc["by_motion"]["move"]["monitors"][0].update(
        {"tolerance_signal": BAND, "operand_ids": ["dist"]}
    )
    doc["schema_hash"] = _hash_doc(doc)
    return doc


def _frames(doc: dict, short: bool) -> list[dict]:
    """A run's frames with the gated quantity given something to do."""
    recorded = short_run_frames(doc) if short else full_run_frames(doc)
    for frame in recorded:
        frame["q0"] = 0.03 * math.sin(frame["step"] / 3)
    return recorded


def _design(root: Path, stamp: str, gain: float, end: str) -> dict:
    """A design graph the way a generated one looks: the model, plus its generation provenance.

    The provenance names this generation's own output path and the moment it ran, so two
    generations of one source are never byte-identical and a diff keeping it never empties.
    """
    doc = model_jsonld()
    doc["@graph"] = [node for node in doc["@graph"] if node["@id"] != S_DONE]
    doc["@graph"] += [
        {"@id": end, "@type": f"{FSM}State"},
        {
            "@id": CTRL_Z,
            "@type": f"{CSTR_HDL}Controller",
            f"{CSTR_HDL}proportional-gain": gain,
            f"{CSTR_HDL}constraint": {"@id": f"{MODEL}hold_height"},
        },
        {
            "@id": f"{DSLPROV}activity/jsonld_generation/test",
            "@type": f"{PROV}Activity",
            f"{PROV}endedAtTime": {"@type": f"{XSD}dateTime", "@value": stamp},
        },
        {
            "@id": f"{DSLPROV}entity/app_manifest/test-app.ld.json",
            "@type": f"{PROV}Entity",
            f"{PROV}wasGeneratedBy": {"@id": f"{DSLPROV}activity/jsonld_generation/test"},
            f"{PROV}generatedAtTime": {"@type": f"{XSD}dateTime", "@value": stamp},
            f"{PROV}atLocation": {"@id": (root / MANIFEST).as_uri()},
        },
    ]
    return doc


def _archive(root: Path, name: str, doc: dict, recorded: list[dict]) -> None:
    """One run under a generation: its log, its own copy of the model, its runtime graph."""
    run = root / "runs" / name
    (run / "logs").mkdir(parents=True)
    (run / "model").mkdir()
    (run / "model" / MANIFEST.name).write_text((root / MANIFEST).read_text())
    log = run / "logs" / "frame_log.pb"
    write_frame_log_pb(log, doc, recorded)
    (run / "manifest.json").write_text(
        json.dumps(
            {
                "run_id": name,
                "files": {"frame_log": "logs/frame_log.pb", "model": f"model/{MANIFEST.name}"},
            }
        )
    )
    contract = frame_log_pb.read_contract(log)
    write_runtime_ttl(run, list(frame_log_pb.frame_records(log, contract)))


def build(tmp_path, name, *, stamp, gain=200.0, end=S_DONE, short=False, runs=True) -> Path:
    """A generation directory the way the dashboard reads one: a design graph under
    generated/model, its contract beside it, and its archived runs under runs/."""
    doc = compare_schema()
    root = tmp_path / name / "20260825T000000Z"
    (root / MANIFEST.parent).mkdir(parents=True)
    (root / MANIFEST).write_text(json.dumps(_design(root, stamp, gain, end)))
    (root / "generated" / "contract").mkdir(parents=True)
    (root / "generated" / "contract" / "frame_layout.json").write_text(
        json.dumps(build_frame_layout(doc))
    )
    if runs:
        _archive(root, "run-1", doc, _frames(doc, short))
    return root


def _deltas(payload: dict) -> list:
    """Every number the payload claims moved, across all three axes."""
    signals = payload["signals"]
    return [
        value
        for group in (
            payload["coordination"]["activities"],
            payload["coordination"]["gates"],
            signals["saturation"],
            *(
                activity[kind]
                for activity in signals["activities"]
                for kind in ("signals", "contact", "modes")
            ),
        )
        for row in group
        for key, value in row.items()
        if key.startswith("delta")
    ]


def _compare(before: Path, after: Path) -> dict:
    return compare_generations(before, after, before / "runs/run-1", after / "runs/run-1")


@pytest.fixture
def left(tmp_path):
    return build(tmp_path, "left", stamp="2026-08-25T00:00:00+00:00")


def test_a_generation_compared_against_itself_shows_no_change(left):
    """The case everything else rests on: nothing generation-incidental reaches the diff."""
    payload = _compare(left, left)

    assert payload["comparable"] == {
        "same_model": True,
        "same_schema_hash": True,
        "same_platform": True,
        "both_completed": True,
        "notes": [],
    }
    model = payload["model"]
    assert (model["values"], model["added"], model["removed"], model["structure"]) == (
        [],
        [],
        [],
        [],
    )
    assert payload["coordination"]["order_differs"] is False
    assert set(_deltas(payload)) <= {0, None}
    # Not vacuous: the signal axis really did compare the signal the gate judges.
    watched = {row["signal"] for a in payload["signals"]["activities"] for row in a["signals"]}
    assert watched == {"dist"}


def test_two_generations_of_one_source_differ_only_in_provenance(tmp_path):
    """Regenerating is not a change: different paths, different stamps, the same model."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00")
    after = build(tmp_path, "after", stamp="2026-08-25T09:30:00+00:00")

    # The two files really are different, or this proves nothing about what was stripped.
    assert (before / MANIFEST).read_text() != (after / MANIFEST).read_text()

    diff = model_diff(before, after)
    assert (diff["values"], diff["added"], diff["removed"], diff["structure"]) == ([], [], [], [])
    assert diff["same_model"] is True


def test_the_model_diff_is_over_the_graph_not_the_text(tmp_path):
    """A reordered file is not a change -- the STOP a text diff would walk straight into."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00")
    after = build(tmp_path, "after", stamp="2026-08-25T00:00:00+00:00")
    shuffled = json.loads((after / MANIFEST).read_text())
    shuffled["@graph"].reverse()
    (after / MANIFEST).write_text(json.dumps(shuffled))

    assert model_diff(before, after)["values"] == []


def test_one_changed_gain_is_one_value_row(tmp_path):
    """The common case, and the one that has to be the most legible thing in the output."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00", gain=200.0)
    after = build(tmp_path, "after", stamp="2026-08-25T09:30:00+00:00", gain=40.0)

    diff = model_diff(before, after)
    assert (diff["added"], diff["removed"], diff["structure"]) == ([], [], [])
    assert diff["values"] == [
        {
            "iri": CTRL_Z,
            "name": "ctrl_z",
            "predicate": f"{CSTR_HDL}proportional-gain",
            "property": "proportional-gain",
            "left": 200.0,
            "right": 40.0,
        }
    ]


def test_different_models_are_said_to_be_different_and_still_align(tmp_path):
    """Refusing is unhelpful and pretending is worse: name it, then show what lines up."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00")
    after = build(tmp_path, "after", stamp="2026-08-25T09:30:00+00:00", end=f"{MODEL}S_FINISH")

    payload = _compare(before, after)
    assert payload["comparable"]["same_model"] is False
    assert payload["comparable"]["notes"]
    aligned = [row for row in payload["coordination"]["activities"] if row["delta_s"] == 0.0]
    assert [row["name"] for row in aligned] == ["S_START", "S_MOVE", "S_HOLD", "S_MOVE"]


def test_an_activity_entered_twice_is_two_rows_not_a_sum(tmp_path):
    """A re-entry that appears or vanishes is a behaviour change, often the interesting one."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00")
    after = build(tmp_path, "after", stamp="2026-08-25T09:30:00+00:00", short=True)

    rows = [
        row
        for row in _compare(before, after)["coordination"]["activities"]
        if row["name"] == "S_MOVE"
    ]
    assert len(rows) == 2
    assert [row["right_s"] for row in rows] == [rows[0]["left_s"], None]
    assert rows[1]["delta_s"] is None


def test_a_run_that_stopped_short_is_not_completed(tmp_path):
    """It never occupied the element its own contract names as the end."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00")
    after = build(tmp_path, "after", stamp="2026-08-25T09:30:00+00:00", short=True)

    payload = _compare(before, after)
    assert payload["comparable"]["both_completed"] is False
    assert any("did not reach its end state" in note for note in payload["comparable"]["notes"])
    assert payload["coordination"]["left_reached_end"] is True


def test_two_generations_with_no_runs_still_have_a_model_diff(tmp_path):
    """Generations nobody ran are most of the corpus; a gain change is still worth seeing."""
    before = build(tmp_path, "before", stamp="2026-08-25T00:00:00+00:00", runs=False)
    after = build(tmp_path, "after", stamp="2026-08-25T09:30:00+00:00", gain=40.0, runs=False)

    payload = compare_generations(before, after)
    assert len(payload["model"]["values"]) == 1
    assert payload["coordination"]["activities"] == []
    assert any("no run picked" in note for note in payload["comparable"]["notes"])


@pytest.fixture
def dashboard(left, tmp_path, monkeypatch):
    monkeypatch.setattr(roots, "GENERATIONS", tmp_path)
    monkeypatch.setattr(roots, "WORKSPACE", tmp_path)
    monkeypatch.setattr(server, "LIFECYCLE", None)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.DashboardHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    host = f"http://127.0.0.1:{httpd.server_address[1]}"

    def get(query):
        with urllib.request.urlopen(f"{host}{query}") as response:
            return json.load(response)

    yield type("Dashboard", (), {"get": staticmethod(get), "root": tmp_path, "generation": left})
    httpd.shutdown()


def test_the_route_answers_a_deep_linked_comparison(dashboard):
    """Two generations and two runs in the URL, so a comparison can be pasted to a colleague."""
    path = urllib.parse.quote(str(dashboard.generation.relative_to(dashboard.root)))
    run = urllib.parse.quote(f"{dashboard.generation.relative_to(dashboard.root)}/runs/run-1")
    payload = dashboard.get(f"/api/compare?left={path}&left_run={run}&right={path}&right_run={run}")

    assert payload["comparable"]["same_model"] is True
    assert payload["model"]["values"] == []
    assert payload["signals"]["restricted_to"]
    assert [row["name"] for row in payload["coordination"]["activities"]] == [
        "S_START",
        "S_MOVE",
        "S_HOLD",
        "S_MOVE",
        "S_DONE",
    ]
