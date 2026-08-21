# SPDX-License-Identifier: MPL-2.0
"""Generation/run discovery, and what counts as a live run."""

from __future__ import annotations

import json
import shutil

from motion_spec.dashboard import server
from motion_spec.dashboard.runs import GenerationCatalog, GenerationInfo, RunInfo
from motion_spec.generation.artifacts import build_frame_layout

from dashboard_fixture import CONSTRAINT, CTRL, MONITOR, QUANTITY, schema
from frame_log_fixture import flat_frame, write_frame_log_pb
from support import _hash_doc

REC_NS = "https://secorolab.github.io/metamodels/rec#"

SETTLED = "https://example.test/settled"
ROBMOT = """
guarded-motion (ns=demo) move {
    while {
        hold-height: dist within <target-height>,
    }
    until {
        settled: err-x below <tol-height>,
    }
}
"""


def _rec(status):
    """A REC run document with one lifecycle state, as the runner writes it."""
    return {
        "@context": {"rec": REC_NS, "run-id": {"@id": "rec:run-id"}},
        "@id": "https://example.test/run/r1",
        "@type": f"rec:{status}",
        "run-id": "r1",
    }


def _generation(tmp_path, name="pick_place_single", stamp="20260811T000000Z"):
    gen = tmp_path / name / stamp
    (gen / "generated" / "contract").mkdir(parents=True)
    (gen / "generated" / "contract" / "frame_layout.json").write_text(
        json.dumps(build_frame_layout(schema()))
    )
    return gen


def _run(gen, run_id="run-1", status="RunningRun", log=b"frames"):
    run = gen / "runs" / run_id
    (run / "logs").mkdir(parents=True)
    (run / "logs" / "frame_log.pb").write_bytes(log)
    (run / "rec.ld.json").write_text(json.dumps(_rec(status)))
    (run / "manifest.json").write_text(json.dumps({"run_id": run_id}))
    return RunInfo(run)


def test_the_catalog_finds_generations_by_their_contract(tmp_path):
    _generation(tmp_path)
    _generation(tmp_path, stamp="20260811T010000Z")
    _generation(tmp_path, name="admittance_arc_single")
    (tmp_path / "not_a_generation").mkdir()

    generations = GenerationCatalog([tmp_path]).generations()
    assert len(generations) == 3
    assert [g.model for g in generations][0] == "pick_place_single"  # newest first
    assert generations[0].timestamp == "20260811T010000Z"
    assert generations[0].built is False
    assert generations[0].layout.schema_hash == schema()["schema_hash"]


def test_a_run_still_ticking_is_live(tmp_path):
    """The REC vocabulary says RUNNING, not STARTED -- a run whose log is still growing is
    live regardless of which non-terminal state it sits in."""
    run = _run(_generation(tmp_path))
    assert run.status == "RUNNING"
    assert run.is_live() is True
    assert run.manifest == {"run_id": "run-1"}


def test_a_finished_run_is_not_live_however_fresh_its_log(tmp_path):
    run = _run(_generation(tmp_path), status="CompletedRun")
    assert run.status == "COMPLETED"
    assert run.is_live() is False


def test_a_run_killed_without_a_terminal_state_goes_stale(tmp_path):
    """Nothing writes a terminal state when a run is killed outright, so a stale log is what
    tells the two apart."""
    run = _run(_generation(tmp_path))
    assert run.is_live() is True
    assert run.is_live(within_s=-1.0) is False


def test_generations_expose_their_runs_newest_first(tmp_path):
    gen = _generation(tmp_path)
    _run(gen, run_id="run-1")
    _run(gen, run_id="run-2")

    runs = GenerationInfo(gen).runs
    assert [r.run_id for r in runs] == ["run-2", "run-1"]
    assert GenerationInfo(gen).runs[0].health is None  # no health sidecar written


def _contract_schema() -> dict:
    """The dashboard schema, with the identity a controller and a monitor slot carry in a header."""
    doc = schema()
    doc["pools"].update({"quantities": 2, "wrenches": 1})
    doc["quantities"] = [
        {"index": 0, "id": "dist", "uri": QUANTITY},
        {"index": 1, "id": "err_x", "uri": "https://example.test/err_x"},
    ]
    doc["spatial"]["wrenches"] = [
        {"index": 0, "id": "cmd_wrench", "uri": "https://example.test/cmd_wrench"}
    ]
    doc["constants"] = [
        {"id": "target_height", "source_id": "target_height", "value": 0.25},
        {"id": "tol_height", "source_id": "tol_height", "value": 0.01},
    ]
    doc["by_motion"]["move"]["controllers"] = [
        {
            "index": 0,
            "id": "ctrl_x",
            "uri": CTRL,
            "constraint": "hold-height",
            "constraint_uri": CONSTRAINT,
            "gains": {"proportional_gain": 12.0},
            "error_signal": "err_x",
            "output_signal": "cmd_wrench",
            "measured_signal": "dist",
            "setpoint_signal": "target_height",
            "tolerance_signal": "tol_height",
        }
    ]
    doc["by_motion"]["move"]["monitors"] = [
        {
            "index": 0,
            "id": "done_mon",
            "uri": MONITOR,
            "phase": "until",
            "constraint_ids": ["settled"],
            "constraint_uris": [SETTLED],
            "error_signal": "err_x",
        }
    ]
    doc["schema_hash"] = _hash_doc(doc)
    return doc


def _archived_run(tmp_path, *, vendored: bool):
    """A run archive whose log carries the contract, with its source vendored or generation-side."""
    doc = _contract_schema()
    run = tmp_path / "demo" / "20260821T000000Z" / "runs" / "run-1"
    (run / "logs").mkdir(parents=True)
    frame = flat_frame(
        doc, t=0.001, step=1, fsm_state=0, active_motion=0, last_event=-1, q0=0.5, q1=0.02
    )
    write_frame_log_pb(run / "logs" / "frame_log.pb", doc, [frame])
    files = {"frame_log": "logs/frame_log.pb"}
    if vendored:
        (run / "source").mkdir()
        (run / "source" / "demo.robmot").write_text(ROBMOT)
        files["sources"] = ["source/demo.robmot"]
    else:
        (run.parent.parent / "generated" / "source").mkdir(parents=True)
        (run.parent.parent / "generated" / "source" / "demo.robmot").write_text(ROBMOT)
    (run / "manifest.json").write_text(json.dumps({"run_id": "run-1", "files": files}))
    return run


def test_constraint_rows_are_read_off_the_log_header(tmp_path):
    """Constraint, phase, signals, gains and tolerance all come from the header the run wrote."""
    rows = {
        row["name"]: row
        for row in server.replay_data(_archived_run(tmp_path, vendored=True))["constraints"]
    }
    assert set(rows) == {"hold-height", "settled"}

    held = rows["hold-height"]
    assert (held["motion"], held["kind"], held["evaluator"]) == ("move", "controlled", CTRL)
    assert (held["line"], held["expression"]) == (4, "dist within <target-height>")
    assert held["error"] == ["err_x"]
    assert held["control"] == sorted(
        f"cmd_wrench.{part}"
        for part in ("force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z")
    )
    # The setpoint is a constant, so it is a band on the plot rather than a signal to fetch.
    assert held["between"] == ["dist", "target_height"] and held["tracking"] == ["dist"]
    assert held["setpoints"] == [{"label": "target_height", "value": 0.25}]
    assert held["gains"] == {"Kp": 12.0} and held["tolerance"] == 0.01
    assert held["window"] == [0, 0] and held["monitors"] == []

    settled = rows["settled"]
    assert (settled["kind"], settled["line"]) == ("monitored", 7)
    assert settled["monitors"] == ["done_mon.value", "done_mon.satisfied"]
    assert settled["error"] == ["err_x"]


def test_the_authored_line_falls_back_to_the_generation_source(tmp_path):
    run = _archived_run(tmp_path, vendored=False)
    rows = {row["name"]: row for row in server.replay_data(run)["constraints"]}
    assert rows["hold-height"]["expression"] == "dist within <target-height>"


def test_a_run_copied_out_of_its_generation_still_plots(tmp_path):
    """Nothing on the replay path resolves the generation: the log and the vendored source suffice."""
    copy = tmp_path / "run_copy"
    shutil.copytree(_archived_run(tmp_path, vendored=True), copy)

    data = server.replay_data(copy)
    assert data["generation"] is None
    rows = {row["name"]: row for row in data["constraints"]}
    assert rows["hold-height"]["line"] == 4
    assert rows["hold-height"]["error"] == ["err_x"] and "dist" in data["signals"]


def test_a_constraint_whose_source_is_gone_keeps_its_row(tmp_path):
    """The source line is display only, so losing it costs the expression, never the plot."""
    copy = tmp_path / "run_copy"
    shutil.copytree(_archived_run(tmp_path, vendored=False), copy)

    rows = {row["name"]: row for row in server.replay_data(copy)["constraints"]}
    assert set(rows) == {"hold-height", "settled"}
    assert rows["hold-height"]["expression"] is None and rows["hold-height"]["line"] is None
    assert rows["hold-height"]["error"] == ["err_x"]


def test_a_generation_counts_its_constraints_before_any_run(tmp_path, monkeypatch):
    monkeypatch.setattr(server, "GENERATIONS", tmp_path)
    gen = _generation(tmp_path)
    (gen / "generated" / "source").mkdir(parents=True)
    (gen / "generated" / "source" / "demo.robmot").write_text(ROBMOT)

    details = server.generation_details(gen)
    assert (details["authored_constraints"], details["motions"]) == (2, 1)
