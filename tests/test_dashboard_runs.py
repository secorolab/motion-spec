# SPDX-License-Identifier: MPL-2.0
"""Generation/run discovery, and what counts as a live run."""

from __future__ import annotations

import json

from motion_spec.dashboard.runs import GenerationCatalog, GenerationInfo, RunInfo
from motion_spec.generation.artifacts import build_frame_layout

from dashboard_fixture import schema

REC_NS = "https://secorolab.github.io/metamodels/rec#"


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
