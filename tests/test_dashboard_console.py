# SPDX-License-Identifier: MPL-2.0
"""Tailing the terminal logs the dashboard shows, and what a page-started generation made.

The spawning paths themselves (`start_generate`, `start_run`) stay untested, as `start_run`
already is: what they launch is the CLI, and a test that runs one is not a unit test.
"""

from __future__ import annotations

import types

import pytest

from motion_spec.dashboard import server


def _job(monkeypatch, tmp_path, announced, name="gen-1", code=0):
    """A finished generation job whose log says where it put the generation."""
    log = tmp_path / f"{name}.log"
    log.write_text(f"parsing model\ngeneration: {announced}\nbuilding\n")
    monkeypatch.setattr(server, "GENERATIONS", tmp_path)
    monkeypatch.setitem(
        server.GENERATING,
        name,
        {
            "process": types.SimpleNamespace(poll=lambda: code, returncode=code, pid=1),
            "log": log,
            "generation": None,
        },
    )
    return name


def test_a_slice_of_a_log_nobody_wrote_is_empty(tmp_path):
    assert server.console_slice(tmp_path / "console.log", 0) == {"text": "", "offset": 0, "size": 0}


def test_the_first_slice_reads_the_whole_log(tmp_path):
    log = tmp_path / "console.log"
    log.write_text("starting\nrunning\n")
    slice_ = server.console_slice(log, 0)
    assert slice_["text"] == "starting\nrunning\n"
    assert slice_["offset"] == slice_["size"] == log.stat().st_size


def test_the_next_slice_reads_only_what_was_appended(tmp_path):
    log = tmp_path / "console.log"
    log.write_text("starting\n")
    first = server.console_slice(log, 0)
    with log.open("a") as fh:
        fh.write("running\n")
    tail = server.console_slice(log, first["offset"])
    assert tail["text"] == "running\n"
    assert tail["offset"] == log.stat().st_size


def test_a_slice_past_the_end_starts_the_log_over(tmp_path):
    """A log shorter than the offset asked for is a new log in the old file's place."""
    log = tmp_path / "console.log"
    log.write_text("run again\n")
    assert server.console_slice(log, 4096)["text"] == "run again\n"


def test_terminal_colors_reach_the_page_raw(tmp_path):
    """ANSI escapes pass through untouched; the page renders them as colors."""
    log = tmp_path / "console.log"
    log.write_text("\x1b[31m[mj_kdl INFO ] scene compiled\x1b[0m\n")
    slice_ = server.console_slice(log, 0)
    assert slice_["text"] == "\x1b[31m[mj_kdl INFO ] scene compiled\x1b[0m\n"
    assert slice_["offset"] == log.stat().st_size


def test_a_generation_means_the_log_of_the_run_the_dashboard_started(tmp_path):
    (tmp_path / "generated" / "model").mkdir(parents=True)
    (tmp_path / "generated" / "model" / "ir.json").write_text("{}")
    assert server.console_log_for(tmp_path) == tmp_path / server.RUN_LOG


def test_anything_else_means_the_console_a_run_archived(tmp_path):
    assert server.console_log_for(tmp_path) == tmp_path / "logs" / "console.log"


def test_a_job_learns_the_generation_the_cli_announced(monkeypatch, tmp_path):
    job = _job(monkeypatch, tmp_path, tmp_path / "model" / "stamp")
    status = server.generate_status(job)
    assert status == {"busy": False, "pid": None, "exit_code": 0, "generation": "model/stamp"}


def test_a_generation_announced_outside_the_root_is_not_one_to_open(monkeypatch, tmp_path):
    job = _job(monkeypatch, tmp_path, tmp_path.parent / "outside" / "stamp")
    assert server.generate_status(job)["generation"] is None


def test_asking_after_a_job_nobody_started(monkeypatch, tmp_path):
    with pytest.raises(ValueError):
        server.generate_status("nope")
    with pytest.raises(ValueError):
        server.generate_console("nope", 0)
