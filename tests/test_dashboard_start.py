# SPDX-License-Identifier: MPL-2.0
"""What the dashboard decides when it spawns a run: whether it may start, with which flags, and
how it is stopped. Exercised with a process of this test's own choosing rather than the CLI."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
from unittest import mock

import pytest

from motion_spec.dashboard import jobs, roots


@pytest.fixture
def generation(tmp_path, monkeypatch):
    """A hardware generation bundle rooted where the dashboard browses, since a run is named
    relative to it; whatever it left running is forgotten afterwards."""
    monkeypatch.setattr(roots, "GENERATIONS", tmp_path.parent)
    monkeypatch.setattr(roots, "WORKSPACE", tmp_path.parent)
    (tmp_path / "generated/contract").mkdir(parents=True)
    (tmp_path / "generated/contract/frame_layout.json").write_text(
        json.dumps({"platform": {"simulated": False}, "cameras": []})
    )
    yield tmp_path
    for key in [key for key, run in jobs.RUNNING.items() if run["generation"] == str(tmp_path)]:
        jobs.RUNNING.pop(key)


@pytest.fixture
def sleeping_run(generation):
    """A registered run of the generation that stays up until it is stopped."""
    process = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True
    )
    jobs.RUNNING[str(generation / "runs" / "run-test")] = {
        "process": process,
        "run_id": "run-test",
        "generation": str(generation),
    }
    yield process
    process.kill()


def test_hardware_never_gets_the_simulators_flags_and_unreachable_hardware_still_starts(
    generation, monkeypatch
):
    """Starting is the operator's call: the devices panel probes, the run does not."""
    with socket.create_server(("127.0.0.1", 0)) as listener:
        port = listener.getsockname()[1]
    (generation / "generated/source").mkdir(parents=True)
    (generation / "generated/source/robot.toml").write_text(
        f'[arm]\nip = "127.0.0.1"\nport = {port}\nconnection_timeout_ms = 200\n'
    )
    popen = mock.Mock(return_value=subprocess.Popen([sys.executable, "-c", ""]))
    monkeypatch.setattr(jobs.subprocess, "Popen", popen)
    answer = jobs.start_run(generation, {"headless": True})
    argv = list(popen.call_args.args[0])
    assert answer["run"].endswith(argv[argv.index("--run-id") + 1])
    assert "--headless" not in argv
    assert "--start-paused" not in argv


def test_the_real_robot_runs_one_thing_at_a_time(generation, sleeping_run):
    with pytest.raises(ValueError, match="already running"):
        jobs.start_run(generation, {})


def test_stopping_a_run_ends_the_process_group_it_was_started_in(generation, sleeping_run):
    status = jobs.stop_run(generation / "runs" / "run-test")
    assert sleeping_run.poll() is not None
    assert status["running"] is False
