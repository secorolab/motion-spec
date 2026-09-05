# SPDX-License-Identifier: MPL-2.0
"""What the dashboard does before, and instead of, starting a run.

These are the paths that spawn: they are exercised with a process of this test's own choosing
rather than the CLI, so what is under test is the dashboard's decisions -- whether a run may
start at all, what it names, how it is stopped -- and never the runner behind them.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time

from motion_spec.dashboard import jobs, roots
from motion_spec.devices import probe_devices, unreachable, where

HARDWARE_TOML = """
[arm]
ip                    = "127.0.0.1"
port                  = {port}
connection_timeout_ms = 200
"""


def _generation(tmp_path, *, simulated, toml_text=None, monkeypatch=None):
    """A generation bundle holding the two files these decisions read.

    Rooted where the dashboard browses, because naming a run means naming it relative to that.
    """
    if monkeypatch is not None:
        monkeypatch.setattr(roots, "GENERATIONS", tmp_path.parent)
        monkeypatch.setattr(roots, "WORKSPACE", tmp_path.parent)
    (tmp_path / "generated/contract").mkdir(parents=True)
    (tmp_path / "generated/contract/frame_layout.json").write_text(
        json.dumps({"platform": {"simulated": simulated}, "cameras": []})
    )
    if toml_text is not None:
        (tmp_path / "generated/source").mkdir(parents=True)
        (tmp_path / "generated/source/robot.toml").write_text(toml_text)
    return tmp_path


def _dead_port() -> int:
    """A port nobody answers on: bound to find a free number, then given back."""
    import socket

    with socket.create_server(("127.0.0.1", 0)) as listener:
        return listener.getsockname()[1]


def _sleeping_job(generation):
    """Register a run of this generation that stays up until it is stopped."""
    process = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True
    )
    jobs.RUNNING[str(generation)] = {"process": process, "run_id": "run-test"}
    return process


def test_a_run_of_something_that_is_not_a_generation_is_refused(tmp_path):
    try:
        jobs.start_run(tmp_path, {})
    except ValueError as refused:
        assert "not a generation" in str(refused)
    else:
        raise AssertionError("a directory with no contract is not a generation")


def test_hardware_that_does_not_answer_still_starts_the_run(tmp_path, monkeypatch):
    """Starting is the operator's call: the devices panel probes, the run does not."""
    generation = _generation(
        tmp_path,
        simulated=False,
        toml_text=HARDWARE_TOML.format(port=_dead_port()),
        monkeypatch=monkeypatch,
    )
    started = {}
    monkeypatch.setattr(jobs.subprocess, "Popen", lambda *a, **k: _record(started, a, k))
    try:
        answer = jobs.start_run(generation, {})
    finally:
        jobs.RUNNING.pop(str(generation), None)
    assert "--run-id" in started["argv"]
    assert answer["run"].endswith(started["argv"][started["argv"].index("--run-id") + 1])
    # Hardware takes neither of the simulator's options, whatever the browser posted.
    assert "--headless" not in started["argv"]
    assert "--start-paused" not in started["argv"]


def test_a_real_run_records_the_cameras_that_name_a_topic(tmp_path, monkeypatch):
    """The standard view is the simulator's; hardware records what ROS can hand it."""
    generation = _generation(
        tmp_path,
        simulated=False,
        toml_text=HARDWARE_TOML.format(port=_dead_port()),
        monkeypatch=monkeypatch,
    )
    (generation / "generated/contract/frame_layout.json").write_text(
        json.dumps(
            {
                "platform": {"simulated": False},
                "cameras": [{"id": "rk", "topic": "/cameras/rk/image_raw"}, {"id": "wrist"}],
            }
        )
    )
    started = {}
    monkeypatch.setattr(jobs.subprocess, "Popen", lambda *a, **k: _record(started, a, k))
    try:
        answer = jobs.start_run(generation, {"cameras": ["default", "rk", "wrist"]})
    finally:
        jobs.RUNNING.pop(str(generation), None)
    assert answer["recording"] == ["rk"]
    assert started["argv"][started["argv"].index("--record") + 1] == "rk"
    assert started["argv"].count("--record") == 1


def test_a_simulation_starts_paused_with_its_run_named(tmp_path, monkeypatch):
    generation = _generation(
        tmp_path, simulated=True, toml_text=HARDWARE_TOML.format(port=1), monkeypatch=monkeypatch
    )
    started = {}
    monkeypatch.setattr(jobs.subprocess, "Popen", lambda *a, **k: _record(started, a, k))
    try:
        answer = jobs.start_run(generation, {})
    finally:
        jobs.RUNNING.pop(str(generation), None)
    assert "--run-id" in started["argv"]
    assert answer["run"].endswith(started["argv"][started["argv"].index("--run-id") + 1])
    assert "--start-paused" in started["argv"]


def test_a_generation_already_running_does_not_start_a_second_run(tmp_path):
    generation = _generation(tmp_path, simulated=True)
    process = _sleeping_job(generation)
    try:
        jobs.start_run(generation, {})
    except ValueError as refused:
        assert "already running" in str(refused)
    else:
        raise AssertionError("one generation runs once at a time")
    finally:
        process.kill()
        jobs.RUNNING.pop(str(generation), None)


def test_stopping_a_run_ends_the_process_group_it_was_started_in(tmp_path):
    generation = _generation(tmp_path, simulated=True)
    process = _sleeping_job(generation)
    try:
        status = jobs.stop_run(generation)
        assert process.poll() is not None
        assert status["running"] is False
    finally:
        process.kill()
        jobs.RUNNING.pop(str(generation), None)


def test_stopping_what_is_not_running_says_so(tmp_path):
    generation = _generation(tmp_path, simulated=True)
    try:
        jobs.stop_run(generation)
    except ValueError as refused:
        assert "running" in str(refused)
    else:
        raise AssertionError("there is nothing to stop")


def test_a_probe_reports_every_endpoint_at_once(tmp_path):
    """Three dead addresses cost one timeout, not three: they are knocked on together."""
    ports = [_dead_port() for _ in range(3)]
    toml_text = "".join(
        f'[arm{index}]\nip = "127.0.0.1"\nport = {port}\nconnection_timeout_ms = 400\n'
        for index, port in enumerate(ports)
    )
    generation = _generation(tmp_path, simulated=False, toml_text=toml_text)
    started = time.monotonic()
    report = probe_devices(generation)
    assert len(unreachable(report)) == 3
    assert time.monotonic() - started < 1.0


def test_a_device_is_reported_by_where_it_was_looked_for(tmp_path):
    port = _dead_port()
    generation = _generation(tmp_path, simulated=False, toml_text=HARDWARE_TOML.format(port=port))
    assert where(unreachable(probe_devices(generation))[0]) == f"127.0.0.1:{port}"


# The real one, kept before any test replaces the name it is reached by.
_POPEN = subprocess.Popen


def _record(started: dict, argv, kwargs):
    """Stand in for the runner: remember the command, and run something that ends at once."""
    started["argv"] = list(argv[0])
    return _POPEN([sys.executable, "-c", ""], **kwargs)
