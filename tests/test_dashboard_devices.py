# SPDX-License-Identifier: MPL-2.0
"""What a hardware generation offers the dashboard: its run options, its devices, its health.

Reachability is tested against a listener this process opens, so the only wire involved is the
loopback one. `check_health` is stubbed: the real checks configure CMake projects.
"""

from __future__ import annotations

import socket
import threading
import time

from motion_spec import devices
from motion_spec.dashboard import catalog, jobs
from motion_spec.health import HealthCheck

ROBOT_TOML = """
[collab-agents.kinova]
ip                    = "192.168.1.10"
user                  = "admin"
password              = "admin"
port                  = 10000
port_real_time        = 10001
session_timeout_ms    = 60000
connection_timeout_ms = 2000

[collab-agents.gripper]
port          = "/dev/ttyUSB1"
baudrate      = 115200
slave_address = 0x09

[kinova.wrist_ft]
port          = "ttyUSB0"
baudrate      = 19200

[ros.joint_states]
topic = "/joint_states"
rate  = 100.0

[poses.look-table]
position = [-0.34, -0.7, 0.4]
"""


def _generation(tmp_path, *, simulated, toml_text=None):
    """A generation bundle with only the two files these endpoints read."""
    (tmp_path / "generated/contract").mkdir(parents=True)
    (tmp_path / "generated/contract/frame_layout.json").write_text(
        f'{{"platform": {{"name": "kinova", "simulated": {str(simulated).lower()}}}}}'
    )
    if toml_text is not None:
        (tmp_path / "generated/source").mkdir(parents=True)
        (tmp_path / "generated/source/robot.toml").write_text(toml_text)
    return tmp_path


def _toml(tmp_path, text):
    path = tmp_path / "robot.toml"
    path.write_text(text)
    return path


def test_the_platform_says_whether_a_display_is_a_choice(tmp_path):
    assert catalog.is_simulated(_generation(tmp_path / "sim", simulated=True))
    assert not catalog.is_simulated(_generation(tmp_path / "real", simulated=False))
    # a directory that is no generation at all claims nothing
    assert not catalog.is_simulated(tmp_path / "nowhere")


def test_every_endpoint_the_config_names_is_what_the_data_says_it_is(tmp_path):
    assert devices.device_endpoints(_toml(tmp_path, ROBOT_TOML)) == [
        {
            "name": "collab-agents.kinova",
            "kind": "network",
            "host": "192.168.1.10",
            "ports": {"port": 10000, "port_real_time": 10001},
            "timeout_ms": 2000,
        },
        {"name": "collab-agents.gripper", "kind": "serial", "device": "/dev/ttyUSB1"},
        {"name": "kinova.wrist_ft", "kind": "serial", "device": "/dev/ttyUSB0"},
    ]


def test_a_topic_or_a_pose_is_nothing_to_connect_to(tmp_path):
    named = {endpoint["name"] for endpoint in devices.device_endpoints(_toml(tmp_path, ROBOT_TOML))}
    assert not any(name.startswith(("ros.", "poses.")) for name in named)


def test_a_host_with_no_port_is_still_a_device_to_name(tmp_path):
    endpoints = devices.device_endpoints(_toml(tmp_path, '[arm.base]\nip = "10.0.0.2"\n'))
    assert endpoints == [
        {
            "name": "arm.base",
            "kind": "network",
            "host": "10.0.0.2",
            "ports": {},
            "timeout_ms": devices.CONNECT_TIMEOUT_MS,
        }
    ]


def test_a_serial_port_written_as_a_path_is_used_as_written(tmp_path):
    endpoints = devices.device_endpoints(_toml(tmp_path, '[ft]\nport = "/dev/serial/by-id/ft"\n'))
    assert endpoints[0]["device"] == "/dev/serial/by-id/ft"


def test_a_generation_that_archived_no_config_has_nothing_to_probe(tmp_path):
    generation = _generation(tmp_path, simulated=False)
    assert devices.probe_devices(generation) == {"devices": [], "config": None}


def test_a_port_something_listens_on_answers(tmp_path):
    with socket.create_server(("127.0.0.1", 0)) as listener:
        port = listener.getsockname()[1]
        generation = _generation(
            tmp_path, simulated=False, toml_text=f'[arm]\nip = "127.0.0.1"\nport = {port}\n'
        )
        report = devices.probe_devices(generation)
    device = report["devices"][0]
    assert report["config"].endswith("generated/source/robot.toml")
    assert device["ok"] is True
    assert device["ports"] == [{"port": port, "ok": True, "detail": None}]


def test_a_port_nothing_listens_on_says_why(tmp_path):
    with socket.create_server(("127.0.0.1", 0)) as listener:
        port = listener.getsockname()[1]
    # the listener is closed: the port is now one nobody answers on
    generation = _generation(
        tmp_path, simulated=False, toml_text=f'[arm]\nip = "127.0.0.1"\nport = {port}\n'
    )
    device = devices.probe_devices(generation)["devices"][0]
    assert device["ok"] is False
    assert device["ports"][0]["detail"]


def test_a_probe_never_carries_what_the_config_logs_in_with(tmp_path):
    generation = _generation(
        tmp_path,
        simulated=False,
        toml_text=(
            '[arm]\nip = "127.0.0.1"\nuser = "admin"\npassword = "hunter2"\n'
            "port = 1\nconnection_timeout_ms = 200\n"
        ),
    )
    assert "hunter2" not in repr(devices.probe_devices(generation))


def test_a_serial_device_nobody_plugged_in_is_not_reachable(tmp_path):
    generation = _generation(
        tmp_path, simulated=False, toml_text='[gripper]\nport = "/dev/nothing-here"\n'
    )
    device = devices.probe_devices(generation)["devices"][0]
    assert device == {
        "name": "gripper",
        "kind": "serial",
        "device": "/dev/nothing-here",
        "ok": False,
        "detail": "no such device",
    }


def _health(monkeypatch, checks, *, delay=0.0):
    """Stand in for the real checks, which configure CMake projects to answer."""
    monkeypatch.setattr(jobs, "HEALTH", {"checks": None, "stamp": None, "thread": None})

    def fake(profiles, targets=()):
        time.sleep(delay)
        return checks

    monkeypatch.setattr("motion_spec.health.check_health", fake)


CHECKS = [
    HealthCheck("base", "rdflib", "Python module", "/usr/lib/rdflib", True, "pip install"),
    HealthCheck("build", "cmake", "executable", None, False, "apt install cmake"),
]


def _progress(report):
    """How far the checks have got. The report also carries the environment, which is this
    machine's own versions and roots -- not something a run of the checks can be asserted on."""
    return {key: report[key] for key in ("running", "checks", "stamp")}


def test_the_checks_are_run_once_and_then_remembered(monkeypatch):
    _health(monkeypatch, CHECKS, delay=0.2)
    assert _progress(jobs.health_report()) == {"running": True, "checks": None, "stamp": None}
    jobs.HEALTH["thread"].join(5)
    report = jobs.health_report()
    assert report["running"] is False
    assert report["stamp"] > 0
    reported = report["checks"][1]
    assert {key: reported[key] for key in ("profile", "dependency", "what", "path", "ok")} == {
        "profile": "build",
        "dependency": "cmake",
        "what": "executable",
        "path": None,
        "ok": False,
    }
    assert reported["detail"] == "apt install cmake"
    # nothing was started again: the answer is the one already taken
    assert not jobs.HEALTH["thread"].is_alive()


def test_a_re_check_runs_them_again(monkeypatch):
    _health(monkeypatch, CHECKS)
    jobs.health_report()
    jobs.HEALTH["thread"].join(5)
    first = jobs.HEALTH["thread"]
    jobs.health_report(refresh=True)
    assert jobs.HEALTH["thread"] is not first


def test_a_report_asked_for_while_they_run_says_so(monkeypatch):
    _health(monkeypatch, CHECKS, delay=0.5)
    jobs.health_report()
    assert _progress(jobs.health_report()) == {"running": True, "checks": None, "stamp": None}
    assert isinstance(jobs.HEALTH["thread"], threading.Thread)
    jobs.HEALTH["thread"].join(5)
