# SPDX-License-Identifier: MPL-2.0
"""What a hardware generation connects to, and whether it answers right now.

A real run drives devices over the network or a serial line, and a driver that cannot reach one
does not fail: it waits. So the endpoints its deployment config names are knocked on first --
by the CLI before it starts a run, and by the dashboard behind its Devices panel.

The wire only. A port that accepts a connection says nothing about the protocol behind it, and
the credentials in the config stay in the config: nothing here reads or reports them.
"""

from __future__ import annotations

import os
import socket
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import tomllib

ROBOT_TOML_REL = "generated/source/robot.toml"
# What a device says to wait for a connection, where its table says nothing.
CONNECT_TIMEOUT_MS = 1000


def _endpoints(table: dict, name: str):
    """Every endpoint one table tree names, as what it is rather than what it is called.

    A table with a host is reached over the network on every port it names; one whose port is a
    device path is reached over serial. Everything else -- topics, poses, tunings -- names
    nothing to connect to.
    """
    port = table.get("port")
    if name and isinstance(table.get("ip"), str):
        yield {
            "name": name,
            "kind": "network",
            "host": table["ip"],
            "ports": {
                key: value
                for key, value in table.items()
                if key.startswith("port") and isinstance(value, int) and not isinstance(value, bool)
            },
            "timeout_ms": table.get("connection_timeout_ms") or CONNECT_TIMEOUT_MS,
        }
    elif name and isinstance(port, str):
        # every driver here opens the path it is given; a bare name is taken under /dev/
        yield {"name": name, "kind": "serial", "device": port if "/" in port else f"/dev/{port}"}
    for key, value in table.items():
        if isinstance(value, dict):
            yield from _endpoints(value, f"{name}.{key}" if name else key)


def device_endpoints(toml_path: Path) -> list[dict]:
    """What the config connects to: derived from the data, never from section names."""
    with toml_path.open("rb") as fh:
        return list(_endpoints(tomllib.load(fh), ""))


def _tcp_probe(host: str, port: int, timeout_s: float) -> dict:
    """Whether something answers on one port, right now."""
    try:
        socket.create_connection((host, port), timeout_s).close()
    except OSError as exc:
        return {"port": port, "ok": False, "detail": str(exc)}
    return {"port": port, "ok": True, "detail": None}


def _serial_probe(device: str) -> dict:
    """Whether one serial device is there and this user may open it."""
    if not Path(device).exists():
        return {"ok": False, "detail": "no such device"}
    if not os.access(device, os.R_OK | os.W_OK):
        return {"ok": False, "detail": "device is not readable and writable by this user"}
    return {"ok": True, "detail": None}


def probe_devices(generation_dir: Path) -> dict:
    """Try each endpoint one generation names once, now: TCP connect, or stat for serial.

    All of them at the same time. Every unanswered address costs its own connect timeout, and
    waiting them out one after another is how a refusal takes as long as the robot has ports.
    """
    toml_path = generation_dir / ROBOT_TOML_REL
    if not toml_path.is_file():
        return {"devices": [], "config": None}
    endpoints = device_endpoints(toml_path)
    # One job per address, so a host with two ports is two jobs and every job is a leaf. Flat
    # on purpose: a probe that waits inside the pool for another probe in the same pool stops
    # dead as soon as the pool runs out of threads.
    jobs = [
        (index, port)
        for index, endpoint in enumerate(endpoints)
        for port in (endpoint["ports"].values() if endpoint["kind"] == "network" else (None,))
    ]
    if not jobs:
        return {"devices": [dict(endpoint) for endpoint in endpoints], "config": str(toml_path)}
    with ThreadPoolExecutor(max_workers=min(16, len(jobs))) as pool:
        answers = list(pool.map(lambda job: _probe(endpoints[job[0]], job[1]), jobs))
    replies: dict[int, list] = {}
    for (index, _), answer in zip(jobs, answers):
        replies.setdefault(index, []).append(answer)
    return {
        "devices": [
            _answered(endpoint, replies.get(index, [])) for index, endpoint in enumerate(endpoints)
        ],
        "config": str(toml_path),
    }


def _probe(endpoint: dict, port: int | None) -> dict:
    """One address, tried once: a port on a host, or a device node."""
    if port is None:
        return _serial_probe(endpoint["device"])
    return _tcp_probe(endpoint["host"], port, endpoint["timeout_ms"] / 1000)


def _answered(endpoint: dict, replies: list[dict]) -> dict:
    """One endpoint and what its addresses said."""
    if endpoint["kind"] != "network":
        return {**endpoint, **replies[0]}
    # A host named with no port to knock on is listed, and answers for nothing.
    return {**endpoint, "ports": replies, "ok": all(p["ok"] for p in replies) if replies else None}


def unreachable(report: dict) -> list[dict]:
    """The devices in a report that did not answer. A device with nothing to knock on did."""
    return [device for device in report["devices"] if device["ok"] is False]


def where(device: dict) -> str:
    """Where a device was looked for, as one line: host and ports, or the device node."""
    if device["kind"] != "network":
        return device["device"]
    return f"{device['host']}:{','.join(str(probe['port']) for probe in device['ports'])}"
