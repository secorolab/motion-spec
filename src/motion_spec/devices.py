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
import tomllib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROBOT_TOML_REL = "generated/source/robot.toml"
CONNECT_TIMEOUT_MS = 1000


def endpoints(table: dict, name: str):
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
        # a bare name is taken under /dev/
        yield {"name": name, "kind": "serial", "device": port if "/" in port else f"/dev/{port}"}
    for key, value in table.items():
        if isinstance(value, dict):
            yield from endpoints(value, f"{name}.{key}" if name else key)


def probe_devices(generation_dir: Path) -> dict:
    """Try each endpoint one generation names once, now: TCP connect, or stat for serial.

    All of them at the same time. Every unanswered address costs its own connect timeout, and
    waiting them out one after another is how a refusal takes as long as the robot has ports.
    """
    toml_path = generation_dir / ROBOT_TOML_REL
    if not toml_path.is_file():
        return {"devices": [], "config": None}
    found = list(endpoints(tomllib.loads(toml_path.read_text()), ""))
    # flat: a probe waiting on another in the same pool can deadlock it
    jobs = [
        (index, port)
        for index, endpoint in enumerate(found)
        for port in (endpoint["ports"].values() if endpoint["kind"] == "network" else (None,))
    ]
    if not jobs:
        return {"devices": found, "config": str(toml_path)}
    with ThreadPoolExecutor(max_workers=min(16, len(jobs))) as pool:
        answers = list(pool.map(lambda job: probe(found[job[0]], job[1]), jobs))
    replies: dict[int, list] = {}
    for (index, _), answer in zip(jobs, answers):
        replies.setdefault(index, []).append(answer)
    devices = []
    for index, endpoint in enumerate(found):
        said = replies.get(index, [])
        if endpoint["kind"] != "network":
            devices.append({**endpoint, **said[0]})
        else:
            devices.append(
                {**endpoint, "ports": said, "ok": all(p["ok"] for p in said) if said else None}
            )
    return {"devices": devices, "config": str(toml_path)}


def probe(endpoint: dict, port: int | None) -> dict:
    """One address, tried once: a port on a host, or a device node."""
    if port is None:
        device = endpoint["device"]
        if not Path(device).exists():
            return {"ok": False, "detail": "no such device"}
        if not os.access(device, os.R_OK | os.W_OK):
            return {"ok": False, "detail": "device is not readable and writable by this user"}
        return {"ok": True, "detail": None}
    try:
        socket.create_connection((endpoint["host"], port), endpoint["timeout_ms"] / 1000).close()
    except OSError as exc:
        return {"port": port, "ok": False, "detail": str(exc)}
    return {"port": port, "ok": True, "detail": None}
