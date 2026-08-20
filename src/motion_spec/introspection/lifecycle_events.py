# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Best-effort local delivery of durable REC lifecycle transitions."""

from __future__ import annotations

import json
import os
import socket
from pathlib import Path


def socket_path() -> Path:
    """Return the per-user lifecycle socket location."""
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    return runtime / "motion-spec-lifecycle.sock"


def publish_lifecycle(run_dir: Path, run_id: str, status: str) -> None:
    """Notify a local listener after REC durably records a lifecycle transition."""
    message = json.dumps({"run": str(run_dir), "run_id": run_id, "status": status}).encode()
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as channel:
        try:
            channel.sendto(message, str(socket_path()))
        except OSError:
            return
