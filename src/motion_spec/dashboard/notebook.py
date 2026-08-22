# SPDX-License-Identifier: MPL-2.0

"""JupyterLab, started beside the dashboard and pointed at one run."""

from __future__ import annotations

import json
import secrets
import shutil
import socket
import subprocess
import tempfile
from pathlib import Path

from motion_spec.dashboard import roots

JUPYTER = {}


def lab_settings() -> Path:
    """A settings directory of our own, so the framed lab is dark without touching ~/.jupyter."""
    directory = Path(tempfile.gettempdir()) / "motion-spec-lab-settings"
    themes = directory / "@jupyterlab" / "apputils-extension"
    themes.mkdir(parents=True, exist_ok=True)
    (themes / "themes.jupyterlab-settings").write_text(
        json.dumps({"theme": "JupyterLab Dark", "adaptive-theme": False}, indent=1)
    )
    return directory


def jupyter_server() -> dict:
    """The embedded JupyterLab, started on first use and framed by this dashboard only."""
    if JUPYTER.get("process") and JUPYTER["process"].poll() is None:
        return {"url": JUPYTER["url"], "root": str(roots.WORKSPACE)}
    if shutil.which("jupyter") is None:
        raise ValueError(
            "JupyterLab is not installed. Install it with: pip install 'motion-spec[replay]'"
        )
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    token = secrets.token_urlsafe(24)
    settings = lab_settings()
    framing = json.dumps(
        {
            "headers": {
                "Content-Security-Policy": "frame-ancestors 'self' http://127.0.0.1:8080 http://localhost:8080"
            }
        }
    )
    JUPYTER["process"] = subprocess.Popen(
        [
            "jupyter",
            "lab",
            "--no-browser",
            f"--port={port}",
            f"--ServerApp.root_dir={roots.WORKSPACE}",
            f"--IdentityProvider.token={token}",
            f"--LabServerApp.user_settings_dir={settings}",
            f"--ServerApp.tornado_settings={framing}",
            "--ServerApp.disable_check_xsrf=True",
            "--ServerApp.open_browser=False",
        ],
        cwd=roots.WORKSPACE,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    JUPYTER["url"] = f"http://127.0.0.1:{port}/lab?token={token}"
    JUPYTER["base"] = f"http://127.0.0.1:{port}"
    JUPYTER["token"] = token
    return {"url": JUPYTER["url"], "root": str(roots.WORKSPACE)}


def stop_jupyter() -> None:
    """Take the embedded lab down with the dashboard that started it."""
    process = JUPYTER.get("process")
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(5)
    except subprocess.TimeoutExpired:
        process.kill()


def run_notebook(run_dir: Path) -> dict:
    """Seed a notebook beside the run that loads its signals, and open it in JupyterLab."""
    jupyter_server()
    notebook = run_dir / "analysis.ipynb"
    if not notebook.exists():
        notebook.write_text(json.dumps(NOTEBOOK_TEMPLATE(run_dir), indent=1))
    relative = notebook.relative_to(roots.WORKSPACE)
    return {"url": f"{JUPYTER['base']}/lab/tree/{relative}?token={JUPYTER['token']}"}


def NOTEBOOK_TEMPLATE(run_dir: Path) -> dict:
    """One notebook whose first cells load this run and draw one of its signals."""
    cells = [
        [
            "from pathlib import Path\n",
            "\n",
            "from motion_spec.dashboard.replay import plot_data, replay_data\n",
            "\n",
            f"run = Path({str(run_dir)!r})\n",
            "meta = replay_data(run)\n",
            "meta['frames'], meta['duration'], len(meta['signals'])",
        ],
        ["# every signal this run can answer for\n", "meta['signals'][:20]"],
        [
            "import matplotlib.pyplot as plt\n",
            "\n",
            "names = meta['constraints'][0]['tracking'] or meta['signals'][:1]\n",
            "data = plot_data(run, names)\n",
            "step = data['sample_step']\n",
            "\n",
            "figure, axes = plt.subplots(figsize=(7, 3))\n",
            "for name, series in data['signals'].items():\n",
            "    axes.plot([point * step for point in range(len(series))], series, label=name, lw=1)\n",
            "axes.set_xlabel('frame')\n",
            "axes.legend(fontsize=7)\n",
            "figure.tight_layout()\n",
            "figure.savefig('plot.pdf')  # vector, for LaTeX",
        ],
    ]
    return {
        "nbformat": 4,
        "nbformat_minor": 5,
        "metadata": {"kernelspec": {"name": "python3", "display_name": "Python 3"}},
        "cells": [
            {
                "cell_type": "code",
                "execution_count": None,
                "metadata": {},
                "outputs": [],
                "source": source,
            }
            for source in cells
        ],
    }
