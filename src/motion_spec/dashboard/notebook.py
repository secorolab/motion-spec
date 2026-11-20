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


def jupyter_server(dashboard_port: int) -> dict:
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
    # own settings directory, so ~/.jupyter stays untouched
    settings = Path(tempfile.gettempdir()) / "motion-spec-lab-settings"
    themes = settings / "@jupyterlab" / "apputils-extension"
    themes.mkdir(parents=True, exist_ok=True)
    (themes / "themes.jupyterlab-settings").write_text(
        json.dumps({"theme": "JupyterLab Dark", "adaptive-theme": False}, indent=1)
    )
    framing = json.dumps(
        {
            "headers": {
                "Content-Security-Policy": (
                    f"frame-ancestors 'self' http://127.0.0.1:{dashboard_port}"
                    f" http://localhost:{dashboard_port}"
                )
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
    # lets `dashboard -k` find a lab its killed dashboard left behind
    JUPYTER["pidfile"] = roots.pidfile(dashboard_port, "lab")
    JUPYTER["pidfile"].write_text(f"{JUPYTER['process'].pid}\n")
    JUPYTER["url"] = f"http://127.0.0.1:{port}/lab?token={token}"
    JUPYTER["base"] = f"http://127.0.0.1:{port}"
    JUPYTER["token"] = token
    return {"url": JUPYTER["url"], "root": str(roots.WORKSPACE)}


def stop_jupyter() -> None:
    """Take the embedded lab down with the dashboard that started it."""
    process = JUPYTER.get("process")
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(5)
        except subprocess.TimeoutExpired:
            process.kill()
    if JUPYTER.get("pidfile") is not None:
        JUPYTER["pidfile"].unlink(missing_ok=True)


def run_notebook(run_dir: Path, dashboard_port: int) -> dict:
    """Seed a notebook in the dashboard's store that loads the run's signals, and open it in
    JupyterLab."""
    jupyter_server(dashboard_port)
    notebook = (
        roots.GENERATIONS
        / roots.STATE_REL
        / run_dir.relative_to(roots.GENERATIONS)
        / "analysis.ipynb"
    )
    if not notebook.exists():
        cells = [
            [
                "from pathlib import Path\n",
                "\n",
                "from motion_spec.dashboard.replay import plot_series, replay_metadata\n",
                "\n",
                f"run = Path({str(run_dir)!r})\n",
                "meta = replay_metadata(run)\n",
                "meta['frames'], meta['duration'], len(meta['signals'])",
            ],
            ["# every signal this run can answer for\n", "meta['signals'][:20]"],
            [
                "import matplotlib.pyplot as plt\n",
                "\n",
                "names = meta['constraints'][0]['tracking'] or meta['signals'][:1]\n",
                "data = plot_series(run, names)\n",
                "step = data['sample_step']\n",
                "\n",
                "figure, axes = plt.subplots(figsize=(7, 3))\n",
                "for name, series in data['signals'].items():\n",
                "    axes.plot(\n",
                "        [point * step for point in range(len(series))], series, label=name, lw=1\n",
                "    )\n",
                "axes.set_xlabel('frame')\n",
                "axes.legend(fontsize=7)\n",
                "figure.tight_layout()\n",
                "figure.savefig('plot.pdf')  # vector, for LaTeX",
            ],
        ]
        template = {
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
        notebook.parent.mkdir(parents=True, exist_ok=True)
        notebook.write_text(json.dumps(template, indent=1))
    relative = notebook.relative_to(roots.WORKSPACE)
    return {"url": f"{JUPYTER['base']}/lab/tree/{relative}?token={JUPYTER['token']}"}
