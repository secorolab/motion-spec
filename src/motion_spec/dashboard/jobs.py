# SPDX-License-Identifier: MPL-2.0

"""What the dashboard starts, refuses to start, follows and stops.

A generation is made by one process and run by another; both are watched through their
terminal output.
"""

from __future__ import annotations

import dataclasses
import os
import platform
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from motion_spec.dashboard import roots
from motion_spec.dashboard.catalog import (
    generation_cameras,
    generation_platform,
    run_ended,
    run_recorded,
)
from motion_spec.dashboard.roots import HEADER_REL, RUN_LOG, trace
from motion_spec.setup import GENERATION_VARIABLE

RUNNING: dict[str, dict] = {}

RUNS_KEPT = 20


def run_status(run_dir: Path) -> dict:
    """Whether this run is in progress, and where its output is going.

    A run is over when its archive is written, not when its process exits.
    """
    started = RUNNING.get(str(run_dir))
    process = started["process"] if started else None
    pid = process.pid if process is not None and process.poll() is None else None
    busy = pid is not None
    live = busy and not (run_dir.is_dir() and run_ended(run_dir))
    trace(f"run_status {run_dir.name}: busy={busy} running={live}")
    recorded = run_recorded(run_dir)
    if recorded is None and started is not None:
        recorded = started.get("recorded")
    return {
        "id": run_dir.name,
        "path": str(run_dir.relative_to(roots.GENERATIONS)),
        "running": live,
        "busy": busy,
        "pid": pid,
        "run":str(run_dir.relative_to(roots.GENERATIONS)) if live else None,
        "exit_code": None if busy or process is None else process.returncode,
        "stopped": bool(started and started.get("stopped")),
        "log": str(Path(started["generation"]) / RUN_LOG) if started else None,
        "recorded": recorded,
    }


def generation_status(generation_dir: Path) -> dict:
    """Every run of this generation the dashboard started and still remembers, newest first."""
    runs = [
        run_status(Path(key))
        for key, started in sorted(RUNNING.items(), reverse=True)
        if started["generation"] == str(generation_dir)
    ]
    return {"running": any(run["running"] for run in runs), "runs": runs}


def start_run(generation_dir: Path, options: dict) -> dict:
    """Run a generation again, with the options `motion-spec run` takes for one."""
    if not (generation_dir / HEADER_REL).exists():
        raise ValueError("not a generation")
    simulated = bool(generation_platform(generation_dir).get("simulated"))
    if not simulated:
        other = next(
            (
                Path(key)
                for key, started in RUNNING.items()
                if started["process"].poll() is None
                and not generation_platform(Path(started["generation"])).get("simulated")
            ),
            None,
        )
        if other is not None:
            raise ValueError(f"the real robot is already running {other.name}")
    from motion_spec.generation.pipeline import new_id

    # named here so the page can open the run before its directory exists
    run_id = new_id("run")
    argv = [
        "motion-spec",
        "run",
        str(generation_dir),
        "--cwd",
        str(roots.WORKSPACE),
        "--run-id",
        run_id,
    ]
    recorded = options.get("log", True) is not False
    if not recorded:
        argv.append("--no-log")
    argv += ["--env", ENVIRONMENT["script"]] if ENVIRONMENT["script"] else ["--no-env"]
    recordable = {
        camera["id"]
        for camera in generation_cameras(generation_dir)
        if simulated or camera.get("topic")
    }
    if simulated:
        recordable.add("default")
    recording = [camera for camera in options.get("cameras") or () if camera in recordable]
    for camera in recording:
        argv += ["--record", camera]
    finished = sorted(
        key for key, started in RUNNING.items() if started["process"].poll() is not None
    )
    for key in finished[:-RUNS_KEPT]:
        RUNNING.pop(key)
    # Popen dups the fd for the child; ours would leak
    with (generation_dir / RUN_LOG).open("wb") as sink:
        process = subprocess.Popen(
            argv, cwd=roots.WORKSPACE, stdout=sink, stderr=sink, start_new_session=True
        )
    run_dir = generation_dir / "runs" / run_id
    RUNNING[str(run_dir)] = {
        "process": process,
        "run_id": run_id,
        "generation": str(generation_dir),
        "recorded": recorded,
    }
    return {
        **run_status(run_dir),
        "command": argv,
        "recording": recording,
        "run": str(run_dir.relative_to(roots.GENERATIONS)),
    }


STOP_GRACE_S = 5


def mark_stopped(run_dir: Path) -> None:
    """Record that this run's end was asked for, whoever asked.

    A stopped run exits non-zero as a failed one does; only this mark tells them apart.
    """
    started = RUNNING.get(str(run_dir))
    if started is not None:
        started["stopped"] = True


def stop_run(run_dir: Path) -> dict:
    """End a run this dashboard started.

    A run rec has started is cancelled as `motion-spec cancel` does it. A job still generating
    or building has no run yet, so its process group is signalled.
    """
    from rec import State

    from motion_spec.runs.provenance import rec_run

    started = RUNNING.get(str(run_dir))
    process = started["process"] if started else None
    if process is None or process.poll() is not None:
        raise ValueError(f"{run_dir.name} is not running here")
    mark_stopped(run_dir)
    run = rec_run(run_dir)
    if run.lifecycle()[0] is State.IN_PROGRESS:
        run.cancel()
    else:
        # the CLI starts the runtime as a child
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    try:
        process.wait(timeout=2 * STOP_GRACE_S)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=STOP_GRACE_S)
    return run_status(run_dir)


HEALTH: dict = {"checks": None, "stamp": None, "thread": None, "progress": None}
ENVIRONMENT: dict = {"script": None, "captured": None}


def use_environment(script: str | None) -> dict:
    """Report under the environment SCRIPT leaves, or under the server's own when None."""
    from motion_spec.setup import capture_environment

    if not script:
        ENVIRONMENT.update(script=None, captured=None)
    else:
        path = Path(script).expanduser()
        if not path.is_file():
            raise ValueError(f"no environment file at {path}")
        ENVIRONMENT.update(script=str(path), captured=capture_environment(path))
    HEALTH["checks"] = None
    return health_report(refresh=True)


def pick_environment() -> dict:
    """Choose an environment file with the host file chooser, and report under it."""
    start = ENVIRONMENT["script"] or f"{roots.WORKSPACE}/"
    return use_environment(roots.choose(start, directory=False))


def health_report(refresh: bool = False) -> dict:
    """The CLI's health checks, run once and remembered; refresh reruns them.

    The checks take seconds, so they run on a thread the page polls.
    """
    thread = HEALTH["thread"]
    running = thread is not None and thread.is_alive()
    if not running and (refresh or HEALTH["checks"] is None):
        from motion_spec.health import check_health

        def collect() -> None:
            def progress(done: int, dependency: str) -> None:
                HEALTH["progress"] = {"done": done, "dependency": dependency}

            checks = check_health(
                ("all",),
                ("mujoco", "robif2b"),
                env=ENVIRONMENT.get("captured"),
                on_progress=progress,
            )
            HEALTH["checks"] = [dataclasses.asdict(check) for check in checks]
            HEALTH["stamp"] = time.time()
            HEALTH["progress"] = None

        HEALTH["progress"] = {"done": 0, "dependency": ""}
        HEALTH["thread"] = threading.Thread(target=collect, daemon=True)
        HEALTH["thread"].start()
        running = True
    from motion_spec.health import environment_values, ros_summary
    from motion_spec.setup import ENVIRONMENT_FILES

    env = ENVIRONMENT["captured"]
    return {
        "running": running,
        "checks": HEALTH["checks"],
        "stamp": HEALTH["stamp"],
        "progress": HEALTH["progress"] if running else None,
        "environment": {
            "motion_spec": roots.package_version(),
            "python": platform.python_version(),
            "executable": sys.executable,
            "ros": ros_summary(env),
            "variables": environment_values(env),
            "script": ENVIRONMENT["script"],
            "choices": [
                str(roots.WORKSPACE / name)
                for name in ENVIRONMENT_FILES
                if (roots.WORKSPACE / name).is_file()
            ],
            "generations": str(roots.GENERATIONS),
        },
    }


CONSOLE_CHUNK = 256 * 1024


def console_slice(log: Path, offset: int) -> dict:
    """One poll of a text log: the bytes from offset, and where to ask from next.

    Raw text, ANSI escapes included: the page renders the colors and drops the rest.
    """
    if not log.is_file():
        return {"text": "", "offset": 0, "size": 0}
    size = log.stat().st_size
    if offset > size:
        offset = 0  # the log was replaced; start over
    with log.open("rb") as fh:
        fh.seek(offset)
        data = fh.read(CONSOLE_CHUNK)
    return {
        "text": data.decode("utf-8", errors="replace"),
        "offset": offset + len(data),
        "size": size,
    }


def console_log_for(path: Path) -> Path:
    """Which terminal log a dashboard path means.

    A run directory keeps its own console. A generation has two: the dashboard's log of a run it
    started, and the CLI's own console of generating and building it. The newest is the one that
    says what just happened; without either, the name a run would have written.
    """
    if not (path / HEADER_REL).is_file():
        return path / "logs" / "console.log"
    candidates = (path / RUN_LOG, path / "logs" / "console.log")
    written = [log for log in candidates if log.is_file()]
    return max(written, key=lambda log: log.stat().st_mtime) if written else path / RUN_LOG


GENERATING: dict[str, dict] = {}

GENERATE_LOGS = ".dashboard"

GENERATION_LINE_SCAN = 8192

GENERATE_LOGS_KEPT = 20


def start_generate(source: Path) -> dict:
    """Generate a .robmot from its page, and build what it made.

    It does not run it: a run starts from the generation's own page.
    """
    from motion_spec.generation.pipeline import new_id

    job = new_id("gen")
    log_dir = roots.GENERATIONS / GENERATE_LOGS
    log_dir.mkdir(exist_ok=True)
    log = log_dir / f"{job}.log"
    # pipefail, else a failed gen reaches build with an empty path; dash has no pipefail
    env = f" --env {shlex.quote(ENVIRONMENT['script'])}" if ENVIRONMENT["script"] else " --no-env"
    chain = (
        "set -o pipefail; "
        f"g=$(motion-spec gen code {shlex.quote(str(source))} | tail -n 1)"
        f' && printf "generation: %s\\n" "$g" && exec motion-spec build{env} "$g"'
    )
    argv = ["bash", "-c", chain]
    finished = sorted(
        job for job, started in GENERATING.items() if started["process"].poll() is not None
    )
    for finished_job in finished[:-GENERATE_LOGS_KEPT]:
        GENERATING.pop(finished_job)
    kept = {started["log"] for started in GENERATING.values()}

    # another dashboard may reap the same logs
    def mtime(path: Path) -> float:
        try:
            return os.path.getmtime(path)
        except OSError:
            return 0.0

    written = sorted(log_dir.glob("gen-*.log"), key=mtime)
    for stale in written[:-GENERATE_LOGS_KEPT]:
        if stale not in kept:
            stale.unlink(missing_ok=True)
    with log.open("wb") as sink:
        process = subprocess.Popen(
            argv,
            cwd=roots.WORKSPACE,
            env={**os.environ, GENERATION_VARIABLE: str(roots.GENERATIONS)},
            stdout=sink,
            stderr=sink,
            start_new_session=True,
        )
    GENERATING[job] = {"process": process, "log": log, "generation": None}
    return {"job": job, **generate_status(job)}


def generation_named_in(log: Path) -> str | None:
    """The generation the runner announced, from the head of its log.

    The chain prints `generation: <path>` before it builds, so only the head is read.
    """
    if not log.is_file():
        return None
    with log.open("r", errors="replace") as fh:
        head = fh.read(GENERATION_LINE_SCAN)
    for line in head.splitlines():
        if line.startswith("generation: "):
            made = Path(line.removeprefix("generation: ").strip()).resolve()
            root = roots.GENERATIONS.resolve()
            return str(made.relative_to(root)) if root in made.parents else None
    return None


def generate_status(job: str) -> dict:
    """How far a page-started generation has got, and which generation it made."""
    started = GENERATING.get(job)
    if started is None:
        raise ValueError("unknown job")
    process = started["process"]
    busy = process.poll() is None
    if started["generation"] is None:
        started["generation"] = generation_named_in(started["log"])
    return {
        "busy": busy,
        "pid": process.pid if busy else None,
        "exit_code": None if busy else process.returncode,
        "generation": started["generation"],
    }


def generate_console(job: str, offset: int) -> dict:
    """The terminal output of a page-started generation, followed by byte offset."""
    started = GENERATING.get(job)
    if started is None:
        raise ValueError("unknown job")
    return console_slice(started["log"], offset)
