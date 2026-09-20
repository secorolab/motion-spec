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
from motion_spec.dashboard.catalog import generation_cameras, is_simulated, run_ended, run_recorded
from motion_spec.dashboard.roots import LAYOUT_REL, RUN_LOG, trace

# One entry per run this dashboard started, keyed by the run directory: several runs of one
# generation can be up at once.
RUNNING: dict[str, dict] = {}

# Finished runs whose status a page may still ask for; older ones are the past.
RUNS_KEPT = 20


def _real_run_active() -> Path | None:
    """The run directory of a real-hardware run still in progress, if any is.

    Two runs can each believe they own the real robot, so this is the one place that looks
    across all of them at once.
    """
    for key, started in RUNNING.items():
        if started["process"].poll() is None and not is_simulated(Path(started["generation"])):
            return Path(key)
    return None


def run_status(run_dir: Path) -> dict:
    """Whether this run is in progress, and where its output is going.

    A run is over when its archive is written, not when the process that started it exits:
    the CLI still verifies and reports for a while after, which is no longer this run.
    """
    started = RUNNING.get(str(run_dir))
    process = started["process"] if started else None
    busy = process is not None and process.poll() is None
    # The dashboard names the run when it starts it, so the directory to watch is known
    # before anything exists on disk.
    live = busy and not (run_dir.is_dir() and run_ended(run_dir))
    trace(f"run_status {run_dir.name}: busy={busy} running={live}")
    return {
        "id": run_dir.name,
        "path": str(run_dir.relative_to(roots.GENERATIONS)),
        "running": live,
        "busy": busy,
        "pid": process.pid if busy else None,
        "run": str(run_dir.relative_to(roots.GENERATIONS)) if live else None,
        "exit_code": None if busy or process is None else process.returncode,
        # Ended on request, not by failing. A signalled run exits non-zero either way, so the
        # page cannot tell a cancel from a crash by the code alone.
        "stopped": bool(started and started.get("stopped")),
        "log": str(Path(started["generation"]) / RUN_LOG) if started else None,
        # Whether the run keeps a frame log: from its manifest once written, before that from the
        # choice this dashboard started it with. False means the page must not wait for a log.
        "recorded": _recorded(run_dir, started),
    }


def generation_status(generation_dir: Path) -> dict:
    """Every run of this generation the dashboard started and still remembers, newest first."""
    runs = [
        run_status(Path(key))
        for key, started in sorted(RUNNING.items(), reverse=True)
        if started["generation"] == str(generation_dir)
    ]
    return {"running": any(run["running"] for run in runs), "runs": runs}


def _recorded(run_dir: Path | None, started: dict | None) -> bool | None:
    recorded = run_recorded(run_dir)
    if recorded is None and started is not None:
        return started.get("recorded")
    return recorded


def run_arguments(options: dict, simulated: bool) -> list[str]:
    """The run flags a browser's choices amount to. Only a simulator has a display or a pace.

    A window paces itself, but a headless loop runs as fast as the machine allows, which is
    nothing to watch live plots of: realtime asks the runtime for `--rtf 1`, which the CLI
    forwards to the executable untouched. Fast keeps the uncapped loop.
    """
    argv = (
        ["--headless"] + (["--rtf", "1"] if options.get("realtime", True) else [])
        if simulated and options.get("headless")
        else []
    )
    # Whether to record is a choice on any platform: no log means no replay and no live plots.
    if options.get("log", True) is False:
        argv.append("--no-log")
    # A run started from the page arms and waits for play: the transport is right there, and a
    # simulation that takes off on its own is already past what the operator wanted to watch.
    if simulated:
        argv.append("--start-paused")
    return argv


def start_run(generation_dir: Path, options: dict) -> dict:
    """Run a generation again, with the options `motion-spec run` takes for one.

    The CLI owns what a run is -- the run directory, the archive, the recovery afterwards --
    so start it rather than reimplementing it here, and let it say what it made.
    """
    if not (generation_dir / LAYOUT_REL).exists():
        raise ValueError("not a generation")
    # Only a simulator has a display to drop, whatever the browser posted.
    simulated = is_simulated(generation_dir)
    # Simulations run side by side; the real robot runs one thing at a time, whichever
    # generation asks. Whether the hardware answers is not asked here: the devices panel
    # probes when the operator asks it to.
    if not simulated:
        other = _real_run_active()
        if other is not None:
            raise ValueError(f"the real robot is already running {other.name}")
    from motion_spec.generation.pipeline import new_id

    # Name the run here rather than letting the CLI pick: the browser can then open the run
    # page at once and wait for the log, instead of polling for a directory to appear.
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
    # Only what a browser can meaningfully choose; the CLI decides the rest.
    argv += run_arguments(options, simulated)
    # A simulator renders any declared camera and its standard view; a real platform records
    # the cameras that name a ROS image topic to read.
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
    _reap_runs()
    # Closed here: Popen dups the descriptor for the child, and a handle per run never
    # collected is a file descriptor leaked for the life of the server.
    with (generation_dir / RUN_LOG).open("wb") as sink:
        process = subprocess.Popen(
            argv, cwd=roots.WORKSPACE, stdout=sink, stderr=sink, start_new_session=True
        )
    run_dir = generation_dir / "runs" / run_id
    RUNNING[str(run_dir)] = {
        "process": process,
        "run_id": run_id,
        "generation": str(generation_dir),
        "recorded": options.get("log", True) is not False,
    }
    return {
        **run_status(run_dir),
        "command": argv,
        "recording": recording,
        "run": str(run_dir.relative_to(roots.GENERATIONS)),
    }


def _reap_runs() -> None:
    """Forget finished runs beyond the last few: their status is on disk, in rec's record."""
    finished = sorted(
        key for key, started in RUNNING.items() if started["process"].poll() is not None
    )
    for key in finished[:-RUNS_KEPT]:
        RUNNING.pop(key)


# How long a run gets to end on a TERM before it is killed outright.
STOP_GRACE_S = 5


def mark_stopped(run_dir: Path) -> None:
    """Record that this run's end was asked for, whoever asked.

    A run signalled from the generation page and one cancelled through its control block both
    exit non-zero, exactly as a failed run does. Only the asking tells them apart, so it is the
    asking that is remembered.
    """
    started = RUNNING.get(str(run_dir))
    if started is not None:
        started["stopped"] = True


def stop_run(run_dir: Path) -> dict:
    """End a run this dashboard started, by signalling the process group it was started in.

    The control block is the polite way to stop a loop, and only a loop already ticking reads
    it: a run still connecting to its hardware, or one that never gets that far, answers
    nothing. This is the signal for that.
    """
    started = RUNNING.get(str(run_dir))
    process = started["process"] if started else None
    if process is None or process.poll() is not None:
        raise ValueError(f"{run_dir.name} is not running here")
    # Remembered before the signal, so the exit this sets up is already known to be asked for
    # by the time anything reads the status back.
    mark_stopped(run_dir)
    # The whole session: the CLI starts the runtime as a child, and it is the one holding the
    # devices open.
    group = os.getpgid(process.pid)
    os.killpg(group, signal.SIGTERM)
    try:
        process.wait(timeout=STOP_GRACE_S)
    except subprocess.TimeoutExpired:
        os.killpg(group, signal.SIGKILL)
        process.wait(timeout=STOP_GRACE_S)
    return run_status(run_dir)


HEALTH: dict = {"checks": None, "stamp": None, "thread": None, "progress": None}
# The environment file the page asked health to report under, and what sourcing it gave.
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


def environment_choices() -> list[str]:
    """The environment files a workspace offers, for the page to choose between."""
    from motion_spec.setup import ENVIRONMENT_FILES

    return [
        str(roots.WORKSPACE / name)
        for name in ENVIRONMENT_FILES
        if (roots.WORKSPACE / name).is_file()
    ]


def health_report(refresh: bool = False) -> dict:
    """The CLI's health checks, run once and remembered; refresh reruns them.

    Some checks configure CMake projects, so they take seconds: run them on a thread the page
    polls rather than holding a request open, and keep the answer for the server's lifetime.
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
    return {
        "running": running,
        "checks": HEALTH["checks"],
        "stamp": HEALTH["stamp"],
        "progress": HEALTH["progress"] if running else None,
        "environment": _environment(),
    }


def _environment() -> dict:
    """What this installation is, beside whether it works: versions, interpreter, roots."""
    from importlib import metadata

    from motion_spec.health import environment_values, ros_summary

    try:
        version = metadata.version("motion_spec")
    except metadata.PackageNotFoundError:
        version = None
    env = ENVIRONMENT["captured"]
    return {
        "motion_spec": version,
        "python": platform.python_version(),
        "executable": sys.executable,
        "ros": ros_summary(env),
        "variables": environment_values(env),
        "script": ENVIRONMENT["script"],
        "choices": environment_choices(),
        "generations": str(roots.GENERATIONS),
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
    if not (path / LAYOUT_REL).is_file():
        return path / "logs" / "console.log"
    candidates = (path / RUN_LOG, path / "logs" / "console.log")
    written = [log for log in candidates if log.is_file()]
    return max(written, key=lambda log: log.stat().st_mtime) if written else path / RUN_LOG


GENERATING: dict[str, dict] = {}


# under roots.GENERATIONS; the catalog only lists generation dirs, but keep the logs out of the way
GENERATE_LOGS = ".dashboard"


# How far into a job's log the generation it made can still be announced.
GENERATION_LINE_SCAN = 8192


# Finished jobs whose logs are kept for a page still reading them; older ones are the past.
GENERATE_LOGS_KEPT = 20


def start_generate(source: Path) -> dict:
    """Generate a .robmot from its page, and build what it made.

    Making is all this does: a run starts from the generation's own page, where the devices are
    tested, the run is named and its page follows it. `gen` prints the directory it made on
    stdout, which is what hands it to `build`; the terminal words are the record.
    """
    from motion_spec.generation.pipeline import new_id

    job = new_id("gen")
    log_dir = roots.GENERATIONS / GENERATE_LOGS
    log_dir.mkdir(exist_ok=True)
    log = log_dir / f"{job}.log"
    # The capture eats gen's narration, so the path is announced into the log here for
    # generate_status to read. No tee back into it: a /dev/fd reopen starts at offset 0 and
    # overwrites what the others wrote.
    # pipefail, or the status is tail's, build is reached with an empty path, and click's
    # "Directory '' does not exist" buries the rejection. bash, not sh: dash has no pipefail.
    chain = (
        "set -o pipefail; "
        f"g=$(motion-spec gen code {shlex.quote(str(source))}"
        f" -o {shlex.quote(str(roots.GENERATIONS))} | tail -n 1)"
        f' && printf "generation: %s\\n" "$g" && exec motion-spec build "$g"'
    )
    argv = ["bash", "-c", chain]
    _reap_jobs()
    with log.open("wb") as sink:
        process = subprocess.Popen(
            argv, cwd=roots.WORKSPACE, stdout=sink, stderr=sink, start_new_session=True
        )
    GENERATING[job] = {"process": process, "log": log, "generation": None}
    return {"job": job, **generate_status(job)}


def _reap_jobs() -> None:
    """Forget finished jobs and delete the logs nobody will ask for again.

    A job is a process, a log and what it made; the page reads all three while it runs and for
    as long as it is looking at the console afterwards. Keeping the last few covers that. The
    rest are runs of the build from days ago, holding a Popen each and a file on disk.
    """
    finished = sorted(
        job for job, started in GENERATING.items() if started["process"].poll() is not None
    )
    for job in finished[:-GENERATE_LOGS_KEPT]:
        GENERATING.pop(job)
    # Logs left by earlier dashboards go too: their jobs died with the process that started
    # them. Another dashboard may be reaping the same ones, so a file that vanishes between
    # the listing and the stat is not an error.
    log_dir = roots.GENERATIONS / GENERATE_LOGS
    if not log_dir.is_dir():
        return
    kept = {started["log"] for started in GENERATING.values()}

    def _mtime(path: Path) -> float:
        try:
            return os.path.getmtime(path)
        except OSError:
            return 0.0

    written = sorted(log_dir.glob("gen-*.log"), key=_mtime)
    for log in written[:-GENERATE_LOGS_KEPT]:
        if log not in kept:
            log.unlink(missing_ok=True)


def _generation_named_in(log: Path) -> str | None:
    """The generation the runner announced, from the head of its log.

    The chain prints `generation: <path>` before it builds anything, so the answer is in the
    first lines or it is not coming -- and reading only those keeps a poll every two seconds
    off a log that grows for the length of a build.
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
        started["generation"] = _generation_named_in(started["log"])
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
