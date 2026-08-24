# SPDX-License-Identifier: MPL-2.0

"""What the dashboard starts, refuses to start, follows and stops.

A generation is made by one process and run by another; both are watched through their
terminal output, and a hardware run is refused before it is named if nothing answers.
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
from motion_spec.devices import probe_devices, unreachable

RUNNING: dict[str, dict] = {}


def _real_run_active() -> Path | None:
    """The generation directory of a real-hardware run still in progress, if any is.

    Two generations can each believe they own the real robot; the dashboard tracks one entry
    per generation, so this is the one place that looks across all of them at once.
    """
    for key, started in RUNNING.items():
        if started["process"].poll() is None and not is_simulated(Path(key)):
            return Path(key)
    return None


def run_status(generation_dir: Path) -> dict:
    """Whether this generation has a run in progress, and where its output is going.

    A run is over when its archive is written, not when the process that started it exits:
    the CLI still verifies and reports for a while after, which is no longer this run.
    """
    started = RUNNING.get(str(generation_dir))
    process = started["process"] if started else None
    busy = process is not None and process.poll() is None
    # The dashboard names the run when it starts it, so the directory to watch is known
    # before anything exists on disk.
    run_dir = generation_dir / "runs" / started["run_id"] if started else None
    run = run_dir if run_dir is not None and run_dir.is_dir() else None
    live = busy and not (run is not None and run_ended(run))
    trace(
        f"run_status {generation_dir.name}: busy={busy} running={live} run={run.name if run else None}"
    )
    return {
        "running": live,
        "busy": busy,
        "pid": process.pid if busy else None,
        "run": str(run.relative_to(roots.GENERATIONS)) if live and run else None,
        "exit_code": None if busy or process is None else process.returncode,
        "log": str(generation_dir / RUN_LOG) if process is not None else None,
        # What the run says about its own frame log, once it has written a manifest to say it in:
        # false means it kept none on purpose, and the page shows its console instead of waiting.
        "recorded": run_recorded(run_dir),
    }


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


class Unreachable(ValueError):
    """A refusal that carries the probe behind it, so the page shows rows and not a sentence."""

    def __init__(self, message: str, report: dict):
        super().__init__(message)
        self.report = report


def start_run(generation_dir: Path, options: dict) -> dict:
    """Run a generation again, with the options `motion-spec run` takes for one.

    The CLI owns what a run is -- the run directory, the archive, the recovery afterwards --
    so start it rather than reimplementing it here, and let it say what it made.
    """
    if not (generation_dir / LAYOUT_REL).exists():
        raise ValueError("not a generation")
    if run_status(generation_dir)["busy"]:
        raise ValueError("this generation is already running")
    # Only a simulator has a display to drop or a frame to record, whatever the browser posted.
    simulated = is_simulated(generation_dir)
    # Hardware that does not answer is not a run to start: the driver would block on the
    # connect, and the run would exist as a named, empty directory that never records a frame.
    if not simulated:
        other = _real_run_active()
        if other is not None:
            raise ValueError(f"the real robot is already running {other.name}")
        report = probe_devices(generation_dir)
        missing = [device["name"] for device in unreachable(report)]
        if missing:
            raise Unreachable(f"not reachable: {', '.join(missing)}", report)
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
    # What a browser can meaningfully choose: the rest the dashboard already knows or the CLI
    # decides. A run always verifies what it archived; a recording nobody checked is not
    # worth the disk it sits on.
    argv += run_arguments(options, simulated)
    # The runtime records: it holds the rendered frame, so it writes the video itself.
    declared = {camera["id"] for camera in generation_cameras(generation_dir)}
    recording = [
        camera
        for camera in (options.get("cameras") or () if simulated else ())
        if camera in declared or camera == "default"
    ]
    for camera in recording:
        argv += ["--record", camera]
    sink = (generation_dir / RUN_LOG).open("wb")
    process = subprocess.Popen(
        argv, cwd=roots.WORKSPACE, stdout=sink, stderr=sink, start_new_session=True
    )
    RUNNING[str(generation_dir)] = {"process": process, "run_id": run_id}
    return {
        **run_status(generation_dir),
        "command": argv,
        "recording": recording,
        "run": str((generation_dir / "runs" / run_id).relative_to(roots.GENERATIONS)),
    }


# How long a run gets to end on a TERM before it is killed outright.
STOP_GRACE_S = 5


def stop_run(path: Path) -> dict:
    """End a run this dashboard started, by signalling the process group it was started in.

    The control block is the polite way to stop a loop, and only a loop already ticking reads
    it: a run still connecting to its hardware, or one that never gets that far, answers
    nothing. This is the signal for that. Takes a run or its generation, as `run_control` does.
    """
    generation_dir = path if (path / LAYOUT_REL).exists() else path.parent.parent
    started = RUNNING.get(str(generation_dir))
    process = started["process"] if started else None
    if process is None or process.poll() is not None:
        raise ValueError("no run of this generation is running here")
    # The whole session: the CLI starts the runtime as a child, and it is the one holding the
    # devices open.
    group = os.getpgid(process.pid)
    os.killpg(group, signal.SIGTERM)
    try:
        process.wait(timeout=STOP_GRACE_S)
    except subprocess.TimeoutExpired:
        os.killpg(group, signal.SIGKILL)
        process.wait(timeout=STOP_GRACE_S)
    return run_status(generation_dir)


HEALTH: dict = {"checks": None, "stamp": None, "thread": None}


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
            checks = check_health(("all",), ("mujoco", "robif2b"))
            HEALTH["checks"] = [dataclasses.asdict(check) for check in checks]
            HEALTH["stamp"] = time.time()

        HEALTH["thread"] = threading.Thread(target=collect, daemon=True)
        HEALTH["thread"].start()
        running = True
    return {
        "running": running,
        "checks": HEALTH["checks"],
        "stamp": HEALTH["stamp"],
        "environment": _environment(),
    }


def _environment() -> dict:
    """What this installation is, beside whether it works: versions, interpreter, roots."""
    from importlib import metadata

    try:
        version = metadata.version("motion_spec")
    except metadata.PackageNotFoundError:
        version = None
    return {
        "motion_spec": version,
        "python": platform.python_version(),
        "executable": sys.executable,
        "ros_distro": os.environ.get("ROS_DISTRO"),
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
    """Which terminal log a dashboard path means: a run's console, or a generation's runner log."""
    if (path / LAYOUT_REL).is_file():
        return path / RUN_LOG
    return path / "logs" / "console.log"


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
    # gen narrates on stdout and ends with the bare generation path; the capture eats both.
    # Announce the path into the log ourselves -- generate_status reads it from there -- and
    # let build's output carry the rest. (No tee back into the log: a /dev/fd reopen starts at
    # offset 0 and overwrites what the others wrote.)
    chain = (
        f"g=$(motion-spec gen code {shlex.quote(str(source))}"
        f" -o {shlex.quote(str(roots.GENERATIONS))} | tail -n 1)"
        f' && printf "generation: %s\\n" "$g" && exec motion-spec build "$g"'
    )
    argv = ["sh", "-c", chain]
    _reap_jobs()
    sink = log.open("wb")
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
    # On disk too, including logs left by earlier dashboards: their jobs went with the process
    # that started them, so nothing is reading those at all. The directory appears with the
    # first job; a root nothing generated under has nothing to reap. Another dashboard may be
    # reaping the same logs, so one vanishing between the listing and the stat is not an error.
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
