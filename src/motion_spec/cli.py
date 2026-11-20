# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Unified Click command-line interface for motion-spec."""

import contextlib
import json
import logging
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
import traceback
import warnings
from contextlib import contextmanager
from importlib.metadata import distribution
from pathlib import Path
from typing import ClassVar

import click

from motion_spec.setup import (
    BUILD_TYPE,
    BUILD_TYPE_VARIABLE,
    GENERATION_VARIABLE,
    WORKSPACE_VARIABLE,
)
from motion_spec.utils import (
    LEVEL_LABELS,
    LOG_DATEFMT,
    LOG_DIRECTORY,
    RESET,
    SGR,
    STAMP_COLOUR,
    command_log,
    log_header,
    machine_facts,
    mirrored_stderr,
    show_warning,
)


class Reported(click.ClickException):
    """A failure already phrased for the reader, shown in the same format as every other line."""

    def show(self, file=None) -> None:
        for line in str(self.message).splitlines():
            stamp("error")
            click.echo(line, err=True)


def internal_failure(what: str, exc: Exception) -> click.ClickException:
    """Report a failure that is ours, not the model's, with the stack that produced it.

    Anything reaching here is a defect in the generator or the environment, and the one thing
    needed to fix it -- where it came from -- is exactly what wrapping the message throws away.
    """
    traceback.print_exception(exc, file=sys.stderr)

    return click.ClickException(f"{what}: {exc}")


def stamp(level: str, err: bool = True) -> None:
    """The prefix every line carries, for one assembled in pieces."""
    click.echo(
        f"{SGR}{STAMP_COLOUR}m{time.strftime(LOG_DATEFMT)}{RESET}  {LEVEL_LABELS[level]} ",
        nl=False,
        err=err,
    )


def say(level: str, message: str = "", err: bool = True) -> None:
    """One log line: when, how loud, and what.

    Progress goes to stderr; a report is the command's result and goes to stdout.
    """
    stamp(level, err)
    click.echo(message, err=err)


# width of `  STATUS   `, the indent of a health row's detail lines
STATUS_WIDTH = 11
# a hue no name, source or status uses
COMMAND_COLOUR = 180
LATEST_LINK = "latest"


def new_generation(model: Path, name: str | None = None) -> Path:
    """Create this run's generation directory, and say where it is before anything fills it.

    Announced up front rather than only on success: the DSL and the compiler write pages of
    their own output next, and where it all landed is the one thing needed to go look at it --
    including when the run fails partway and never reaches the closing line. Stderr, because
    `gen` writes that closing line to stdout for a caller to read.
    """
    from motion_spec.generation.pipeline import create_generation_dir

    base = generation_root()
    generation = create_generation_dir(model, base, name)
    say("info", f"generation {generation}")
    # relative, and replaced atomically so it is never missing
    link = base / LATEST_LINK
    pending = base / f".{LATEST_LINK}.new"
    try:
        pending.unlink(missing_ok=True)
        pending.symlink_to(os.path.relpath(generation, base), target_is_directory=True)
        os.replace(pending, link)
    except OSError:
        # a convenience: never fails the generation
        pending.unlink(missing_ok=True)

    return generation


def generation_root() -> Path:
    """Where generations live: $MOTION_SPEC_GEN."""
    from motion_spec.setup import generations_root

    try:
        return generations_root()
    except RuntimeError as exc:
        raise click.UsageError(str(exc)) from exc


def model_rejected(exc: Exception) -> click.ClickException:
    """Report a model the checks refused, with the check that refused it.

    The message says what is wrong with the model and is the last line read. The stack says
    which check decided that, which is the only way to find the rule from the sentence it
    prints -- there are hundreds of them, and the sentence names none.
    """
    traceback.print_exception(exc, file=sys.stderr)

    return click.ClickException(f"the model was rejected: {exc}")


@contextmanager
def manual_section(formatter: click.HelpFormatter, title: str):
    formatter.write(f"{click.style(title, bold=True)}\n")
    formatter.indent()
    yield
    formatter.dedent()
    formatter.write_paragraph()


class MotionSpecGroup(click.Group):
    """Render the top-level help as a Unix manual page."""

    def format_help(self, ctx: click.Context, formatter: click.HelpFormatter) -> None:
        with manual_section(formatter, "NAME"):
            formatter.write_text(
                "motion-spec - validate, compile, run, and inspect motion specifications"
            )
        with manual_section(formatter, "SYNOPSIS"):
            formatter.write_text("motion-spec [OPTIONS] COMMAND [ARGS]...")
        with manual_section(formatter, "DESCRIPTION"):
            formatter.write_text(
                "motion-spec lowers motion-model JSON-LD to an intermediate representation, "
                "generates a C++ controller, and records or inspects controller executions."
            )
        with manual_section(formatter, "WORKFLOW"):
            formatter.write_dl(
                [
                    (
                        "setup",
                        "Install the pinned tools and libraries needed to build controllers.",
                    ),
                    ("install", "Install the optional DSL, dashboard, or replay features."),
                    ("health", "Check the tools and libraries required by the selected target."),
                    ("gen", "Generate IR or C++ from a .robmot model."),
                    ("build", "Configure and compile a generation."),
                    ("run", "Execute the controller and record the run."),
                    ("rerun", "Run the last generation again, without naming it."),
                    ("cancel", "Ask a run in progress to stop; rec records it canceled."),
                    ("control", "Pause, resume, step or pace a simulated run in progress."),
                    ("replay", "Verify, summarize, or recover data from a recorded run."),
                    ("compare", "Whether two generations had the same model, inputs and tools."),
                    ("view", "Compose a .scenex in MuJoCo and view it."),
                ]
            )
        with manual_section(formatter, "ARTIFACTS"):
            formatter.write_dl(
                [
                    ("generated/source/", "Authored DSL inputs."),
                    ("generated/model/", "JSON-LD graphs, FSM artifacts, and IR."),
                    ("generated/controller/", "Generated C++ and CMake project."),
                    ("generated/contract/", "Schema, frame layout, and frame-log protocol."),
                    ("generated/provenance/", "Generation and derivation provenance."),
                    ("build/", "Reusable compiled controller."),
                    ("runs/RUN/", "Run-owned logs and provenance/."),
                    ("latest", f"Symlink to the newest generation, under ${GENERATION_VARIABLE}."),
                ]
            )
        with manual_section(formatter, "ENVIRONMENT"):
            formatter.write_dl(
                [
                    (
                        GENERATION_VARIABLE,
                        (
                            "Where 'gen' and 'run' put a new generation when given no -o. "
                            "Unset, they stop and say so. "
                            "'latest' there names the one 'rerun' takes."
                        ),
                    )
                ]
            )
        option_records = [
            record for param in self.get_params(ctx) if (record := param.get_help_record(ctx))
        ]
        if option_records:
            with manual_section(formatter, "OPTIONS"):
                formatter.write_dl(option_records)
        command_records = [
            (name, command.help or command.get_short_help_str())
            for name in self.list_commands(ctx)
            if (command := self.get_command(ctx, name)) is not None and not command.hidden
        ]
        with manual_section(formatter, "COMMANDS"):
            formatter.write_dl(command_records)
        with manual_section(formatter, "EXAMPLES"):
            examples = (
                (
                    "Generate and build a model",
                    ("motion-spec gen model.robmot", 'motion-spec build "$MOTION_SPEC_GEN/latest"'),
                ),
                ("Generate only the IR", ("motion-spec gen ir model.robmot --name scratch",)),
                (
                    "Generate, build, run, and inspect",
                    (
                        "motion-spec run model.robmot --run-id run-1",
                        'motion-spec replay "$MOTION_SPEC_GEN/latest/runs/run-1"',
                    ),
                ),
            )
            prompt = " " * (formatter.current_indent + 2) + click.style("$ ", fg="cyan")
            for example_index, (label, commands) in enumerate(examples):
                if example_index:
                    formatter.write_paragraph()
                formatter.write_text(click.style(label, bold=True))
                for command in commands:
                    formatter.write(f"{prompt}{command}\n")
        with manual_section(formatter, "SEE ALSO"):
            formatter.write_text("motion-spec COMMAND --help")


INSTALLS = {"textx": "motion-spec setup motion-spec-dsl", "pyshacl": "pip install motion_spec"}


@contextmanager
def requirements_reported():
    """Turn a missing optional dependency into its box instead of a raw traceback."""
    try:
        yield
    except ImportError as exc:
        from motion_spec.health import DETAILS, SETUP_PROVIDES, remedy

        dependency = (exc.name or "").partition(".")[0]
        spec = DETAILS.get(dependency)
        install = remedy(dependency) if dependency in SETUP_PROVIDES else INSTALLS.get(dependency)
        if spec is None or install is None:
            raise
        rows = [("why", spec["why"]), ("source", spec["source"]), ("install", install)]
        body = [f"│ {key:<8} {value}" for key, value in rows]
        width = max(len(dependency) + 14, *(len(line) for line in body)) + 1
        top = f"┌─ {dependency} not found " + "─" * (width - len(dependency) - 14)
        raise click.ClickException("\n".join(["", top, *body, "└" + "─" * width])) from exc


@click.group(cls=MotionSpecGroup, no_args_is_help=True)
@click.version_option(package_name="motion_spec")
@click.pass_context
def main(ctx: click.Context) -> None:
    """The motion-spec toolchain."""
    # setup installs motion-spec-dsl, so runs without it
    if ctx.invoked_subcommand != "setup":
        from rdf_utils.resolver import install_resolver

        install_resolver()
    handler = SayHandler()
    for name in TOOL_LOGGERS:
        logger = logging.getLogger(name)
        if any(isinstance(existing, SayHandler) for existing in logger.handlers):
            continue
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    warnings.showwarning = show_warning


# only these, so third-party loggers stay quiet
TOOL_LOGGERS = ("motion_spec", "motion_spec_dsl", "scene_dsl", "coord_dsl", "textx")


class SayHandler(logging.Handler):
    """A library's log record, in the format every other line of the CLI is written in."""

    LEVELS: ClassVar = {logging.WARNING: "warn", logging.ERROR: "error", logging.CRITICAL: "error"}

    def emit(self, record: logging.LogRecord) -> None:
        say(self.LEVELS.get(record.levelno, "info"), record.getMessage())


def environment_options(command):
    """``--env`` and ``--no-env``, for every command that shells out to a build or a run."""
    command = click.option(
        "--no-env",
        is_flag=True,
        help="Ignore any environment file and inherit this shell unchanged.",
    )(command)
    return click.option(
        "--env",
        "env_script",
        type=click.Path(exists=True, dir_okay=False, path_type=Path),
        help=(
            "Shell file to source first; defaults to $MOTION_SPEC_ENV, "
            "then the nearest setup-motion-spec.bash above the generation."
        ),
    )(command)


def run_options(command):
    """What one run of a built controller takes, for `run` and `rerun` alike."""
    command = click.option(
        "--no-log", is_flag=True, help="Do not write the frame log; the run has no replay."
    )(command)
    command = click.option(
        "--record",
        "record",
        multiple=True,
        metavar="CAMERA",
        help="Record this camera to MP4 beside the log. Simulations render declared "
        "cameras; real runs record their declared ROS image topics. Repeatable.",
    )(command)
    command = click.option(
        "--cwd",
        type=click.Path(exists=True, file_okay=False, path_type=Path),
        help="Working directory of the executable; scene assets resolve relative to it.",
    )(command)
    return click.option(
        "--run-id", help="Name of this run's directory under <generation>/runs."
    )(command)


def environment(
    env_script: Path | None, no_env: bool, start: Path | None = None
) -> tuple[dict[str, str] | None, Path | None]:
    """The environment to run under, and the file it was sourced from, which a run keeps."""
    from motion_spec.setup import capture_environment, find_environment

    if no_env:
        return None, None
    try:
        script = env_script or find_environment(start)
        if script is None:
            return None, None
        captured = capture_environment(script)
    except (OSError, RuntimeError) as exc:
        raise click.ClickException(str(exc)) from exc
    say("info", f"environment {script}")
    return captured, script


@main.command()
@click.option("--port", default=8080, show_default=True, help="Port to serve on.")
@click.option(
    "--logs",
    type=click.Path(file_okay=False, path_type=Path),
    help=f"Generation root to browse. Default: ${GENERATION_VARIABLE}.",
)
@click.option(
    "--sources",
    type=click.Path(file_okay=False, path_type=Path),
    help="Model source root. Default: the logs root's parent.",
)
@click.option(
    "-b", "--background", is_flag=True, help="Serve detached and return, logging to a file."
)
@click.option(
    "-k", "--kill", is_flag=True, help="Stop a dashboard already serving, on --port if given."
)
@click.option("-r", "--restart", is_flag=True, help="Stop whatever is serving, then serve again.")
@click.option(
    "--log-file",
    type=click.Path(dir_okay=False, path_type=Path),
    help="Where --background writes output. Default: dashboard.log in the logs root.",
)
@environment_options
def dashboard(
    port: int,
    logs: Path | None,
    sources: Path | None,
    env_script: Path | None,
    no_env: bool,
    background: bool,
    kill: bool,
    restart: bool,
    log_file: Path | None,
) -> None:
    """Browse generations, replay runs, and query them in a browser."""
    from motion_spec.dashboard.jobs import use_environment
    from motion_spec.dashboard.server import serve
    from motion_spec.setup import find_environment

    named_port = (
        port
        if click.get_current_context().get_parameter_source("port")
        is not click.core.ParameterSource.DEFAULT
        else None
    )
    if kill:
        return stop_dashboards(named_port)
    if restart:
        # the port frees asynchronously
        stop_dashboards(named_port)
        for _ in range(50):
            if not port_taken(port):
                break
            time.sleep(0.1)
    logs = (logs or generation_root()).expanduser()
    if port_taken(port):
        raise click.ClickException(
            f"port {port} is already serving. Stop it, or pass --port for a second dashboard."
        )
    script = None if no_env else (env_script or find_environment(logs))
    if not background:
        if script:
            use_environment(str(script))
            say("info", f"environment {script}")
        return serve(port, logs, sources)

    destination = (log_file or logs / "dashboard.log").expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    argv = [sys.argv[0], "dashboard", "--port", str(port), "--logs", str(logs)]
    if sources is not None:
        argv += ["--sources", str(sources)]
    argv += ["--env", str(script)] if script else ["--no-env"]
    with destination.open("ab") as sink:
        child = subprocess.Popen(argv, stdout=sink, stderr=sink, start_new_session=True)
    say("info", f"pid {child.pid}, logging to {destination}")
    say("info", f"stop it with: kill {child.pid}")
    say("done", f"dashboard http://127.0.0.1:{port}")
    click.echo(f"http://127.0.0.1:{port}")


def stop_dashboards(port: int | None) -> None:
    """Stop the dashboards serving, on PORT or all, and the labs they started.

    Each records its pid on start, and its lab's; a lab whose dashboard had to be killed
    outright is stopped from that record too, or the next dashboard starts a second one.
    """
    from motion_spec.dashboard.roots import pidfile

    records = [
        path
        for pattern in (pidfile(port or "*"), pidfile(port or "*", "lab"))
        for path in pattern.parent.glob(pattern.name)
    ]
    stopped = []
    for path in records:
        try:
            pid = int(path.read_text().strip())
        except (OSError, ValueError):
            pass
        else:
            if stop(pid):
                stopped.append(pid)
        path.unlink(missing_ok=True)
    if not stopped:
        where = f" on port {port}" if port else ""
        raise click.ClickException(f"no dashboard is serving{where}.")
    say("done", f"stopped {', '.join(map(str, stopped))}")


def stop(pid: int) -> bool:
    """SIGTERM, then SIGKILL after 5 s; False when PID was already gone."""
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.2)
    with contextlib.suppress(ProcessLookupError):
        os.kill(pid, signal.SIGKILL)
    return True


def port_taken(port: int) -> bool:
    """Whether something already answers on the loopback port."""
    with socket.socket() as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


@main.command()
@click.argument("repositories", nargs=-1)
@click.option(
    "--workspace",
    "workspace_argument",
    type=click.Path(file_okay=False, path_type=Path),
    help=(
        "Workspace to install into; defaults to $MOTION_SPEC_WS. Builds go to "
        "WORKSPACE/install and the environment files to WORKSPACE."
    ),
)
@click.option(
    "--clean",
    is_flag=True,
    help="Trash the workspace's build, install and log trees and its environment files, then "
    "exit. Sources and generations stay.",
)
@click.option(
    "--force",
    is_flag=True,
    help="Rebuild even when a package is already installed, from a cleared CMake cache.",
)
@click.option(
    "--build-type",
    default=BUILD_TYPE,
    show_default=True,
    envvar=BUILD_TYPE_VARIABLE,
    help="CMAKE_BUILD_TYPE for the sources.",
)
@click.option(
    "--dev",
    is_flag=True,
    help="Install the Python packages editable (`pip install -e`), for working on their "
    "checkouts in WORKSPACE/src. Without it they install as snapshots.",
)
@click.option(
    "--real",
    is_flag=True,
    help="Also install motion_spec.real.repos: the device drivers a real platform needs.",
)
@click.option(
    "--ros/--no-ros",
    "ros_flag",
    default=None,
    help="Treat the workspace as a colcon one: build each CMake package with colcon and write "
    "an environment file that sources ROS and the overlay. Overrides [ros] workspace.",
)
@click.option(
    "-j",
    "--jobs",
    type=click.IntRange(min=1),
    envvar="CMAKE_BUILD_PARALLEL_LEVEL",
    help="Compilers to run at once. Defaults to the lesser of the usable cores and one per "
    "2 GiB of memory.",
)
def setup(
    repositories: tuple[str, ...],
    workspace_argument: Path | None,
    clean: bool,
    force: bool,
    build_type: str,
    dev: bool,
    real: bool,
    ros_flag: bool | None,
    jobs: int | None,
) -> None:
    """Install what the .repos manifests list, in their order.

    Sources are imported by hand first (`vcs import src < motion_spec.repos`); setup fetches
    nothing. Python packages go into the active virtual environment
    (else WORKSPACE/.venv, made here), and CMake packages into WORKSPACE/install with the
    arguments in WORKSPACE/colcon.meta. REPOSITORIES narrows the run to those entries, by
    manifest path or by name.
    """
    if clean and repositories:
        raise click.UsageError("--clean takes the whole workspace's outputs; it takes no names")
    from motion_spec.config import setting
    from motion_spec.setup import (
        INSTALL_DIRECTORY,
        SOURCE_DIRECTORY,
        STST_REPOSITORY,
        build_jobs,
        discover_packages,
        ignore_thirdparty,
        install_package,
        install_stst,
        manifest_files,
        manifest_in_force,
        missing_prerequisites,
        package_installed,
        source_state,
        stst_installed,
        target_environment,
        workspace,
        write_environment,
    )

    # a usage error: no stack
    try:
        root = workspace(workspace_argument)
        ros = bool(setting("ros.workspace", ros_flag, root).value)
    except (RuntimeError, ValueError) as exc:
        raise click.UsageError(str(exc)) from exc
    if clean:
        from motion_spec.setup import clean_workspace

        trashed = clean_workspace(root)
        for path, where in trashed:
            say("done", f"trashed {path}" + (f" → {where}" if where else ""))
        say("info", "sources and generations left in place" if trashed else "nothing to clean")
        return
    files = manifest_files(real)
    try:
        listed = manifest_in_force(files)
    except (OSError, ValueError) as exc:
        raise click.UsageError(str(exc)) from exc
    names = {r.path for r in listed} | {r.name for r in listed}
    unknown = [n for n in repositories if n not in names]
    if unknown:
        raise click.UsageError(
            f"not in {', '.join(str(f) for f in files)}: {', '.join(unknown)}; listed are "
            f"{', '.join(r.name for r in listed)}"
        )
    selected = [
        r for r in listed if not repositories or r.path in repositories or r.name in repositories
    ]
    jobs = build_jobs(jobs)
    prefix = root / INSTALL_DIRECTORY
    log = command_log(root, "setup")
    manifests = ", ".join(str(path) for path in files)
    # held to the end, so the final failure is logged too
    click.get_current_context().with_resource(mirrored_stderr(log))
    log_header(
        log,
        "setup",
        {
            "workspace": root,
            "manifests": manifests,
            "repositories": ", ".join(repository.name for repository in selected),
            "build type": build_type,
            "jobs": jobs,
            "python": "editable" if dev else "snapshot",
            "colcon": ros,
            **machine_facts(),
        },
    )
    say("info", f"workspace {root}")
    say("info", f"installing into {prefix}")
    say("info", f"manifests {manifests}")
    say("info", f"console log {log}")
    say("info", f"building with {jobs} job{'s' if jobs != 1 else ''}")
    import motion_spec

    # setup cannot check out the code running it
    sources = root / SOURCE_DIRECTORY
    if dev and sources.resolve() not in Path(motion_spec.__file__).resolve().parents:
        say(
            "warn",
            f"--dev leaves motion-spec itself installed from {Path(motion_spec.__file__).parent}; "
            f"clone it under {sources} and reinstall it with `pip install -e` "
            "to edit it too",
        )
    # before any build: setup cannot install these itself
    say("info", "checking prerequisites")
    packages, others = missing_prerequisites(selected, ros)
    if packages or others:
        for requirement in others:
            say("error", requirement)
        if packages:
            say("error", f"apt: sudo apt-get install -y {' '.join(packages)}")
        raise Reported("setup needs these before it can start; nothing was built.")
    if not any((sources / r.path / ".git").is_dir() for r in selected):
        raise Reported(
            f"nothing imported under {sources}: run "
            f"`vcs import {sources} < {files[0]}` first; nothing was built."
        )
    skipped: list[str] = []
    try:
        python = target_environment(root, ros, dev, log)
        say("info", f"python packages into {python}")
        if ros:
            ignore_thirdparty(root)
        for repository in selected:
            state = source_state(repository, root)
            if not state.usable:
                say("warn", f"skipped {repository.name}: {state.path} {state.reason}")
                skipped.append(repository.name)
                continue
            if state.drift:
                say("warn", f"{repository.name}: {state.path} {state.drift}")
            if repository.path == STST_REPOSITORY:
                already = not force and stst_installed(root)
                if not already:
                    say("step", "stst")
                launcher = install_stst(root, state, force=force, log=log)
                say(
                    "info" if already else "done",
                    f"stst {'already installed' if already else 'installed'}, launcher {launcher}",
                )
                continue
            packages = discover_packages(state.path)
            if not packages:
                say("warn", f"skipped {repository.name}: {state.path} holds no package to build")
                skipped.append(repository.name)
                continue
            for package in packages:
                if not force and package_installed(package.name, prefix, state.path):
                    say("info", f"{package.name} already installed, source {state.path}")
                    continue
                say("step", package.name)
                install_package(
                    package,
                    state,
                    root,
                    python,
                    fresh=force,
                    build_type=build_type,
                    log=log,
                    ros=ros,
                    jobs=jobs,
                    dev=dev,
                )
                say("done", f"{package.name} installed, source {state.path}")
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise internal_failure("setup failed", exc) from exc
    from motion_spec.config import write_sample

    sample = write_sample(root, declared=workspace_argument is not None, ros=ros)
    if sample:
        say("info", f"settings written to {sample}")
    try:
        written = write_environment(root, ros, python)
    except RuntimeError as exc:
        raise click.UsageError(str(exc)) from exc
    say("done", f"source {written} before generating, building or running")
    if skipped:
        say("error", f"not built: {', '.join(skipped)}")
        raise click.exceptions.Exit(1)


@main.command()
@click.argument(
    "features",
    nargs=-1,
    type=click.Choice(tuple(distribution("motion_spec").metadata.get_all("Provides-Extra") or [])),
)
def install(features: tuple[str, ...]) -> None:
    """Install optional FEATURES of motion-spec."""
    if not features:
        raise click.UsageError("name at least one feature")
    from motion_spec.setup import installer

    try:
        result = subprocess.run([*installer(), f"motion_spec[{','.join(features)}]"], check=False)
    except RuntimeError as exc:
        raise click.ClickException(str(exc)) from exc
    if result.returncode:
        raise click.ClickException("installation failed")


EXAMPLES_DIRECTORY = "ms-examples"


@main.command()
@click.option(
    "--into",
    type=click.Path(file_okay=False, path_type=Path),
    help=f"Where to copy them; defaults to WORKSPACE/src/{EXAMPLES_DIRECTORY}.",
)
def examples(into: Path | None) -> None:
    """Copy the example models that ship with motion-spec-dsl into the workspace.

    A file already at the destination is never touched: the copy is yours to edit.
    """
    from importlib.resources import files

    from motion_spec.setup import SOURCE_DIRECTORY, workspace

    try:
        source = Path(str(files("motion_spec_dsl") / "models"))
    except ImportError:
        source = None
    if source is None or not source.is_dir():
        raise click.ClickException(
            "no example models in this motion_spec_dsl: it predates the ones that ship with "
            "the package; reinstall it with `motion-spec setup --force motion-spec-dsl`"
        )
    if into is None:
        try:
            into = workspace() / SOURCE_DIRECTORY / EXAMPLES_DIRECTORY
        except RuntimeError as exc:
            raise click.UsageError(str(exc)) from exc
    destination = into.expanduser()
    copied, kept = [], []
    for path in source.rglob("*"):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        relative = path.relative_to(source)
        target = destination / relative
        if target.exists():
            kept.append(str(relative))
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        copied.append(str(relative))
    if kept:
        shown = ", ".join(kept[:3]) + (f", and {len(kept) - 3} more" if len(kept) > 3 else "")
        say("info", f"kept {len(kept)} file{'' if len(kept) == 1 else 's'} already there: {shown}")
    say("done", f"{len(copied)} model file{'' if len(copied) == 1 else 's'} copied to {into}")


@main.command()
@click.option(
    "--profile",
    "profiles",
    multiple=True,
    type=click.Choice(("base", "telemetry", "dsl", "codegen", "ros", "build", "runtime", "all")),
    default=("all",),
    show_default=True,
)
@click.option(
    "--target",
    "targets",
    type=click.Choice(("mujoco", "robif2b")),
    multiple=True,
    default=("mujoco", "robif2b"),
    show_default=True,
)
@environment_options
def health(
    profiles: tuple[str, ...], targets: tuple[str, ...], env_script: Path | None, no_env: bool
) -> None:
    """Report health of selected installation profiles."""
    from motion_spec.health import (
        ENVIRONMENT_DEFAULTS,
        apt_packages,
        check_health,
        environment_values,
        ros_summary,
        verdicts,
    )

    # the environment a build and run get, not this shell's
    env, _ = environment(env_script, no_env)

    interactive = sys.stderr.isatty()

    def progress(done: int, dependency: str) -> None:
        if interactive:
            click.echo(f"\r\033[2K  checking {dependency} ({done} done)", err=True, nl=False)

    checks = check_health(profiles, targets, env=env, on_progress=progress)
    if interactive:
        click.echo("\r\033[2K", err=True, nl=False)
    requirements = {
        "base": "required for every installation",
        "telemetry": "required for recording and replay",
        "dsl": "required for DSL generation",
        "codegen": "required for C++ generation",
        "ros": "required only for models with ROS communication",
        "build": "required to build generated C++",
        "runtime": "required to run generated controllers",
        "build[mujoco]": "required to build the MuJoCo target",
        "runtime[mujoco]": "required to run the MuJoCo target",
        "build[robif2b]": "required to build the robif2b target",
        "runtime[robif2b]": "required to run the robif2b target",
    }
    say("info", click.style("motion-spec health", bold=True, fg="magenta"), err=False)
    values = environment_values(env)
    say("info", click.style("ENVIRONMENT — what motion-spec reads", bold=True, fg="blue"), False)
    name_width = max(len(name) for name in values)
    for name, value in values.items():
        fallback = ENVIRONMENT_DEFAULTS.get(name)
        stamp("info" if value or fallback else "warn", err=False)
        click.secho(f"{name:<{name_width}}  ", fg="cyan", nl=False)
        if value:
            click.secho(value)
        else:
            click.secho(
                f"{fallback} (default)" if fallback else "unset", fg=None if fallback else "yellow"
            )
    current_profile = None
    dependency_width = max(len(check.dependency) for check in checks)
    what_width = max(len(check.what) for check in checks)
    for check in checks:
        if check.profile != current_profile:
            current_profile = check.profile
            click.echo()
            heading = f"{current_profile.upper()} — {requirements[current_profile]}"
            say("info", click.style(heading, bold=True, fg="blue"), err=False)
            if current_profile == "ros":
                say("warn", ros_summary(env), err=False)
        status = "OK" if check.ok else "ABSENT" if check.optional else "MISSING"
        colour = "green" if check.ok else "yellow" if check.optional else "red"
        stamp("info" if check.ok or check.optional else "error", err=False)
        click.secho(f"{status:<7}  ", fg=colour, bold=not check.ok, nl=False)
        click.secho(f"{check.dependency:<{dependency_width}}", fg="cyan", nl=False)
        click.echo(f"  {check.what:<{what_width}}  {check.path or '—'}")
        if not check.ok and not check.optional:
            # apt packages go into the one install line below
            fix = "" if check.apt else check.detail
            for label, value, colour in (
                ("why", check.why, "white"),
                ("source", check.source, "blue"),
                ("fix", fix, COMMAND_COLOUR),
                ("or", check.alternative if fix else "", COMMAND_COLOUR),
            ):
                if not value:
                    continue
                stamp("error", err=False)
                click.echo(" " * (STATUS_WIDTH + 4), nl=False)
                click.secho(f"{label:<6} ", fg="yellow", dim=True, nl=False)
                click.secho(value, fg=colour)
    missing = sum(not check.ok and not check.optional for check in checks)
    absent = sum(not check.ok and check.optional for check in checks)
    click.echo()
    stamp("error" if missing else "done", err=False)
    click.secho(f"{sum(check.ok for check in checks)} good", fg="green", nl=False)
    click.secho(f", {missing} missing", fg="red" if missing else None, nl=False)
    click.echo(f", {absent} absent by build option" if absent else "")
    for target, blocking in verdicts(checks).items():
        # by package: one install, not one per CMake target
        owners = list(dict.fromkeys(dependency.partition("::")[0] for dependency in blocking))
        named = ", ".join(owners[:3]) + (f" +{len(owners) - 3}" if len(owners) > 3 else "")
        stamp("error" if blocking else "done", err=False)
        click.secho(
            f"{target} NOT ready ({named})" if blocking else f"{target} ready",
            fg="red" if blocking else "green",
        )
    packages = apt_packages(checks)
    if packages:
        click.echo()
        say("info", click.style("Install what apt provides, in one line:", fg="blue"), False)
        stamp("info", err=False)
        click.secho(f"sudo apt-get install -y {' '.join(packages)}", fg=COMMAND_COLOUR)
    if missing:
        raise click.exceptions.Exit(1)


@main.command()
@click.argument("stage-or-model")
@click.argument(
    "model", required=False, type=click.Path(exists=True, dir_okay=False, path_type=Path)
)
@click.option(
    "--name",
    help="Name the generation tree under $MOTION_SPEC_GEN; defaults to the model's stem.",
)
def gen(stage_or_model: str, model: Path | None, name: str | None) -> None:
    """Generate IR or C++ from a .robmot MODEL; CODE is the default stage."""
    from rdf_utils.constraints import ConstraintViolation

    with requirements_reported():
        from motion_spec.generation.codegen import MissingAssets
        from motion_spec.generation.pipeline import generate_model

    if stage_or_model in {"ir", "code"}:
        if model is None:
            raise click.UsageError(f"MODEL is required after '{stage_or_model}'")
        stage = stage_or_model
    else:
        if model is not None:
            raise click.UsageError("use 'motion-spec gen [ir|code] MODEL.robmot'")
        stage = "code"
        model = Path(stage_or_model)
        if not model.is_file():
            raise click.BadParameter(f"file not found: {model}", param_hint="MODEL")
    if model.suffix != ".robmot":
        raise click.BadParameter("MODEL must be a .robmot file", param_hint="MODEL")
    try:
        generation = new_generation(model, name)
        with mirrored_stderr(generation / LOG_DIRECTORY / "console.log"):
            generate_model(model, generation, stage=stage)
    except ConstraintViolation as exc:
        raise model_rejected(exc) from exc
    except MissingAssets as exc:
        raise Reported(str(exc)) from exc
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise internal_failure("generation failed", exc) from exc
    say("done", f"generated {generation}")
    click.echo(generation)


@main.command()
@click.argument("generation", type=click.Path(exists=True, file_okay=False, path_type=Path))
@click.option(
    "--prefix",
    "prefixes",
    multiple=True,
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    help="Additional CMake package prefix; repeat as needed.",
)
@click.option("-j", "--jobs", type=click.IntRange(min=1))
@environment_options
def build(
    generation: Path,
    prefixes: tuple[Path, ...],
    jobs: int | None,
    env_script: Path | None,
    no_env: bool,
) -> None:
    """Configure and compile GENERATION/generated into GENERATION/build."""
    from motion_spec.generation.pipeline import build_generation

    generation = generation.resolve()
    env, _ = environment(env_script, no_env, generation)
    try:
        with mirrored_stderr(generation / LOG_DIRECTORY / "console.log"):
            executable = build_generation(generation, prefixes=prefixes, jobs=jobs, env=env)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise internal_failure("build failed", exc) from exc
    say("done", f"built {executable}")
    click.echo(executable)


@main.command()
@click.argument("manifest", type=click.Path(exists=True, dir_okay=False, path_type=Path))
def check(manifest: Path) -> None:
    """Validate MANIFEST against its SHACL constraints."""
    with requirements_reported():
        from motion_spec_dsl.vocab import APP
        from rdf_utils.constraints import SHACLViolation, check_shacl_constraints
        from rdf_utils.resolver import install_resolver
        from rdflib import Dataset
        from rdflib.util import guess_format

    install_resolver()
    dataset = Dataset(default_union=True)
    dataset.parse(str(manifest), format=guess_format(str(manifest)))
    shapes = {str(o) for _, _, o, _ in dataset.quads((None, APP["constraints"], None, None))}
    try:
        check_shacl_constraints(dataset, dict.fromkeys(shapes, "turtle"))
    except SHACLViolation as exc:
        click.echo(exc)
        raise click.exceptions.Exit(1) from exc
    say("done", f"{manifest.name} conforms")


@main.command()
@click.option("--init", "initialize", is_flag=True, help="Write a commented sample and exit.")
@click.option(
    "--workspace",
    "workspace_argument",
    type=click.Path(file_okay=False, path_type=Path),
    help="Workspace to write the sample into; defaults to $MOTION_SPEC_WS.",
)
def config(initialize: bool, workspace_argument: Path | None) -> None:
    """Show this workspace's settings, and where each one came from."""
    from motion_spec.config import CONFIG_FILE, KEYS, find_config, setting, write_sample
    from motion_spec.setup import workspace as resolve_workspace

    if initialize:
        try:
            root = resolve_workspace(workspace_argument)
        except RuntimeError as exc:
            raise click.UsageError(str(exc)) from exc
        written = write_sample(root, declared=workspace_argument is not None)
        if written:
            say("done", f"settings written to {written}")
            click.echo(written)
        else:
            say("info", f"{root / CONFIG_FILE} already exists, left as it is")
        return

    path = find_config()
    click.secho(f"file: {path or f'none found ({CONFIG_FILE})'}", fg="blue")
    defaults = {
        "workspace.root": path.parent if path else None,
        "workspace.generations": "<workspace>/generations",
        "workspace.environment": "the nearest setup-motion-spec file",
        "ros.workspace": False,
    }
    width = max(len(name) for name in KEYS)
    for name in KEYS:
        try:
            resolved = setting(name)
        except ValueError as exc:
            raise click.ClickException(str(exc)) from exc
        shown = resolved.value if resolved.value is not None else defaults.get(name)
        click.secho(f"  {name:<{width}}  ", fg="cyan", nl=False)
        click.secho(f"{shown}", nl=False)
        click.secho(f"  ({resolved.source})", fg="yellow", dim=True)


@main.command()
@click.argument("log", type=click.Path(path_type=Path))
@click.option("--jsonl", is_flag=True, help="Emit decoded frames as JSON Lines.")
@click.option(
    "--verify",
    is_flag=True,
    help="Check the run: every file its provenance records, the log against the "
    "generation's contract, and every frame decoded against what the writer reported.",
)
def replay(log: Path, jsonl: bool, verify: bool) -> None:
    """Inspect a recorded run LOG."""
    from motion_spec.runs.replay import decode_frames, summarize, verify_recording
    from motion_spec.runs.verify import RecordError

    try:
        if verify:
            say("done", f"run OK, {verify_recording(log)} frames")
        elif jsonl:
            for record in decode_frames(log):
                click.echo(json.dumps(record, separators=(",", ":")))
        else:
            click.echo(summarize(log))
    except RecordError as exc:
        raise click.ClickException(str(exc)) from exc


@main.command()
@click.argument("scenex", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option(
    "--config",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Robot config whose [<agent>] home puts each driven chain at its start pose.",
)
@click.option("--headless", is_flag=True, help="Compose and describe the scene; open no viewer.")
def view(scenex: Path, config: Path | None, headless: bool) -> None:
    """Compose a .scenex in MuJoCo and view it, without generating or building anything."""
    from rdf_utils.constraints import ConstraintViolation

    with requirements_reported():
        from motion_spec.view import view as view_scene

    try:
        view_scene(scenex, config, headless)
    except ConstraintViolation as exc:
        raise model_rejected(exc) from exc
    except FileNotFoundError as exc:
        raise click.ClickException(str(exc)) from exc


@main.command()
@click.argument("input", type=click.Path(exists=True, path_type=Path))
@click.option(
    "--prefix",
    "prefixes",
    multiple=True,
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    help="Install prefix the build resolves packages from. Repeatable.",
)
@click.option("-j", "--jobs", type=click.IntRange(min=1), help="Parallel build jobs.")
@click.option(
    "--name",
    help="Name the generation tree under $MOTION_SPEC_GEN; defaults to the model's stem. "
    "Requires a .robmot INPUT.",
)
@run_options
@environment_options
def run(
    input: Path,
    prefixes: tuple[Path, ...],
    jobs: int | None,
    name: str | None,
    run_id: str | None,
    cwd: Path | None,
    record: tuple[str, ...],
    no_log: bool,
    env_script: Path | None,
    no_env: bool,
) -> None:
    """Run a .robmot INPUT, generating and building it first, or an existing GENERATION.

    How a simulation runs -- headless, its steps, its pace, whether it starts paused -- is the
    [run] section of the robot config the model's deployment declares.
    """
    from rdf_utils.constraints import ConstraintViolation

    with requirements_reported():
        from motion_spec.generation.codegen import MissingAssets
        from motion_spec.generation.pipeline import build_generation, generate_model, new_id
        from motion_spec.runs.provenance import built_executable
        from motion_spec.runs.runner import RunnerError, run_cataloged
        from motion_spec.runs.verify import RecordError

    if input.suffix == ".robmot":
        try:
            generation = new_generation(input, name)
            env, environment_script = environment(env_script, no_env, generation)
            with mirrored_stderr(generation / LOG_DIRECTORY / "console.log"):
                say("step", "generating")
                generate_model(input, generation, stage="code", env=env)
                say("step", "building")
                executable = build_generation(generation, prefixes=prefixes, jobs=jobs, env=env)
                say("done", f"built {executable}")
        except ConstraintViolation as exc:
            raise model_rejected(exc) from exc
        except MissingAssets as exc:
            raise Reported(str(exc)) from exc
        except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
            raise internal_failure("pipeline failed", exc) from exc
    else:
        if not input.is_dir():
            raise click.BadParameter(
                "INPUT must be a .robmot model or a generation directory", param_hint="INPUT"
            )
        if prefixes or jobs is not None or name:
            raise click.UsageError("generation and build options require a .robmot INPUT")
        generation = input.resolve()
        executable = built_executable(generation / "generated")
        if executable is None:
            raise click.ClickException(
                f"{generation}: no build is recorded. Build it with `motion-spec build`."
            )
        env, environment_script = environment(env_script, no_env, generation)

    run_dir = generation / "runs" / (run_id or new_id("run"))
    say("step", f"running {generation.name}")
    say("info", f"run directory {run_dir}")
    # end a SIGTERM like Ctrl-C, so rec records the end
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        returncode = run_cataloged(
            run_dir,
            source_dir=generation / "generated",
            executable=executable,
            run_id=run_id,
            cwd=cwd,
            record=list(record),
            record_log=not no_log,
            env=env,
            environment_script=environment_script,
        )
    except (RecordError, RunnerError) as exc:
        raise click.ClickException(str(exc)) from exc
    if returncode:
        say("error", f"run failed ({returncode}) {run_dir}")
        raise click.exceptions.Exit(returncode)
    say("done", f"run {run_dir}")


@main.command()
@click.argument(
    "generation", required=False, type=click.Path(exists=True, file_okay=False, path_type=Path)
)
@run_options
@environment_options
@click.pass_context
def rerun(ctx: click.Context, generation: Path | None, **options) -> None:
    """Run a generation again, in a run of its own.

    `run` for the generation `latest` points at, so the timestamped path a `gen` just made need
    not be pasted back. Name a GENERATION to take that one instead. Generates and builds nothing.
    """
    if generation is None:
        link = generation_root() / LATEST_LINK
        if not link.is_dir():
            raise click.ClickException(
                f"{link}: nothing generated here to rerun. `gen` and `run` point it at what they "
                f"make, so generate once, or name a generation directory. Generations go under "
                f"${GENERATION_VARIABLE} when it is set, else ${{{WORKSPACE_VARIABLE}}}/generations."
            )
        from motion_spec.runs.provenance import built_executable

        generation = link.resolve()
        executable = built_executable(generation / "generated")
        if executable is None or not executable.is_file():
            raise click.ClickException(
                f"{generation}: the latest generation is not built. Build it with `motion-spec "
                "build`, or name a generation directory."
            )
    generation = generation.resolve()
    say("info", f"generation {generation}")
    ctx.invoke(run, input=generation, prefixes=(), jobs=None, name=None, **options)


@main.command()
@click.argument("run_dir", metavar="RUN", type=click.Path(exists=True, file_okay=False, path_type=Path))
def cancel(run_dir: Path) -> None:
    """Ask the run in RUN, while it is in progress, to stop.

    rec records it canceling; the runner running it stops the program within its grace period
    and ends the run, which rec records canceled.
    """
    from motion_spec.runs.provenance import rec_run

    try:
        rec_run(run_dir).cancel()
    except RuntimeError as exc:
        raise click.ClickException(str(exc)) from exc
    say("done", f"asked {run_dir.name} to stop")


@main.command()
@click.argument("run_dir", metavar="RUN", type=click.Path(exists=True, file_okay=False, path_type=Path))
@click.argument("action", type=click.Choice(["pause", "resume", "step", "speed", "status"]))
@click.argument("value", required=False, type=float)
def control(run_dir: Path, action: str, value: float | None) -> None:
    """Pause, resume, step or pace the simulated run in RUN while its loop runs.

    `step [N]` advances a paused run N ticks, 1 by default; `speed X` paces the simulation clock
    at X times real time, 0.1 to 10; `status` asks the loop to acknowledge and reports the block.
    A run started paused (`[run] start_paused`) waits here for `resume` or `step`.
    """
    from motion_spec.runs.control import control_run

    try:
        status = control_run(run_dir, action, value)
    except ValueError as exc:
        raise click.ClickException(str(exc)) from exc
    if not status["available"]:
        raise click.ClickException(
            f"{run_dir.name} has no control block: it is not a simulated run in progress"
        )
    say(
        "done" if status["alive"] else "warn",
        f"{run_dir.name}: {'paused' if status['paused'] else 'running'} at {status['speed']:g}x"
        + ("" if status["alive"] else ", and the loop did not acknowledge"),
    )


@main.command()
@click.argument(
    "generation_a", metavar="GEN_A", type=click.Path(exists=True, file_okay=False, path_type=Path)
)
@click.argument(
    "generation_b", metavar="GEN_B", type=click.Path(exists=True, file_okay=False, path_type=Path)
)
def compare(generation_a: Path, generation_b: Path) -> None:
    """Whether GEN_A and GEN_B were generated from the same input by the same tools.

    The same input is an isomorphic model graph and the same recorded input files, by hash; the
    same tools are the same recorded package versions and revisions. Then the IR and the code
    are the same up to order. Exits 1 on any difference. The ROS interface definitions a model
    reads at generation are not recorded, so they are not compared.
    """
    from rdf_utils.models.prov import get_file_path
    from rdflib import Graph
    from rdflib.compare import graph_diff, to_isomorphic
    from rdflib.namespace import DCTERMS, PROV, RDF, SDO

    from motion_spec.runs.provenance import (
        GENERATION_DOCUMENT,
        generation_model,
        read_generation_dataset,
    )

    recorded = []
    for generation in (generation_a, generation_b):
        try:
            model_dataset = generation_model(generation / "generated").graph
        except ValueError as exc:
            raise click.ClickException(str(exc)) from exc
        # bundles hold one generation's times and ids
        model, provenance = Graph(), Graph()
        document = read_generation_dataset(generation / "generated" / GENERATION_DOCUMENT)
        for graph in model_dataset.graphs():
            if (graph.identifier, RDF.type, PROV.Bundle) in graph:
                provenance += graph
            else:
                model += graph
        for graph in document.graphs():
            if (graph.identifier, RDF.type, PROV.Bundle) in graph:
                provenance += graph
        packages = {
            str(provenance.value(package, SDO.name)): (
                str(version),
                str(provenance.value(package, SDO.identifier)),
            )
            for package, version in provenance.subject_objects(SDO.softwareVersion)
        }
        # inputs: hashed files no activity generated and no tool wrote
        inputs = {}
        for entity, digest in provenance.subject_objects(DCTERMS.identifier):
            if (entity, PROV.wasGeneratedBy, None) in provenance or (
                entity, PROV.wasAttributedTo, None
            ) in provenance:
                continue
            try:
                inputs[str(digest)] = get_file_path(provenance, entity) or entity
            except ValueError:
                inputs[str(digest)] = entity
        recorded.append((to_isomorphic(model), packages, inputs))

    (model_a, packages_a, inputs_a), (model_b, packages_b, inputs_b) = recorded
    differences = []
    if model_a != model_b:
        _, only_a, only_b = graph_diff(model_a, model_b)
        differences.append(
            f"model graphs differ: {len(only_a)} triples only in GEN_A, {len(only_b)} only in GEN_B"
        )
    differences += [
        f"{name}: {packages_a.get(name)} in GEN_A, {packages_b.get(name)} in GEN_B"
        for name in packages_a.keys() | packages_b.keys()
        if packages_a.get(name) != packages_b.get(name)
    ]
    differences += [f"input only in GEN_A: {inputs_a[key]}" for key in inputs_a.keys() - inputs_b]
    differences += [f"input only in GEN_B: {inputs_b[key]}" for key in inputs_b.keys() - inputs_a]
    if not differences:
        say("done", "same model, inputs and tools: the IR and code are the same up to order")
        return
    for difference in differences:
        say("warn", difference)
    raise SystemExit(1)
