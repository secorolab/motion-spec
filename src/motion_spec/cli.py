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

from motion_spec.setup import BUILD_TYPE, BUILD_TYPE_VARIABLE, WORKSPACE_VARIABLE
from motion_spec.utils import (
    LEVEL_LABELS,
    LOG_DATEFMT,
    STAMP_COLOUR,
    command_log,
    generation_log,
    log_header,
    machine_facts,
    mirrored_stderr,
    paint,
    show_warning,
)


class _Reported(click.ClickException):
    """A failure already phrased for the reader, shown in the same format as every other line."""

    def show(self, file=None) -> None:
        for line in str(self.message).splitlines():
            _stamp("error")
            click.echo(line, err=True)


def _editable_here(root: Path) -> bool:
    """Whether this motion-spec is running from a checkout in ROOT's source tree."""
    import motion_spec

    running = Path(motion_spec.__file__).resolve()
    return (root / "src" / "motion-spec").resolve() in running.parents


def _internal_failure(what: str, exc: Exception) -> click.ClickException:
    """Report a failure that is ours, not the model's, with the stack that produced it.

    Anything reaching here is a defect in the generator or the environment, and the one thing
    needed to fix it -- where it came from -- is exactly what wrapping the message throws away.
    """
    traceback.print_exception(exc, file=sys.stderr)

    return click.ClickException(f"{what}: {exc}")


GENERATION_DIR_ENV = "MOTION_SPEC_GEN"


# Every line the CLI says about its own progress, so one command reads like the next. Results
# -- a generation path, an executable, a launcher -- stay bare on stdout for a caller to read.
def _stamp(level: str, err: bool = True) -> None:
    """The prefix every line carries, for one assembled in pieces."""
    click.echo(paint(time.strftime(LOG_DATEFMT), STAMP_COLOUR) + "  ", nl=False, err=err)
    click.echo(LEVEL_LABELS[level] + " ", nl=False, err=err)


def _say(level: str, message: str = "", err: bool = True) -> None:
    """One log line: when, how loud, and what.

    Progress goes to stderr; a report is the command's result and goes to stdout.
    """
    _stamp(level, err)
    click.echo(message, err=err)


# `  STATUS   ` — what a health row's detail lines indent by, to sit under the dependency.
_STATUS_WIDTH = 11
# A command to run, in a hue no name, source or status uses.
_COMMAND_COLOUR = 180
LATEST_LINK = "latest"


def _generation_base(output_dir: Path | None) -> Path | None:
    """Where a new generation goes: `-o` if given, else the workspace's generations root."""
    if output_dir is not None:
        return output_dir
    return _generation_root()


def _new_generation(model: Path, output_dir: Path | None, name: str | None = None) -> Path:
    """Create this run's generation directory, and say where it is before anything fills it.

    Announced up front rather than only on success: the DSL and the compiler write pages of
    their own output next, and where it all landed is the one thing needed to go look at it --
    including when the run fails partway and never reaches the closing line. Stderr, because
    `gen` writes that closing line to stdout for a caller to read.
    """
    from motion_spec.generation.pipeline import create_generation_dir

    generation = create_generation_dir(model, _generation_base(output_dir), name)
    _say("info", f"generation {generation}")
    _point_latest(generation)

    return generation


def _point_latest(generation: Path) -> None:
    """Point the one `latest` at the generation just made.

    A generation's directory is a timestamp nobody types twice, so the tree carries a name that
    does not change -- the way a colcon workspace has `log/latest`. One link, under the root
    `rerun` reads, because a link per output directory is a link nobody asked for. The target is
    relative, so moving the tree keeps it pointing at what it names, and it is replaced in one
    step, so a reader never finds it missing.
    """
    try:
        base = _generation_root()
    except click.UsageError:
        # An explicit -o works without a workspace; there is no default tree to link from.
        return
    link = base / LATEST_LINK
    try:
        target = os.path.relpath(generation, base)
        pending = link.with_name(f".{LATEST_LINK}.new")
        pending.unlink(missing_ok=True)
        pending.symlink_to(target, target_is_directory=True)
        os.replace(pending, link)
    except OSError:
        # A convenience, not the generation: a filesystem without symlinks, or a tree only
        # readable, must not fail a generation that otherwise worked. `rerun` says what to
        # do when it finds no link.
        link.with_name(f".{LATEST_LINK}.new").unlink(missing_ok=True)


def _generation_root() -> Path:
    """Where generations live: $MOTION_SPEC_GEN, else the workspace's generations/."""
    from motion_spec.setup import generations_root

    try:
        return generations_root()
    except RuntimeError as exc:
        raise click.UsageError(str(exc)) from exc


def _latest_generation() -> Path:
    """The generation `latest` points at under the configured generation root.

    Raises:
        ClickException: nothing has been generated there, or what was is not built.
    """
    base = _generation_root()
    link = base / LATEST_LINK
    if not link.is_dir():
        raise click.ClickException(
            f"{link}: nothing generated here to rerun. `gen` and `run` point it at what they "
            f"make, so generate once, or name a generation directory. Generations go under "
            f"${GENERATION_DIR_ENV} when it is set, else ${{{WORKSPACE_VARIABLE}}}/generations."
        )
    generation = link.resolve()
    if not (generation / "build" / "main").is_file():
        raise click.ClickException(
            f"{generation}: the latest generation is not built. Build it with `motion-spec "
            "build`, or name a generation directory."
        )

    return generation


def _model_rejected(exc: Exception) -> click.ClickException:
    """Report a model the checks refused, with the check that refused it.

    The message says what is wrong with the model and is the last line read. The stack says
    which check decided that, which is the only way to find the rule from the sentence it
    prints -- there are hundreds of them, and the sentence names none.
    """
    traceback.print_exception(exc, file=sys.stderr)

    return click.ClickException(f"the model was rejected: {exc}")


@contextmanager
def _manual_section(formatter: click.HelpFormatter, title: str):
    formatter.write(f"{click.style(title, bold=True)}\n")
    formatter.indent()
    yield
    formatter.dedent()
    formatter.write_paragraph()


class MotionSpecGroup(click.Group):
    """Render the top-level help as a Unix manual page."""

    def format_help(self, ctx: click.Context, formatter: click.HelpFormatter) -> None:
        with _manual_section(formatter, "NAME"):
            formatter.write_text(
                "motion-spec - validate, compile, run, and inspect motion specifications"
            )
        with _manual_section(formatter, "SYNOPSIS"):
            formatter.write_text("motion-spec [OPTIONS] COMMAND [ARGS]...")
        with _manual_section(formatter, "DESCRIPTION"):
            formatter.write_text(
                "motion-spec lowers motion-model JSON-LD to an intermediate representation, "
                "generates a C++ controller, and records or inspects controller executions."
            )
        with _manual_section(formatter, "WORKFLOW"):
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
                    ("run", "Execute the controller and create a recorded run archive."),
                    ("rerun", "Run the last generation again, without naming it."),
                    ("replay", "Verify, summarize, or recover data from a recorded run."),
                    ("view", "Compose a .scenex in MuJoCo and view it."),
                ]
            )
        with _manual_section(formatter, "ARTIFACTS"):
            formatter.write_dl(
                [
                    ("generated/source/", "Authored DSL inputs."),
                    ("generated/model/", "JSON-LD graphs, FSM artifacts, and IR."),
                    ("generated/controller/", "Generated C++ and CMake project."),
                    ("generated/contract/", "Schema, frame layout, and frame-log protocol."),
                    ("generated/provenance.ld.json", "DSL, coordinate and motion-spec provenance."),
                    ("build/", "Reusable compiled controller."),
                    ("runs/RUN/", "Run-owned logs, REC graph, and manifest."),
                    ("latest", f"Symlink to the newest generation, under ${GENERATION_DIR_ENV}."),
                ]
            )
        with _manual_section(formatter, "ENVIRONMENT"):
            formatter.write_dl(
                [
                    (
                        GENERATION_DIR_ENV,
                        (
                            "Where 'gen' and 'run' put a new generation when given no -o. "
                            "Unset, they use the workspace's generations directory. "
                            "'latest' there names the one 'rerun' takes."
                        ),
                    )
                ]
            )
        option_records = [
            record for param in self.get_params(ctx) if (record := param.get_help_record(ctx))
        ]
        if option_records:
            with _manual_section(formatter, "OPTIONS"):
                formatter.write_dl(option_records)
        command_records = []
        for name in self.list_commands(ctx):
            command = self.get_command(ctx, name)
            if command is not None and not command.hidden:
                command_records.append((name, command.help or command.get_short_help_str()))
        with _manual_section(formatter, "COMMANDS"):
            formatter.write_dl(command_records)
        with _manual_section(formatter, "EXAMPLES"):
            examples = (
                (
                    "Generate and build a model",
                    (
                        ("motion-spec gen model.robmot -o generation/demo",),
                        ("motion-spec build generation/demo",),
                    ),
                ),
                (
                    "Generate only the IR",
                    (("motion-spec gen ir model.robmot -o generation/demo-ir",),),
                ),
                (
                    "Generate, build, run, and inspect",
                    (
                        (
                            "motion-spec run model.robmot -o generation/demo \\",
                            "  --run-id run-1 --headless",
                        ),
                        ("motion-spec replay generation/demo/runs/run-1",),
                    ),
                ),
            )
            indent = " " * (formatter.current_indent + 2)
            for example_index, (label, commands) in enumerate(examples):
                formatter.write_text(click.style(label, bold=True))
                for command in commands:
                    for index, line in enumerate(command):
                        prompt = click.style("$ ", fg="cyan") if index == 0 else "  "
                        formatter.write(f"{indent}{prompt}{line}\n")
                if example_index < len(examples) - 1:
                    formatter.write_paragraph()
        with _manual_section(formatter, "SEE ALSO"):
            formatter.write_text("motion-spec COMMAND --help")


def _install_features() -> tuple[str, ...]:
    return tuple(sorted(distribution("motion_spec").metadata.get_all("Provides-Extra") or []))


# What arrives some other way than as a manifest entry of its own.
_INSTALLS = {"textx": "motion-spec setup motion-spec-dsl", "pyshacl": "pip install motion_spec"}


def _requirement_box(dependency: str) -> click.ClickException | None:
    """The dependency's box, when both its details and its install command are known."""
    from motion_spec.health import DETAILS, SETUP_PROVIDES, _remedy

    spec = DETAILS.get(dependency)
    install = _remedy(dependency) if dependency in SETUP_PROVIDES else _INSTALLS.get(dependency)
    if spec is None or install is None:
        return None
    return _missing(dependency, spec["why"], spec["source"], install)


def _missing(name: str, why: str, source: str, install: str) -> click.ClickException:
    """A dependency that is not there, reported as a box: what, why, where from, how to get it."""
    rows = [("why", why), ("source", source), ("install", install)]
    body = [f"│ {key:<8} {value}" for key, value in rows]
    width = max(len(name) + 14, *(len(line) for line in body)) + 1
    top = f"┌─ {name} not found " + "─" * (width - len(name) - 14)
    return click.ClickException("\n".join(["", top, *body, "└" + "─" * width]))


@contextmanager
def _requirements_reported():
    """Turn a missing optional dependency into its box instead of a raw traceback."""
    try:
        yield
    except ImportError as exc:
        box = _requirement_box((getattr(exc, "name", "") or "").partition(".")[0])
        if box is None:
            raise
        raise box from exc


@click.group(cls=MotionSpecGroup, no_args_is_help=True)
@click.version_option(package_name="motion_spec")
@click.pass_context
def main(ctx: click.Context) -> None:
    """The motion-spec toolchain."""
    # setup is what installs motion-spec-dsl, so it has to run before there is one.
    if ctx.invoked_subcommand != "setup":
        from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

        install_metamodel_resolver()
    _route_tool_logging()
    warnings.showwarning = show_warning


# The toolchain's own libraries, named so a third-party logger cannot make the CLI chatty.
_TOOL_LOGGERS = ("motion_spec", "motion_spec_dsl", "scene_dsl", "coord_dsl", "textx")


class _SayHandler(logging.Handler):
    """A library's log record, in the format every other line of the CLI is written in."""

    _LEVELS: ClassVar = {logging.WARNING: "warn", logging.ERROR: "error", logging.CRITICAL: "error"}

    def emit(self, record: logging.LogRecord) -> None:
        _say(self._LEVELS.get(record.levelno, "info"), record.getMessage())


def _route_tool_logging() -> None:
    handler = _SayHandler()
    for name in _TOOL_LOGGERS:
        logger = logging.getLogger(name)
        if any(isinstance(existing, _SayHandler) for existing in logger.handlers):
            continue
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False


def _clean(root: Path) -> None:
    """`setup --clean`: trash the workspace's builds, installs, logs and environment files."""
    from motion_spec.setup import clean_workspace

    trashed = clean_workspace(root)
    for path, where in trashed:
        _say("done", f"trashed {path}" + (f" → {where}" if where else ""))
    _say("info", "sources and generations left in place" if trashed else "nothing to clean")


def _environment_options(command):
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


def _run_options(command):
    """What one run of a built controller takes, for `run` and `rerun` alike."""
    for option in reversed(
        (
            click.option("--run-id", help="Name of this run's directory under <generation>/runs."),
            click.option(
                "--cwd",
                type=click.Path(exists=True, file_okay=False, path_type=Path),
                help="Working directory of the executable; scene assets resolve relative to it.",
            ),
            click.option("--headless", is_flag=True, help="Run without a GUI."),
            click.option(
                "--rtf",
                type=click.FloatRange(min=0.0),
                help="Real-time factor: 1.0 paces the loop to wall time, 0.5 half speed, 2.0 "
                "double. Headless runs uncapped (as fast as the machine allows) unless given; "
                "with a GUI the viewer's live speed setting applies. Only the wall duration "
                "changes -- the simulation clock advances one control period per tick either way.",
            ),
            click.option(
                "--start-paused",
                is_flag=True,
                help="Arm the run paused; the loop holds on its first tick until the dashboard "
                "resumes it.",
            ),
            click.option(
                "--record",
                "record",
                multiple=True,
                metavar="CAMERA",
                help="Record this camera to MP4 beside the log. Simulations render declared "
                "cameras; real runs record their declared ROS image topics. Repeatable.",
            ),
            click.option(
                "--steps", type=click.IntRange(min=1), help="Maximum headless simulation steps."
            ),
            click.option(
                "--no-log", is_flag=True, help="Do not write the frame log; the run has no replay."
            ),
        )
    ):
        command = option(command)
    return command


def _environment(
    env_script: Path | None, no_env: bool, start: Path | None = None
) -> tuple[dict[str, str] | None, dict | None]:
    """The environment to run under, and what a run archives about where it came from."""
    from motion_spec.setup import capture_environment, environment_provenance, find_environment

    if no_env:
        return None, None
    try:
        script = env_script or find_environment(start)
        if script is None:
            return None, None
        captured = capture_environment(script)
    except (OSError, RuntimeError) as exc:
        raise click.ClickException(str(exc)) from exc
    # A build under a different toolchain than the terminal should not have to be deduced.
    _say("info", f"environment {script}")
    return captured, environment_provenance(script, captured)


@main.command()
@click.option("--port", default=8080, show_default=True, help="Port to serve on.")
@click.option(
    "--logs",
    type=click.Path(file_okay=False, path_type=Path),
    help=f"Generation root to browse. Default: ${GENERATION_DIR_ENV}, else the working directory.",
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
@click.option(
    "--lan",
    is_flag=True,
    help="Reachable from the network: replay and simulated runs only, no delete or source access.",
)
@_environment_options
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
    lan: bool,
) -> None:
    """Browse generations, replay runs, and query them in a browser."""
    from motion_spec.dashboard.jobs import use_environment
    from motion_spec.dashboard.server import serve
    from motion_spec.setup import find_environment

    named_port = (
        click.get_current_context().get_parameter_source("port")
        is not click.core.ParameterSource.DEFAULT
    )
    if kill:
        return _stop_dashboards(port if named_port else None)
    if restart:
        # Stopping frees the port asynchronously; serving before it does would only report the
        # port as taken, so wait for the socket to actually let go.
        _stop_dashboards(port if named_port else None)
        for _ in range(50):
            if not _port_taken(port):
                break
            time.sleep(0.1)
    logs = (logs or _generation_root()).expanduser()
    if _port_taken(port):
        raise click.ClickException(
            f"port {port} is already serving. Stop it, or pass --port for a second dashboard."
        )
    script = None if no_env else (env_script or find_environment(logs))
    if not background:
        if script:
            use_environment(str(script))
            _say("info", f"environment {script}")
        return serve(port, logs, sources, lan)

    destination = (log_file or logs / "dashboard.log").expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    argv = [sys.argv[0], "dashboard", "--port", str(port), "--logs", str(logs)]
    if sources is not None:
        argv += ["--sources", str(sources)]
    argv += ["--env", str(script)] if script else ["--no-env"]
    if lan:
        argv.append("--lan")
    with destination.open("ab") as sink:
        child = subprocess.Popen(argv, stdout=sink, stderr=sink, start_new_session=True)
    _say("info", f"pid {child.pid}, logging to {destination}")
    _say("info", f"stop it with: kill {child.pid}")
    _say("done", f"dashboard http://127.0.0.1:{port}")
    click.echo(f"http://127.0.0.1:{port}")


def _stop_dashboards(port: int | None) -> None:
    """Stop the dashboards serving, on PORT or all, and the labs they started.

    Each records its pid on start, and its lab's; a lab whose dashboard had to be killed
    outright is stopped from that record too, or the next dashboard starts a second one.
    """
    from motion_spec.dashboard.roots import pidfile

    every = pidfile("*")
    ports = [port] if port else [path.stem.rpartition("-")[2] for path in every.parent.glob(every.name)]
    stopped = []
    for each in ports:
        for path in (pidfile(each), pidfile(each, "lab")):
            pid = _recorded_pid(path)
            if pid is not None and _stop(pid):
                stopped.append(pid)
            path.unlink(missing_ok=True)
    if not stopped:
        where = f" on port {port}" if port else ""
        raise click.ClickException(f"no dashboard is serving{where}.")
    _say("done", f"stopped {', '.join(map(str, stopped))}")


def _recorded_pid(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return None


def _stop(pid: int) -> bool:
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


def _port_taken(port: int) -> bool:
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

    Sources come in by `vcs import`, Python packages go into the active virtual environment
    (else WORKSPACE/.venv, made here), and CMake packages into WORKSPACE/install with the
    arguments in WORKSPACE/colcon.meta. REPOSITORIES narrows the run to those entries, by
    manifest path or by name.
    """
    if clean and repositories:
        raise click.UsageError("--clean takes the whole workspace's outputs; it takes no names")
    from motion_spec.config import setting
    from motion_spec.setup import (
        STST_REPOSITORY,
        _ignore_thirdparty,
        build_jobs,
        discover_packages,
        import_sources,
        install_package,
        install_prefix,
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

    # A mistake in the command, not a failure inside it: no stack.
    try:
        root = workspace(workspace_argument)
        ros = bool(setting("ros.workspace", ros_flag, root).value)
    except (RuntimeError, ValueError) as exc:
        raise click.UsageError(str(exc)) from exc
    if clean:
        _clean(root)
        return
    files = manifest_files(real)
    try:
        listed = manifest_in_force(files)
    except (OSError, ValueError) as exc:
        raise click.UsageError(str(exc)) from exc
    unknown = [n for n in repositories if not any(n in (r.path, r.name) for r in listed)]
    if unknown:
        raise click.UsageError(
            f"not in {', '.join(str(f) for f in files)}: {', '.join(unknown)}; listed are "
            f"{', '.join(r.name for r in listed)}"
        )
    selected = [r for r in listed if not repositories or {r.path, r.name} & set(repositories)]
    jobs = build_jobs(jobs)
    prefix = install_prefix(root)
    log = command_log(root, "setup")
    # Held to the end of the command, so the failure that ends it is in the log too.
    click.get_current_context().with_resource(mirrored_stderr(log))
    log_header(
        log,
        "setup",
        {
            "workspace": root,
            "manifests": ", ".join(str(path) for path in files),
            "repositories": ", ".join(repository.name for repository in selected),
            "build type": build_type,
            "jobs": jobs,
            "python": "editable" if dev else "snapshot",
            "colcon": ros,
            **machine_facts(),
        },
    )
    _say("info", f"workspace {root}")
    _say("info", f"installing into {prefix}")
    _say("info", f"manifests {', '.join(str(path) for path in files)}")
    _say("info", f"console log {log}")
    _say("info", f"building with {jobs} job{'s' if jobs != 1 else ''}")
    # setup cannot check itself out: it is the code running. So it says so instead.
    if dev and not _editable_here(root):
        import motion_spec

        _say(
            "warn",
            f"--dev leaves motion-spec itself installed from {Path(motion_spec.__file__).parent}; "
            f"clone it into {root / 'src' / 'motion-spec'} and reinstall it with `pip install -e` "
            "to edit it too",
        )
    # Before the first import: a prerequisite setup cannot install itself is a failure the
    # operator has to act on, and finding it after four checkouts and a build helps nobody.
    _say("info", "checking prerequisites")
    packages, others = missing_prerequisites(selected, ros)
    if packages or others:
        for requirement in others:
            _say("error", requirement)
        if packages:
            _say("error", f"apt: sudo apt-get install -y {' '.join(packages)}")
        raise _Reported("setup needs these before it can start; nothing was imported or built.")
    skipped: list[str] = []
    try:
        python = target_environment(root, ros, dev, log)
        _say("info", f"python packages into {python}")
        if ros:
            _ignore_thirdparty(root)
        imported = import_sources(files, listed, root, log)
        for repository in selected:
            state = source_state(repository, root, repository.path in imported)
            if not state.usable:
                _say("warn", f"skipped {repository.name}: {state.path} {state.reason}")
                skipped.append(repository.name)
                continue
            if state.drift:
                _say("warn", f"{repository.name}: {state.path} {state.drift}")
            if repository.path == STST_REPOSITORY:
                already = not force and stst_installed(root)
                if not already:
                    _say("step", "stst")
                launcher = install_stst(root, state, force=force, log=log)
                _say(
                    "info" if already else "done",
                    f"stst {'already installed' if already else 'installed'}, launcher {launcher}",
                )
                continue
            packages = discover_packages(state.path)
            if not packages:
                _say("warn", f"skipped {repository.name}: {state.path} holds no package to build")
                skipped.append(repository.name)
                continue
            for package in packages:
                if not force and package_installed(package.name, prefix, state.path):
                    _say("info", f"{package.name} already installed, source {state.path}")
                    continue
                # Announced before the build, so its console output has a heading.
                _say("step", package.name)
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
                _say("done", f"{package.name} installed, source {state.path}")
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("setup failed", exc) from exc
    from motion_spec.config import write_sample

    sample = write_sample(root, declared=workspace_argument is not None, ros=ros)
    if sample:
        _say("info", f"settings written to {sample}")
    try:
        written = write_environment(root, ros, python)
    except RuntimeError as exc:
        raise click.UsageError(str(exc)) from exc
    _say("done", f"source {written} before generating, building or running")
    if skipped:
        # Each said why above; a caller that asked for them did not get them.
        _say("error", f"not built: {', '.join(skipped)}")
        raise click.exceptions.Exit(1)


@main.command()
@click.argument("features", nargs=-1, type=click.Choice(_install_features()))
def install(features: tuple[str, ...]) -> None:
    """Install optional FEATURES of motion-spec."""
    if not features:
        raise click.UsageError("name at least one feature")
    requirements = [f"motion_spec[{','.join(sorted(features))}]"]
    from motion_spec.setup import installer

    try:
        result = subprocess.run([*installer(), *requirements], check=False)
    except RuntimeError as exc:
        raise click.ClickException(str(exc)) from exc
    if result.returncode:
        raise click.ClickException("installation failed")


# Where `examples` puts them, under the developer's tree: they are yours to edit from the
# moment they land, which is the whole reason they are copied out of the package.
EXAMPLES_DIRECTORY = "ms-examples"


def _packaged_models() -> Path | None:
    """The example models shipped inside motion-spec-dsl, wherever pip put the package."""
    from importlib.resources import files

    try:
        models = Path(str(files("motion_spec_dsl") / "models"))
    except (ImportError, ModuleNotFoundError):
        return None
    return models if models.is_dir() else None


def _copy_examples(source: Path, destination: Path) -> tuple[list[str], list[str]]:
    """Copy what is not there yet, and report what was left alone."""
    copied, kept = [], []
    for path in sorted(source.rglob("*")):
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
    return copied, kept


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
    from motion_spec.setup import SOURCE_DIRECTORY, workspace

    source = _packaged_models()
    if source is None:
        raise click.ClickException(
            "no example models in this motion_spec_dsl: it predates the ones that ship with "
            "the package; reinstall it with `motion-spec setup --force motion_spec_dsl`"
        )
    if into is None:
        try:
            into = workspace() / SOURCE_DIRECTORY / EXAMPLES_DIRECTORY
        except RuntimeError as exc:
            raise click.UsageError(str(exc)) from exc
    copied, kept = _copy_examples(source, into.expanduser())
    if kept:
        shown = ", ".join(kept[:3]) + (f", and {len(kept) - 3} more" if len(kept) > 3 else "")
        _say("info", f"kept {len(kept)} file{'' if len(kept) == 1 else 's'} already there: {shown}")
    _say("done", f"{len(copied)} model file{'' if len(copied) == 1 else 's'} copied to {into}")


@main.command()
@click.option(
    "--profile",
    "profiles",
    multiple=True,
    type=click.Choice(
        ("base", "telemetry", "dsl", "codegen", "ros", "build", "runtime", "all")
    ),
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
@_environment_options
def health(
    profiles: tuple[str, ...], targets: tuple[str, ...], env_script: Path | None, no_env: bool
) -> None:
    """Report health of selected installation profiles."""
    from motion_spec.health import (
        APT_REMEDY,
        ENVIRONMENT_DEFAULTS,
        apt_packages,
        check_health,
        environment_values,
        ros_summary,
        verdicts,
    )

    # The environment a build and a run would be given, not this shell's.
    env, _ = _environment(env_script, no_env)

    def progress(done: int, dependency: str) -> None:
        # Overwritten in place on stderr, so the report itself stays clean on stdout.
        click.echo(f"\r\033[2K  checking {dependency} ({done} done)", err=True, nl=False)

    interactive = sys.stderr.isatty()
    checks = check_health(profiles, targets, env=env, on_progress=progress if interactive else None)
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
    _say("info", click.style("motion-spec health", bold=True, fg="magenta"), err=False)
    values = environment_values(env)
    _say("info", click.style("ENVIRONMENT — what motion-spec reads", bold=True, fg="blue"), False)
    name_width = max(len(name) for name in values)
    for name, value in values.items():
        fallback = ENVIRONMENT_DEFAULTS.get(name)
        _stamp("info" if value or fallback else "warn", err=False)
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
            _say("info", click.style(heading, bold=True, fg="blue"), err=False)
            # ABSENT means one thing with no distribution installed, another with two unsourced.
            if current_profile == "ros":
                _say("warn", ros_summary(env), err=False)
        status = "OK" if check.ok else "ABSENT" if check.optional else "MISSING"
        colour = "green" if check.ok else "yellow" if check.optional else "red"
        _stamp("info" if check.ok or check.optional else "error", err=False)
        click.secho(f"{status:<7}  ", fg=colour, bold=not check.ok, nl=False)
        click.secho(f"{check.dependency:<{dependency_width}}", fg="cyan", nl=False)
        click.echo(f"  {check.what:<{what_width}}  {check.path or '—'}")
        if not check.ok and not check.optional:
            # What apt provides is gathered into the one install line below instead.
            fix = "" if check.detail.startswith(APT_REMEDY) else check.detail
            # Magenta is a command to run; cyan is a name, and the two sat side by side.
            for label, value, colour in (
                ("why", check.why, "white"),
                ("source", check.source, "blue"),
                ("fix", fix, _COMMAND_COLOUR),
                ("or", check.alternative if fix else "", _COMMAND_COLOUR),
            ):
                if not value:
                    continue
                _stamp("error", err=False)
                click.echo(" " * (_STATUS_WIDTH + 4), nl=False)
                click.secho(f"{label:<6} ", fg="yellow", dim=True, nl=False)
                click.secho(value, fg=colour)
    missing = sum(not check.ok and not check.optional for check in checks)
    absent = sum(not check.ok and check.optional for check in checks)
    click.echo()
    _stamp("error" if missing else "done", err=False)
    click.secho(f"{sum(check.ok for check in checks)} good", fg="green", nl=False)
    click.secho(f", {missing} missing", fg="red" if missing else None, nl=False)
    # An absent optional is a fact about this workspace, not a fault: say it, do not fail on it.
    click.echo(f", {absent} absent by build option" if absent else "")
    # A line each: one target being short of a driver says nothing about the other.
    for target, blocking in verdicts(checks).items():
        # By package: three missing robif2b targets are one thing to install, not three.
        owners = sorted({dependency.split("::")[0].split()[0] for dependency in blocking})
        named = ", ".join(owners[:3]) + (f" +{len(owners) - 3}" if len(owners) > 3 else "")
        _stamp("error" if blocking else "done", err=False)
        click.secho(
            f"{target} NOT ready ({named})" if blocking else f"{target} ready",
            fg="red" if blocking else "green",
        )
    packages = apt_packages(checks)
    if packages:
        click.echo()
        _say("info", click.style("Install what apt provides, in one line:", fg="blue"), False)
        _stamp("info", err=False)
        click.secho(f"sudo apt-get install -y {' '.join(packages)}", fg=_COMMAND_COLOUR)
    if missing:
        raise click.exceptions.Exit(1)


@main.command()
@click.argument("stage-or-model")
@click.argument(
    "model", required=False, type=click.Path(exists=True, dir_okay=False, path_type=Path)
)
@click.option("-o", "--output-dir", type=click.Path(file_okay=False, path_type=Path))
@click.option(
    "--name",
    help="Name the generation tree under the base directory; defaults to the model's stem.",
)
def gen(stage_or_model: str, model: Path | None, output_dir: Path | None, name: str | None) -> None:
    """Generate IR or C++ from a .robmot MODEL; CODE is the default stage."""
    from rdf_utils.constraints import ConstraintViolation

    with _requirements_reported():
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
        generation = _new_generation(model, output_dir, name)
        with mirrored_stderr(generation_log(generation)):
            generate_model(model, generation, stage=stage)
    except ConstraintViolation as exc:
        raise _model_rejected(exc) from exc
    except MissingAssets as exc:
        raise _Reported(str(exc)) from exc
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("generation failed", exc) from exc
    _say("done", f"generated {generation}")
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
@_environment_options
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
    env, _ = _environment(env_script, no_env, generation)
    try:
        with mirrored_stderr(generation_log(generation)):
            executable = build_generation(generation, prefixes=prefixes, jobs=jobs, env=env)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("build failed", exc) from exc
    _say("done", f"built {executable}")
    click.echo(executable)


@main.command()
@click.argument("manifest", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option(
    "--meta-shacl", is_flag=True, help="Validate the SHACL shape graph against SHACL-of-SHACL too."
)
def check(manifest: Path, meta_shacl: bool) -> None:
    """Validate MANIFEST against its SHACL constraints."""
    with _requirements_reported():
        from motion_spec_dsl.rdf_parser.check import validate_manifest

    conforms, report = validate_manifest(manifest, meta_shacl=meta_shacl)
    click.echo(report)
    if not conforms:
        raise click.exceptions.Exit(1)


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
            _say("done", f"settings written to {written}")
            click.echo(written)
        else:
            _say("info", f"{root / CONFIG_FILE} already exists, left as it is")
        return

    path = find_config()
    click.secho(f"file: {path or f'none found ({CONFIG_FILE})'}", fg="blue")
    # What each key falls back to when nothing sets it, as the commands that read it decide.
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
    help="Check the archive: every file its manifest names, the log against the generation's "
    "contract, and every frame decoded against what the writer reported.",
)
def replay(log: Path, jsonl: bool, verify: bool) -> None:
    """Inspect a recorded run LOG."""
    from motion_spec.runs.archive import ArchiveError
    from motion_spec.runs.replay import decode_frames, summarize, verify_archive

    try:
        if verify:
            _say("done", f"archive OK, {verify_archive(log)} frames")
        elif jsonl:
            for record in decode_frames(log):
                click.echo(json.dumps(record, separators=(",", ":")))
        else:
            click.echo(summarize(log))
    except ArchiveError as exc:
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

    with _requirements_reported():
        from motion_spec.view import view as view_scene

    try:
        view_scene(scenex, config, headless)
    except ConstraintViolation as exc:
        raise _model_rejected(exc) from exc
    except FileNotFoundError as exc:
        raise click.ClickException(str(exc)) from exc


def _require_devices(generation: Path) -> None:
    """Refuse a hardware run whose devices do not answer, before anything names a run.

    A driver handed an address nothing listens on does not fail, it waits -- so the run would
    exist as a directory with no frames in it, and have to be killed. Knocking first costs one
    connect attempt per endpoint and turns that into a sentence.
    """
    from motion_spec.devices import probe_devices, unreachable, where

    missing = unreachable(probe_devices(generation))
    if not missing:
        return
    raise click.ClickException(
        "these devices did not answer:\n"
        + "\n".join(f"  {device['name']} at {where(device)}" for device in missing)
    )


def _is_simulated(generation: Path) -> bool:
    """Whether this generation's controller drives a simulator.

    The platform the model declared, read back from the IR the generation carries -- the backend
    token is an implementation detail and never the thing asked about.
    """
    ir_path = generation / "generated" / "model" / "ir.json"
    if not ir_path.is_file():
        raise click.ClickException(f"generation has no IR to read the platform from: {ir_path}")
    platform = json.loads(ir_path.read_text())["configuration"]["platform"]
    return bool(platform["simulated"])


@main.command(context_settings={"ignore_unknown_options": True})
@click.argument("input", type=click.Path(exists=True, path_type=Path))
@click.option(
    "-o",
    "--output-dir",
    type=click.Path(file_okay=False, path_type=Path),
    help="Base directory for a new generation (<model>/<timestamp> is appended). "
    "Rejected when INPUT is an existing generation.",
)
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
    help="Name the generation tree under the base directory; defaults to the model's stem. "
    "Requires a .robmot INPUT.",
)
@_run_options
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
@_environment_options
def run(
    input: Path,
    output_dir: Path | None,
    prefixes: tuple[Path, ...],
    jobs: int | None,
    name: str | None,
    run_id: str | None,
    cwd: Path | None,
    headless: bool,
    rtf: float | None,
    start_paused: bool,
    record: tuple[str, ...],
    steps: int | None,
    no_log: bool,
    executable_args: tuple[str, ...],
    env_script: Path | None,
    no_env: bool,
) -> None:
    """Run a .robmot INPUT, generating and building it first, or an existing GENERATION.

    Anything after the options that this command does not recognise is handed to the
    executable unchanged.
    """
    from rdf_utils.constraints import ConstraintViolation

    with _requirements_reported():
        from motion_spec.generation.codegen import MissingAssets
        from motion_spec.generation.pipeline import build_generation, generate_model, new_id
        from motion_spec.runs.archive import ArchiveError
        from motion_spec.runs.runner import RunnerError, run_cataloged

    if steps is not None and not headless:
        raise click.UsageError("--steps requires --headless")
    if input.suffix == ".robmot":
        try:
            generation = _new_generation(input, output_dir, name)
            env, environment_record = _environment(env_script, no_env, generation)
            with mirrored_stderr(generation_log(generation)):
                _say("step", "generating")
                generate_model(input, generation, stage="code", env=env)
                _say("step", "building")
                executable = build_generation(generation, prefixes=prefixes, jobs=jobs, env=env)
                _say("done", f"built {executable}")
        except ConstraintViolation as exc:
            raise _model_rejected(exc) from exc
        except MissingAssets as exc:
            raise _Reported(str(exc)) from exc
        except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
            raise _internal_failure("pipeline failed", exc) from exc
    else:
        if not input.is_dir():
            raise click.BadParameter(
                "INPUT must be a .robmot model or a generation directory", param_hint="INPUT"
            )
        if output_dir is not None or prefixes or jobs is not None or name:
            raise click.UsageError("generation and build options require a .robmot INPUT")
        generation = input.resolve()
        env, environment_record = _environment(env_script, no_env, generation)

    if (headless or steps is not None) and not _is_simulated(generation):
        raise click.UsageError(
            "--headless and --steps are simulator options; this generation runs on hardware"
        )
    # Probing before a hardware run is off for now: start the controller directly and let a
    # device that does not answer say so in the run's own console.
    # if not _is_simulated(generation):
    #     _require_devices(generation)

    run_dir = generation / "runs" / (run_id or new_id("run"))
    _say("step", f"running {generation.name}")
    _say("info", f"run directory {run_dir}")
    arguments = (
        (["--headless"] if headless else [])
        + (["--steps", str(steps)] if steps is not None else [])
        + (["--rtf", str(rtf)] if rtf is not None else [])
        + (["--start-paused"] if start_paused else [])
        + list(executable_args)
    )
    try:
        returncode = run_cataloged(
            run_dir,
            source_dir=generation / "generated",
            executable=generation / "build" / "main",
            executable_args=arguments,
            run_id=run_id,
            cwd=cwd,
            record=list(record),
            record_log=not no_log,
            env=env,
            environment_provenance=environment_record,
        )
    except (ArchiveError, RunnerError) as exc:
        raise click.ClickException(str(exc)) from exc
    # Said before the exit: an interrupt leaves a complete archive, and that is when the
    # caller most needs the path.
    if returncode:
        _say("error", f"run failed ({returncode}) {run_dir}")
        raise click.exceptions.Exit(returncode)
    _say("done", f"run {run_dir}")


@main.command(context_settings={"ignore_unknown_options": True})
@click.argument(
    "generation", required=False, type=click.Path(exists=True, file_okay=False, path_type=Path)
)
@_run_options
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
@_environment_options
@click.pass_context
def rerun(ctx: click.Context, generation: Path | None, **options) -> None:
    """Run a generation again, in a run of its own.

    `run` for the generation `latest` points at, so the timestamped path a `gen` just made need
    not be pasted back. Name a GENERATION to take that one instead. Generates and builds nothing.
    """
    generation = (generation or _latest_generation()).resolve()
    _say("info", f"generation {generation}")
    ctx.invoke(
        run, input=generation, output_dir=None, prefixes=(), jobs=None, name=None, **options
    )
