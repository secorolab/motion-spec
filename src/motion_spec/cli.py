# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Unified Click command-line interface for motion-spec."""

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
from importlib.metadata import PackageNotFoundError, distribution
from importlib.util import find_spec
from pathlib import Path
from typing import ClassVar
from urllib.parse import urlparse

import click

from motion_spec.setup import (
    BUILD_TYPE,
    BUILD_TYPE_VARIABLE,
    COMPONENT_NAMES,
    COMPONENT_OPTIONS,
    DEFAULT_COMPONENTS,
    WORKSPACE_VARIABLE,
)
from motion_spec.utils import (
    LEVEL_LABELS,
    LOG_DATEFMT,
    STAMP_COLOUR,
    command_log,
    generation_log,
    human_bytes,
    log_header,
    machine_facts,
    mirrored_stderr,
    paint,
    show_warning,
    trash_if_present,
    tree_size,
)


class _Reported(click.ClickException):
    """A failure already phrased for the reader, shown in the same format as every other line."""

    def show(self, file=None) -> None:
        for line in str(self.message).splitlines():
            _stamp("error")
            click.echo(line, err=True)


def _clean_offer(name: str, root: Path, prefix: Path) -> list[tuple[str, int]]:
    """What removing one component would take, as the lines the prompt shows before asking."""
    from motion_spec.setup import (
        COMPONENTS_BY_NAME,
        build_directory,
        install_marker,
        installed_files,
    )

    entries = []
    build = build_directory(root, name)
    if build.is_dir():
        entries.append((_under(build, root), tree_size(build)))
    component = COMPONENTS_BY_NAME.get(name)
    if component is None:
        launcher = prefix / "bin" / "stst"
        if launcher.is_file():
            entries.append((_under(launcher, root), tree_size(launcher)))
    elif files := installed_files(component, root, prefix):
        entries.append(
            (
                f"{len(files)} installed file{'' if len(files) == 1 else 's'} under "
                f"{_under(prefix, root)}",
                sum(tree_size(path) for path in files),
            )
        )
    marker = install_marker(name, prefix)
    if marker.is_file():
        entries.append((_under(marker, root), tree_size(marker)))
    return entries


def _under(path: Path, root: Path) -> str:
    """PATH as the workspace spells it, so a prompt is not a wall of absolute paths."""
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def _confirm_removal(title: str, entries: list[tuple[str, int]], assume_yes: bool) -> bool:
    """Show what would go, with its size, and ask. Nothing is taken on silence."""
    if not entries:
        return False
    _say("step", title)
    for label, size in entries:
        click.echo(f"  {label:<44} {human_bytes(size):>10}", err=True)
    if assume_yes:
        return True
    try:
        return click.confirm("  remove?", default=False, err=True)
    except click.Abort:
        # End of input rather than an answer: a script reached a prompt it cannot see.
        raise click.UsageError(
            "nothing answered the prompt: pass --yes to clean unattended"
        ) from None


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
LAB_SETTINGS_DIR = "motion-spec-lab-settings"


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
                    ("setup", "Install the pinned tools and libraries needed to build controllers."),
                    ("install", "Install the optional DSL, dashboard, or replay features."),
                    ("health", "Check the tools and libraries required by the selected target."),
                    ("gen", "Generate IR or C++ from a .robmot model."),
                    ("build", "Configure and compile a generation."),
                    ("run", "Execute the controller and create a recorded run archive."),
                    ("rerun", "Run the last generation again, without naming it."),
                    ("replay", "Verify, summarize, or recover data from a recorded run."),
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
    extras = distribution("motion_spec").metadata.get_all("Provides-Extra") or []
    return (*sorted(extras), "dsl")


def _editable_source_root() -> Path | None:
    """Editable-install source root from the distribution's direct_url.json, or None."""
    try:
        direct_url = distribution("motion_spec").read_text("direct_url.json")
    except (PackageNotFoundError, FileNotFoundError):
        return None
    if not direct_url:
        return None
    try:
        url = json.loads(direct_url).get("url", "")
    except json.JSONDecodeError:
        return None
    parsed = urlparse(url)
    return Path(parsed.path) if parsed.scheme == "file" else None


# None of these are on PyPI: motion-spec-dsl and the compilers it depends on each come
# from a sibling checkout when there is one, from GitHub otherwise.
_DSL_REPOS = {
    "motion_spec_dsl": "motion-spec-dsl",
    "coord_dsl": "coord-dsl",
    "scene_dsl": "scene-dsl",
}


def _dsl_requirements() -> list[str]:
    root = _editable_source_root()
    requirements = []
    for module, repo in _DSL_REPOS.items():
        # An already-importable dependency stays as installed; pip would rebuild it from git.
        if module != "motion_spec_dsl" and find_spec(module) is not None:
            continue
        sibling = root.parent / repo if root else None
        requirements.append(
            str(sibling)
            if sibling and sibling.is_dir()
            else f"{module} @ git+https://github.com/secorolab/{repo}.git"
        )
    return requirements


# How each optional dependency arrives; its why and source live in health.DETAILS, once.
_INSTALLS = {
    "motion_spec_dsl": "motion-spec install dsl",
    "coord_dsl": "motion-spec install dsl",
    "scene_dsl": "motion-spec install dsl",
    "textx": "motion-spec install dsl",
    "pyshacl": "pip install motion_spec",
    "rec": "pip install motion_spec",
    "google.protobuf": "pip install motion_spec",
    "stst": "motion-spec setup",
}


def _requirement_box(dependency: str) -> click.ClickException | None:
    """The dependency's box, when both its details and its install command are known."""
    from motion_spec.health import DETAILS

    spec = DETAILS.get(dependency)
    install = _INSTALLS.get(dependency)
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
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    from motion_spec.introspection.journal import record

    # Before the work, so a command that never returned is still in the record.
    record(ctx.invoked_subcommand or "")
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


def _clear_shadowing_stst(prefix: Path) -> None:
    """Trash a broken launcher an older motion-spec left on PATH; warn about a working one."""
    from motion_spec.setup import shadowing_stst
    from motion_spec.utils import trash

    found = shadowing_stst(prefix)
    if found is None:
        return
    other, has_jar = found
    if has_jar:
        _say("warn", f"PATH reaches {other} before {prefix / 'bin' / 'stst'}; remove it or reorder")
        return
    trash(other)
    _say("done", f"trashed {other}, an older install whose jar is gone")


def _cmake_options(component, configured: dict, extra: tuple[str, ...]) -> tuple[str, ...]:
    """What a component is built with: its own list unless the config names one, plus extras."""
    declared = configured.get(component.name)
    return (*(component.options if declared is None else declared), *extra)


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
) -> None:
    """Browse generations, replay runs, and query them in a browser."""
    from motion_spec.dashboard.jobs import use_environment
    from motion_spec.dashboard.server import serve
    from motion_spec.setup import find_environment

    named_port = "--port" in sys.argv or "-p" in sys.argv
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
        return serve(port, logs, sources)

    destination = (log_file or logs / "dashboard.log").expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    argv = [sys.executable, "-m", "motion_spec.dashboard", "--port", str(port), "--logs", str(logs)]
    if sources is not None:
        argv += ["--sources", str(sources)]
    if script:
        argv += ["--env", str(script)]
    with destination.open("ab") as sink:
        child = subprocess.Popen(argv, stdout=sink, stderr=sink, start_new_session=True)
    _say("info", f"pid {child.pid}, logging to {destination}")
    _say("info", f"stop it with: kill {child.pid}")
    _say("done", f"dashboard http://127.0.0.1:{port}")
    click.echo(f"http://127.0.0.1:{port}")


def _stop_dashboards(port: int | None) -> None:
    """Stop the dashboards this machine is serving, whoever started them, and their labs.

    A dashboard takes its JupyterLab with it when asked to stop, so a lab still running with
    no dashboard left was orphaned by one that had to be killed outright. Sweep those too, or
    the next dashboard starts a second lab beside a stranded one.
    """
    found = _dashboard_pids(port)
    for pid in found:
        os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and _dashboard_pids(port):
        time.sleep(0.2)
    for pid in _dashboard_pids(port):
        os.kill(pid, signal.SIGKILL)

    orphans = [] if _dashboard_pids(None) else _lab_pids()
    for pid in orphans:
        os.kill(pid, signal.SIGTERM)

    if not found and not orphans:
        where = f" on port {port}" if port else ""
        raise click.ClickException(f"no dashboard is serving{where}.")
    if found:
        _say("done", f"stopped {len(found)} dashboard{'' if len(found) == 1 else 's'}: {found}")
    if orphans:
        _say("done", f"stopped {len(orphans)} stranded JupyterLab: {orphans}")


def _dashboard_pids(port: int | None) -> list[int]:
    """Processes serving the dashboard, by what they were started as."""
    return _matching_pids(
        lambda argv: "motion_spec.dashboard" in argv and (port is None or str(port) in argv)
    )


def _lab_pids() -> list[int]:
    """JupyterLabs a dashboard started, known by the settings directory it hands them."""
    return _matching_pids(lambda argv: any(part.endswith(LAB_SETTINGS_DIR) for part in argv))


def _matching_pids(wanted) -> list[int]:
    """Live processes whose argv this accepts, never this one.

    Matching whole arguments, not a substring of the line: a shell that merely mentions the
    dashboard in its own command would otherwise be killed along with it.
    """
    found = []
    for entry in Path("/proc").glob("[0-9]*"):
        pid = int(entry.name)
        if pid != os.getpid() and wanted(_argv_of(pid)):
            found.append(pid)
    return sorted(found)


def _argv_of(pid: int) -> list[str]:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().decode().split("\0")
    except OSError:
        return []  # it exited while we looked


def _port_taken(port: int) -> bool:
    """Whether something already answers on the loopback port."""
    with socket.socket() as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


@main.command()
@click.argument("components", nargs=-1, type=click.Choice(COMPONENT_NAMES))
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
    "--prefix",
    type=click.Path(file_okay=False, path_type=Path),
    help="Install somewhere other than WORKSPACE/install; the environment files still "
    "describe it from the workspace root.",
)
@click.option(
    "--clean",
    is_flag=True,
    help="Remove the managed installations and exit, asking before each one. Sources are "
    "never removed.",
)
@click.option(
    "--all",
    "everything",
    is_flag=True,
    help="With --clean: offer the whole build, install and log trees and the environment "
    "files, including what setup did not install itself.",
)
@click.option(
    "-y",
    "--yes",
    "assume_yes",
    is_flag=True,
    help="Answer yes to every --clean prompt, for a script with no terminal to ask at.",
)
@click.option(
    "--force", is_flag=True, help="Rebuild even when the component is already installed."
)
@click.option(
    "--clear-cache",
    is_flag=True,
    help="Clear selected CMake build caches and rebuild those components.",
)
@click.option(
    "--build-type", default=BUILD_TYPE, show_default=True, help="CMAKE_BUILD_TYPE for the sources."
)
@click.option(
    "--dev/--no-dev",
    "dev_flag",
    default=None,
    help="Check the Python components out into WORKSPACE/src, to work on them. Without it pip "
    "fetches the pinned ref and nothing new lands under src; a checkout already there is "
    "installed either way.",
)
@click.option(
    "--editable/--no-editable",
    "editable_flag",
    default=None,
    help="Whether a Python checkout is installed with `pip install -e`. On by default, so "
    "site-packages points back at the tree you are editing; --no-editable installs a snapshot.",
)
@click.option(
    "--repos",
    "repos",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Pins to use instead of the shipped manifest. Every component it omits must already "
    "be checked out in the workspace, or setup stops and says which.",
)
@click.option(
    "--external",
    "external",
    multiple=True,
    type=click.Choice(COMPONENT_NAMES),
    help="A component you supply yourself: never cloned, built or installed. Repeatable; "
    "adds to [setup] external.",
)
@click.option(
    "--ros/--no-ros",
    "ros_flag",
    default=None,
    help="Treat the workspace as a colcon one: build each component with colcon and write an "
    "environment file that sources ROS and the overlay. Overrides [ros] workspace.",
)
@click.option(
    "--cmake-arg",
    "cmake_args",
    multiple=True,
    help="Extra cmake option for the named components; repeat as needed.",
)
@click.option(
    "-j",
    "--jobs",
    type=click.IntRange(min=1),
    help="Compilers to run at once. Defaults to the lesser of the usable cores and one per "
    "2 GiB of memory; $CMAKE_BUILD_PARALLEL_LEVEL and [setup] jobs also set it.",
)
def setup(
    components: tuple[str, ...],
    workspace_argument: Path | None,
    prefix: Path | None,
    clean: bool,
    everything: bool,
    assume_yes: bool,
    force: bool,
    clear_cache: bool,
    build_type: str,
    dev_flag: bool | None,
    editable_flag: bool | None,
    repos: Path | None,
    external: tuple[str, ...],
    ros_flag: bool | None,
    cmake_args: tuple[str, ...],
    jobs: int | None,
) -> None:
    """Install the external tools and libraries motion-spec builds against.

    Installs into WORKSPACE/install, with the environment files written to WORKSPACE. With no
    COMPONENTS, installs the ones every model needs, in dependency order; the device drivers
    (serial, robotiq_driver_noros, robif2b) are built only when named. A source already in the
    workspace is what gets built; --dev decides where a missing one is checked out.
    """
    if clean and clear_cache:
        raise click.UsageError("--clean and --clear-cache cannot be used together")
    if everything and not clean:
        raise click.UsageError("--all applies to --clean: it names what a clean may take")
    if everything and components:
        raise click.UsageError(
            "--all removes the workspace's build, install and log trees; it takes no components"
        )
    from motion_spec.setup import (
        COMPONENTS_BY_NAME,
        build_jobs,
        component_installed,
        install_component,
        install_prefix,
        install_stst,
        is_installed,
        manifest_in_force,
        missing_prerequisites,
        remove_component,
        remove_environment,
        remove_stst,
        repository_of,
        source_directory,
        source_tree,
        stst_installed,
        uncovered,
        workspace_outputs,
        workspace,
        write_environment,
    )

    selected = components or DEFAULT_COMPONENTS
    # Declaration order, not the order typed: mj_kdl_wrapper links orocos_kdl.
    ordered = [name for name in COMPONENT_NAMES if name in selected]
    # A mistake in the command, not a failure inside it: no stack.
    try:
        root = workspace(workspace_argument)
        from motion_spec.config import settings as config_settings

        configured, config_path = config_settings(root)
    except (RuntimeError, ValueError) as exc:
        raise click.UsageError(str(exc)) from exc
    declared = configured.get("setup", {})
    configured_args = declared.get("cmake_args", {})
    in_file = bool(configured.get("ros", {}).get("workspace"))
    ros = ros_flag if ros_flag is not None else in_file
    dev = dev_flag if dev_flag is not None else bool(declared.get("dev", False))
    # Editable is the point of a checkout, and a checkout is installed wherever one is found,
    # so this is on unless --no-editable says otherwise. It reaches nothing else: a component
    # pip fetches has no tree to point at, and a compiled extension takes it only under --dev.
    editable = editable_flag if editable_flag is not None else bool(declared.get("editable", True))
    if not components and declared.get("components"):
        ordered = [name for name in COMPONENT_NAMES if name in declared["components"]]
    manifest = repos or (Path(declared["repos"]) if declared.get("repos") else None)
    try:
        sources = manifest_in_force(manifest)
    except (OSError, ValueError) as exc:
        raise click.UsageError(str(exc)) from exc
    if manifest is not None:
        _say("info", f"pins from {manifest}")
    supplied = {*external, *declared.get("external", [])}
    # Dropped before anything reads the list, so no clone, build, marker or prerequisite of an
    # external component is ever considered; health still reports whether it resolves.
    skipped_external = [name for name in ordered if name in supplied]
    ordered = [name for name in ordered if name not in supplied]
    from motion_spec.config import resolve

    build_type = resolve(
        build_type if build_type != BUILD_TYPE else None,
        BUILD_TYPE_VARIABLE,
        declared.get("build_type"),
        BUILD_TYPE,
        os.environ,
    ).value
    # CMake's own variable fills the environment slot; an explicit --parallel would shadow it.
    asked = resolve(jobs, "CMAKE_BUILD_PARALLEL_LEVEL", declared.get("jobs"), None, os.environ)
    try:
        jobs = build_jobs(int(asked.value) if asked.value is not None else None)
    except ValueError as exc:
        raise click.UsageError(f"jobs ({asked.source}) is not a number: {asked.value!r}") from exc
    prefix = prefix or (Path(declared["prefix"]) if declared.get("prefix") else None)
    prefix = (root / prefix).resolve() if prefix else install_prefix(root)
    log = command_log(root, "setup")
    # Held to the end of the command, so the failure that ends it is in the log too.
    click.get_current_context().with_resource(mirrored_stderr(log))
    log_header(
        log,
        "setup",
        {
            "workspace": root,
            "prefix": prefix,
            "components": ", ".join(ordered),
            "build type": build_type,
            "jobs": f"{jobs} ({asked.source})",
            "python": "checkout, editable" if dev and editable else "checkout" if dev else "git",
            "colcon": ros,
            **machine_facts(),
        },
    )
    _say("info", f"workspace {root}")
    _say("info", f"installing into {prefix}")
    _say("info", f"console log {log}")
    _say("info", f"building with {jobs} job{'s' if jobs != 1 else ''} ({asked.source})")
    if skipped_external:
        _say("info", f"supplied by you, left alone: {', '.join(skipped_external)}")
    # setup cannot check itself out: it is the code running. So it says so instead.
    if dev and not _editable_here(root):
        import motion_spec

        _say(
            "warn",
            f"--dev leaves motion-spec itself installed from {Path(motion_spec.__file__).parent}; "
            f"clone it into {root / 'src' / 'motion-spec'} and reinstall it with `pip install -e` "
            "to edit it too",
        )
    # The sample is written only when there is no config, so an existing one is left disagreeing.
    if config_path and ros != in_file:
        _say(
            "warn",
            f"--{'ros' if ros else 'no-ros'} applies to this run only; edit [ros] workspace in "
            f"{config_path} to keep it",
        )
    if not clean:
        # Before the first clone: a prerequisite setup cannot install itself is a failure the
        # operator has to act on, and finding it after four checkouts and a build helps nobody.
        _say("info", "checking prerequisites")
        unsupplied = uncovered(ordered, root, dev, sources)
        if unsupplied:
            for name in unsupplied:
                _say("error", f"{name}: no entry in {manifest} and no checkout in the workspace")
            raise _Reported("every component needs a pin or a source; nothing was cloned.")
        packages, others = missing_prerequisites(ordered, ros)
        if packages or others:
            for requirement in others:
                _say("error", requirement)
            if packages:
                _say("error", f"apt: sudo apt-get install -y {' '.join(packages)}")
            raise _Reported(
                "setup needs these before it can start; nothing was cloned or built."
            )
    if ros and not clean:
        from motion_spec.setup import write_colcon_meta

        # Every package, not just the ones installed now: colcon builds whatever is in src/,
        # and it is written before the first build, which reads it.
        built_with = {
            component.name: _cmake_options(component, configured_args, ())
            for component in COMPONENTS_BY_NAME.values()
        }
        _say("info", f"colcon options in {write_colcon_meta(root, built_with)}")
    skipped: list[str] = []
    try:
        if clean:
            if everything:
                removed = [
                    _under(path, root)
                    for path in workspace_outputs(root, prefix)
                    if _confirm_removal(
                        _under(path, root), [(_under(path, root), tree_size(path))], assume_yes
                    )
                    and trash_if_present(path)
                ]
            else:
                removed = [
                    name
                    for name in ordered
                    if _confirm_removal(name, _clean_offer(name, root, prefix), assume_yes)
                    and (
                        remove_stst(root, prefix)
                        if name == "stst"
                        else remove_component(COMPONENTS_BY_NAME[name], root, prefix)
                    )
                ]
                # With nothing installed they are a map to an empty prefix.
                if removed and not any(
                    is_installed(component, prefix) for component in COMPONENTS_BY_NAME.values()
                ):
                    remove_environment(root)
            if removed:
                _say("done", f"moved to trash: {', '.join(removed)}")
                left = sorted(
                    {
                        _under(tree, root)
                        for name in (ordered if everything else removed)
                        if (tree := source_tree(root, repository_of(name), dev)).is_dir()
                    }
                )
                if left:
                    _say("info", f"sources left in place: {', '.join(left)}")
            else:
                _say("info", "nothing removed")
            return
        for name in ordered:
            component = COMPONENTS_BY_NAME.get(name)
            # Otherwise the miss is a find_package error pages into a build log.
            for required in component.requires if component else ():
                if required in ordered or is_installed(COMPONENTS_BY_NAME[required], prefix):
                    continue
                _say(
                    "warn",
                    f"{name} takes {required} from {prefix}, which has none installed; run "
                    f"`motion-spec setup {required}` first, or point CMAKE_PREFIX_PATH at one",
                )
            already = not (force or (clear_cache and component and not component.python)) and (
                stst_installed(root, prefix, dev)
                if name == "stst"
                else component_installed(component, prefix, dev, root, sources)
            )
            # Announced before the build, so its console output has a heading.
            if not already:
                _say("step", name)
            if name == "stst":
                launcher = install_stst(root, prefix, force=force, log=log, dev=dev, sources=sources)
                _say(
                    "info" if already else "done",
                    f"stst {'already installed' if already else 'installed'}, launcher {launcher}",
                )
                _clear_shadowing_stst(prefix)
                continue
            state = install_component(
                COMPONENTS_BY_NAME[name],
                root,
                prefix,
                force=force,
                clear_cache=clear_cache,
                build_type=build_type,
                log=log,
                options=_cmake_options(COMPONENTS_BY_NAME[name], configured_args, cmake_args),
                ros=ros,
                editable=editable,
                jobs=jobs,
                dev=dev,
                sources=sources,
            )
            if not state.usable:
                _say("warn", f"skipped {name}: {state.path} {state.reason}")
                skipped.append(name)
            else:
                # Not the pip route, which reports its requirement as the origin and has no tree.
                if not state.origin and state.path != source_directory(
                    root, COMPONENTS_BY_NAME[name].repository, dev
                ):
                    _say("info", f"{name}: adopting the checkout at {state.path}")
                if state.drift:
                    _say("warn", f"{name}: {state.path} {state.drift}")
                _say(
                    "info" if already else "done",
                    f"{name} {'already installed' if already else 'installed'}, "
                    f"source {state.origin or state.path}",
                )
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("setup failed", exc) from exc
    from motion_spec.config import write_sample

    sample = write_sample(
        root,
        COMPONENT_OPTIONS,
        DEFAULT_COMPONENTS,
        declared=workspace_argument is not None,
        ros=ros,
        dev=dev,
        editable=editable,
    )
    if sample:
        _say("info", f"settings written to {sample}")
    try:
        written = write_environment(root, prefix, ros=ros)
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
    """Install optional FEATURES and motion-spec-dsl."""
    if not features:
        raise click.UsageError("name at least one feature")
    extras = sorted(set(features) - {"dsl"})
    requirements = [f"motion_spec[{','.join(extras)}]"] if extras else []
    if "dsl" in features:
        requirements.extend(_dsl_requirements())
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
        ("base", "introspection", "dsl", "codegen", "ros", "build", "runtime", "all")
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
        "introspection": "required for recording and replay",
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
                f"{fallback} (default)" if fallback else "unset",
                fg=None if fallback else "yellow",
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


@main.command("ir")
@click.argument("manifest", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option("-o", "--output", type=click.Path(dir_okay=False, path_type=Path))
@click.option("-c", "--console", is_flag=True, help="Print IR to stdout.")
def generate_ir(manifest: Path, output: Path | None, console: bool) -> None:
    """Lower MANIFEST to motion-spec IR."""
    from motion_spec.classes.base import DataclassJSONEncoder
    from motion_spec.rdf_parser.ir import generate_ir as build_ir

    if console == (output is not None):
        raise click.UsageError("choose exactly one of --output or --console")
    payload = json.dumps(build_ir(manifest), cls=DataclassJSONEncoder, indent=4, sort_keys=True)
    if console or output == Path("-"):
        click.echo(payload)
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload)


@main.command()
@click.argument("input", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option("-o", "--output-dir", required=True, type=click.Path(file_okay=False, path_type=Path))
@click.option("--stst-bin", help="STSTv4 executable; defaults to managed STST, then PATH.")
def codegen(input: Path, output_dir: Path, stst_bin: str | None) -> None:
    """Generate C++ from motion-spec IR INPUT."""
    from motion_spec.generation.codegen import MissingAssets, generate_code
    from motion_spec.setup import find_stst

    stst = stst_bin or find_stst()
    if not stst:
        raise _requirement_box("stst") or click.ClickException("stst not found")
    try:
        generate_code(input.resolve(), output_dir.resolve(), stst)
    except MissingAssets as exc:
        raise _Reported(str(exc)) from exc
    except RuntimeError as exc:
        raise _internal_failure("code generation failed", exc) from exc


@main.command()
@click.argument("run-dir", type=click.Path(file_okay=False, path_type=Path))
@click.option("--source-dir", type=click.Path(file_okay=False, path_type=Path))
@click.option("--run-id")
@click.option("--frame-log", type=click.Path(dir_okay=False, path_type=Path))
@click.option("--log-producer-executable", type=click.Path(dir_okay=False, path_type=Path))
@click.option("--rec", type=click.Path(dir_okay=False, path_type=Path))
@click.option("--verify", is_flag=True, help="Verify an existing archive.")
def archive(
    run_dir: Path,
    source_dir: Path | None,
    run_id: str | None,
    frame_log: Path | None,
    log_producer_executable: Path | None,
    rec: Path | None,
    verify: bool,
) -> None:
    """Create or verify a run archive at RUN_DIR."""
    from motion_spec.introspection.archive import (
        ArchiveError,
        create_archive_manifest,
        verify_manifest,
    )

    try:
        if verify:
            verify_manifest(run_dir)
            _say("done", "archive OK")
        else:
            create_archive_manifest(
                run_dir,
                source_dir=source_dir,
                run_id=run_id,
                frame_log=frame_log,
                log_producer_executable=log_producer_executable,
                rec=rec,
            )
            click.echo(run_dir / "manifest.json")
    except ArchiveError as exc:
        raise click.ClickException(str(exc)) from exc


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
    from motion_spec.config import (
        CONFIG_FILE,
        settings,
        write_sample,
    )
    from motion_spec.config import (
        shell as config_shell,
    )
    from motion_spec.setup import (
        BUILD_TYPE as DEFAULT_BUILD_TYPE,
    )
    from motion_spec.setup import (
        GENERATION_DIRECTORY,
        INSTALL_DIRECTORY,
        build_jobs,
    )
    from motion_spec.setup import (
        workspace as resolve_workspace,
    )

    if initialize:
        try:
            root = resolve_workspace(workspace_argument)
        except RuntimeError as exc:
            raise click.UsageError(str(exc)) from exc
        written = write_sample(
            root, COMPONENT_OPTIONS, DEFAULT_COMPONENTS, declared=workspace_argument is not None
        )
        if written:
            _say("done", f"settings written to {written}")
            click.echo(written)
        else:
            _say("info", f"{root / CONFIG_FILE} already exists, left as it is")
        return

    try:
        configured, path = settings()
    except ValueError as exc:
        raise click.ClickException(str(exc)) from exc
    click.secho(f"file: {path or f'none found ({CONFIG_FILE})'}", fg="blue")
    workspace_keys = configured.get("workspace", {})
    setup_keys = configured.get("setup", {})
    # The file's own directory is the workspace when it does not name one.
    located = str(path.parent) if path else None
    rows = [
        ("workspace.root", workspace_keys, "root", WORKSPACE_VARIABLE, located),
        (
            "workspace.generations",
            workspace_keys,
            "generations",
            GENERATION_DIR_ENV,
            f"<workspace>/{GENERATION_DIRECTORY}",
        ),
        (
            "workspace.environment",
            workspace_keys,
            "environment",
            "MOTION_SPEC_ENV",
            "the nearest setup-motion-spec file",
        ),
        ("workspace.shell", workspace_keys, "shell", None, config_shell(configured)),
        ("setup.prefix", setup_keys, "prefix", None, f"<workspace>/{INSTALL_DIRECTORY}"),
        ("setup.build_type", setup_keys, "build_type", BUILD_TYPE_VARIABLE, DEFAULT_BUILD_TYPE),
        ("setup.dev", setup_keys, "dev", None, False),
        ("setup.editable", setup_keys, "editable", None, False),
        ("setup.jobs", setup_keys, "jobs", "CMAKE_BUILD_PARALLEL_LEVEL", build_jobs()),
        ("setup.components", setup_keys, "components", None, list(DEFAULT_COMPONENTS)),
        ("setup.external", setup_keys, "external", None, []),
    ]
    width = max(len(name) for name, *_ in rows)
    for name, section, key, variable, default in rows:
        from_env = os.environ.get(variable) if variable else None
        if from_env:
            shown, source = from_env, "environment"
        elif key in section:
            shown, source = section[key], "file"
        else:
            shown = default
            source = "this file's directory" if default is located and located else "default"
        click.secho(f"  {name:<{width}}  ", fg="cyan", nl=False)
        click.secho(f"{shown}", nl=False)
        click.secho(f"  ({source})", fg="yellow", dim=True)
    for component, options in setup_keys.get("cmake_args", {}).items():
        click.secho(f"  setup.cmake_args.{component}  ", fg="cyan", nl=False)
        click.echo(" ".join(options))


@main.command()
@click.option("-n", "--count", type=click.IntRange(min=1), help="Show only the last COUNT.")
@click.option("--json", "as_json_flag", is_flag=True, help="Emit the entries as JSON Lines.")
def journal(count: int | None, as_json_flag: bool) -> None:
    """Show what has been asked of this workspace, newest last."""
    from motion_spec.introspection.journal import entries, journal_path

    path = journal_path()
    if path is None:
        raise click.UsageError(
            f"no journal: set {WORKSPACE_VARIABLE} to the workspace whose history to read, "
            "or MOTION_SPEC_JOURNAL to a file"
        )
    read = entries(path)
    read = read[-count:] if count else read
    if as_json_flag:
        for entry in read:
            click.echo(json.dumps(entry))
        return
    if not read:
        _say("info", f"{path}: nothing recorded yet")
        return
    width = max(len(entry.get("command", "")) for entry in read)
    for entry in read:
        arguments = " ".join(entry.get("argv", []))
        click.secho(f"{entry.get('ts', '')}  ", fg="blue", nl=False)
        click.secho(f"{entry.get('command', ''):<{width}}  ", fg="cyan", nl=False)
        click.echo(arguments)


@main.command()
@click.argument("generation_a", type=click.Path(exists=True, path_type=Path))
@click.argument("generation_b", type=click.Path(exists=True, path_type=Path))
@click.option("--json", "as_json_flag", is_flag=True, help="Emit the full delta as JSON.")
def diff(generation_a: Path, generation_b: Path, as_json_flag: bool) -> None:
    """Diff two generations' specification graphs, classified by layer (task/binding)."""
    from motion_spec.introspection.spec_diff import (
        as_json,
        diff_graphs,
        load_model_graph,
        summarize,
    )

    deltas = diff_graphs(load_model_graph(generation_a), load_model_graph(generation_b))
    if as_json_flag:
        click.echo(as_json(deltas))
        return
    for layer, row in sorted(summarize(deltas).items()):
        click.echo(
            f"{layer:>9}: {row['subjects']} subjects (+{row['added']} / -{row['removed']} triples)"
        )
    for delta in deltas:
        if delta.layer == "metadata":
            continue
        click.echo(f"  [{delta.layer}] {delta.subject} (+{len(delta.added)}/-{len(delta.removed)})")


@main.command()
@click.argument("log", type=click.Path(path_type=Path))
@click.option("--jsonl", is_flag=True, help="Emit decoded frames as JSON Lines.")
@click.option("--verify", is_flag=True, help="Verify the manifest and frame-log header.")
def replay(log: Path, jsonl: bool, verify: bool) -> None:
    """Inspect a recorded run LOG."""
    from motion_spec.introspection.archive import ArchiveError
    from motion_spec.introspection.replay import (
        decode_frames,
        resolve_archive,
        summarize,
        validate_header,
    )

    try:
        if verify:
            _run_dir, log_path, _manifest, schema = resolve_archive(log)
            validate_header(log_path, schema)
            _say("done", "archive OK")
        elif jsonl:
            for record in decode_frames(log):
                click.echo(json.dumps(record, separators=(",", ":")))
        else:
            click.echo(summarize(log))
    except ArchiveError as exc:
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
@click.option("--run-id", help="Name of this run's directory under <generation>/runs.")
@click.option(
    "--cwd",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    help="Working directory of the executable; scene assets resolve relative to it.",
)
@click.option("--headless", is_flag=True, help="Run without a GUI.")
@click.option(
    "--rtf",
    type=click.FloatRange(min=0.0),
    help="Real-time factor: 1.0 paces the loop to wall time, 0.5 half speed, 2.0 double. "
    "Headless runs uncapped (as fast as the machine allows) unless given; with a GUI the "
    "viewer's live speed setting applies. Only the wall duration changes -- the simulation "
    "clock advances one control period per tick either way.",
)
@click.option(
    "--start-paused",
    is_flag=True,
    help="Arm the run paused; the loop holds on its first tick until the dashboard resumes it.",
)
@click.option(
    "--record",
    "record",
    multiple=True,
    metavar="CAMERA",
    help="Record this camera to MP4 beside the log. Simulations render declared cameras; "
    "real runs record their declared ROS image topics. Repeatable.",
)
@click.option("--steps", type=click.IntRange(min=1), help="Maximum headless simulation steps.")
@click.option("--no-log", is_flag=True, help="Do not write the frame log; the run has no replay.")
@click.option(
    "--seed",
    type=click.IntRange(min=0),
    help="Seed the run's draw of every sampled quantity; unseeded runs draw from OS entropy. "
    "The seed and the draw are recorded in logs/sampling.json either way.",
)
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
    seed: int | None,
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
        from motion_spec.introspection.archive import ArchiveError
        from motion_spec.introspection.runner import RunnerError, run_cataloged

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
        + (["--seed", str(seed)] if seed is not None else [])
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


@main.command()
@click.argument("model", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option("-o", "--output-dir", type=click.Path(file_okay=False, path_type=Path))
@click.option("--list-sites", is_flag=True, help="Print the mutation sites and run nothing.")
@click.option("--operators", help="Comma-separated operator tags to keep; default is all of them.")
@click.option(
    "--natural-faults",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Run the hand-recorded faults in this JSON file instead of enumerating operator sites.",
)
@click.option("--limit", type=click.IntRange(min=1), help="Take only the first K sites.")
@click.option(
    "--reference-runs",
    default=3,
    show_default=True,
    type=click.IntRange(min=1),
    help="Reference runs.",
)
@click.option(
    "--steps", default=12000, show_default=True, type=click.IntRange(min=1), help="Steps per run."
)
def mutate(
    model: Path,
    output_dir: Path | None,
    list_sites: bool,
    operators: str | None,
    natural_faults: Path | None,
    limit: int | None,
    reference_runs: int,
    steps: int,
) -> None:
    """Mutate MODEL one site at a time and score what each mutant's run points at.

    With --natural-faults, the sites are the hand-recorded faults in that file rather than the
    operator sites the model's text admits. Those run through the same pipeline and are written to
    report_natural.jsonl beside the operator campaign's report, against the same reference: pointed
    at a campaign directory that already has one, they reuse it rather than measure a new envelope.
    """
    from motion_spec.mutation import metric, runner, scorer
    from motion_spec.mutation.operators import discover_sites, natural_sites

    if model.suffix != ".robmot":
        raise click.BadParameter("MODEL must be a .robmot file", param_hint="MODEL")
    text = model.read_text()
    if natural_faults is None:
        sites = discover_sites(text, str(model))
    else:
        try:
            sites = natural_sites(
                json.loads(natural_faults.read_text())["faults"], text, str(model)
            )
        except (KeyError, ValueError) as exc:
            raise click.ClickException(f"{natural_faults}: {exc}") from exc
    if operators:
        wanted = {tag.strip() for tag in operators.split(",") if tag.strip()}
        unknown = wanted - {site.operator for site in sites}
        if unknown:
            raise click.UsageError(f"no site uses these operators: {', '.join(sorted(unknown))}")
        sites = [site for site in sites if site.operator in wanted]
    if limit is not None:
        sites = sites[:limit]
    if not sites:
        raise click.ClickException(f"{model}: nothing to mutate")

    if list_sites:
        width = max(len(site.operator) for site in sites)
        names = max(len(site.name) for site in sites)
        for index, site in enumerate(sites):
            click.echo(
                f"{index:3d}  {site.operator:<{width}}  {site.name:<{names}}  "
                f"{site.original} -> {site.mutated}"
            )
        click.echo(f"\n{len(sites)} sites")
        return
    if output_dir is None:
        raise click.UsageError("-o/--output-dir is required unless --list-sites is given")

    out = output_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    reference = runner.existing_reference(out / "reference") if natural_faults else None
    if reference is None:
        _say("step", f"reference: {reference_runs} runs of the unmutated model")
        try:
            reference = runner.reference_runs(model, out / "reference", reference_runs, steps)
        except (OSError, RuntimeError) as exc:
            raise click.ClickException(f"the reference did not run: {exc}") from exc
    else:
        _say("info", f"reusing {len(reference[1])} reference runs in {out / 'reference'}")
    generation, decoded = reference
    envelope = metric.envelope([metric.features(metric.load_frames(path)) for path in decoded])
    slots = scorer.state_controllers(generation, scorer.introspection(generation))

    records = []
    report = out / ("report_natural.jsonl" if natural_faults else "report.jsonl")
    report.unlink(missing_ok=True)
    for index, site in enumerate(sites):
        _say("step", f"[{index + 1}/{len(sites)}] {site.operator} {site.name}")
        record = runner.run_mutant(site, index, model, out, steps)
        if record["frames"]:
            feature = metric.features(metric.load_frames(Path(record["frames"])))
            record |= scorer.score(
                metric.deviation(feature, envelope),
                slots,
                site.operator,
                site.name,
                site.element_uri,
            )
        records.append(record)
        with report.open("a") as sink:
            sink.write(json.dumps(record) + "\n")
    _mutation_summary(records, report)


@main.command("mutate-rescore")
@click.argument("campaign", type=click.Path(exists=True, file_okay=False, path_type=Path))
def mutate_rescore(campaign: Path) -> None:
    """Re-rank the finished mutation campaign in CAMPAIGN with scorer v2.

    Reads the campaign's own report and mutant frames and writes report_v2.jsonl beside them: the
    same records, ranked by deviation onset, with v1's ranking kept alongside for comparison. A
    naturals report in the same campaign is rescored the same way, into report_natural_v2.jsonl.
    """
    from motion_spec.mutation import scorer_v2

    report = campaign / "report.jsonl"
    if not report.is_file():
        raise click.ClickException(f"{report}: no campaign report to rescore")
    try:
        reference = scorer_v2.load(campaign)
    except (OSError, RuntimeError, ValueError) as exc:
        raise click.ClickException(f"the reference could not be read: {exc}") from exc
    _say("info", f"{len(reference.candidates)} candidates")
    _rescore(campaign, report, campaign / "report_v2.jsonl", reference)
    naturals = campaign / "report_natural.jsonl"
    if naturals.is_file():
        _rescore(campaign, naturals, campaign / "report_natural_v2.jsonl", reference)


def _rescore(campaign: Path, report: Path, into: Path, reference) -> None:
    """Rank one report's mutants with v2 and write the rescored records to `into`."""
    from motion_spec.mutation import metric, scorer_v2

    records = [json.loads(line) for line in report.read_text().splitlines() if line.strip()]
    rescored = []
    into.unlink(missing_ok=True)
    for index, record in enumerate(records, start=1):
        _say("step", f"[{index}/{len(records)}] {record['mutant']}")
        # The deviation rule is frozen, so whether a run deviated is carried over as it stands;
        # v2 changes only the ranking that follows from it.
        fresh = dict(record) | {
            "target_rank_v1": record.get("target_rank"),
            "top_v1": record.get("top"),
        }
        frames = campaign / "mutants" / record["mutant"] / "frames.jsonl"
        if not frames.is_file() and record.get("frames"):
            frames = Path(record["frames"])
        if frames.is_file():
            fresh |= scorer_v2.score(
                metric.load_frames(frames), reference, record["name"], record.get("element_uri")
            )
        rescored.append(fresh)
        with into.open("a") as sink:
            sink.write(json.dumps(fresh) + "\n")
    _mutation_summary(rescored, into)


def _mutation_summary(records: list[dict], report: Path) -> None:
    """Outcome counts, and how well the ranking found the mutated constraint."""
    import statistics

    click.echo()
    for outcome in sorted({record["outcome"] for record in records}):
        count = sum(record["outcome"] == outcome for record in records)
        click.echo(f"  {outcome:<12} {count}")
    deviated = [record for record in records if record.get("deviated")]
    ranks = [record["target_rank"] for record in deviated if record.get("target_rank")]
    click.echo(f"  {'deviated':<12} {len(deviated)} of {len(records)}")
    if ranks:
        click.echo(
            f"  {'rank':<12} mean {statistics.mean(ranks):.2f}, median {statistics.median(ranks)}"
        )
    click.echo()
    for operator in sorted({record["operator"] for record in deviated}):
        hit = [record for record in deviated if record["operator"] == operator]
        first = sum(record.get("target_rank") == 1 for record in hit)
        unranked = sum(record.get("target_rank") is None for record in hit)
        click.echo(f"  {operator:<24} top-1 {first}/{len(hit)}, unranked {unranked}")
    click.echo(f"\n{report}")


@main.command("mutate-taxonomy")
@click.argument("campaign", type=click.Path(exists=True, file_okay=False, path_type=Path))
def mutate_taxonomy(campaign: Path) -> None:
    """Sort every mutant of the campaign in CAMPAIGN into one of four outcome classes.

    Rejected before it ran, silent once it did, detected and attributed to the element that was
    damaged, or detected without being attributed. Ranks come from the best scorer the campaign
    holds -- report_v2.jsonl where a rescore was run, the original report otherwise -- and the
    naturals report is counted alongside the operator mutants as its own operator.
    """
    from motion_spec.mutation import taxonomy

    rows = taxonomy.load(campaign)
    if not rows:
        raise click.ClickException(f"{campaign}: no campaign report to classify")
    counts = taxonomy.tally(rows)
    width = max(len(operator) for operator in counts["per_operator"])
    header = "  ".join(f"{name:>{len(name)}}" for name in taxonomy.CLASSES)
    click.echo(f"\n  {'operator':<{width}}  {header}")
    for operator, per_class in sorted(counts["per_operator"].items()):
        click.echo(f"  {operator:<{width}}  {_taxonomy_row(per_class)}")
    click.echo(f"  {'all':<{width}}  {_taxonomy_row(counts['overall'])}")

    for class_name in ("rejected", "detected-unattributed"):
        mutants = taxonomy.named(rows, class_name)
        click.echo(f"\n  {class_name} ({len(mutants)})")
        for mutant in mutants:
            click.echo(f"    {mutant}")
    written = campaign / "taxonomy.json"
    written.write_text(
        json.dumps({"campaign": str(campaign), "counts": counts, "records": rows}, indent=2) + "\n"
    )
    click.echo(f"\n{written}")


def _taxonomy_row(per_class: dict[str, int]) -> str:
    """One row of class counts, each under the width of its own class heading."""
    from motion_spec.mutation import taxonomy

    return "  ".join(f"{per_class[name]:>{len(name)}}" for name in taxonomy.CLASSES)


@main.command(context_settings={"ignore_unknown_options": True})
@click.argument(
    "generation", required=False, type=click.Path(exists=True, file_okay=False, path_type=Path)
)
@click.option("--run-id")
@click.option("--cwd", type=click.Path(exists=True, file_okay=False, path_type=Path))
@click.option("--headless", is_flag=True, help="Run without a GUI.")
@click.option(
    "--record",
    "record",
    multiple=True,
    metavar="CAMERA",
    help="Record this camera to MP4 beside the log. Simulations render declared cameras; "
    "real runs record their declared ROS image topics. Repeatable.",
)
@click.option("--steps", type=click.IntRange(min=1), help="Maximum headless simulation steps.")
@click.option("--no-log", is_flag=True, help="Do not write the frame log; the run has no replay.")
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
@_environment_options
@click.pass_context
def rerun(
    ctx: click.Context,
    generation: Path | None,
    run_id: str | None,
    cwd: Path | None,
    headless: bool,
    record: tuple[str, ...],
    steps: int | None,
    no_log: bool,
    executable_args: tuple[str, ...],
    env_script: Path | None,
    no_env: bool,
) -> None:
    """Run a generation again, in a run of its own.

    `run` for the generation `latest` points at, so the timestamped path a `gen` just made need
    not be pasted back. Name a GENERATION to take that one instead. Generates and builds nothing.
    """
    generation = (generation or _latest_generation()).resolve()
    _say("info", f"generation {generation}")
    ctx.invoke(
        run,
        input=generation,
        output_dir=None,
        prefixes=(),
        jobs=None,
        run_id=run_id,
        cwd=cwd,
        headless=headless,
        record=record,
        steps=steps,
        no_log=no_log,
        executable_args=executable_args,
        env_script=env_script,
        no_env=no_env,
    )
