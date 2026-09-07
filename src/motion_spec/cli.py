# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Unified Click command-line interface for motion-spec."""

import json
import os
import signal
import socket
import subprocess
import sys
import time
import traceback
from contextlib import contextmanager
from importlib.metadata import PackageNotFoundError, distribution
from importlib.util import find_spec
from pathlib import Path
from urllib.parse import urlparse

import click

from motion_spec.setup import DEFAULT_PREFIX


def _internal_failure(what: str, exc: Exception) -> click.ClickException:
    """Report a failure that is ours, not the model's, with the stack that produced it.

    Anything reaching here is a defect in the generator or the environment, and the one thing
    needed to fix it -- where it came from -- is exactly what wrapping the message throws away.
    """
    traceback.print_exception(exc, file=sys.stderr)

    return click.ClickException(f"{what}: {exc}")


GENERATION_DIR_ENV = "MOTION_SPEC_GEN"
LATEST_LINK = "latest"
LAB_SETTINGS_DIR = "motion-spec-lab-settings"


def _generation_base(output_dir: Path | None) -> Path | None:
    """Where a new generation goes: `-o` if given, else `$MOTION_SPEC_GEN`.

    Neither set leaves the decision to `create_generation_dir`, which puts it under the working
    directory -- workable, but it scatters generations across wherever the command was run from,
    so say so once rather than let them accumulate unnoticed. The notice goes to stderr: `gen`
    writes the generation path to stdout for a caller to read.
    """
    if output_dir is not None:
        return output_dir
    configured = os.environ.get(GENERATION_DIR_ENV, "").strip()
    if configured:
        return Path(configured).expanduser()
    click.echo(
        f"{GENERATION_DIR_ENV} is not set, so this generation goes under the working directory. "
        f"Export {GENERATION_DIR_ENV}=/path/to/generations to keep them all in one place.",
        err=True,
    )

    return None


def _new_generation(model: Path, output_dir: Path | None, name: str | None = None) -> Path:
    """Create this run's generation directory, and say where it is before anything fills it.

    Announced up front rather than only on success: the DSL and the compiler write pages of
    their own output next, and where it all landed is the one thing needed to go look at it --
    including when the run fails partway and never reaches the closing line. Stderr, because
    `gen` writes that closing line to stdout for a caller to read.
    """
    from motion_spec.generation.pipeline import create_generation_dir

    generation = create_generation_dir(model, _generation_base(output_dir), name)
    click.echo(f"generation: {generation}", err=True)
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
    base = _generation_root()
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
    """Where `latest` lives: $MOTION_SPEC_GEN, or the working directory when it is unset."""
    configured = os.environ.get(GENERATION_DIR_ENV, "").strip()
    return Path(configured).expanduser() if configured else Path.cwd()


def _latest_generation() -> Path:
    """The generation `latest` points at, under `-o`'s default or the working directory.

    Raises:
        ClickException: nothing has been generated there, or what was is not built.
    """
    base = _generation_root()
    if not os.environ.get(GENERATION_DIR_ENV, "").strip():
        click.echo(
            f"{GENERATION_DIR_ENV} is not set, so this looks for `latest` under the working "
            f"directory ({base}). Export {GENERATION_DIR_ENV}=/path/to/generations to keep them "
            "all in one place.",
            err=True,
        )
    link = base / LATEST_LINK
    if not link.is_dir():
        raise click.ClickException(
            f"{link}: nothing generated here to rerun. `gen` and `run` point it at what they "
            f"make, so generate once, or name a generation directory. Generations go under "
            f"${GENERATION_DIR_ENV} when it is set, else the working directory."
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
                    ("setup", "Install the pinned STST code-generation tool."),
                    ("install", "Install the optional DSL, validation, or introspection features."),
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
                    ("generated/provenance/", "DSL, coordinate, and motion-spec provenance."),
                    ("build/", "Reusable compiled controller."),
                    ("runs/RUN/", "Run-owned logs, runtime RDF, REC graph, and manifest."),
                    ("latest", f"Symlink to the newest generation, under ${GENERATION_DIR_ENV}."),
                ]
            )
        with _manual_section(formatter, "ENVIRONMENT"):
            formatter.write_dl(
                [
                    (
                        GENERATION_DIR_ENV,
                        "Where 'gen' and 'run' put a new generation when given no -o. "
                        "Unset, they fall back to the working directory and say so. "
                        "'latest' there names the one 'rerun' takes.",
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
    "pyshacl": "motion-spec install validation",
    "rec": "motion-spec install introspection",
    "google.protobuf": "motion-spec install introspection",
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
def main() -> None:
    """The motion-spec toolchain."""
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    install_metamodel_resolver()


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
def dashboard(
    port: int,
    logs: Path | None,
    sources: Path | None,
    background: bool,
    kill: bool,
    restart: bool,
    log_file: Path | None,
) -> None:
    """Browse generations, replay runs, and query them in a browser."""
    from motion_spec.dashboard.server import serve
    from motion_spec.introspection.journal import record as _journal_record

    _journal_record("dashboard")

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
    if not background:
        return serve(port, logs, sources)

    destination = (log_file or logs / "dashboard.log").expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    argv = [sys.executable, "-m", "motion_spec.dashboard", "--port", str(port), "--logs", str(logs)]
    if sources is not None:
        argv += ["--sources", str(sources)]
    with destination.open("ab") as sink:
        child = subprocess.Popen(argv, stdout=sink, stderr=sink, start_new_session=True)
    click.echo(f"motion-spec dashboard: http://127.0.0.1:{port}")
    click.echo(f"  pid {child.pid}, logging to {destination}")
    click.echo(f"  stop it with: kill {child.pid}")


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
        click.echo(f"stopped {len(found)} dashboard{'' if len(found) == 1 else 's'}: {found}")
    if orphans:
        click.echo(f"stopped {len(orphans)} stranded JupyterLab: {orphans}")


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
@click.option(
    "--prefix",
    type=click.Path(file_okay=False, path_type=Path),
    default=DEFAULT_PREFIX,
    show_default=True,
    help="Installation prefix; the launcher is written to PREFIX/bin.",
)
@click.option("--clean", is_flag=True, help="Remove the managed STST installation and exit.")
def setup(prefix: Path, clean: bool) -> None:
    """Install the pinned STST code-generation tool."""
    from motion_spec.setup import install_stst, remove_stst

    try:
        prefix = prefix.resolve()
        if clean:
            click.echo("removed" if remove_stst(prefix) else "nothing to remove")
            return
        launcher = install_stst(prefix)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("STST setup failed", exc) from exc
    click.echo(launcher)


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
    result = subprocess.run([sys.executable, "-m", "pip", "install", *requirements])
    if result.returncode:
        raise click.ClickException("installation failed")


@main.command()
@click.option(
    "--profile",
    "profiles",
    multiple=True,
    type=click.Choice(
        ("base", "validation", "introspection", "dsl", "codegen", "build", "runtime", "all")
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
def health(profiles: tuple[str, ...], targets: tuple[str, ...]) -> None:
    """Report health of selected installation profiles."""
    from motion_spec.health import check_health

    checks = check_health(profiles, targets)
    requirements = {
        "base": "required for every installation",
        "validation": "required for validation",
        "introspection": "required for recording and replay",
        "dsl": "required for DSL generation",
        "codegen": "required for C++ generation",
        "build": "required to build generated C++",
        "runtime": "required to run generated controllers",
        "build[mujoco]": "required to build the MuJoCo target",
        "runtime[mujoco]": "required to run the MuJoCo target",
        "build[robif2b]": "required to build the robif2b target",
        "runtime[robif2b]": "required to run the robif2b target",
    }
    click.secho("motion-spec health", bold=True, fg="magenta")
    current_profile = None
    dependency_width = max(len(check.dependency) for check in checks)
    what_width = max(len(check.what) for check in checks)
    for check in checks:
        if check.profile != current_profile:
            current_profile = check.profile
            click.secho(
                f"\n{current_profile.upper()} — {requirements[current_profile]}",
                bold=True,
                fg="blue",
            )
        status = "OK" if check.ok else "ABSENT" if check.optional else "MISSING"
        colour = "green" if check.ok else "yellow" if check.optional else "red"
        click.secho(f"  {status:<7}  ", fg=colour, nl=False)
        click.secho(f"{check.dependency:<{dependency_width}}", fg="cyan", nl=False)
        click.echo(f"  {check.what:<{what_width}}  {check.path or '—'}")
        if not check.ok and not check.optional:
            if check.why:
                click.secho(f"          why: {check.why}", fg="yellow")
            if check.source:
                click.secho(f"          source: {check.source}", fg="yellow")
            click.secho(f"          fix: {check.detail}", fg="yellow")
    missing = sum(not check.ok and not check.optional for check in checks)
    absent = sum(not check.ok and check.optional for check in checks)
    click.echo()
    click.secho("Summary:", bold=True, fg="blue", nl=False)
    click.echo(" ", nl=False)
    click.secho(f"{sum(check.ok for check in checks)} good", fg="green", nl=False)
    click.echo(", ", nl=False)
    if missing:
        click.secho(f"{missing} missing", fg="red", nl=False)
    else:
        click.echo("0 missing", nl=False)
    # An absent optional is a fact about this workspace, not a fault: say it, do not fail on it.
    click.echo(f", {absent} absent by build option" if absent else "")
    if missing:
        raise click.exceptions.Exit(1)


@main.command()
@click.argument("stage-or-model")
@click.argument(
    "model", required=False, type=click.Path(exists=True, dir_okay=False, path_type=Path)
)
@click.option("-o", "--output-dir", type=click.Path(file_okay=False, path_type=Path))
@click.option(
    "--seed",
    type=int,
    help="Seed the draws of any sampled quantity; recorded either way in "
    "generated/provenance/motion-spec.ld.json.",
)
@click.option(
    "--name",
    help="Name the generation tree under the base directory; defaults to the model's stem.",
)
def gen(
    stage_or_model: str,
    model: Path | None,
    output_dir: Path | None,
    seed: int | None,
    name: str | None,
) -> None:
    """Generate IR or C++ from a .robmot MODEL; CODE is the default stage."""
    from rdf_utils.constraints import ConstraintViolation

    with _requirements_reported():
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
        generate_model(model, generation, stage=stage, seed=seed)
    except ConstraintViolation as exc:
        raise _model_rejected(exc) from exc
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("generation failed", exc) from exc
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
def build(generation: Path, prefixes: tuple[Path, ...], jobs: int | None) -> None:
    """Configure and compile GENERATION/generated into GENERATION/build."""
    from motion_spec.generation.pipeline import build_generation

    try:
        executable = build_generation(generation.resolve(), prefixes=prefixes, jobs=jobs)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise _internal_failure("build failed", exc) from exc
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
    from motion_spec.introspection.journal import record as _journal_record
    from motion_spec.rdf_parser.ir import generate_ir as build_ir

    _journal_record("ir")

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
    from motion_spec.generation.codegen import generate_code
    from motion_spec.setup import find_stst

    stst = stst_bin or find_stst()
    if not stst:
        raise _requirement_box("stst") or click.ClickException("stst not found")
    try:
        generate_code(input.resolve(), output_dir.resolve(), stst)
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
            click.echo("archive OK")
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
@click.argument("generation_a", type=click.Path(exists=True, path_type=Path))
@click.argument("generation_b", type=click.Path(exists=True, path_type=Path))
@click.option("--json", "as_json_flag", is_flag=True, help="Emit the full delta as JSON.")
def diff(generation_a: Path, generation_b: Path, as_json_flag: bool) -> None:
    """Diff two generations' specification graphs, classified by layer (task/binding)."""
    from motion_spec.introspection.journal import record
    from motion_spec.introspection.spec_diff import (
        as_json,
        diff_graphs,
        load_model_graph,
        summarize,
    )

    record("diff")
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
@click.option("--recover-runtime-ttl", is_flag=True, help="Recover runtime.ttl from the log.")
def replay(log: Path, jsonl: bool, verify: bool, recover_runtime_ttl: bool) -> None:
    """Inspect or recover a recorded run LOG."""
    from motion_spec.introspection.journal import record as _journal_record

    _journal_record("replay")
    from motion_spec.introspection.archive import ArchiveError
    from motion_spec.introspection.replay import (
        decode_frames,
        resolve_archive,
        runtime_frames,
        summarize,
        validate_header,
    )

    try:
        if recover_runtime_ttl:
            from motion_spec.introspection.archive import consolidate_provenance
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            run_dir, log_path, _manifest, _schema = resolve_archive(log)
            records, _frame_count = runtime_frames(log_path)
            out = write_runtime_ttl(run_dir, records)
            consolidate_provenance(run_dir)
            click.echo(out)
        elif verify:
            _run_dir, log_path, _manifest, schema = resolve_archive(log)
            validate_header(log_path, schema)
            click.echo("archive OK")
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
    "--runtime-ttl",
    is_flag=True,
    help="Recover runtime.ttl from the log when the run ends; otherwise "
    "'motion-spec replay <run> --recover-runtime-ttl' writes it later.",
)
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
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
    runtime_ttl: bool,
    executable_args: tuple[str, ...],
) -> None:
    """Run a .robmot INPUT, generating and building it first, or an existing GENERATION.

    Anything after the options that this command does not recognise is handed to the
    executable unchanged.
    """
    from rdf_utils.constraints import ConstraintViolation

    from motion_spec.generation.pipeline import build_generation, generate_model, new_id
    from motion_spec.introspection.archive import ArchiveError
    from motion_spec.introspection.runner import RunnerError, run_cataloged

    if steps is not None and not headless:
        raise click.UsageError("--steps requires --headless")
    if input.suffix == ".robmot":
        try:
            generation = _new_generation(input, output_dir, name)
            generate_model(input, generation, stage="code")
            build_generation(generation, prefixes=prefixes, jobs=jobs)
        except ConstraintViolation as exc:
            raise _model_rejected(exc) from exc
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

    if (headless or steps is not None) and not _is_simulated(generation):
        raise click.UsageError(
            "--headless and --steps are simulator options; this generation runs on hardware"
        )
    # Probing before a hardware run is off for now: start the controller directly and let a
    # device that does not answer say so in the run's own console.
    # if not _is_simulated(generation):
    #     _require_devices(generation)

    run_dir = generation / "runs" / (run_id or new_id("run"))
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
            recover_runtime_ttl=runtime_ttl,
            record=list(record),
            record_log=not no_log,
        )
    except (ArchiveError, RunnerError) as exc:
        raise click.ClickException(str(exc)) from exc
    # Echoed before the exit: an interrupt leaves a complete archive, and that is when the
    # caller most needs the path.
    click.echo(run_dir)
    if returncode:
        raise click.exceptions.Exit(returncode)


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
        click.echo(f"reference: {reference_runs} runs of the unmutated model", err=True)
        try:
            reference = runner.reference_runs(model, out / "reference", reference_runs, steps)
        except (OSError, RuntimeError) as exc:
            raise click.ClickException(f"the reference did not run: {exc}") from exc
    else:
        click.echo(f"reference: reusing {len(reference[1])} runs in {out / 'reference'}", err=True)
    generation, decoded = reference
    envelope = metric.envelope([metric.features(metric.load_frames(path)) for path in decoded])
    slots = scorer.state_controllers(generation, scorer.introspection(generation))

    records = []
    report = out / ("report_natural.jsonl" if natural_faults else "report.jsonl")
    report.unlink(missing_ok=True)
    for index, site in enumerate(sites):
        click.echo(f"[{index + 1}/{len(sites)}] {site.operator} {site.name}", err=True)
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
    click.echo(f"{len(reference.candidates)} candidates", err=True)
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
        click.echo(f"[{index}/{len(records)}] {record['mutant']}", err=True)
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
@click.option(
    "--runtime-ttl",
    is_flag=True,
    help="Recover runtime.ttl from the log when the run ends; otherwise "
    "'motion-spec replay <run> --recover-runtime-ttl' writes it later.",
)
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
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
    runtime_ttl: bool,
    executable_args: tuple[str, ...],
) -> None:
    """Run a generation again, in a run of its own.

    `run` for the generation `latest` points at, so the timestamped path a `gen` just made need
    not be pasted back. Name a GENERATION to take that one instead. Generates and builds nothing.
    """
    generation = (generation or _latest_generation()).resolve()
    click.echo(f"generation: {generation}", err=True)
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
        runtime_ttl=runtime_ttl,
        executable_args=executable_args,
    )
