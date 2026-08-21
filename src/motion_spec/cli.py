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
from importlib.metadata import distribution
from pathlib import Path

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


def _new_generation(model: Path, output_dir: Path | None) -> Path:
    """Create this run's generation directory, and say where it is before anything fills it.

    Announced up front rather than only on success: the DSL and the compiler write pages of
    their own output next, and where it all landed is the one thing needed to go look at it --
    including when the run fails partway and never reaches the closing line. Stderr, because
    `gen` writes that closing line to stdout for a caller to read.
    """
    from motion_spec.generation.pipeline import create_generation_dir

    generation = create_generation_dir(model, _generation_base(output_dir))
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


def _dsl_requirement() -> str:
    from motion_spec.generation.codegen import _source_root_from_distribution

    root = _source_root_from_distribution()
    sibling = root.parent / "motion-spec-dsl" if root else None
    return (
        str(sibling)
        if sibling and sibling.is_dir()
        else "motion_spec_dsl @ git+https://github.com/secorolab/motion-spec-dsl.git"
    )


@click.group(cls=MotionSpecGroup, no_args_is_help=True)
@click.version_option(package_name="motion_spec")
def main() -> None:
    """The motion-spec toolchain."""


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
    log_file: Path | None,
) -> None:
    """Browse generations, replay runs, and query them in a browser."""
    from motion_spec.dashboard.server import serve

    if kill:
        return _stop_dashboards(port if "--port" in sys.argv or "-p" in sys.argv else None)
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
    return _matching_pids(
        lambda argv: any(part.endswith(LAB_SETTINGS_DIR) for part in argv)
    )


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
        requirements.append(_dsl_requirement())
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
        click.secho(
            f"  {'OK' if check.ok else 'MISSING':<7}  ", fg="green" if check.ok else "red", nl=False
        )
        click.secho(f"{check.dependency:<{dependency_width}}", fg="cyan", nl=False)
        click.echo(f"  {check.what:<{what_width}}  {check.path or '—'}")
        if not check.ok:
            click.secho(f"          Fix: {check.detail}", fg="yellow")
    missing = sum(not check.ok for check in checks)
    click.echo()
    click.secho("Summary:", bold=True, fg="blue", nl=False)
    click.echo(" ", nl=False)
    click.secho(f"{len(checks) - missing} good", fg="green", nl=False)
    click.echo(", ", nl=False)
    if missing:
        click.secho(f"{missing} missing", fg="red")
    else:
        click.echo("0 missing")
    if not all(check.ok for check in checks):
        raise click.exceptions.Exit(1)


@main.command()
@click.argument("stage-or-model")
@click.argument(
    "model", required=False, type=click.Path(exists=True, dir_okay=False, path_type=Path)
)
@click.option("-o", "--output-dir", type=click.Path(file_okay=False, path_type=Path))
def gen(stage_or_model: str, model: Path | None, output_dir: Path | None) -> None:
    """Generate IR or C++ from a .robmot MODEL; CODE is the default stage."""
    from rdf_utils.constraints import ConstraintViolation

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
        generation = _new_generation(model, output_dir)
        generate_model(model, generation, stage=stage)
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
    from motion_spec.generation.codegen import generate_code
    from motion_spec.setup import find_stst

    try:
        generate_code(input.resolve(), output_dir.resolve(), stst_bin or find_stst() or "stst")
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
@click.argument("log", type=click.Path(path_type=Path))
@click.option("--jsonl", is_flag=True, help="Emit decoded frames as JSON Lines.")
@click.option("--verify", is_flag=True, help="Verify the manifest and frame-log header.")
@click.option("--recover-runtime-ttl", is_flag=True, help="Recover runtime.ttl from the log.")
def replay(log: Path, jsonl: bool, verify: bool, recover_runtime_ttl: bool) -> None:
    """Inspect or recover a recorded run LOG."""
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
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            run_dir, log_path, _manifest, _schema = resolve_archive(log)
            records, frame_count = runtime_frames(log_path)
            click.echo(write_runtime_ttl(run_dir, records, frame_count=frame_count))
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
@click.option("-o", "--output-dir", type=click.Path(file_okay=False, path_type=Path))
@click.option(
    "--prefix",
    "prefixes",
    multiple=True,
    type=click.Path(exists=True, file_okay=False, path_type=Path),
)
@click.option("-j", "--jobs", type=click.IntRange(min=1))
@click.option("--run-id")
@click.option("--cwd", type=click.Path(exists=True, file_okay=False, path_type=Path))
@click.option("--no-verify", is_flag=True)
@click.option("--headless", is_flag=True, help="Run without a GUI.")
@click.option("--steps", type=click.IntRange(min=1), help="Maximum headless simulation steps.")
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
def run(
    input: Path,
    output_dir: Path | None,
    prefixes: tuple[Path, ...],
    jobs: int | None,
    run_id: str | None,
    cwd: Path | None,
    no_verify: bool,
    headless: bool,
    steps: int | None,
    executable_args: tuple[str, ...],
) -> None:
    """Run a .robmot INPUT, generating and building it first, or an existing GENERATION."""
    from rdf_utils.constraints import ConstraintViolation

    from motion_spec.generation.pipeline import build_generation, generate_model, new_id
    from motion_spec.introspection.archive import ArchiveError
    from motion_spec.introspection.runner import RunnerError, run_cataloged

    if steps is not None and not headless:
        raise click.UsageError("--steps requires --headless")
    if input.suffix == ".robmot":
        try:
            generation = _new_generation(input, output_dir)
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
        if output_dir is not None or prefixes or jobs is not None:
            raise click.UsageError("generation and build options require a .robmot INPUT")
        generation = input.resolve()

    if (headless or steps is not None) and not _is_simulated(generation):
        raise click.UsageError(
            "--headless and --steps are simulator options; this generation runs on hardware"
        )

    run_dir = generation / "runs" / (run_id or new_id("run"))
    arguments = (
        (["--headless"] if headless else [])
        + (["--steps", str(steps)] if steps is not None else [])
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
            recover_runtime_ttl=True,
            verify=not no_verify,
        )
    except (ArchiveError, RunnerError) as exc:
        raise click.ClickException(str(exc)) from exc
    if returncode:
        raise click.exceptions.Exit(returncode)
    click.echo(run_dir)


@main.command(context_settings={"ignore_unknown_options": True})
@click.argument(
    "generation", required=False, type=click.Path(exists=True, file_okay=False, path_type=Path)
)
@click.option("--run-id")
@click.option("--cwd", type=click.Path(exists=True, file_okay=False, path_type=Path))
@click.option("--no-verify", is_flag=True)
@click.option("--headless", is_flag=True, help="Run without a GUI.")
@click.option("--steps", type=click.IntRange(min=1), help="Maximum headless simulation steps.")
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
@click.pass_context
def rerun(
    ctx: click.Context,
    generation: Path | None,
    run_id: str | None,
    cwd: Path | None,
    no_verify: bool,
    headless: bool,
    steps: int | None,
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
        no_verify=no_verify,
        headless=headless,
        steps=steps,
        executable_args=executable_args,
    )
