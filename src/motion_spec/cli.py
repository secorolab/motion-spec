# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: OpenAI

"""Unified Click command-line interface for motion-spec."""

import json
import subprocess
import sys
from contextlib import contextmanager
from importlib.metadata import distribution
from pathlib import Path

import click

from motion_spec.setup import DEFAULT_PREFIX


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
            formatter.write_text("motion-spec - validate, compile, run, and inspect motion specifications")
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
                ]
            )
        option_records = [record for param in self.get_params(ctx) if (record := param.get_help_record(ctx))]
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
    from motion_spec.codegen import _source_root_from_distribution

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
        raise click.ClickException(f"STST setup failed: {exc}") from exc
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
            f"  {'OK' if check.ok else 'MISSING':<7}  ",
            fg="green" if check.ok else "red",
            nl=False,
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
@click.argument("model", required=False, type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option("-o", "--output-dir", type=click.Path(file_okay=False, path_type=Path))
def gen(stage_or_model: str, model: Path | None, output_dir: Path | None) -> None:
    """Generate IR or C++ from a .robmot MODEL; CODE is the default stage."""
    from motion_spec.pipeline import create_generation_dir, generate_model

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
        generation = create_generation_dir(model, output_dir)
        generate_model(model, generation, stage=stage)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise click.ClickException(f"generation failed: {exc}") from exc
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
    from motion_spec.pipeline import build_generation

    try:
        executable = build_generation(generation.resolve(), prefixes=prefixes, jobs=jobs)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise click.ClickException(f"build failed: {exc}") from exc
    click.echo(executable)


@main.command()
@click.argument("manifest", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option(
    "--meta-shacl",
    is_flag=True,
    help="Validate the SHACL shape graph against SHACL-of-SHACL too.",
)
def check(manifest: Path, meta_shacl: bool) -> None:
    """Validate MANIFEST against its SHACL constraints."""
    from motion_spec.check import validate_manifest

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
    from motion_spec.entities import DataclassJSONEncoder
    from motion_spec.ir_gen import generate_ir as build_ir

    if console == (output is not None):
        raise click.UsageError("choose exactly one of --output or --console")
    payload = json.dumps(build_ir(manifest), cls=DataclassJSONEncoder, indent=4)
    if console or output == Path("-"):
        click.echo(payload)
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload)


@main.command()
@click.argument("input", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option(
    "-o", "--output-dir", required=True, type=click.Path(file_okay=False, path_type=Path)
)
@click.option("--stst-bin", help="STSTv4 executable; defaults to managed STST, then PATH.")
def codegen(input: Path, output_dir: Path, stst_bin: str | None) -> None:
    """Generate C++ from motion-spec IR INPUT."""
    from motion_spec.codegen import generate_code
    from motion_spec.setup import find_stst

    try:
        generate_code(input.resolve(), output_dir.resolve(), stst_bin or find_stst() or "stst")
    except RuntimeError as exc:
        raise click.ClickException(str(exc)) from exc


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


@main.command(context_settings={"ignore_unknown_options": True})
@click.argument("input", type=click.Path(path_type=Path))
@click.option(
    "--source-dir", type=click.Path(exists=True, file_okay=False, path_type=Path)
)
@click.option(
    "--executable", type=click.Path(exists=True, dir_okay=False, path_type=Path)
)
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
@click.option("--recover-runtime-ttl", is_flag=True)
@click.option("--no-verify", is_flag=True)
@click.option("--headless", is_flag=True, help="Run a .robmot model without a GUI.")
@click.option("--steps", type=click.IntRange(min=1), help="Maximum headless simulation steps.")
@click.argument("executable-args", nargs=-1, type=click.UNPROCESSED)
def run(
    input: Path,
    source_dir: Path | None,
    executable: Path | None,
    output_dir: Path | None,
    prefixes: tuple[Path, ...],
    jobs: int | None,
    run_id: str | None,
    cwd: Path | None,
    recover_runtime_ttl: bool,
    no_verify: bool,
    headless: bool,
    steps: int | None,
    executable_args: tuple[str, ...],
) -> None:
    """Generate, build, and run a .robmot INPUT, or run an explicit executable."""
    from motion_spec.introspection.archive import ArchiveError
    from motion_spec.introspection.runner import RunnerError, run_cataloged

    generated_from_model = input.suffix == ".robmot"
    if generated_from_model:
        from motion_spec.pipeline import build_generation, create_generation_dir, generate_model, new_id

        if not input.is_file():
            raise click.BadParameter(f"file not found: {input}", param_hint="INPUT")
        if source_dir is not None or executable is not None:
            raise click.UsageError("--source-dir and --executable are only for an existing build")
        if steps is not None and not headless:
            raise click.UsageError("--steps requires --headless")
        try:
            generation = create_generation_dir(input, output_dir)
            source_dir = generate_model(input, generation, stage="code")
            executable = build_generation(generation, prefixes=prefixes, jobs=jobs)
        except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
            raise click.ClickException(f"pipeline failed: {exc}") from exc
        run_id = run_id or new_id(f"{input.stem}-run")
        run_dir = generation / "runs" / run_id
        recover_runtime_ttl = True
        arguments = (["--headless"] if headless else []) + (
            ["--steps", str(steps)] if steps is not None else []
        ) + list(executable_args)
    else:
        if source_dir is None or executable is None:
            raise click.UsageError(
                "INPUT must be a .robmot model, or --source-dir and --executable must be provided"
            )
        if output_dir is not None or prefixes or jobs is not None or headless or steps is not None:
            raise click.UsageError("generation and build options require a .robmot INPUT")
        run_dir = input
        arguments = list(executable_args)

    try:
        returncode = run_cataloged(
            run_dir,
            source_dir=source_dir,
            executable=executable,
            executable_args=arguments,
            run_id=run_id,
            cwd=cwd,
            recover_runtime_ttl=recover_runtime_ttl,
            verify=not no_verify,
        )
    except (ArchiveError, RunnerError) as exc:
        raise click.ClickException(str(exc)) from exc
    if returncode:
        raise click.exceptions.Exit(returncode)
    if generated_from_model:
        click.echo(run_dir)
