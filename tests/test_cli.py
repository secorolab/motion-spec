# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

import json

from click.testing import CliRunner
from pathlib import Path
from types import SimpleNamespace

from motion_spec.cli import main
from motion_spec import setup as stst_setup


def test_cli_exposes_lazy_click_commands(monkeypatch, tmp_path) -> None:
    runner = CliRunner()

    result = runner.invoke(main, ["--help"])
    assert result.exit_code == 0
    assert all(
        command in result.output
        for command in ("install", "health", "gen", "build", "check", "ir", "codegen", "run")
    )

    result = runner.invoke(main, ["codegen", "--help"])
    assert result.exit_code == 0
    assert "--stst-bin" in result.output

    generation = tmp_path / "generation"
    (generation / "generated").mkdir(parents=True)
    (generation / "build").mkdir()
    (generation / "build" / "main").write_text("")
    received = {}
    monkeypatch.setattr(
        "motion_spec.introspection.runner.run_cataloged",
        lambda *args, **kwargs: received.update(args=args, kwargs=kwargs) or 0,
    )
    result = runner.invoke(
        main, ["run", str(generation), "--run-id", "run-2", "--", "--headless", "--steps", "10"]
    )
    assert result.exit_code == 0
    assert received["kwargs"]["executable_args"] == ["--headless", "--steps", "10"]
    assert received["kwargs"]["source_dir"] == generation / "generated"
    assert received["kwargs"]["executable"] == generation / "build" / "main"
    assert received["args"][0] == generation / "runs" / "run-2"


def test_rerun_runs_a_generation_again_under_a_new_id(monkeypatch, tmp_path) -> None:
    """It is `run` for a generation already built: a run of its own, generating nothing."""
    generation = tmp_path / "generation"
    (generation / "generated").mkdir(parents=True)
    (generation / "build").mkdir()
    (generation / "build" / "main").write_text("")
    received = {}
    monkeypatch.setattr(
        "motion_spec.introspection.runner.run_cataloged",
        lambda *args, **kwargs: received.update(args=args, kwargs=kwargs) or 0,
    )

    result = CliRunner().invoke(
        main, ["rerun", str(generation), "--run-id", "run-2", "--", "--headless"]
    )

    assert result.exit_code == 0
    assert received["kwargs"]["executable_args"] == ["--headless"]
    assert received["kwargs"]["source_dir"] == generation.resolve() / "generated"
    assert received["kwargs"]["executable"] == generation.resolve() / "build" / "main"
    assert received["args"][0] == generation.resolve() / "runs" / "run-2"


def test_rerun_takes_the_generation_latest_points_at(monkeypatch, tmp_path) -> None:
    """The timestamped path a `gen` just made is the one nobody should have to paste back."""
    from motion_spec import cli

    generation = tmp_path / "generation" / "model" / "20260815T000000000000Z"
    (generation / "build").mkdir(parents=True)
    (generation / "build" / "main").write_text("")
    monkeypatch.setenv(cli.GENERATION_DIR_ENV, str(tmp_path / "generation"))
    cli._point_latest(generation)
    received = {}
    monkeypatch.setattr(
        "motion_spec.introspection.runner.run_cataloged",
        lambda *args, **kwargs: received.update(args=args, kwargs=kwargs) or 0,
    )

    result = CliRunner().invoke(main, ["rerun", "--run-id", "run-1"])

    assert result.exit_code == 0
    assert received["args"][0] == generation / "runs" / "run-1"
    assert received["kwargs"]["executable"] == generation / "build" / "main"
    assert f"generation: {generation}" in result.output


def test_latest_follows_the_newest_generation(monkeypatch, tmp_path) -> None:
    """One link, under the generation root, relative, replaced in place."""
    from motion_spec import cli

    monkeypatch.setenv(cli.GENERATION_DIR_ENV, str(tmp_path))
    first = tmp_path / "model" / "20260815T000000000000Z"
    second = tmp_path / "model" / "20260815T111111111111Z"
    other = tmp_path / "named-run" / "other-model" / "20260815T222222222222Z"
    for generation in (first, second, other):
        generation.mkdir(parents=True)
        cli._point_latest(generation)

    assert (tmp_path / cli.LATEST_LINK).resolve() == other
    # No link per output directory or per model: one name, where `rerun` reads it.
    assert not (tmp_path / "model" / cli.LATEST_LINK).exists()
    assert not (tmp_path / "named-run" / cli.LATEST_LINK).exists()
    # Relative, so the tree can be moved or copied without the link pointing back at the old one.
    assert not Path((tmp_path / cli.LATEST_LINK).readlink()).is_absolute()


def test_rerun_says_where_it_looked_for_a_generation(monkeypatch, tmp_path) -> None:
    from motion_spec import cli

    monkeypatch.setenv(cli.GENERATION_DIR_ENV, str(tmp_path))

    result = CliRunner().invoke(main, ["rerun"])

    assert result.exit_code != 0
    assert f"{tmp_path / cli.LATEST_LINK}: nothing generated here to rerun" in result.output


def test_rerun_says_when_env_var_is_unset(monkeypatch, tmp_path) -> None:
    from motion_spec import cli

    monkeypatch.delenv(cli.GENERATION_DIR_ENV, raising=False)
    monkeypatch.chdir(tmp_path)

    result = CliRunner().invoke(main, ["rerun"])

    assert result.exit_code != 0
    assert f"{cli.GENERATION_DIR_ENV} is not set" in result.stderr
    assert str(tmp_path) in result.stderr


def test_rerun_reports_a_latest_generation_that_is_not_built(monkeypatch, tmp_path) -> None:
    from motion_spec import cli

    generation = tmp_path / "model" / "20260815T000000000000Z"
    generation.mkdir(parents=True)
    monkeypatch.setenv(cli.GENERATION_DIR_ENV, str(tmp_path))
    cli._point_latest(generation)

    result = CliRunner().invoke(main, ["rerun"])

    assert result.exit_code != 0
    assert "is not built" in result.output


def test_gen_and_run_compose_the_model_pipeline(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("MOTION_SPEC_GEN", str(tmp_path))  # `latest` belongs to this tree, not the user's
    model = tmp_path / "demo.robmot"
    model.write_text("")
    received = {}

    def create_generation(_model, output):
        output.mkdir()
        return output

    def generate(_model, generation, *, stage):
        received.setdefault("stages", []).append(stage)
        generated = generation / "generated"
        (generated / "model").mkdir(parents=True)
        # `run` reads the platform back from here to decide whether simulator options apply.
        (generated / "model" / "ir.json").write_text(
            json.dumps({"configuration": {"platform": {"simulated": True}}})
        )
        return generated

    def build(generation, *, prefixes, jobs):
        received["build"] = (generation, prefixes, jobs)
        executable = generation / "build" / "main"
        executable.parent.mkdir()
        executable.touch()
        return executable

    monkeypatch.setattr("motion_spec.generation.pipeline.create_generation_dir", create_generation)
    monkeypatch.setattr("motion_spec.generation.pipeline.generate_model", generate)
    monkeypatch.setattr("motion_spec.generation.pipeline.build_generation", build)
    monkeypatch.setattr("motion_spec.generation.pipeline.new_id", lambda name: f"{name}-1")
    monkeypatch.setattr(
        "motion_spec.introspection.runner.run_cataloged",
        lambda *args, **kwargs: received.update(run=(args, kwargs)) or 0,
    )

    ir_generation = tmp_path / "ir-generation"
    result = CliRunner().invoke(main, ["gen", "ir", str(model), "-o", str(ir_generation)])
    assert result.exit_code == 0
    assert received["stages"] == ["ir"]

    run_generation = tmp_path / "run-generation"
    result = CliRunner().invoke(
        main, ["run", str(model), "-o", str(run_generation), "--headless", "--steps", "10"]
    )
    assert result.exit_code == 0
    assert received["stages"] == ["ir", "code"]
    assert received["run"][1]["executable_args"] == ["--headless", "--steps", "10"]
    assert str(run_generation / "runs" / "run-1") in result.output


def test_generation_base_prefers_o_then_the_environment(monkeypatch, tmp_path) -> None:
    """-o wins over $MOTION_SPEC_GEN, which wins over the working-directory fallback."""
    monkeypatch.chdir(tmp_path)  # the unset case points `latest` at the working directory
    model = tmp_path / "demo.robmot"
    model.write_text("")
    received = {}

    def create_generation(_model, output):
        received["output"] = output
        generation = output or tmp_path / "fallback"
        generation.mkdir(exist_ok=True)
        return generation

    monkeypatch.setattr("motion_spec.generation.pipeline.create_generation_dir", create_generation)
    monkeypatch.setattr(
        "motion_spec.generation.pipeline.generate_model",
        lambda _model, generation, *, stage: generation / "generated",
    )

    explicit, configured = tmp_path / "explicit", tmp_path / "configured"
    monkeypatch.setenv("MOTION_SPEC_GEN", str(configured))
    assert CliRunner().invoke(main, ["gen", "ir", str(model), "-o", str(explicit)]).exit_code == 0
    assert received["output"] == explicit

    result = CliRunner().invoke(main, ["gen", "ir", str(model)])
    assert result.exit_code == 0
    assert received["output"] == configured
    assert "MOTION_SPEC_GEN is not set" not in result.stderr
    # Where it is goes out before the DSL and the compiler bury it in their own output.
    assert result.stderr.splitlines()[0] == f"generation: {configured}"

    # Unset, the library keeps deciding: the CLI passes no base and says where things will land.
    monkeypatch.delenv("MOTION_SPEC_GEN")
    result = CliRunner().invoke(main, ["gen", "ir", str(model)])
    assert result.exit_code == 0
    assert received["output"] is None
    assert "MOTION_SPEC_GEN is not set" in result.stderr
    # The generation path stays the only thing on stdout, for a caller reading it.
    assert result.stdout.strip() == str(tmp_path / "fallback")


def test_install_uses_package_extras(monkeypatch) -> None:
    received = {}
    monkeypatch.setattr(
        "motion_spec.cli.subprocess.run",
        lambda args: received.update(args=args) or SimpleNamespace(returncode=0),
    )

    result = CliRunner().invoke(main, ["install", "validation"])

    assert result.exit_code == 0
    assert received["args"][-1] == "motion_spec[validation]"


def test_setup_installs_stst_in_prefix(monkeypatch, tmp_path) -> None:
    received = {}
    monkeypatch.setattr(
        "motion_spec.setup.install_stst",
        lambda prefix: received.update(prefix=prefix) or prefix / "bin" / "stst",
    )

    result = CliRunner().invoke(main, ["setup", "--prefix", str(tmp_path)])

    assert result.exit_code == 0
    assert received["prefix"] == tmp_path
    assert result.output.strip() == str(tmp_path / "bin" / "stst")

    monkeypatch.setattr("motion_spec.setup.remove_stst", lambda prefix: prefix == tmp_path)
    result = CliRunner().invoke(main, ["setup", "--prefix", str(tmp_path), "--clean"])
    assert result.exit_code == 0
    assert result.output.strip() == "removed"


def test_stst_setup_builds_pinned_launcher_once(monkeypatch, tmp_path) -> None:
    calls = []
    monkeypatch.setattr(stst_setup.shutil, "which", lambda command: f"/usr/bin/{command}")

    def run(args, check):
        calls.append(args)
        if args[1] == "clone":
            (Path(args[-1]) / ".git").mkdir(parents=True)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(stst_setup.subprocess, "run", run)
    monkeypatch.setattr(
        stst_setup, "urlretrieve", lambda _url, path: Path(path).write_bytes(b"jar")
    )

    launcher = stst_setup.install_stst(tmp_path)
    first_call_count = len(calls)

    assert launcher.is_file()
    assert "jjs.stst.STStandaloneTool" in launcher.read_text()
    assert stst_setup.STST_REF in calls[2]
    assert stst_setup.install_stst(tmp_path) == launcher
    assert len(calls) == first_call_count
    assert stst_setup.remove_stst(tmp_path) is True
    assert not launcher.exists()
    assert not (tmp_path / "share" / "motion-spec").exists()
    assert stst_setup.remove_stst(tmp_path) is False


def test_health_is_profile_scoped() -> None:
    result = CliRunner().invoke(main, ["health", "--profile", "validation"])

    assert result.exit_code == 0
    assert "motion-spec health" in result.output
    assert "BASE — required for every installation" in result.output
    assert "rdflib     Python module" in result.output
    assert "VALIDATION — required for validation" in result.output
    assert "pyshacl    Python module" in result.output
    assert "stst" not in result.output
    assert "Summary: 4 good, 0 missing" in result.output


def test_managed_stst_wins_over_path(monkeypatch, tmp_path) -> None:
    managed = tmp_path / "bin" / "stst"
    managed.parent.mkdir()
    managed.touch()
    monkeypatch.setattr(stst_setup, "DEFAULT_PREFIX", tmp_path)
    monkeypatch.setattr(stst_setup.shutil, "which", lambda _command: "/old/venv/bin/stst")

    assert stst_setup.find_stst() == str(managed)
