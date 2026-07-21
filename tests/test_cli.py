# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: OpenAI

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
    assert all(
        section in result.output
        for section in ("NAME", "SYNOPSIS", "DESCRIPTION", "WORKFLOW", "ARTIFACTS", "EXAMPLES", "SEE ALSO")
    )
    assert "motion-spec COMMAND --help" in result.output

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


def test_gen_and_run_compose_the_model_pipeline(monkeypatch, tmp_path) -> None:
    model = tmp_path / "demo.robmot"
    model.write_text("")
    received = {}

    def create_generation(_model, output):
        output.mkdir()
        return output

    def generate(_model, generation, *, stage):
        received.setdefault("stages", []).append(stage)
        generated = generation / "generated"
        generated.mkdir()
        return generated

    def build(generation, *, prefixes, jobs):
        received["build"] = (generation, prefixes, jobs)
        executable = generation / "build" / "main"
        executable.parent.mkdir()
        executable.touch()
        return executable

    monkeypatch.setattr("motion_spec.pipeline.create_generation_dir", create_generation)
    monkeypatch.setattr("motion_spec.pipeline.generate_model", generate)
    monkeypatch.setattr("motion_spec.pipeline.build_generation", build)
    monkeypatch.setattr("motion_spec.pipeline.new_id", lambda name: f"{name}-1")
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
        main,
        ["run", str(model), "-o", str(run_generation), "--headless", "--steps", "10"],
    )
    assert result.exit_code == 0
    assert received["stages"] == ["ir", "code"]
    assert received["run"][1]["executable_args"] == ["--headless", "--steps", "10"]
    assert str(run_generation / "runs" / "run-1") in result.output


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
        stst_setup,
        "urlretrieve",
        lambda _url, path: Path(path).write_bytes(b"jar"),
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
