# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""What the CLI and setup must never do: trash sources, run an unbounded build, read a malformed
config, or write generations where nobody asked."""

import subprocess
from unittest import mock

import pytest
from click.testing import CliRunner

from motion_spec import setup as stst_setup
from motion_spec.cli import main


def test_clean_trashes_the_whole_output_tree_and_never_sources_or_generations(
    monkeypatch, tmp_path
) -> None:
    for directory in ("build", "install", "log"):
        (tmp_path / directory / "inside").mkdir(parents=True)
    (tmp_path / "setup-motion-spec.bash").write_text("x\n")
    (tmp_path / "src" / "coord2b").mkdir(parents=True)
    (tmp_path / "generations").mkdir()
    trashed = []
    monkeypatch.setattr(stst_setup, "trash", trashed.append)

    result = CliRunner().invoke(main, ["setup", "--workspace", str(tmp_path), "--clean"])

    assert result.exit_code == 0, result.output
    assert [path.name for path in trashed] == ["build", "install", "log", "setup-motion-spec.bash"]
    assert (tmp_path / "src" / "coord2b").is_dir() and (tmp_path / "generations").is_dir()
    named = CliRunner().invoke(main, ["setup", "coord2b", "--workspace", str(tmp_path), "--clean"])
    assert named.exit_code != 0 and "takes no names" in named.output


def test_a_build_job_count_is_never_unlimited_and_bounded_by_memory(monkeypatch, tmp_path) -> None:
    """A bare `cmake --build --parallel` is `make -j`, which swaps the machine to death."""
    checkout = tmp_path / "src" / "coord2b"
    (checkout / ".git").mkdir(parents=True)
    package = stst_setup.Package("coord2b", checkout, True, False)
    state = stst_setup.SourceState(checkout, True, ref="c" * 40)
    tee = mock.Mock()
    monkeypatch.setattr(stst_setup, "tee", tee)
    monkeypatch.setattr(stst_setup, "extension_options", mock.Mock(return_value=()))

    stst_setup.install_package(package, state, tmp_path, tmp_path / "python")
    build = next(call.args[0] for call in tee.call_args_list if "--build" in call.args[0])
    assert int(build[build.index("--parallel") + 1]) >= 1

    monkeypatch.setattr(stst_setup, "usable_cores", mock.Mock(return_value=32))
    monkeypatch.setattr(stst_setup, "total_memory", mock.Mock(return_value=8 * 1024**3))
    assert stst_setup.build_jobs() == 4
    # Never zero on a machine too small to hold one job.
    monkeypatch.setattr(stst_setup, "total_memory", mock.Mock(return_value=1024**3))
    assert stst_setup.build_jobs() == 1


def test_generations_go_where_named_and_never_into_the_working_directory(monkeypatch, tmp_path):
    """-o wins over $MOTION_SPEC_GEN, which wins over the workspace; with none, it is refused."""
    monkeypatch.chdir(tmp_path)
    model = tmp_path / "demo.robmot"
    model.write_text("")
    (tmp_path / "generation").mkdir()
    create_generation = mock.Mock(return_value=tmp_path / "generation")
    monkeypatch.setattr("motion_spec.generation.pipeline.create_generation_dir", create_generation)
    monkeypatch.setattr(
        "motion_spec.generation.pipeline.generate_model",
        mock.Mock(return_value=tmp_path / "generation" / "generated"),
    )
    explicit, configured = tmp_path / "explicit", tmp_path / "configured"
    monkeypatch.setenv(stst_setup.GENERATION_VARIABLE, str(configured))
    assert CliRunner().invoke(main, ["gen", "ir", str(model), "-o", str(explicit)]).exit_code == 0
    assert create_generation.call_args.args[1] == explicit
    assert CliRunner().invoke(main, ["gen", "ir", str(model)]).exit_code == 0
    assert create_generation.call_args.args[1] == configured

    monkeypatch.delenv(stst_setup.GENERATION_VARIABLE)
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    result = CliRunner().invoke(main, ["gen", "ir", str(model)])
    assert result.exit_code != 0 and stst_setup.WORKSPACE_VARIABLE in result.output
    with pytest.raises(RuntimeError, match=stst_setup.GENERATION_VARIABLE):
        stst_setup.generations_root()
    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    assert stst_setup.generations_root() == tmp_path / "generations"


def test_a_failing_tool_fails_the_command_that_ran_it(tmp_path) -> None:
    from motion_spec.utils import tee

    with pytest.raises(subprocess.CalledProcessError) as raised:
        tee(["bash", "-c", "exit 3"], log=tmp_path / "setup.log")
    assert raised.value.returncode == 3


def test_a_config_is_refused_with_an_unknown_key_or_a_newer_version(monkeypatch, tmp_path):
    """A key nobody reads is a setting that silently does nothing; a newer file is not half-read."""
    from motion_spec import config, formats

    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    monkeypatch.chdir(tmp_path)
    CliRunner().invoke(main, ["config", "--init", "--workspace", str(tmp_path)])
    written = tmp_path / config.CONFIG_FILE
    current = formats.FORMATS["config"].current
    assert f"\nversion = {current}\n" in written.read_text()

    written.write_text(written.read_text().replace(f"version = {current}", "version = 9"))
    with pytest.raises(ValueError, match="version 9") as raised:
        config.load(written)
    assert "reads 1" in str(raised.value)

    for text, message in (
        ("[nonsense]\nkey = 1\n", r"unknown key key in \[nonsense\]"),
        ('[setup]\nbuild_type = "Debug"\n', r"unknown key build_type in \[setup\]"),
    ):
        written.write_text(text)
        with pytest.raises(ValueError, match=message):
            config.load(written)
