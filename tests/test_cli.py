# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

import json
import re
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
from click.testing import CliRunner

from motion_spec import config as config_module
from motion_spec import setup as stst_setup
from motion_spec.cli import main


def _declare_simulated(generation: Path) -> None:
    """The IR slice `run` reads before touching hardware: a simulated platform, nothing else."""
    model = generation / "generated" / "model"
    model.mkdir(parents=True, exist_ok=True)
    (model / "ir.json").write_text(json.dumps({"configuration": {"platform": {"simulated": True}}}))


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
    _declare_simulated(generation)
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
    _declare_simulated(generation)
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
    _declare_simulated(generation)
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
    assert f"generation {generation}" in result.output


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
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    monkeypatch.chdir(tmp_path)

    result = CliRunner().invoke(main, ["rerun"])

    # Not the working directory, and not a stack: the two variables that answer the question.
    assert result.exit_code != 0
    assert cli.GENERATION_DIR_ENV in result.output
    assert stst_setup.WORKSPACE_VARIABLE in result.output
    assert "Traceback" not in result.output


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
    monkeypatch.setenv(
        "MOTION_SPEC_GEN", str(tmp_path)
    )  # `latest` belongs to this tree, not the user's
    model = tmp_path / "demo.robmot"
    model.write_text("")
    received = {}

    def create_generation(_model, output, _name=None):
        output.mkdir()
        return output

    def generate(_model, generation, *, stage, env=None):
        received.setdefault("stages", []).append(stage)
        generated = generation / "generated"
        (generated / "model").mkdir(parents=True)
        # `run` reads the platform back from here to decide whether simulator options apply.
        (generated / "model" / "ir.json").write_text(
            json.dumps({"configuration": {"platform": {"simulated": True}}})
        )
        return generated

    def build(generation, *, prefixes, jobs, env=None):
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
        main,
        [
            "run",
            str(model),
            "-o",
            str(run_generation),
            "--headless",
            "--steps",
            "10",
            "--seed",
            "7",
        ],
    )
    assert result.exit_code == 0
    assert received["stages"] == ["ir", "code"]
    assert received["run"][1]["executable_args"] == ["--headless", "--steps", "10", "--seed", "7"]
    assert str(run_generation / "runs" / "run-1") in result.output


def test_generation_base_prefers_o_then_the_environment(monkeypatch, tmp_path) -> None:
    """-o wins over $MOTION_SPEC_GEN, which wins over the workspace default."""
    monkeypatch.chdir(tmp_path)
    model = tmp_path / "demo.robmot"
    model.write_text("")
    received = {}

    def create_generation(_model, output, _name=None):
        received["output"] = output
        generation = output or tmp_path / "fallback"
        generation.mkdir(exist_ok=True)
        return generation

    monkeypatch.setattr("motion_spec.generation.pipeline.create_generation_dir", create_generation)
    monkeypatch.setattr(
        "motion_spec.generation.pipeline.generate_model",
        lambda _model, generation, *, stage, seed=None: generation / "generated",
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
    assert result.stderr.splitlines()[0].endswith(f"generation {configured}")

    # Unset, with no workspace either: refused rather than written to the working directory.
    monkeypatch.delenv("MOTION_SPEC_GEN")
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    result = CliRunner().invoke(main, ["gen", "ir", str(model)])
    assert result.exit_code != 0
    assert stst_setup.WORKSPACE_VARIABLE in result.output

    # With an explicit destination, generation needs no workspace or latest link.
    result = CliRunner().invoke(main, ["gen", "ir", str(model), "-o", str(explicit)])
    assert result.exit_code == 0, result.output
    assert received["output"] == explicit

    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    assert CliRunner().invoke(main, ["gen", "ir", str(model)]).exit_code == 0
    assert received["output"] == tmp_path / "generations"


def test_install_uses_package_extras(monkeypatch) -> None:
    received = {}
    monkeypatch.setattr(
        "motion_spec.cli.subprocess.run",
        lambda args, **_kwargs: received.update(args=args) or SimpleNamespace(returncode=0),
    )

    result = CliRunner().invoke(main, ["install", "dashboard"])

    assert result.exit_code == 0
    assert received["args"][-1] == "motion_spec[dashboard]"


def _quiet_setup(monkeypatch, imported: frozenset = frozenset()) -> list:
    """`setup` with nothing fetched or built for real; returns the manifests it imported."""
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    monkeypatch.setattr("motion_spec.setup.missing_prerequisites", lambda *_a, **_k: ([], []))
    monkeypatch.setattr(
        "motion_spec.setup.target_environment",
        lambda root, *_a, **_k: root / ".venv" / "bin" / "python",
    )
    imports = []
    monkeypatch.setattr(
        "motion_spec.setup.import_sources",
        lambda files, _listed, _root, _log=None: imports.append(files) or set(imported),
    )
    monkeypatch.setattr(
        "motion_spec.setup.source_state",
        lambda repository, root, imported=False: stst_setup.SourceState(
            stst_setup.source_directory(root, repository.path), imported, True, ref="r" * 40
        ),
    )
    return imports


def _one_package_per_repository(monkeypatch) -> None:
    monkeypatch.setattr(
        "motion_spec.setup.discover_packages",
        lambda checkout: [stst_setup.Package(checkout.name, checkout, True, False)],
    )


def test_setup_installs_into_the_workspace_it_was_given(monkeypatch, tmp_path) -> None:
    received = {}
    _quiet_setup(monkeypatch)
    monkeypatch.setattr(
        "motion_spec.setup.install_stst",
        lambda root, state, prefix=None, force=False, **_kwargs: (
            received.update(root=root, state=state, prefix=prefix, force=force)
            or prefix / "bin" / "stst"
        ),
    )
    install = tmp_path / "install"

    result = CliRunner().invoke(main, ["setup", "STSTv4", "--workspace", str(tmp_path)])

    assert result.exit_code == 0, result.output
    # WORKSPACE/install, never the user's ~/.local: what setup installs is a toolchain.
    assert received["prefix"] == install
    assert received["force"] is False
    assert received["state"].path == tmp_path / "src" / "thirdparty" / "STSTv4"
    assert str(install / "bin" / "stst") in result.output
    # One file, for the shell in force, at the workspace root rather than in the install tree.
    written = [name for name in stst_setup.ENVIRONMENT_FILES if (tmp_path / name).is_file()]
    assert written == [f"setup-motion-spec.{config_module.shell()}"]
    assert not [name for name in stst_setup.ENVIRONMENT_FILES if (install / name).exists()]

    CliRunner().invoke(main, ["setup", "STSTv4", "--workspace", str(tmp_path), "--force"])
    assert received["force"] is True

    launcher = install / "bin" / "stst"
    monkeypatch.setattr(
        "motion_spec.setup.remove_stst",
        lambda root, prefix=None: (
            [stst_setup.Removed(str(launcher), TRASH / "stst")] if prefix == install else []
        ),
    )
    # Something for the prompt to offer: with nothing installed there is nothing to ask about.
    launcher.parent.mkdir(parents=True, exist_ok=True)
    launcher.write_text("#!/usr/bin/env bash\n")
    result = CliRunner().invoke(
        main, ["setup", "STSTv4", "--workspace", str(tmp_path), "--clean", "--yes"]
    )
    assert result.exit_code == 0
    # Each item with where the trash keeps it, so it can be found and restored.
    assert f"trashed install/bin/stst → {TRASH / 'stst'}" in result.output
    assert "cleaned: stst" in result.output


def test_setup_without_a_workspace_says_so_instead_of_choosing_one(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    # Away from any config file above the checkout, which would answer the question for it.
    monkeypatch.chdir(tmp_path)

    result = CliRunner().invoke(main, ["setup", "STSTv4"])

    assert result.exit_code != 0
    assert "set MOTION_SPEC_WS, pass --workspace" in result.output
    # A mistake in the command, not a failure inside it: no stack, no traceback.
    assert "Traceback" not in result.output

    # The variable is the first answer, so a set-up shell needs no flag at all.
    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    assert stst_setup.workspace(None) == tmp_path.resolve()
    assert stst_setup.install_prefix(tmp_path) == tmp_path / "install"


def test_setup_installs_every_repository_in_manifest_order(monkeypatch, tmp_path) -> None:
    order = []
    _quiet_setup(monkeypatch)
    _one_package_per_repository(monkeypatch)
    monkeypatch.setattr(
        "motion_spec.setup.install_stst",
        lambda root, state, prefix=None, **_kwargs: (
            order.append(("stst", None)) or prefix / "bin" / "stst"
        ),
    )
    monkeypatch.setattr(
        "motion_spec.setup.install_package",
        lambda package, _state, _root, _prefix, _python, build_type="", **_kwargs: order.append(
            (package.name, build_type)
        ),
    )

    result = CliRunner().invoke(main, ["setup", "--workspace", str(tmp_path)])

    assert result.exit_code == 0, result.output
    # The manifest's order is the build order: nothing else knows mj_kdl_wrapper links KDL.
    shipped = stst_setup.read_manifest(stst_setup.shipped(stst_setup.MANIFEST))
    expected = [
        "stst" if repository.path == stst_setup.STST_REPOSITORY else repository.name
        for repository in shipped
    ]
    assert [name for name, _ in order] == expected
    assert {build_type for name, build_type in order if name != "stst"} == {stst_setup.BUILD_TYPE}
    # The device drivers are a layer of their own, installed only with --real.
    assert not {"robif2b", "serial", "robotiq_driver_noros"} & {name for name, _ in order}
    # Sourcing it is what makes the installation usable, so setup must leave it.
    environment = tmp_path / f"setup-motion-spec.{config_module.shell()}"
    assert f"export MOTION_SPEC_PREFIX={tmp_path}" in environment.read_text()
    assert f"source {environment}" in result.output


def test_setup_asked_for_repositories_installs_only_those(monkeypatch, tmp_path) -> None:
    installed = []
    imports = _quiet_setup(monkeypatch)
    _one_package_per_repository(monkeypatch)
    monkeypatch.setattr(
        "motion_spec.setup.install_package",
        lambda package, *_a, **_k: installed.append(package.name),
    )

    # By name or by the path the manifest lists it under.
    result = CliRunner().invoke(
        main, ["setup", "mj_kdl_wrapper", "thirdparty/scene-dsl", "--workspace", str(tmp_path)]
    )

    assert result.exit_code == 0, result.output
    assert installed == ["scene-dsl", "mj_kdl_wrapper"]
    # The import is the whole manifest: vcs skips what is there, and narrowing is for building.
    assert imports == [[stst_setup.shipped(stst_setup.MANIFEST)]]

    unknown = CliRunner().invoke(main, ["setup", "no-such-repo", "--workspace", str(tmp_path)])
    assert unknown.exit_code != 0
    assert "no-such-repo" in unknown.output and "mj_kdl_wrapper" in unknown.output
    assert "Traceback" not in unknown.output


def test_setup_real_layers_the_device_drivers_on_top(monkeypatch, tmp_path) -> None:
    installed = []
    imports = _quiet_setup(monkeypatch)
    _one_package_per_repository(monkeypatch)
    monkeypatch.setattr(
        "motion_spec.setup.install_package",
        lambda package, *_a, **_k: installed.append(package.name),
    )

    result = CliRunner().invoke(main, ["setup", "robif2b", "--real", "--workspace", str(tmp_path)])

    assert result.exit_code == 0, result.output
    assert installed == ["robif2b"]
    assert imports == [
        [stst_setup.shipped(stst_setup.MANIFEST), stst_setup.shipped(stst_setup.REAL_MANIFEST)]
    ]
    # Without --real the drivers are not listed at all.
    plain = CliRunner().invoke(main, ["setup", "robif2b", "--workspace", str(tmp_path)])
    assert plain.exit_code != 0 and "robif2b" in plain.output


def test_setup_repos_replaces_the_shipped_manifest(monkeypatch, tmp_path) -> None:
    installed = []
    imports = _quiet_setup(monkeypatch)
    _one_package_per_repository(monkeypatch)
    monkeypatch.setattr(
        "motion_spec.setup.install_package",
        lambda package, *_a, **_k: installed.append(package.name),
    )
    mine = tmp_path / "mine.repos"
    mine.write_text(
        "repositories:\n  mylib:\n    type: git\n    url: https://example.org/mylib.git\n"
        "    version: main\n"
    )

    result = CliRunner().invoke(main, ["setup", "--repos", str(mine), "--workspace", str(tmp_path)])

    assert result.exit_code == 0, result.output
    assert installed == ["mylib"] and imports == [[mine]]


def test_setup_clear_cache_rebuilds_selected_cmake_package(monkeypatch, tmp_path) -> None:
    _quiet_setup(monkeypatch)
    checkout = stst_setup.source_directory(tmp_path, "coord2b")
    (checkout / ".git").mkdir(parents=True)
    (checkout / "CMakeLists.txt").write_text("project(coord2b)\n")
    build = stst_setup.build_directory(tmp_path, "coord2b")
    (build / "CMakeFiles").mkdir(parents=True)
    (build / "CMakeCache.txt").write_text("stale")
    monkeypatch.setattr(stst_setup, "package_installed", lambda *_a, **_k: True)
    monkeypatch.setattr(stst_setup, "extension_options", lambda *_a, **_k: ())
    commands = []
    monkeypatch.setattr(stst_setup, "tee", lambda command, **_k: commands.append(command))

    result = CliRunner().invoke(
        main, ["setup", "coord2b", "--workspace", str(tmp_path), "--clear-cache"]
    )

    assert result.exit_code == 0, result.output
    assert not (build / "CMakeCache.txt").exists()
    assert not (build / "CMakeFiles").exists()
    assert any("-S" in command for command in commands)

    commands.clear()
    # Installed and not asked to clear: nothing is built again.
    result = CliRunner().invoke(main, ["setup", "coord2b", "--workspace", str(tmp_path)])
    assert result.exit_code == 0 and "coord2b already installed" in result.output
    assert commands == []

    conflict = CliRunner().invoke(
        main, ["setup", "coord2b", "--workspace", str(tmp_path), "--clean", "--clear-cache"]
    )
    assert conflict.exit_code != 0
    assert "cannot be used together" in conflict.output


def test_manifest_pins_what_setup_builds_and_the_template_asks_for() -> None:
    core = stst_setup.read_manifest(stst_setup.shipped(stst_setup.MANIFEST))
    real = stst_setup.read_manifest(stst_setup.shipped(stst_setup.REAL_MANIFEST))

    # colcon must not build what thirdparty/ holds: the Python packages and the ant-built STST.
    thirdparty = [r.name for r in core if stst_setup.is_thirdparty(r)]
    assert set(thirdparty) == {
        "rec",
        "motion-spec-dsl",
        "coord-dsl",
        "scene-dsl",
        "rdf-utils",
        "STSTv4",
    }
    # rdf-utils last: every other Python package pins it by git URL, so it must win.
    python = [r.name for r in core if stst_setup.is_thirdparty(r) and r.name != "STSTv4"]
    assert python[-1] == "rdf-utils"
    assert [r.name for r in real] == ["serial", "robotiq_driver_noros", "robif2b"]
    assert all(r.url.startswith("https://") for r in (*core, *real))
    # A pin that moves without the template is an install the build then rejects.
    template = (Path(stst_setup.__file__).parent / "templates" / "entry_build.stg").read_text()
    pinned = stst_setup.shipped_pin("mj_kdl_wrapper").version.lstrip("v")
    assert f"find_package(mj_kdl_wrapper {pinned} " in template


def _manifest(path: Path, body: str) -> Path:
    path.write_text(f"repositories:\n{body}")
    return path


def test_a_manifest_is_read_in_its_order_and_refused_when_malformed(tmp_path) -> None:
    good = _manifest(
        tmp_path / "good.repos",
        "  thirdparty/b:\n    type: git\n    url: https://x/b.git\n    version: 1\n"
        "  a:\n    type: git\n    url: https://x/a.git\n    version: main\n",
    )
    repositories = stst_setup.read_manifest(good)
    assert [(r.path, r.name, r.version) for r in repositories] == [
        ("thirdparty/b", "b", "1"),
        ("a", "a", "main"),
    ]

    for body, reason in (
        ("  a:\n    type: git\n    url: https://x/a.git\n", "declares no version"),
        ("  a:\n    type: svn\n    url: https://x/a\n    version: 3\n", "only git"),
        ("  a: just-a-string\n", "not a mapping"),
    ):
        with pytest.raises(ValueError, match=reason):
            stst_setup.read_manifest(_manifest(tmp_path / "bad.repos", body))
    (tmp_path / "empty.repos").write_text("something: else\n")
    with pytest.raises(ValueError, match="no `repositories:`"):
        stst_setup.read_manifest(tmp_path / "empty.repos")


def test_manifests_layer_file_after_file_and_refuse_a_path_listed_twice(tmp_path) -> None:
    assert stst_setup.manifest_files() == [stst_setup.shipped(stst_setup.MANIFEST)]
    assert stst_setup.manifest_files(real=True)[-1] == stst_setup.shipped(stst_setup.REAL_MANIFEST)
    mine = _manifest(
        tmp_path / "mine.repos", "  a:\n    type: git\n    url: https://x/a.git\n    version: 1\n"
    )
    # A manifest given replaces the shipped core; --real still layers the drivers on top.
    assert stst_setup.manifest_files((mine,), real=True) == [
        mine,
        stst_setup.shipped(stst_setup.REAL_MANIFEST),
    ]
    twice = _manifest(
        tmp_path / "twice.repos", "  a:\n    type: git\n    url: https://y/a.git\n    version: 2\n"
    )
    with pytest.raises(ValueError, match="listed in both"):
        stst_setup.manifest_in_force([mine, twice])


def test_packages_are_found_the_way_colcon_finds_them(tmp_path) -> None:
    root_package = tmp_path / "wrapper"
    root_package.mkdir()
    (root_package / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\nproject(mj_kdl_wrapper CXX)\n"
    )
    (root_package / "pyproject.toml").write_text('[project]\nname = "mj-kdl-wrapper"\n')
    (root_package / "sub").mkdir()
    (root_package / "sub" / "CMakeLists.txt").write_text("project(ignored)\n")
    # The root is the package; its subdirectories are not looked into.
    [found] = stst_setup.discover_packages(root_package)
    assert (found.name, found.cmake, found.python) == ("mj_kdl_wrapper", True, True)

    repo = tmp_path / "orocos"
    for name, depends in (("python_orocos_kdl", "orocos_kdl"), ("orocos_kdl", "eigen")):
        (repo / name).mkdir(parents=True)
        (repo / name / "CMakeLists.txt").write_text(f"project({name})\n")
        (repo / name / "package.xml").write_text(
            f"<package><name>{name}</name><depend>{depends}</depend></package>"
        )
    (repo / "docs").mkdir()
    (repo / ".github").mkdir()
    # Dependency order, not the alphabet: python_orocos_kdl builds against orocos_kdl.
    assert [p.name for p in stst_setup.discover_packages(repo)] == [
        "orocos_kdl",
        "python_orocos_kdl",
    ]

    pure = tmp_path / "dsl"
    pure.mkdir()
    (pure / "pyproject.toml").write_text('[project]\nname = "motion_spec_dsl"\n')
    [dsl] = stst_setup.discover_packages(pure)
    assert (dsl.name, dsl.cmake, dsl.python) == ("motion_spec_dsl", False, True)

    empty = tmp_path / "empty"
    empty.mkdir()
    assert stst_setup.discover_packages(empty) == []


def test_colcon_meta_is_seeded_once_and_read_for_every_build(tmp_path) -> None:
    meta = stst_setup.workspace_colcon_meta(tmp_path)
    assert meta == tmp_path / "colcon.meta"
    assert "-DMJ_KDL_OROCOS_KDL_FROM_PACKAGE=ON" in stst_setup.cmake_arguments(
        meta, "mj_kdl_wrapper"
    )
    assert stst_setup.cmake_arguments(meta, "coord2b") == ()

    # Edited by hand from then on: setup never writes over it.
    meta.write_text(json.dumps({"names": {"robif2b": {"cmake-args": ["-DENABLE_KORTEX=ON"]}}}))
    assert stst_setup.workspace_colcon_meta(tmp_path) == meta
    assert stst_setup.cmake_arguments(meta, "robif2b") == ("-DENABLE_KORTEX=ON",)
    assert stst_setup.cmake_arguments(meta, "mj_kdl_wrapper") == ()


def test_sources_are_imported_by_vcs_into_src(monkeypatch, tmp_path) -> None:
    assert stst_setup.source_directory(tmp_path, "coord2b") == tmp_path / "src" / "coord2b"

    monkeypatch.setattr(stst_setup.shutil, "which", lambda command: f"/usr/bin/{command}")
    commands = []
    monkeypatch.setattr(stst_setup, "tee", lambda command, **_k: commands.append(command))
    files = stst_setup.manifest_files(real=True)
    listed = stst_setup.manifest_in_force(files)
    (stst_setup.source_directory(tmp_path, "coord2b") / ".git").mkdir(parents=True)

    imported = stst_setup.import_sources(files, listed, tmp_path)

    # The manifests as they are, one import each; --skip-existing leaves a checkout alone.
    assert commands == [
        ["vcs", "import", "--skip-existing", "--input", str(path), str(tmp_path / "src")]
        for path in files
    ]
    # What had no checkout before is what setup cloned, and so what it may clean.
    assert imported == {r.path for r in listed} - {"coord2b"}

    monkeypatch.setattr(stst_setup.shutil, "which", lambda _command: None)
    with pytest.raises(RuntimeError, match="vcs"):
        stst_setup.import_sources(files, listed, tmp_path)


def test_a_new_workspace_gets_its_own_environment_unless_one_is_active(monkeypatch, tmp_path):
    active = tmp_path / "active"
    monkeypatch.setenv("VIRTUAL_ENV", str(active))
    assert stst_setup.target_environment(tmp_path) == active / "bin" / "python"

    monkeypatch.delenv("VIRTUAL_ENV")
    commands, installed = [], []
    monkeypatch.setattr(stst_setup, "tee", lambda command, **_k: commands.append(command))
    monkeypatch.setattr(
        stst_setup.subprocess, "run", lambda *_a, **_k: SimpleNamespace(returncode=1)
    )
    monkeypatch.setattr(
        stst_setup,
        "_pip_install",
        lambda source, _log, editable, python: installed.append((source, editable, python)),
    )

    python = stst_setup.target_environment(tmp_path, ros=True, dev=True)

    assert python == tmp_path / ".venv" / "bin" / "python"
    # Under ROS the distribution's packages must stay reachable from it.
    assert commands[0][1:] == ["-m", "venv", "--system-site-packages", str(tmp_path / ".venv")]
    # The CLI itself goes in too, so one environment holds the whole toolchain.
    assert installed == [(stst_setup.own_source(), True, python)]


def test_stst_setup_builds_pinned_launcher_once(monkeypatch, tmp_path) -> None:
    calls = []
    monkeypatch.setattr(stst_setup.shutil, "which", lambda command: f"/usr/bin/{command}")

    def ant(args, **_kwargs):
        calls.append(args)
        jar = Path(args[-1]).parent / "build" / "jar" / "stst.jar"
        jar.parent.mkdir(parents=True, exist_ok=True)
        jar.write_bytes(b"jar")

    monkeypatch.setattr(stst_setup, "tee", ant)
    monkeypatch.setattr(
        stst_setup, "urlretrieve", lambda _url, path: Path(path).write_bytes(b"jar")
    )
    trashed = []
    monkeypatch.setattr(stst_setup, "trash", _trashing(trashed))
    source = stst_setup.source_directory(tmp_path, stst_setup.STST_REPOSITORY)
    (source / ".git").mkdir(parents=True)
    pin = stst_setup.shipped_pin(stst_setup.STST_REPOSITORY).version
    state = stst_setup.SourceState(source, True, True, ref=pin)

    launcher = stst_setup.install_stst(tmp_path, state)
    first_call_count = len(calls)

    assert launcher == tmp_path / "install" / "bin" / "stst"
    assert launcher.is_file()
    assert "jjs.stst.STStandaloneTool" in launcher.read_text()
    assert source == tmp_path / "src" / "thirdparty" / "STSTv4"
    # The commit the checkout is on, which is what the next run compares HEAD against.
    marker = tmp_path / "install" / "share" / "motion-spec" / ".stst-managed"
    assert marker.read_text().splitlines() == [pin, "cloned"]
    assert stst_setup.install_stst(tmp_path, state) == launcher
    assert len(calls) == first_call_count
    # A launcher whose jar went missing is a broken install, not a done one.
    (source / "build" / "jar" / "stst.jar").unlink()
    assert stst_setup.install_stst(tmp_path, state) == launcher
    assert len(calls) > first_call_count
    rebuilt_call_count = len(calls)
    assert stst_setup.install_stst(tmp_path, state, force=True) == launcher
    assert len(calls) > rebuilt_call_count

    # The launcher goes to the trash, the one-line marker is deleted, and the source stays.
    assert stst_setup.remove_stst(tmp_path) == [
        stst_setup.Removed(str(launcher), TRASH / "stst"),
        stst_setup.Removed(str(marker), None, deleted=True),
    ]
    assert trashed == [launcher]
    assert source.is_dir()
    assert stst_setup.remove_stst(tmp_path) == []

    def failing_ant(command, **_kwargs):
        if command[0] == "ant":
            raise RuntimeError("build failed")

    monkeypatch.setattr(stst_setup, "tee", failing_ant)
    with pytest.raises(RuntimeError, match="build failed"):
        stst_setup.install_stst(tmp_path, state, force=True)
    assert not stst_setup.stst_installed(tmp_path)

    unusable = stst_setup.SourceState(source, False, False, "was not imported")
    with pytest.raises(RuntimeError, match="was not imported"):
        stst_setup.install_stst(tmp_path / "other", unusable)


def test_health_is_profile_scoped() -> None:
    result = CliRunner().invoke(main, ["health", "--profile", "dsl"])

    assert result.exit_code == 0
    assert "motion-spec health" in result.output
    assert "BASE — required for every installation" in result.output
    # Padded to the longest name in the table, so the columns line up across profiles.
    assert f"{'rdflib':<15}  Python module" in result.output
    assert "DSL — required for DSL generation" in result.output
    assert f"{'textx':<15}  Python module" in result.output
    assert "stst" not in result.output


def test_health_gathers_apt_remedies_into_one_line(monkeypatch) -> None:
    from motion_spec.health import APT_REMEDY, HealthCheck

    checks = [
        HealthCheck("build", "cmake", "executable", None, False, f"{APT_REMEDY}cmake"),
        HealthCheck("build", "c++", "executable", None, False, f"{APT_REMEDY}build-essential"),
        HealthCheck(
            "build",
            "Protobuf",
            "CMake package",
            None,
            False,
            f"{APT_REMEDY}libprotobuf-dev protobuf-compiler",
        ),
        HealthCheck("codegen", "stst", "executable", None, False, "motion-spec setup"),
        HealthCheck(
            "ros",
            "rclcpp",
            "CMake package",
            None,
            False,
            f"{APT_REMEDY}ros-$ROS_DISTRO-rclcpp",
            optional=True,
        ),
    ]
    monkeypatch.setattr("motion_spec.health.check_health", lambda *_a, **_kw: checks)

    result = CliRunner().invoke(main, ["health"])

    assert (
        "sudo apt-get install -y cmake build-essential libprotobuf-dev protobuf-compiler"
        in result.output
    )
    # One command for everything apt answers, not one per row.
    assert APT_REMEDY not in result.output
    # A remedy apt cannot serve keeps its own line; an absent optional asks for nothing.
    assert "fix    motion-spec setup" in result.output
    assert "ros-$ROS_DISTRO-rclcpp" not in result.output


def _package(tmp_path: Path, name: str, cmake: bool = True, python: bool = False):
    checkout = tmp_path / "src" / name
    (checkout / ".git").mkdir(parents=True, exist_ok=True)
    return (
        stst_setup.Package(name, checkout, cmake, python),
        stst_setup.SourceState(checkout, True, True, ref="c" * 40),
    )


def test_a_build_is_never_given_an_unlimited_job_count(monkeypatch, tmp_path) -> None:
    """A bare `cmake --build --parallel` is `make -j`, which swaps the machine to death."""
    package, state = _package(tmp_path, "coord2b")
    prefix, python = stst_setup.install_prefix(tmp_path), tmp_path / "python"
    commands = []
    monkeypatch.setattr(stst_setup, "tee", lambda command, **_kwargs: commands.append(command))
    monkeypatch.setattr(stst_setup, "extension_options", lambda *_a, **_k: ())

    stst_setup.install_package(package, state, tmp_path, prefix, python, jobs=3)
    build = next(c for c in commands if "--build" in c)
    assert build[build.index("--parallel") + 1] == "3"

    commands.clear()
    stst_setup.install_package(package, state, tmp_path, prefix, python)
    build = next(c for c in commands if "--build" in c)
    # Whatever this machine computes, --parallel is never the last word.
    assert int(build[build.index("--parallel") + 1]) >= 1

    captured = {}

    def record(command, **kwargs):
        captured.update(kwargs)
        commands.append(command)

    monkeypatch.setattr(stst_setup, "tee", record)
    monkeypatch.setattr(stst_setup.shutil, "which", lambda command: f"/usr/bin/{command}")
    stst_setup.install_package(package, state, tmp_path, prefix, python, ros=True, jobs=2)
    assert captured["env"]["MAKEFLAGS"] == "-j2 -l2"

    commands.clear()
    stst_setup.install_package(
        package, state, tmp_path, prefix, python, ros=True, clear_cache=True, jobs=2
    )
    assert "--cmake-clean-cache" in commands[0]


def test_a_cmake_build_takes_colcon_meta_the_interpreter_and_this_runs_args(monkeypatch, tmp_path):
    package, state = _package(tmp_path, "mj_kdl_wrapper", cmake=True, python=True)
    prefix, python = stst_setup.install_prefix(tmp_path), tmp_path / "python"
    commands, pips = [], []
    monkeypatch.setattr(stst_setup, "tee", lambda command, **_kwargs: commands.append(command))
    monkeypatch.setattr(
        stst_setup, "extension_options", lambda *_a, **_k: ("-DPython3_EXECUTABLE=p",)
    )
    monkeypatch.setattr(
        stst_setup,
        "_pip_install",
        lambda source, _log, editable, py, ros, defines: pips.append(
            (source, editable, py, ros, defines)
        ),
    )
    meta = stst_setup.workspace_colcon_meta(tmp_path)
    meta.write_text(json.dumps({"names": {"mj_kdl_wrapper": {"cmake-args": ["-DFROM_META=ON"]}}}))

    stst_setup.install_package(package, state, tmp_path, prefix, python, extra=("-DONCE=1",))

    configure = next(c for c in commands if "-S" in c)
    # The same arguments colcon would read from colcon.meta, then this machine, then this run.
    assert configure[-3:] == ["-DFROM_META=ON", "-DPython3_EXECUTABLE=p", "-DONCE=1"]
    # The bindings link what the CMake build linked, into the environment setup targets.
    [(source, editable, into, ros, defines)] = pips
    assert (source, editable, into, ros) == (package.path, False, python, False)
    assert defines[:3] == ("-DFROM_META=ON", "-DPython3_EXECUTABLE=p", "-DONCE=1")
    marker = prefix / "share" / "motion-spec" / ".mj_kdl_wrapper-managed"
    assert marker.read_text().splitlines() == [state.ref, "cloned"]


def test_a_python_package_is_installed_editable_only_under_dev(monkeypatch, tmp_path) -> None:
    package, state = _package(tmp_path, "motion_spec_dsl", cmake=False, python=True)
    prefix, python = stst_setup.install_prefix(tmp_path), tmp_path / "python"
    commands = []
    monkeypatch.setattr(stst_setup, "tee", lambda command, **_kwargs: commands.append(command))
    monkeypatch.setattr(stst_setup, "installer", lambda py=None: [str(py), "-m", "pip", "install"])

    stst_setup.install_package(package, state, tmp_path, prefix, python, dev=True)
    # Into the target environment, pointing back at the checkout, and nothing built by cmake.
    assert commands == [[str(python), "-m", "pip", "install", "--editable", str(package.path)]]

    commands.clear()
    stst_setup.install_package(package, state, tmp_path, prefix, python, dev=False)
    assert commands == [[str(python), "-m", "pip", "install", str(package.path)]]


def test_a_missing_scene_asset_stops_codegen(monkeypatch, tmp_path) -> None:
    from motion_spec.generation import codegen

    monkeypatch.chdir(tmp_path)
    ir = {"resources": {"robot": {"path": "models/there.xml"}, "extras": [{"path": "gone.xml"}]}}
    (tmp_path / "models").mkdir()
    (tmp_path / "models" / "there.xml").write_text("<mujoco/>")
    assert codegen.unresolved_assets(ir) == ["gone.xml"]

    # The wrapper's own assets come from its cache, wherever the scene spells them.
    cache = tmp_path / "cache"
    (cache / "mj_kdl_wrapper" / "assets").mkdir(parents=True)
    (cache / "mj_kdl_wrapper" / "assets" / "cube.xml").write_text("<mujoco/>")
    monkeypatch.setenv("XDG_CACHE_HOME", str(cache))
    vendored = {"path": "src/mj_kdl_wrapper/assets/cube.xml"}
    assert codegen.unresolved_assets(vendored) == []


def test_the_job_count_is_bounded_by_memory_not_only_by_cores(monkeypatch) -> None:
    monkeypatch.setattr(stst_setup, "usable_cores", lambda: 32)
    monkeypatch.setattr(stst_setup, "total_memory", lambda: 8 * 1024**3)
    assert stst_setup.build_jobs() == 4
    monkeypatch.setattr(stst_setup, "total_memory", lambda: 512 * 1024**3)
    assert stst_setup.build_jobs() == 32
    # Never zero on a machine too small to hold one job.
    monkeypatch.setattr(stst_setup, "total_memory", lambda: 1024**3)
    assert stst_setup.build_jobs() == 1
    assert stst_setup.build_jobs(64) == 64


def test_package_install_is_skipped_until_its_checkout_moves(monkeypatch, tmp_path) -> None:
    checkout = tmp_path / "src" / "coord2b"
    (checkout / ".git").mkdir(parents=True)
    prefix = stst_setup.install_prefix(tmp_path)
    head = "a" * 40
    answers = {"rev-parse": head, "status": ""}
    monkeypatch.setattr(stst_setup, "_git", lambda _repository, *a: answers.get(a[0]))
    marker = prefix / "share" / "motion-spec" / ".coord2b-managed"
    marker.parent.mkdir(parents=True)
    marker.write_text(f"{head}\ncloned\n")

    assert stst_setup.package_installed("coord2b", prefix, checkout)
    # A checkout that moved is a rebuild without being asked; --force is for repairing.
    answers["rev-parse"] = "b" * 40
    assert not stst_setup.package_installed("coord2b", prefix, checkout)
    # Edits are in no commit, so nothing recorded can say the install matches them.
    answers.update({"rev-parse": head, "status": " M src/main.cpp"})
    assert not stst_setup.package_installed("coord2b", prefix, checkout)
    # Half-built is not built.
    answers["status"] = ""
    marker.write_text("installing\ncloned\n")
    assert not stst_setup.package_installed("coord2b", prefix, checkout)


def test_an_existing_checkout_is_used_at_whatever_ref_it_is_on(monkeypatch, tmp_path):
    pinned_repository = stst_setup.Repository("coord2b", "https://x/coord2b.git", "master")
    checkout = stst_setup.source_directory(tmp_path, "coord2b")
    pinned = "a" * 40
    answers = {}
    monkeypatch.setattr(
        stst_setup, "_git", lambda _repository, *arguments: answers.get(arguments[0])
    )

    missing = stst_setup.source_state(pinned_repository, tmp_path)
    assert (missing.usable, missing.reason) == (False, "was not imported")
    (checkout / ".git").mkdir(parents=True)

    # Just imported: setup's own clone, so `--clean` knows it made it.
    answers.update({"rev-parse": pinned, "status": ""})
    imported = stst_setup.source_state(pinned_repository, tmp_path, imported=True)
    assert (imported.usable, imported.cloned, imported.ref) == (True, True, pinned)

    # On the pin and clean: built where it stands, and not cloned, so `--clean` leaves it.
    monkeypatch.setattr(stst_setup, "_pinned_commit", lambda *_a: pinned)
    state = stst_setup.source_state(pinned_repository, tmp_path)
    assert (state.usable, state.cloned, state.drift) == (True, False, "")

    # Work in progress is built, since it is the whole reason to keep a checkout here.
    answers["status"] = " M src/main.cpp"
    edited = stst_setup.source_state(pinned_repository, tmp_path)
    assert edited.usable is True and "uncommitted changes" in edited.drift

    # A checkout on another commit is the answer to which version this workspace wants: it is
    # built as it stands, said out loud, and recorded as that ref rather than as the pin.
    answers.update({"status": "", "rev-parse": "b" * 40, "describe": "heads/my-feature"})
    moved = stst_setup.source_state(pinned_repository, tmp_path)
    assert moved.usable is True and moved.ref == "b" * 40
    assert "heads/my-feature" in moved.drift and "not the pinned master" in moved.drift

    # A ref the remote has never heard of is still theirs to build.
    monkeypatch.setattr(stst_setup, "_pinned_commit", lambda *_a: None)
    unknown = stst_setup.source_state(pinned_repository, tmp_path)
    assert unknown.usable is True and unknown.ref == "b" * 40


def test_clean_refuses_a_checkout_it_did_not_make(tmp_path) -> None:
    checkout = stst_setup.source_directory(tmp_path, "coord2b")
    build = stst_setup.build_directory(tmp_path, "coord2b")

    assert stst_setup.remove_package("coord2b", tmp_path) == []

    checkout.mkdir(parents=True)
    (checkout / "work-in-progress").write_text("mine")
    build.mkdir(parents=True)
    with pytest.raises(RuntimeError, match="unmanaged"):
        stst_setup.remove_package("coord2b", tmp_path)
    assert (checkout / "work-in-progress").is_file()


def test_clean_leaves_a_checkout_it_only_adopted(monkeypatch, tmp_path) -> None:
    marker = stst_setup.install_prefix(tmp_path) / "share" / "motion-spec" / ".coord2b-managed"
    marker.parent.mkdir(parents=True)
    marker.write_text("master\nadopted\n")
    stst_setup.build_directory(tmp_path, "coord2b").mkdir(parents=True)
    trashed = []
    monkeypatch.setattr(stst_setup, "trash", _trashing(trashed))

    build = stst_setup.build_directory(tmp_path, "coord2b")
    assert stst_setup.remove_package("coord2b", tmp_path) == [
        stst_setup.Removed(str(build), TRASH / "coord2b"),
        stst_setup.Removed(str(marker), None, deleted=True),
    ]

    # The build is setup's; the source it merely built in is not, whoever cleans up after it.
    assert trashed == [build]


def test_clean_asks_for_each_path_and_takes_nothing_on_a_no(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    prefix = stst_setup.install_prefix(tmp_path)
    installed = prefix / "lib" / "libcoord2b.so"
    installed.parent.mkdir(parents=True)
    installed.write_text("o" * 2048)
    marker = prefix / "share" / "motion-spec" / ".coord2b-managed"
    marker.parent.mkdir(parents=True)
    marker.write_text("master\ncloned\n")
    build = stst_setup.build_directory(tmp_path, "coord2b")
    build.mkdir(parents=True)
    (build / "install_manifest.txt").write_text(f"{installed}\n")
    checkout = stst_setup.source_directory(tmp_path, "coord2b")
    (checkout / ".git").mkdir(parents=True)
    # Clean finds the package names in the checkout, as colcon would.
    (checkout / "CMakeLists.txt").write_text("project(coord2b)\n")
    trashed = []
    monkeypatch.setattr(stst_setup, "trash", _trashing(trashed))

    refused = CliRunner().invoke(
        main, ["setup", "coord2b", "--workspace", str(tmp_path), "--clean"], input="n\n"
    )

    # Every path it would take, with its size, before the question -- and a no takes none.
    assert "build/coord2b" in refused.output and "1 installed file" in refused.output
    assert "2.0 KiB" in refused.output
    assert trashed == [] and installed.is_file() and build.is_dir()

    accepted = CliRunner().invoke(
        main, ["setup", "coord2b", "--workspace", str(tmp_path), "--clean"], input="y\n"
    )

    assert accepted.exit_code == 0, accepted.output
    assert not installed.exists() and not build.exists()
    # Each item said with where the trash keeps it; the marker is deleted, and says so.
    assert f"trashed 1 installed files from install → {TRASH / '.removed-coord2b-install'}" in (
        accepted.output
    )
    assert f"trashed build/coord2b → {TRASH / 'coord2b'}" in accepted.output
    assert "deleted install/share/motion-spec/.coord2b-managed" in accepted.output
    # The one thing a clean never takes, whoever cloned it.
    assert (checkout / ".git").is_dir()
    assert "sources left in place: src/coord2b" in accepted.output


def test_clean_all_offers_the_whole_output_tree(monkeypatch, tmp_path) -> None:
    for directory in ("build", "install", "log"):
        (tmp_path / directory / "inside").mkdir(parents=True)
    (tmp_path / "setup-motion-spec.bash").write_text("x\n")
    (tmp_path / "src" / "coord2b").mkdir(parents=True)
    (tmp_path / "generations").mkdir()
    trashed = []
    monkeypatch.setattr("motion_spec.utils.trash", _trashing(trashed))

    result = CliRunner().invoke(
        main, ["setup", "--workspace", str(tmp_path), "--clean", "--all", "--yes"]
    )

    assert result.exit_code == 0, result.output
    assert [path.name for path in trashed] == ["build", "install", "log", "setup-motion-spec.bash"]
    assert f"trashed build → {TRASH / 'build'}" in result.output
    # Sources and generations are not outputs, so --all never offers them.
    assert (tmp_path / "src" / "coord2b").is_dir() and (tmp_path / "generations").is_dir()

    named = CliRunner().invoke(
        main, ["setup", "coord2b", "--workspace", str(tmp_path), "--clean", "--all"]
    )
    assert named.exit_code != 0 and "takes no repositories" in named.output


def test_clean_only_removes_manifest_files_inside_prefix(monkeypatch, tmp_path) -> None:
    prefix = tmp_path / "install"
    owned = prefix / "lib" / "owned.so"
    owned.parent.mkdir(parents=True)
    owned.write_text("owned")
    outside = tmp_path / "outside.so"
    outside.write_text("keep")
    manifest = tmp_path / "install_manifest.txt"
    manifest.write_text(f"{owned}\n{outside}\n{prefix / '..' / outside.name}\n")
    monkeypatch.setattr(stst_setup, "trash", _trashing([]))
    removed = []

    stst_setup._trash_installed(manifest, prefix, "test", removed)

    assert not owned.exists()
    assert outside.read_text() == "keep"
    # One trash entry for all of them, reported with where the trash keeps it.
    assert removed == [
        stst_setup.Removed(f"1 installed files from {prefix}", TRASH / ".removed-test")
    ]


TRASH = Path("/trash/files")


def _trashing(recorded: list) -> object:
    """A stand-in for the desktop trash: records what it took, takes it, and says where."""

    def trash(path: Path) -> Path:
        recorded.append(path)
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists() or path.is_symlink():
            path.unlink()
        return TRASH / path.name

    return trash


def _env_script(tmp_path: Path, name: str = "setup-motion-spec.bash", **exports: str) -> Path:
    script = tmp_path / name
    script.write_text("\n".join(f"export {key}={value}" for key, value in exports.items()) + "\n")
    return script


def test_environment_is_captured_from_the_script_a_command_will_source(monkeypatch, tmp_path):
    script = _env_script(tmp_path, MOTION_SPEC_PREFIX=str(tmp_path), FROM_SCRIPT="yes")
    monkeypatch.delenv(stst_setup.ENVIRONMENT_VARIABLE, raising=False)

    captured = stst_setup.capture_environment(script)

    assert captured["FROM_SCRIPT"] == "yes"
    assert captured["MOTION_SPEC_PREFIX"] == str(tmp_path)
    # The script and the variables a build depends on, not the operator's whole shell.
    record = stst_setup.environment_provenance(script, captured)
    assert record["script"] == str(script)
    assert record["variables"]["MOTION_SPEC_PREFIX"] == str(tmp_path)
    assert "FROM_SCRIPT" not in record["variables"]

    broken = tmp_path / "broken.bash"
    broken.write_text("echo nope >&2\nexit 3\n")
    try:
        stst_setup.capture_environment(broken)
    except RuntimeError as exc:
        assert "nope" in str(exc)
    else:
        raise AssertionError("a script that fails must say so, not yield a half-built environment")


def test_environment_discovery_prefers_the_variable_then_the_nearest_file(monkeypatch, tmp_path):
    workspace = tmp_path / "ws"
    (workspace / "generations" / "gen-1").mkdir(parents=True)
    script = _env_script(workspace)
    monkeypatch.delenv(stst_setup.ENVIRONMENT_VARIABLE, raising=False)
    monkeypatch.chdir(tmp_path)

    assert stst_setup.find_environment(workspace / "generations" / "gen-1") == script
    # Nothing above the working directory, and no variable: inherit the shell as before.
    assert stst_setup.find_environment(tmp_path) is None

    named = _env_script(tmp_path, name="other.bash")
    monkeypatch.setenv(stst_setup.ENVIRONMENT_VARIABLE, str(named))
    assert stst_setup.find_environment(workspace / "generations" / "gen-1") == named

    monkeypatch.setenv(stst_setup.ENVIRONMENT_VARIABLE, str(tmp_path / "gone.bash"))
    try:
        stst_setup.find_environment(None)
    except RuntimeError as exc:
        assert "names no file" in str(exc)
    else:
        raise AssertionError("a variable naming nothing is a mistake worth reporting")


def test_build_runs_under_the_environment_it_found(monkeypatch, tmp_path) -> None:
    generation = tmp_path / "gen-1"
    (generation / "generated" / "controller").mkdir(parents=True)
    _env_script(tmp_path, FROM_SCRIPT="yes")
    monkeypatch.delenv(stst_setup.ENVIRONMENT_VARIABLE, raising=False)
    # Away from any config above the checkout, whose own environment key would win.
    monkeypatch.chdir(tmp_path)
    received = {}
    monkeypatch.setattr(
        "motion_spec.generation.pipeline.build_generation",
        lambda generation, **kwargs: received.update(kwargs) or generation / "build" / "main",
    )

    result = CliRunner().invoke(main, ["build", str(generation)])

    assert result.exit_code == 0, result.output
    assert received["env"]["FROM_SCRIPT"] == "yes"

    # --no-env is the way back to inheriting this shell, whatever files sit above.
    received.clear()
    result = CliRunner().invoke(main, ["build", str(generation), "--no-env"])
    assert result.exit_code == 0, result.output
    assert received["env"] is None


def test_a_run_records_the_environment_it_happened_in() -> None:
    from motion_spec.introspection.provenance import host_info

    assert "environment" not in host_info()
    record = {"script": "/ws/setup-motion-spec.bash", "variables": {"ROS_DISTRO": "jazzy"}}
    assert host_info(record)["environment"] == record


def _ros_root(tmp_path: Path, *distros: str) -> Path:
    """A /opt/ros stand-in holding the named distributions."""
    for distro in distros:
        (tmp_path / distro).mkdir(parents=True)
        (tmp_path / distro / "setup.bash").touch()
    return tmp_path


def test_health_names_the_installed_ros_distributions(monkeypatch, tmp_path) -> None:
    from motion_spec import health

    monkeypatch.setattr(health, "ROS_ROOT", _ros_root(tmp_path, "jazzy", "rolling"))
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    monkeypatch.delenv("ROS_VERSION", raising=False)
    # The ROS CMake probes configure a project apiece; this test is about what is reported.
    monkeypatch.setattr(health, "_cmake_package_path", lambda *_a, **_kw: None)

    # Absent and absent read the same in a report; which distributions are there, and whether
    # one is sourced, is the difference between installing ROS and typing one line.
    assert health.installed_ros_distros() == ["jazzy", "rolling"]
    assert health.active_ros_distro() is None
    summary = health.ros_summary()
    assert "jazzy, rolling installed" in summary and "none sourced" in summary

    def distribution_check():
        checks = health.check_health(("ros",))
        return next(check for check in checks if check.dependency == "ROS distribution")

    distribution = distribution_check()
    assert distribution.ok is False and distribution.optional is True
    assert str(tmp_path / "jazzy") in distribution.path

    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    assert health.active_ros_distro() == "jazzy"
    assert health.ros_summary() == "jazzy sourced; rolling also installed"
    assert distribution_check().ok is True


def test_health_uses_the_environment_being_checked_for_ros(monkeypatch, tmp_path) -> None:
    from motion_spec import health

    monkeypatch.setattr(health, "ROS_ROOT", _ros_root(tmp_path, "jazzy", "rolling"))
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    monkeypatch.setattr(health, "_module_path", lambda *_args: None)
    monkeypatch.setattr(health, "_cmake_package_path", lambda *_args, **_kwargs: None)

    checks = health.check_health(("ros",), env={"ROS_DISTRO": "rolling"})
    distribution = next(check for check in checks if check.dependency == "ROS distribution")
    rclcpp = next(check for check in checks if check.dependency == "rclcpp")

    assert distribution.ok and distribution.path == str(tmp_path / "rolling")
    assert "rolling sourced" in distribution.detail
    assert rclcpp.detail == "apt install ros-rolling-rclcpp"


def test_health_reports_the_variables_it_reads_and_the_sourced_distro(monkeypatch, tmp_path):
    from motion_spec import health

    monkeypatch.setattr(health, "ROS_ROOT", _ros_root(tmp_path, "jazzy", "rolling"))

    assert health.ros_summary({}).startswith("jazzy, rolling installed")
    assert health.ros_summary({"ROS_DISTRO": "jazzy", "ROS_VERSION": "2"}) == (
        "jazzy sourced (ROS 2); rolling also installed"
    )

    values = health.environment_values({"MOTION_SPEC_WS": "/ws", "ROS_DISTRO": ""})
    assert values["MOTION_SPEC_WS"] == "/ws"
    assert values["ROS_DISTRO"] is None
    assert set(values) == set(health.ENVIRONMENT_VARIABLES)

    result = CliRunner().invoke(main, ["health", "--profile", "dsl"])
    assert "ENVIRONMENT — what motion-spec reads" in result.output
    assert "MOTION_SPEC_WS" in result.output

    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    monkeypatch.setattr(
        "motion_spec.cli._environment", lambda *_args: ({"ROS_DISTRO": "rolling"}, None)
    )
    monkeypatch.setattr(
        health,
        "check_health",
        lambda *_args, **_kwargs: [
            health.HealthCheck("ros", "ROS distribution", "sourced", None, True, "")
        ],
    )
    result = CliRunner().invoke(main, ["health", "--profile", "ros"])
    assert "rolling sourced" in result.output
    assert "jazzy sourced" not in result.output


def test_health_announces_each_probe_before_it_runs(monkeypatch, tmp_path) -> None:
    from motion_spec import health

    monkeypatch.setattr(health, "_cmake_package_path", lambda *_a, **_kw: None)
    seen = []

    health.check_health(
        ("base", "codegen"), on_progress=lambda done, name: seen.append((done, name))
    )

    base = list(health.PROFILE_IMPORTS["base"])
    assert [name for _, name in seen[: len(base)]] == base
    assert "stst" in [name for _, name in seen]
    # Announced before the probe, so a slow one is named while it is running, not after.
    assert [done for done, _ in seen] == sorted(done for done, _ in seen)
    assert seen[0][0] == 0


def test_ros_remedies_name_a_distribution_this_machine_has(monkeypatch, tmp_path) -> None:
    from motion_spec import health

    monkeypatch.delenv("ROS_DISTRO", raising=False)
    # The workspace the suite runs in may name a distribution; this is about the installed ones.
    monkeypatch.setattr(health, "_configured_distro", lambda: None)
    monkeypatch.setattr(health, "ROS_ROOT", _ros_root(tmp_path / "one", "jazzy"))
    assert health._remedy("rclcpp") == "apt install ros-jazzy-rclcpp"
    assert health._remedy("rosidl_runtime_py") == "source /opt/ros/jazzy/setup.bash"

    # With two installed and none sourced there is nothing to name: the choice is the user's.
    monkeypatch.setattr(health, "ROS_ROOT", _ros_root(tmp_path / "two", "jazzy", "rolling"))
    assert health._remedy("rclcpp") == "apt install ros-$ROS_DISTRO-rclcpp"


def test_every_remedy_is_one_this_installation_can_run() -> None:
    from motion_spec import health

    named = [*health.DETAILS, *health._REMEDIES, *health.SETUP_PROVIDES]
    remedies = [health._remedy(dependency) for dependency in named]
    remedies += [health._device_remedy(target) for target in health._ROBIF2B_DEVICE_FLAGS]

    # Nothing may point at a second repository or at a workspace tool this installation does
    # not have: every remedy is apt, pip, motion-spec itself, cmake, or a ROS setup file.
    assert not [r for r in remedies if "grc" in r or "colcon build" in r or "install dsl" in r]
    # Named by the manifest entry setup takes it from, which a package need not share.
    assert health._remedy("mj_kdl_wrapper") == "motion-spec setup mj_kdl_wrapper"
    assert health._remedy("orocos_kdl") == "motion-spec setup orocos_kinematics_dynamics"
    assert health._remedy("stst") == "motion-spec setup STSTv4"
    assert health._remedy("scene_dsl") == "motion-spec setup scene-dsl"
    assert health._remedy("robif2b") == "motion-spec setup --real robif2b"
    # A device wrapper is a lasting cmake argument, so it belongs in colcon.meta.
    assert "colcon.meta" in health._device_remedy("robif2b::kinova_gen3")
    assert "ENABLE_KORTEX" in health._device_remedy("robif2b::kinova_gen3")


def test_where_a_dependency_comes_from_is_the_manifests_pin() -> None:
    from motion_spec import health

    for dependency, name in health.SETUP_PROVIDES.items():
        pinned = next(
            r
            for r in stst_setup.manifest_in_force(stst_setup.manifest_files(real=True))
            if r.name == name
        )
        assert health.DETAILS[dependency]["source"] == pinned.url.removesuffix(".git")
    assert health.mujoco_build_packages() == (
        ("mj_kdl_wrapper", stst_setup.shipped_pin("mj_kdl_wrapper").version.lstrip("v")),
    )


def test_provenance_names_the_stst_commit_the_manifest_pins() -> None:
    from motion_spec.introspection import provenance

    assert provenance._stst_commit() == stst_setup.shipped_pin(stst_setup.STST_REPOSITORY).version


def test_stst_on_path_wins_over_managed(monkeypatch, tmp_path) -> None:
    managed = stst_setup.install_prefix(tmp_path) / "bin" / "stst"
    managed.parent.mkdir(parents=True)
    managed.touch()
    # Derived from the workspace, not from a prefix variable of its own: one fact, one place.
    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    monkeypatch.setattr(stst_setup.shutil, "which", lambda _command: "/ws/bin/stst")

    assert stst_setup.find_stst() == "/ws/bin/stst"

    monkeypatch.setattr(stst_setup.shutil, "which", lambda _command: None)
    assert stst_setup.find_stst() == str(managed)

    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE)
    assert stst_setup.find_stst() is None

    other = tmp_path / "other" / "install" / "bin" / "stst"
    other.parent.mkdir(parents=True)
    other.touch()
    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    monkeypatch.setattr(stst_setup.shutil, "which", lambda _command, **_kwargs: None)
    assert stst_setup.find_stst(path="", workspace=str(tmp_path / "other")) == str(other)
    assert stst_setup.find_stst(path="") is None


def test_a_tool_is_shown_as_it_runs_and_kept_in_the_log(tmp_path) -> None:
    from motion_spec.utils import command_log, generation_log, tee

    log = tmp_path / "logs" / "setup.log"
    # `test -t 1` passes only under a pty: the tool must still believe it has a terminal, or
    # git's progress and cmake's colour quietly become their non-interactive output.
    code = tee(["bash", "-c", "echo out; echo err >&2; test -t 1 && echo tty"], log=log)

    assert code == 0
    kept = log.read_text()
    assert "out" in kept and "err" in kept and "tty" in kept
    # A stamped command heads the entry, and its outcome closes it.
    assert re.match(r"\n\[\d\d:\d\d:\d\d\] \$ bash -c", kept)
    assert re.search(r"\n\[\d\d:\d\d:\d\d\] # exit 0 after [\d.]+s\n$", kept)
    # Indented once, by the indenter alone: the file copy must not pad what is already padded.
    assert "\n  out\n" in kept

    tee(["bash", "-c", "echo second"], log=log)
    assert kept in log.read_text()  # appended, never replaced

    try:
        tee(["bash", "-c", "exit 3"], log=log)
    except subprocess.CalledProcessError as exc:
        assert exc.returncode == 3
    else:
        raise AssertionError("a tool that failed must fail the command that ran it")

    # Where each kind of log lives: the workspace's own record, and the generation it is about.
    assert command_log(tmp_path, "setup").parent == tmp_path / ".motion-spec" / "logs"
    # One console per generation: generating and building are read as the one sitting they were.
    assert generation_log(tmp_path / "gen-1") == tmp_path / "gen-1" / "logs" / "console.log"


def test_a_config_file_says_what_the_workspace_is_and_how_it_builds(monkeypatch, tmp_path):
    from motion_spec import config

    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    monkeypatch.delenv(stst_setup.GENERATION_VARIABLE, raising=False)
    monkeypatch.chdir(tmp_path)

    # Nowhere to write it: no workspace named, and no config to take one from.
    assert CliRunner().invoke(main, ["config", "--init"]).exit_code != 0

    result = CliRunner().invoke(main, ["config", "--init", "--workspace", str(tmp_path)])
    assert result.exit_code == 0
    written = tmp_path / config.CONFIG_FILE
    assert written.is_file()
    assert "already exists" in CliRunner().invoke(main, ["config", "--init"]).output

    written.write_text('[workspace]\ngenerations = "gen-out"\n\n[ros]\nworkspace = true\n')
    # No variable and no flag: the file's own directory is the workspace.
    assert stst_setup.workspace() == tmp_path.resolve()
    assert stst_setup.generations_root() == tmp_path / "gen-out"

    settings, path = config.settings()
    assert path == written
    assert settings["ros"]["workspace"] is True

    # An environment variable still wins over the file.
    monkeypatch.setenv(stst_setup.GENERATION_VARIABLE, str(tmp_path / "elsewhere"))
    assert stst_setup.generations_root() == tmp_path / "elsewhere"

    written.write_text("[nonsense]\nkey = 1\n")
    try:
        config.settings()
    except ValueError as exc:
        assert "unknown section" in str(exc)
    else:
        raise AssertionError("a key nobody reads is a setting that silently does nothing")

    # What setup installs is the manifests and its flags; a [setup] table would be ignored.
    written.write_text('[setup]\nbuild_type = "Debug"\n')
    with pytest.raises(ValueError, match=r"unknown section \[setup\]"):
        config.settings()


def test_the_sample_sets_the_core_keys_and_the_shell_it_found(monkeypatch, tmp_path) -> None:
    from motion_spec import config

    monkeypatch.setenv("SHELL", "/usr/bin/zsh")
    monkeypatch.delenv(stst_setup.ENVIRONMENT_VARIABLE, raising=False)
    # Away from the checkout, whose own workspace has these files.
    monkeypatch.chdir(tmp_path)
    assert config.detect_shell() == "zsh"
    monkeypatch.setenv("SHELL", "/bin/fish")
    assert config.detect_shell() == "bash"

    monkeypatch.setenv("SHELL", "/usr/bin/zsh")
    CliRunner().invoke(main, ["config", "--init", "--workspace", str(tmp_path)])
    written = (tmp_path / config.CONFIG_FILE).read_text()
    for live in (
        f'root = "{tmp_path}"',
        'generations = "generations"',
        'environment = "setup-motion-spec.zsh"',
        'shell = "zsh"',
    ):
        assert f"\n{live}" in written, live
    assert "[setup]" not in written

    settings, _ = config.settings(tmp_path)
    assert config.shell(settings) == "zsh"
    # Declared but not written yet: discovery carries on rather than failing every command.
    assert stst_setup.find_environment(tmp_path) is None
    (tmp_path / "setup-motion-spec.zsh").write_text("export FROM_ZSH=yes\n")
    assert stst_setup.find_environment(tmp_path) == tmp_path / "setup-motion-spec.zsh"


def test_a_ros_workspace_builds_with_colcon_and_sources_the_overlay(monkeypatch, tmp_path):
    from motion_spec import config, health

    monkeypatch.setattr(health, "ROS_ROOT", _ros_root(tmp_path / "opt", "jazzy"))
    # This is about colcon and the overlay, not about the interpreter setup refuses to use.
    monkeypatch.setattr(health, "system_site_packages", lambda *_a, **_k: True)
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    (tmp_path / config.CONFIG_FILE).write_text('[ros]\nworkspace = true\ndistro = "jazzy"\n')
    _quiet_setup(monkeypatch)
    _one_package_per_repository(monkeypatch)
    received = {}
    monkeypatch.setattr(
        "motion_spec.setup.install_package",
        lambda package, *_a, ros=False, **_k: received.update({package.name: ros}),
    )

    result = CliRunner().invoke(main, ["setup", "coord2b", "--workspace", str(tmp_path)])

    assert result.exit_code == 0, result.output
    assert received == {"coord2b": True}
    # colcon must not wander into the Python packages and STST: they are not its to build.
    assert (tmp_path / "src" / "thirdparty" / "COLCON_IGNORE").is_file()

    # Two sourcings, not exported paths -- except the prefix's bin, which no overlay carries.
    # Each is guarded, so sourcing the file in a shell that already has them changes nothing.
    environment = tmp_path / f"setup-motion-spec.{config.shell()}"
    written = environment.read_text()
    assert '[ "${ROS_DISTRO:-}" = jazzy ] || . /opt/ros/jazzy/setup.' in written
    assert f". {tmp_path / 'install' / 'setup.'}" in written
    assert "COLCON_PREFIX_PATH" in written
    assert 'export PATH="$MOTION_SPEC_PREFIX/bin' in written
    assert "CMAKE_PREFIX_PATH" not in written
    # Sourcing is the one step to a working workspace; the CLI has to be on PATH after it.
    assert environment.stat().st_mode & 0o111


def test_a_generated_file_says_which_version_it_is(monkeypatch, tmp_path) -> None:
    from motion_spec import config, formats
    from motion_spec.introspection import journal

    monkeypatch.chdir(tmp_path)
    CliRunner().invoke(main, ["config", "--init", "--workspace", str(tmp_path)])
    written = tmp_path / config.CONFIG_FILE
    assert f"\nversion = {formats.FORMATS['config'].current}\n" in written.read_text()

    current = formats.FORMATS["config"].current
    written.write_text(written.read_text().replace(f"version = {current}", "version = 9"))
    try:
        config.settings(tmp_path)
    except ValueError as exc:
        # The range, not just a refusal: it says what this build reads and who wrote the file.
        assert "version 9" in str(exc) and "reads 1" in str(exc)
    else:
        raise AssertionError("a file from a newer motion-spec must not be half-read")

    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    monkeypatch.delenv(journal.JOURNAL_VARIABLE, raising=False)
    journal.record("health")
    assert journal.entries()[0]["v"] == formats.FORMATS["journal"].current

    path = journal.journal_path()
    path.write_text(path.read_text() + '{"v": 9, "command": "from-the-future"}\n')
    assert [entry["command"] for entry in journal.entries()] == ["health"]


def test_an_old_install_marker_is_read_as_one_setup_cloned(monkeypatch, tmp_path) -> None:
    marker = stst_setup.install_prefix(tmp_path) / "share" / "motion-spec" / ".coord2b-managed"
    marker.parent.mkdir(parents=True)
    # v1: the ref alone, from before setup could adopt a checkout it had not made.
    marker.write_text("master\n")
    stst_setup.build_directory(tmp_path, "coord2b").mkdir(parents=True)
    trashed = []
    monkeypatch.setattr(stst_setup, "trash", _trashing(trashed))

    assert stst_setup._recorded_origin(marker) == "cloned"
    assert stst_setup.remove_package("coord2b", tmp_path)

    # Cloned or not, the source is the operator's: clean takes the build only.
    assert trashed == [stst_setup.build_directory(tmp_path, "coord2b")]


def test_setup_takes_its_build_options_from_its_flags(monkeypatch, tmp_path) -> None:
    installed = []
    _quiet_setup(monkeypatch)
    _one_package_per_repository(monkeypatch)
    monkeypatch.setattr(
        "motion_spec.setup.install_package",
        lambda package, *_a, build_type="", extra=(), dev=False, **_k: installed.append(
            (package.name, build_type, extra, dev)
        ),
    )

    result = CliRunner().invoke(
        main,
        [
            "setup",
            "coord2b",
            "--workspace",
            str(tmp_path),
            "--build-type",
            "Debug",
            "--cmake-arg",
            "-DX=1",
            "--dev",
        ],
    )

    assert result.exit_code == 0, result.output
    # --cmake-arg is this run's alone; lasting arguments are colcon.meta's.
    assert installed == [("coord2b", "Debug", ("-DX=1",), True)]

    installed.clear()
    CliRunner().invoke(main, ["setup", "coord2b", "--workspace", str(tmp_path)])
    assert installed == [("coord2b", stst_setup.BUILD_TYPE, (), False)]


def test_the_journal_belongs_to_a_workspace_or_nowhere(monkeypatch, tmp_path) -> None:
    from motion_spec.introspection import journal

    monkeypatch.delenv(journal.JOURNAL_VARIABLE, raising=False)
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)

    # Outside a workspace there is nowhere it belongs, so nothing is written -- least of all
    # to the home directory, where every workspace's history would blend into one.
    assert journal.journal_path() is None
    journal.record("gen")

    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    assert journal.journal_path() == tmp_path / ".motion-spec" / "journal.jsonl"

    named = tmp_path / "elsewhere.jsonl"
    monkeypatch.setenv(journal.JOURNAL_VARIABLE, str(named))
    assert journal.journal_path() == named


def test_every_command_appends_one_entry(monkeypatch, tmp_path) -> None:
    from motion_spec.introspection import journal

    monkeypatch.delenv(journal.JOURNAL_VARIABLE, raising=False)
    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))

    monkeypatch.setattr("sys.argv", ["motion-spec", "health", "--profile", "dsl"])
    CliRunner().invoke(main, ["health", "--profile", "dsl"])
    monkeypatch.setattr("sys.argv", ["motion-spec", "setup", "stst"])
    CliRunner().invoke(main, ["setup", "stst"])  # fails on its own terms; still asked for

    recorded = journal.entries()
    assert [entry["command"] for entry in recorded] == ["health", "setup"]
    assert recorded[0]["argv"] == ["health", "--profile", "dsl"]
    assert all(entry["ts"].endswith("+00:00") and entry["cwd"] for entry in recorded)

    # And the reader prints them, newest last, without anyone opening the file.
    result = CliRunner().invoke(main, ["journal", "-n", "3"])
    assert result.exit_code == 0
    # `journal` is a command too, so reading the journal is itself the newest entry.
    assert [line.split()[1] for line in result.output.splitlines()] == [
        "health",
        "setup",
        "journal",
    ]


def test_generations_root_is_the_workspace_and_never_the_working_directory(monkeypatch, tmp_path):
    monkeypatch.delenv(stst_setup.GENERATION_VARIABLE, raising=False)
    monkeypatch.delenv(stst_setup.WORKSPACE_VARIABLE, raising=False)
    monkeypatch.chdir(tmp_path)

    # Scattering generations across wherever a command was run from is what this prevents.
    try:
        stst_setup.generations_root()
    except RuntimeError as exc:
        assert stst_setup.GENERATION_VARIABLE in str(exc)
    else:
        raise AssertionError("with neither variable set there is no generation root to use")

    monkeypatch.setenv(stst_setup.WORKSPACE_VARIABLE, str(tmp_path))
    assert stst_setup.generations_root() == tmp_path / "generations"

    # Named outright, it wins: generations need not live inside the workspace.
    monkeypatch.setenv(stst_setup.GENERATION_VARIABLE, str(tmp_path / "elsewhere"))
    assert stst_setup.generations_root() == tmp_path / "elsewhere"
