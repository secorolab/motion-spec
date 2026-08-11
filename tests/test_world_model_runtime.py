# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the one world model promises, checked against the code that ships.

The claims are numerical and about C++ lifetimes -- that a forest-wide tree pass reproduces KDL's
own chain FK, that a stale read cannot pass, and that the valid cycle allocates nothing -- so the
real `runtime_header` is rendered and compiled, and the fixture is run in several modes rather
than split into a C++ case per accessor.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from motion_spec.generation.codegen import render_template
from motion_spec.setup import find_stst

TESTS = Path(__file__).resolve().parent
FIXTURE = TESTS / "world_model_fixture.cpp"
INSTALL = TESTS.parents[2] / "install"
KDL = INSTALL / "orocos_kdl"


def _render_runtime_header(tmp_path: Path) -> Path:
    """The shipped runtime header, over the smallest payload that renders it."""
    stst = find_stst()
    if stst is None:
        pytest.skip("no stst; run `motion-spec setup`")
    payload = tmp_path / "ir.json"
    payload.write_text(
        json.dumps(
            {"configuration": {"control_period_ns": 1000000}, "resources": {"device_kinds": {}}}
        )
    )
    header = tmp_path / "runtime.hpp"
    render_template(stst, "runtime_header", payload, header)
    return header


def _build(tmp_path: Path, *extra: str) -> Path:
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    if not (KDL / "include" / "kdl" / "tree.hpp").is_file():
        pytest.skip("orocos_kdl is not built in this workspace")
    _render_runtime_header(tmp_path)
    binary = tmp_path / "world_model"
    subprocess.run(
        [
            compiler,
            "-std=c++20",
            "-O3",
            "-DNDEBUG",
            *extra,
            f"-I{tmp_path}",
            f"-I{KDL / 'include'}",
            "-I/usr/include/eigen3",
            str(FIXTURE),
            "-o",
            str(binary),
            f"-L{KDL / 'lib'}",
            "-lorocos-kdl",
            f"-Wl,-rpath,{KDL / 'lib'}",
        ],
        check=True,
    )
    return binary


def _run(binary: Path, mode: str) -> str:
    done = subprocess.run([str(binary), mode], capture_output=True, text=True, check=False)
    print(done.stdout)
    assert done.returncode == 0, done.stdout + done.stderr
    return done.stdout


def test_the_world_model_matches_chain_fk_and_refuses_a_stale_read(tmp_path: Path) -> None:
    """Two arms in one tree plus a second tree, read through one object.

    Every required pose is compared against `ChainFkSolverPos_recursive` on the chain sliced from
    the same tree, and every startup and freshness refusal is executed -- a safety path that
    cannot be triggered is not a safety path.
    """
    _run(_build(tmp_path), "verify")


@pytest.mark.parametrize("eigen_guard", [(), ("-DEIGEN_RUNTIME_NO_MALLOC",)])
def test_the_valid_hot_path_allocates_nothing(tmp_path: Path, eigen_guard: tuple) -> None:
    """Counted only after every buffer has been sized, so what is measured is the cycle."""
    _run(_build(tmp_path, *eigen_guard), "alloc")


@pytest.mark.skipif(
    os.environ.get("PLAN05_WORLD_BENCH") != "1", reason="set PLAN05_WORLD_BENCH=1 to measure"
)
def test_one_control_cycle_of_reads_is_cheaper_through_the_world_model(tmp_path: Path) -> None:
    """Report-only: a wall-clock ratio is not a pass/fail a test suite should carry.

    The unit is one control cycle's reads, not one read: the tree pass computes every segment
    once, so a motion with a single low-index read can legitimately be slower in isolation.
    """
    _run(_build(tmp_path), "bench")
