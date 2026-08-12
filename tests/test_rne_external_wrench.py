# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The frame and sign an RNE solver reads an external wrench in.

The claim is numerical and about KDL's own conventions -- ACHD and RNEA disagree on both the
frame `f_ext[i]` is stated in and the sign it enters the joint torque with -- so the sequence
`solver-assign-f-ext-RNE` emits is run against KDL rather than matched in the generated text.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

TESTS = Path(__file__).resolve().parent
FIXTURE = TESTS / "rne_external_wrench_fixture.cpp"
KDL = TESTS.parents[2] / "install" / "orocos_kdl"


def test_an_rne_solver_is_handed_the_wrench_the_model_generates(tmp_path: Path) -> None:
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    if not (KDL / "include" / "kdl" / "chain.hpp").is_file():
        pytest.skip("orocos_kdl is not built in this workspace")
    binary = tmp_path / "rne_external_wrench"
    subprocess.run(
        [
            compiler,
            "-std=c++20",
            "-O2",
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
    done = subprocess.run([str(binary)], capture_output=True, text=True, check=False)
    print(done.stdout)
    assert done.returncode == 0, done.stdout + done.stderr
