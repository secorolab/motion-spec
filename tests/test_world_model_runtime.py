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
import shutil
import subprocess
from pathlib import Path

import pytest

from motion_spec.generation.codegen import render_template
from motion_spec.setup import find_stst

TESTS = Path(__file__).resolve().parent
FIXTURE = TESTS / "world_model_fixture.cpp"
KDL = TESTS.parents[2] / "install" / "orocos_kdl"
COMPILER = shutil.which("g++") or shutil.which("c++")
STST = find_stst()

# The smallest payload that renders the shipped runtime header: the world model is rendered only
# for a program that declares a scene tree, and the solver runtime only for one that drives a
# chain with the algorithm it uses.
PAYLOAD = {
    "configuration": {"control_period_ns": 1000000, "backend": "mj_kdl"},
    "resources": {
        "device_kinds": {},
        "world_trees": [{"name": "tree", "cpp_name": "tree", "sampled_frames": []}],
        "by_kind": {"serial_chain": [{"id": "arm"}]},
    },
    "computation": {"uses": {"SolverRNE": True}},
}

pytestmark = [
    pytest.mark.skipif(STST is None, reason="no stst; run `motion-spec setup`"),
    pytest.mark.skipif(COMPILER is None, reason="no C++ compiler"),
    pytest.mark.skipif(
        not (KDL / "include" / "kdl" / "tree.hpp").is_file(),
        reason="orocos_kdl is not built in this workspace",
    ),
]


# verify: two arms in one tree plus a second tree, read through one object. Every required pose
# is compared against `ChainFkSolverPos_recursive` on the chain sliced from the same tree, and
# every startup and freshness refusal is executed -- a safety path that cannot be triggered is
# not a safety path. alloc: counted only after every buffer has been sized, so what is measured
# is the cycle.
@pytest.mark.parametrize(
    ("mode", "flags"),
    [("verify", ()), ("alloc", ()), ("alloc", ("-DEIGEN_RUNTIME_NO_MALLOC",))],
    ids=["matches-chain-fk-and-refuses-a-stale-read", "allocates-nothing", "eigen-no-malloc"],
)
def test_the_world_model_fixture_passes(tmp_path: Path, mode: str, flags: tuple) -> None:
    payload = tmp_path / "ir.json"
    payload.write_text(json.dumps(PAYLOAD))
    render_template(STST, "runtime_header", payload, tmp_path / "runtime.hpp")
    binary = tmp_path / "world_model"
    subprocess.run(
        [
            COMPILER,
            "-std=c++20",
            "-O3",
            "-DNDEBUG",
            *flags,
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
    done = subprocess.run([str(binary), mode], capture_output=True, text=True, check=False)
    assert done.returncode == 0, done.stdout + done.stderr
