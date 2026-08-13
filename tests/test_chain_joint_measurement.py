# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A joint-position measurement on a chain joint compiles.

The simulated read has a fallback for a joint the simulator does not answer by name, and it is
reached only when the joint is on the chain -- every maintained model measures a gripper mimic
instead, which renders the other arm. So the branch ships in each generated controller and no
model compiles it, which is how it once carried an expression naming a struct that does not
exist in the scope it renders into.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest
from conftest import requires_workspace

MODELS = Path(__file__).resolve().parents[2] / "motion-spec-dsl" / "models"
MODEL = "pick_place_single"

WORLD_ANCHOR = """    world {
"""
# A joint the chain articulates, so the measurement resolves to a joint index and takes the
# fallback arm rather than the one that throws.
CHAIN_JOINT = """    world {
        joint-position chain-joint-coverage {
            joint: <kinova.joint_4>,
            normalization: (-pi, pi) rad
        },
"""


@requires_workspace
def test_a_chain_joint_measurement_generates_and_compiles(tmp_path: Path) -> None:
    if shutil.which("cmake") is None:
        pytest.skip("no cmake")
    source = MODELS / MODEL / f"{MODEL}.robmot"
    text = source.read_text()
    assert WORLD_ANCHOR in text, "the model's world block is no longer where this patches it"

    # Beside the original, so its relative imports still resolve; removed however this ends.
    patched = source.with_name(f"{MODEL}__chain_joint_coverage.robmot")
    patched.write_text(text.replace(WORLD_ANCHOR, CHAIN_JOINT, 1))
    try:
        generation = tmp_path / "gen"
        subprocess.run(
            ["motion-spec", "gen", "code", patched.name, "-o", str(generation)],
            cwd=patched.parent,
            check=True,
        )
        built = next(generation.glob(f"{patched.stem}/*"))
        emitted = (built / "generated" / "controller" / "main.cpp").read_text()
        # The reading is taken from the robot, which is what the scope it renders into has.
        assert "jnt_pos_msr" in emitted
        assert "normalize_joint_position" in emitted

        build = subprocess.run(
            ["motion-spec", "build", str(built)], capture_output=True, text=True, check=False
        )
        assert build.returncode == 0, build.stdout + build.stderr
        assert (built / "build" / "main").is_file()
    finally:
        patched.unlink(missing_ok=True)
