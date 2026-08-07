# SPDX-License-Identifier: MPL-2.0
"""robot.toml is keyed by config_key. schema:identifier is retired, so ir.py derives that key
from the graph instead of reading it -- these pin the derivation against the checked-in TOMLs."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

from motion_spec.rdf_parser.ir import generate_ir

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"
REAL_WORLD_MODELS = ["real_demo", "real_demo_2f85", "real_demo_separate"]


def _generate_ir(name: str, tmp_path: Path) -> dict:
    model_dir = MODELS / name
    outdir = tmp_path / "generated" / "model"
    subprocess.run(
        ["textx", "generate", f"{name}.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=model_dir,
        check=True,
    )
    return generate_ir(outdir / f"{name}-app.ld.json")


def _config_keys(ir: dict) -> set[str]:
    return {
        device["config_key"]
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        for device in solver.devices
        if device.get("config_key")
    }


def _toml_sections(name: str) -> set[str]:
    return set(
        re.findall(r"^\[([^]]+)\]", (MODELS / name / "robot.toml").read_text(), re.MULTILINE)
    )


@pytest.fixture(scope="module")
def real_demo_ir(tmp_path_factory) -> dict:
    return _generate_ir("real_demo", tmp_path_factory.mktemp("real_demo"))


def test_the_agent_key_is_its_scenex_alias_then_its_leaf(real_demo_ir: dict) -> None:
    """The alias survives in the agent's IRI as the path segment right after /models/."""
    assert "agents.arm1" in _config_keys(real_demo_ir)


def test_a_hosted_sensors_key_is_its_agents_leaf_then_its_own(real_demo_ir: dict) -> None:
    """Not runtime_prefix: that is empty on every single-robot model, so the sensor's own agent
    -- not the scene's dedup prefix -- is what makes the key line up with robot.toml."""
    assert "arm1.wrist_ft" in _config_keys(real_demo_ir)


@pytest.fixture(scope="module")
def real_demo_separate_ir(tmp_path_factory) -> dict:
    return _generate_ir("real_demo_separate", tmp_path_factory.mktemp("real_demo_separate"))


def test_two_agents_in_one_model_get_distinct_keys(real_demo_separate_ir: dict) -> None:
    assert {"agents.arm1", "agents.gripper1"} <= _config_keys(real_demo_separate_ir)


@pytest.mark.parametrize("name", REAL_WORLD_MODELS)
def test_every_derived_key_matches_the_configs_sections(name: str, tmp_path_factory) -> None:
    """The regression test that matters: runner.py:169-188 fails a run in both directions, a
    missing section and an unbound one, so this must be an exact set match."""
    ir = _generate_ir(name, tmp_path_factory.mktemp(name))
    assert _config_keys(ir) == _toml_sections(name)
