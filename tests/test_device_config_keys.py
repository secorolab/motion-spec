# SPDX-License-Identifier: MPL-2.0
"""robot.toml is keyed by config_key. schema:identifier is retired, so ir.py derives that key
from the graph instead of reading it -- these pin the derivation against the checked-in TOMLs."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from support import DSL_MODELS, example, load_model

from motion_spec.rdf_parser.ir import generate_ir

MODELS = DSL_MODELS

from conftest import requires_workspace

pytestmark = requires_workspace(MODELS)
REAL_WORLD_MODELS = ["real_arm_pose_hold", "real_gripper_cycle_ft_monitoring"]


def _generate_ir(name: str, tmp_path: Path) -> dict:
    model, fsm = load_model(example(name) / f"{name}.robmot", tmp_path / "generated" / "model")
    return generate_ir(model, fsm)


def _config_keys(ir: dict) -> set[str]:
    return {
        device.config_key
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        for device in solver.devices
        if device.config_key
    }


def _toml_sections(name: str) -> set[str]:
    """The device sections, the same cut runner.py makes: [ros.*] configures the generated
    publishers, not a device the run binds, so no derived key ever answers to it."""
    found = re.findall(r"^\[([^]]+)\]", (example(name) / "robot.toml").read_text(), re.MULTILINE)
    return {key for key in found if key.split(".")[0] != "ros"}


@pytest.fixture(scope="module")
def one_agent_ir(tmp_path_factory) -> dict:
    return _generate_ir("real_arm_pose_hold", tmp_path_factory.mktemp("real_arm_pose_hold"))


def test_the_agent_key_is_its_scenex_alias_then_its_leaf(one_agent_ir: dict) -> None:
    """The alias survives in the agent's IRI as the path segment right after /models/."""
    assert "agents.arm1" in _config_keys(one_agent_ir)


def test_a_hosted_sensors_key_is_its_agents_leaf_then_its_own(one_agent_ir: dict) -> None:
    """Not runtime_prefix: that is empty on every single-robot model, so the sensor's own agent
    -- not the scene's dedup prefix -- is what makes the key line up with robot.toml."""
    assert "arm1.wrist_ft" in _config_keys(one_agent_ir)


@pytest.fixture(scope="module")
def two_agent_ir(tmp_path_factory) -> dict:
    return _generate_ir("real_gripper_cycle_ft_monitoring", tmp_path_factory.mktemp("real_gripper_cycle_ft_monitoring"))


def test_two_agents_in_one_model_get_distinct_keys(two_agent_ir: dict) -> None:
    assert {"agents.arm1", "agents.gripper1"} <= _config_keys(two_agent_ir)


@pytest.mark.parametrize("name", REAL_WORLD_MODELS)
def test_every_derived_key_matches_the_configs_sections(name: str, tmp_path_factory) -> None:
    """The regression test that matters: runner.py:169-188 fails a run in both directions, a
    missing section and an unbound one, so this must be an exact set match."""
    ir = _generate_ir(name, tmp_path_factory.mktemp(name))
    assert _config_keys(ir) == _toml_sections(name)
