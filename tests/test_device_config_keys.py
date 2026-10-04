# SPDX-License-Identifier: MPL-2.0
"""robot.toml is keyed by config_key, which the IR derives from the graph: a key/section mismatch
only shows on hardware, so the derivation is checked against the shipped TOMLs."""

from __future__ import annotations

import re

import pytest
from motion_spec_dsl.langs import motion_spec_metamodel
from support import EXAMPLES, load_model

from motion_spec.rdf_parser.ir import generate_ir


@pytest.mark.parametrize("name", ["real_arm_pose_hold", "real_gripper_cycle_ft_monitoring"])
def test_every_derived_key_matches_the_configs_sections(name: str, tmp_path) -> None:
    """The runner fails a run in both directions, a missing section and an unbound one, so this
    must be an exact set match. [ros.*] configures publishers, not a device the run binds."""
    ir = generate_ir(
        *load_model(
            motion_spec_metamodel().model_from_file(str(EXAMPLES[name] / f"{name}.robmot")),
            tmp_path / "generated" / "model",
        )
    )
    keys = {
        device.config_key
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        for device in solver.devices
        if device.config_key
    }
    found = re.findall(r"^\[([^]]+)\]", (EXAMPLES[name] / "robot.toml").read_text(), re.MULTILINE)
    assert keys == {key for key in found if key.split(".")[0] != "ros"}
