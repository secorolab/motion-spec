# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Entering a motion is a fresh start, so everything it accumulates over one activation has to be
re-armed there -- an accumulator that survives being left makes the next activation read the last
one's state."""

from __future__ import annotations

import json

import pytest

from motion_spec.generation.codegen import render_template
from motion_spec.setup import find_stst
from tests.conftest import requires_stst

pytestmark = requires_stst()

MOTION = {
    "motion": {
        "id": "motion_probe",
        "fsm_state": "S_PROBE",
        "relative_poses": [{"id": "rp_tool", "fk_pose_id": "pose_tool"}],
        "controllers": [],
        "when_monitors": [],
        "while_monitors": [],
        "until_monitors": [],
        "action_clients": [],
        "path_projections": [],
    }
}


def test_a_relative_pose_recaptures_its_origin_when_the_motion_is_re_entered(tmp_path) -> None:
    # The origin is captured once per activation, behind `_start_captured`. Left set, a re-entered
    # motion measures from where the arm was the first time it ran and the controller drives to a
    # target displaced by everything that happened in between.
    payload = tmp_path / "motion.json"
    payload.write_text(json.dumps(MOTION))
    rendered = tmp_path / "step.cpp"
    render_template(
        find_stst() or "stst",
        "fsm-step-function",
        payload,
        rendered,
        module_template="assembly_coordination",
    )
    step = rendered.read_text()
    entry = step[: step.index("update_motion_probe")]
    assert "motion_probe_state_instance.rp_tool_start_captured = false;" in entry
