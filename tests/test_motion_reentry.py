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

pytestmark = pytest.mark.skipif(find_stst() is None, reason="no stst; run `motion-spec setup`")

MOTION = {
    "motion": {
        "id": "motion_probe",
        "fsm_state": "S_PROBE",
        "entry_snapshots": [{"target_id": "pose_tool_start", "source_id": "pose_tool"}],
        "controllers": [],
        "when_monitors": [],
        "while_monitors": [],
        "until_monitors": [],
        "action_clients": [],
        "path_projections": [],
    }
}


def test_an_entry_snapshot_recaptures_when_the_motion_is_re_entered(tmp_path) -> None:
    # The origin is captured once per activation, behind `snapshot_taken`. Left set, a re-entered
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
    assert "motion_probe_state_instance.snapshot_taken = false;" in entry
