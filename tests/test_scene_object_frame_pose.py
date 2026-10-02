# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A world pose stated of a scene object, or of one of the frames that object carries.

An object is placed in the scene through the asset that models it, so where it is is where that
asset's root body stands -- a frame of the one world model like any other, not something a
simulator answers by name. The `shared_motion` fixture states all three cases: the table's root
frame, a second frame the table carries, and a frame on the arm.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.rdf_parser.ir import generate_ir

from conftest import requires_workspace
from support import example, load_model

MODEL = Path(__file__).parent / "fixtures" / "shared_motion"
SCENE = example("arc_tracing_with_admittance")

pytestmark = requires_workspace(SCENE)


@pytest.fixture(scope="module")
def generated(tmp_path_factory) -> dict:
    """The lowered IR of the shared-motion fixture."""
    outdir = tmp_path_factory.mktemp("shared_motion") / "generated" / "model"
    ir = generate_ir(*load_model(MODEL / "shared_motion.robmot", outdir))
    return json.loads(json.dumps(ir, cls=DataclassJSONEncoder))


@pytest.fixture(scope="module")
def world_output(generated) -> dict:
    """Every world observation the first handler's arm solver makes, by observation id."""
    solver = generated["resources"]["by_id"]["arm_solver_handler_first"]
    return {out["id"]: out for out in solver["world_output"]}


def test_a_pose_of_a_scene_object_is_a_frame_of_the_world_model(world_output):
    """A pose of the object's root frame is a pose of the segment the body stands at, which the
    world tree names after the body."""
    of = world_output["shared_world_pose_table_top"]["of"]
    assert of["type"] == "Frame"
    assert of["id"] == "table"


def test_a_pose_of_a_frame_the_object_carries_is_that_frames_own_segment(world_output):
    """The centre of mass is a frame the scene places on the table, so it is a segment of the
    world tree and resolves to itself -- not to a marker minted for it."""
    of = world_output["shared_world_pose_table_com"]["of"]
    assert of["type"] == "Frame"
    assert of["id"] == "table_com"


def test_no_observation_carries_a_scene_object(generated):
    """Nothing in the published IR is a scene object any more; every endpoint is a frame."""
    assert '"type": "SceneObject"' not in json.dumps(generated)


def test_a_frame_off_the_chains_branch_records_its_trees_root(generated):
    """The joints placing a frame the chain root is not above run from that tree's root, so the
    world read has to say which root to require the path from. A frame on the chain says none.
    """
    frames = generated["resources"]["world_frames"]
    by_id = {frame["frame_id"]: frame for frame in frames}
    assert by_id["table_com"]["tree_root"]
    assert by_id["bracelet_link"]["tree_root"] is None
