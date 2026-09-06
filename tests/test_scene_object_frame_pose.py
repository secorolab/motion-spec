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
import subprocess
from pathlib import Path

import pytest

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.rdf_parser.ir import generate_ir

from conftest import requires_workspace

MODEL = Path(__file__).parent / "fixtures" / "shared_motion"
SCENE = Path(__file__).parents[2] / "motion-spec-dsl" / "models" / "admittance_arc_single"

pytestmark = requires_workspace(SCENE)


@pytest.fixture(scope="module")
def generated(tmp_path_factory) -> dict:
    """The lowered IR of the shared-motion fixture."""
    outdir = tmp_path_factory.mktemp("shared_motion") / "generated" / "model"
    subprocess.run(
        ["textx", "generate", "shared_motion.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=MODEL,
        check=True,
    )
    manifest = outdir / "shared_motion-app.ld.json"
    return json.loads(json.dumps(generate_ir(manifest), cls=DataclassJSONEncoder))


@pytest.fixture(scope="module")
def world_output(generated) -> dict:
    """Every world observation the first handler's arm solver makes, by observation id."""
    solver = generated["resources"]["by_id"]["arm_solver_handler_first"]
    return {out["id"]: out for out in solver["world_output"]}


def test_a_pose_of_a_scene_object_is_a_frame_of_the_world_model(world_output):
    """A pose of the object itself is a pose of the frame its root body stands at, so what the
    observation names is a Frame carrying that frame's own IRI."""
    of = world_output["pose_table_top"]["of"]
    assert of["type"] == "Frame"
    assert of["uri"].endswith("table_top")


def test_a_pose_of_a_frame_the_object_carries_is_that_frames_own_segment(world_output):
    """The centre of mass is a frame the scene places on the table, so it is a segment of the
    world tree and resolves to itself -- not to a marker minted for it."""
    of = world_output["pose_table_com"]["of"]
    assert of["type"] == "Frame"
    assert of["id"] == "table_com"


def test_no_observation_carries_a_scene_object(generated):
    """Nothing in the published IR is a scene object any more; every endpoint is a frame."""
    assert "SceneObject" not in json.dumps(generated)


def test_a_frame_off_the_chains_branch_records_its_trees_root(generated):
    """The joints placing a frame the chain root is not above run from that tree's root, so the
    world read has to say which root to require the path from. A frame on the chain says none.
    """
    frames = generated["resources"]["world_frames"]
    by_id = {frame["frame_id"]: frame for frame in frames}
    assert by_id["table_com"]["tree_root"]
    assert by_id["bracelet_link"]["tree_root"] is None
