# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A world pose stated of a scene object, or of one of the frames that object carries.

The simulator's scene answers where a scene object is, so such a pose must never be asked of the
arm's forward kinematics -- no chain reaches a table. Which marker answers it is the object's own
body when the pose is of the body's own frame, and the site the scene already marks when the pose
is of another frame the object carries.

The `shared_motion` fixture states all three: the table's root frame, a second frame the table
carries, and a frame on the arm. A resolution that matched names rather than reading the graph --
a root recognised only when named `<body>_origin`, or a site invented for every frame -- answers
wrongly on one of them.
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
def world_output(tmp_path_factory) -> dict:
    """Every world observation the first handler's arm solver makes, by observation id."""
    outdir = tmp_path_factory.mktemp("shared_motion") / "generated" / "model"
    subprocess.run(
        ["textx", "generate", "shared_motion.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=MODEL,
        check=True,
    )
    manifest = outdir / "shared_motion-app.ld.json"
    ir = json.loads(json.dumps(generate_ir(manifest), cls=DataclassJSONEncoder))
    solver = ir["resources"]["by_id"]["arm_solver_handler_first"]
    return {out["id"]: out for out in solver["world_output"]}


def test_a_pose_of_a_frame_on_a_scene_object_reads_the_site_marking_that_frame(world_output):
    """The centre of mass is a frame on the table, and the scene already marks it, so that site
    is what the pose is read off -- not the arm's kinematics, which no table is on."""
    of = world_output["pose_table_com"]["of"]
    assert of["type"] == "SceneObject"
    assert of["is_scene_object"] is True
    assert of["id"] == "table"
    assert of["site"] == "table_com"


def test_the_body_named_is_the_one_the_scene_compiled(world_output):
    """The runtime looks the body up by the name the scene mapped it to, which is the simulator's
    own spelling and need not be the id the model generated."""
    assert world_output["pose_table_com"]["of"]["body"] == "table"


def test_a_pose_of_the_objects_own_frame_names_no_site(world_output):
    """The body's own frame is the body: it is answered by the body pose, so there is no site to
    mark and none is named."""
    of = world_output["pose_table_top"]["of"]
    assert of["is_scene_object"] is True
    assert of["body"] == "table"
    assert of["site"] is None


def test_a_pose_of_a_chain_frame_is_no_scene_object(world_output):
    """The bracelet is on the arm, so it stays a chain-resolved frame and gains no scene marker
    -- the site field is only ever set by a frame the scene marks."""
    of = world_output["pose_bracelet_base"]["of"]
    assert of.get("site", "") in ("", None)
    assert of["is_scene_object"] is False
