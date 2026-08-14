# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A world pose stated of a scene object, or of one of the frames that object carries.

The simulator's scene answers where a scene object is, so such a pose must never be asked of the
arm's forward kinematics -- no chain reaches a drawer. Which marker answers it is the object's
own body when the pose is of the object, and the site the scene already marks when the pose is of
one of its other frames.

`collab_sim` is the case that pins this: its object is `drawer`, its body `drawer-body` and its
root `drawer-root`, so a resolution that matched names rather than reading the graph -- the body
id spelled with a dash against a frame id spelled with an underscore, or a root recognised only
when named `<body>_origin` -- answers wrongly on every one of the three.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.rdf_parser.ir import generate_ir

from conftest import requires_workspace

MODEL = Path(__file__).parents[2] / "bdd_collab_bhv_cpp" / "models" / "collab_sim"
ROBMOT = MODEL / "collab_sim.robmot"
# The pose is what these cases are about, so a sibling checkout predating it has nothing to say
# here and skips, the same as a checkout missing the model altogether.
STATES_THE_POSE = ROBMOT.exists() and "pose-handle-base" in ROBMOT.read_text()

pytestmark = [
    requires_workspace(MODEL),
    pytest.mark.skipif(
        not STATES_THE_POSE, reason=f"{ROBMOT} states no pose of a scene object's frame"
    ),
]


@pytest.fixture(scope="module")
def world_output(tmp_path_factory) -> dict:
    """Every world observation the wait handler's arm solver makes, by observation id."""
    outdir = tmp_path_factory.mktemp("collab_sim") / "generated" / "model"
    subprocess.run(
        ["textx", "generate", "collab_sim.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=MODEL,
        check=True,
    )
    manifest = outdir / "collab_sim-app.ld.json"
    ir = json.loads(json.dumps(generate_ir(manifest), cls=DataclassJSONEncoder))
    solver = ir["resources"]["by_id"]["arm_solver_handler_wait"]
    return {out["id"]: out for out in solver["world_output"]}


def test_a_pose_of_a_frame_on_a_scene_object_reads_the_site_marking_that_frame(world_output):
    """The handle is a frame on the drawer, and the scene already marks it, so that site is
    what the pose is read off -- not the arm's kinematics, which no drawer is on."""
    of = world_output["pose_handle_base"]["of"]
    assert of["type"] == "SceneObject"
    assert of["is_scene_object"] is True
    assert of["id"] == "drawer"
    assert of["site"] == "drawer-handle"


def test_the_body_named_is_the_one_the_scene_compiled_dashes_and_all(world_output):
    """The runtime looks the body up by the name MuJoCo compiled it under, which is the scene's
    own spelling; the generated id spells the same name with underscores and would not match."""
    assert world_output["pose_handle_base"]["of"]["body"] == "drawer-body"


def test_a_pose_of_the_object_itself_names_no_site(world_output):
    """The elbow is on the arm, so it stays a chain-resolved frame and gains no scene marker --
    the site field is only ever set by a frame the scene marks."""
    assert world_output["pose_elbow_base"]["of"].get("site", "") == ""
    assert world_output["pose_elbow_base"]["of"]["is_scene_object"] is False
