# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What a scenex-declared camera lowers to, and what a scene without one lowers to.

The camera is the scene's, not a solver's: it renders against the composed scene, so it lowers
onto `composition.scene`. A model that declares none must lower none, or codegen emits a
publisher for a camera the compiled scene cannot render.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
from conftest import requires_workspace
from textx.exceptions import TextXSemanticError

from motion_spec.rdf_parser.ir import generate_ir

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"
SUBSCRIPTION = """        wrist-view: topic "/wrist/color" message "sensor_msgs/msg/Image" {
            observes { <pick_place_scene_mjc.wrist> }
        },
"""

pytestmark = requires_workspace(MODELS)


FIXTURES = Path(__file__).parent / "fixtures"


def _cameras(name, tmp_path, model_dir=None):
    """`composition.scene.cameras` for a model, lowered through the real IR pipeline."""
    outdir = tmp_path / "generated" / "model"
    subprocess.run(
        ["textx", "generate", f"{name}.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=model_dir or MODELS / name,
        check=True,
    )
    return generate_ir(outdir / f"{name}-app.ld.json")["composition"]["scene"].cameras


def test_declared_camera_lowers_with_its_authored_render_settings(tmp_path):
    """pick_place_single authors one 640x480 rgb camera at 30 Hz on the wrist."""
    cameras = _cameras("pick_place_single", tmp_path)

    assert len(cameras) == 1
    camera = cameras[0]
    assert (camera.id, camera.width, camera.height, camera.rate_hz) == ("wrist", 640, 480, 30.0)
    assert camera.uri.endswith("/wrist")


def test_scene_without_a_camera_lowers_none(tmp_path):
    """No camera declared, no camera lowered -- and so no camera code to generate."""
    assert _cameras("pick_place_dual", tmp_path) == []


def test_a_camera_nothing_subscribes_to_has_no_provider(tmp_path):
    """pick_place_single renders its camera and publishes it nowhere: a viewer has no channel
    to read, and must not invent one from the camera's name."""
    camera = _cameras("pick_place_single", tmp_path)[0]

    assert camera.topic is None
    assert camera.message is None


def test_a_subscribed_camera_carries_the_channel_the_model_states(tmp_path):
    """The subscription naming the camera is where its topic comes from -- the whole point of
    stating it, rather than building '/<id>/color' at the far end."""
    camera = _cameras("camera_view", tmp_path, model_dir=FIXTURES / "camera_view")[0]

    assert camera.topic == "/wrist/color"
    assert camera.message == "sensor_msgs/msg/Image"


def _parse(source: str):
    """Parse fixture source as if it sat beside the fixture, so its imports resolve."""
    from motion_spec_dsl.langs import motion_spec_metamodel

    return motion_spec_metamodel().model_from_str(
        source, file_name=str(FIXTURES / "camera_view" / "camera_view.robmot")
    )


def _fixture_source() -> str:
    return (FIXTURES / "camera_view" / "camera_view.robmot").read_text()


def test_a_camera_carried_by_two_channels_is_rejected():
    """Two topics for one camera leaves a viewer choosing, which is the guess this replaced."""
    source = _fixture_source().replace(
        SUBSCRIPTION,
        SUBSCRIPTION
        + SUBSCRIPTION.replace("wrist-view", "wrist-view-2").replace("/wrist/color", "/wrist/rgb"),
        1,
    )
    with pytest.raises(TextXSemanticError, match="one provider"):
        _parse(source)


def test_a_camera_subscription_stating_a_pose_path_is_rejected():
    """An image holds no pose, so the clause that says where a pose sits has nothing to point
    at -- and the grammar tells the two kinds of channel apart by exactly that clause."""
    source = _fixture_source().replace(
        "            observes { <pick_place_scene_mjc.wrist> }\n",
        "            observes { <pick_place_scene_mjc.wrist> }\n            pose from results\n",
        1,
    )
    # The clause sends it down the pose alternative, where a camera is not a world quantity.
    with pytest.raises(TextXSemanticError, match="WorldQuantity"):
        _parse(source)
