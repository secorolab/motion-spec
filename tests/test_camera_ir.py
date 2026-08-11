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

from conftest import requires_workspace

from motion_spec.rdf_parser.ir import generate_ir

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"

pytestmark = requires_workspace(MODELS)


def _cameras(name, tmp_path):
    """`composition.scene.cameras` for a model, lowered through the real IR pipeline."""
    outdir = tmp_path / "generated" / "model"
    subprocess.run(
        ["textx", "generate", f"{name}.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=MODELS / name,
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
