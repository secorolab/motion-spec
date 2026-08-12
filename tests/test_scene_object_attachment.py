# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A robot mounted on a scene object is held by a site of the asset, so a frame the asset states
no site for is rejected while generating rather than when the runtime fails to assemble the scene.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.scene import MjcfSceneObject, MjcfSceneRobot, MjcfSceneSpec
from motion_spec.rdf_parser.resources import _validate_object_attachments

ASSET = """<mujoco model="table_asset">
  <worldbody>
    <body name="root">
      <site name="table_top" pos="0 0 0" size="0.001"/>
    </body>
  </worldbody>
</mujoco>
"""


def _scene(asset: Path, frame: str) -> MjcfSceneSpec:
    scene = MjcfSceneSpec()
    scene.objects.append(
        MjcfSceneObject(id="table-robot", body="robot-table-body", path=str(asset))
    )
    scene.robots.append(
        MjcfSceneRobot(
            id="kinova", path="arm.xml", attach_kind="Site", attach_name=f"robot-table-body_{frame}"
        )
    )
    return scene


@pytest.fixture
def asset(tmp_path) -> Path:
    path = tmp_path / "table.xml"
    path.write_text(ASSET)
    return path


def test_a_site_the_asset_states_is_accepted(asset) -> None:
    _validate_object_attachments(_scene(asset, "table_top"), asset)


def test_a_site_the_asset_lacks_is_rejected(asset) -> None:
    with pytest.raises(ConstraintViolation, match="states no site for"):
        _validate_object_attachments(_scene(asset, "top-center"), asset)
