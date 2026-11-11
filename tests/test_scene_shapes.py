# SPDX-License-Identifier: MPL-2.0
"""Scene-authored shapes: one fixed, massless SceneObject each, on its body or on the world.

Read from a copy of the `scene_shapes` fixture, which draws an STL marker on the static world
body and an ellipsoid, a cylinder and a cube on a free body resting on the table.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec import view
from motion_spec.rdf_parser.agents import robot_setups
from motion_spec.rdf_parser.scene import read_scene
from motion_spec.rdf_parser.shapes import read_shapes

FIXTURE = Path(__file__).parent / "fixtures" / "scene_shapes"
SCENEX = "kinova_with_objects.scenex"
# Shapes are unnamed: scene-dsl names each after its frame and its place on that frame.
MARKER = "static_marker_frame-shape-0"
BALL = "sphere_center-shape-0"
CYLINDER = "cylinder_center-shape-0"
CUBE = "cube_center-shape-0"


@pytest.fixture
def example(tmp_path: Path) -> Path:
    """A copy of the fixture, so a test may edit it; its mesh keeps its relative place."""
    shutil.copytree(FIXTURE, tmp_path / "scene_shapes")
    return tmp_path / "scene_shapes" / SCENEX


def _scene(path: Path):
    model = view.scenex_model(path)
    scene = read_scene(model, robot_setups(model)[2])
    return model, scene


def test_each_shape_is_one_object_on_its_body_or_the_world(example: Path) -> None:
    model, scene = _scene(example)
    shapes = {shape.name: shape for shape in read_shapes(model, scene)}

    assert {name: shape.shape for name, shape in shapes.items()} == {
        MARKER: "MESH",
        BALL: "ELLIPSOID",
        CYLINDER: "CYLINDER",
        CUBE: "BOX",
    }
    # The static world body is built by nothing else: its shape stands in the world.
    marker = shapes[MARKER]
    assert (marker.attach_kind, marker.attach_name) == ("World", "")
    assert marker.pos == pytest.approx([0.65, -0.4, 0.7])
    assert marker.size == pytest.approx([0.05, 0.05, 0.05])  # scale 5 in cm
    assert Path(marker.path).name == "tetra.stl"
    # No colour stated: MuJoCo's default, not one made up here.
    assert marker.rgba is None
    for name in (BALL, CYLINDER, CUBE):
        assert shapes[name].attach_kind == "Body"
        assert shapes[name].rgba is not None
        assert shapes[name].path is None
    assert shapes[CYLINDER].size == pytest.approx([0.05, 0.045, 0.0])  # radius, half height


def test_a_world_body_frame_is_a_site_on_the_world(example: Path) -> None:
    _model, scene = _scene(example)
    [site] = [frame for frame in scene.frames if frame.name == "static_marker_frame"]
    assert site.body == "world"
    assert [site.pos_x, site.pos_y, site.pos_z] == pytest.approx([0.65, -0.4, 0.7])


def test_an_elliptical_cylinder_is_refused(example: Path) -> None:
    source = example.read_text()
    assert "scale: (50, 50, 90)" in source
    example.write_text(source.replace("scale: (50, 50, 90)", "scale: (50, 40, 90)"))
    model, scene = _scene(example)
    with pytest.raises(ConstraintViolation, match="elliptical cylinder"):
        read_shapes(model, scene)


def test_shapes_build_massless_and_colliding(example: Path) -> None:
    mjk = pytest.importorskip("mj_kdl_wrapper")
    mujoco = pytest.importorskip("mujoco")
    model, scene = _scene(example)
    spec = view.build_spec(scene, example)
    view.add_shapes(spec, model, scene, example)
    env = mjk.Env.build(spec)
    try:
        m = env.model
        for name in (MARKER, BALL, CYLINDER, CUBE):
            geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{name}_geom")
            assert geom >= 0, name
            assert m.body_mass[m.geom_bodyid[geom]] == 0.0
            assert m.geom_contype[geom] != 0
    finally:
        env.close()


def _with_a_second_marker_frame(example: Path) -> None:
    """Give box1_body a frame named like the world body's marker frame, with a shape on it."""
    source = example.read_text()
    frame = "            frame cube_center {"
    shape = "            shape sphere {"
    assert frame in source and shape in source
    source = source.replace(
        frame,
        """            frame static_marker_frame {
                pose second_marker_offset {
                    wrt: <box1_root>
                    xyz: (0.0, -0.18, 0.02) m
                    orientation: euler { angles: (0.0, 0.0, 0.0) unit: rad }
                }
            }
"""
        + frame,
    ).replace(shape, "            shape cube { frame: <static_marker_frame> unit: cm scale: 3 }\n" + shape)
    example.write_text(source)


def test_shapes_named_alike_carry_their_body(example: Path) -> None:
    _with_a_second_marker_frame(example)
    model, scene = _scene(example)
    names = {shape.name for shape in read_shapes(model, scene)}
    # Only the names that repeat take their body; the rest stay as they are.
    assert names == {
        f"world_body_{MARKER}", f"box1_body_{MARKER}", BALL, CYLINDER, CUBE,
    }


def test_shapes_named_alike_build(example: Path) -> None:
    mjk = pytest.importorskip("mj_kdl_wrapper")
    pytest.importorskip("mujoco")
    _with_a_second_marker_frame(example)
    model, scene = _scene(example)
    spec = view.build_spec(scene, example)
    view.add_shapes(spec, model, scene, example)
    env = mjk.Env.build(spec)
    env.close()
