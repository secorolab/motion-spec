# SPDX-License-Identifier: MPL-2.0
"""Where a scene's model file is, as the run looks for it: `model_file` on each kind of path."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import scene_dsl
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.rdf_parser.model import model_file

SCENE = """ns n="https://example.test/"
obj set (ns=n) objects { object box }
agn set (ns=n) agents { agent robot }
scene (ns=n) s { obj set <objects> agn set <agents> }
"""

SCENEX = """import "s.scene"
ns x="https://example.test/models/"
scene inst (ns=x) si {
    scene: <s>
    model relative as mjcf { sys path = "src/mj_kdl_wrapper/assets/my table.xml" }
    model absolute as mjcf { sys path = "/opt/assets/table.xml" }
    model in-python as mjcf { py pkg = "scene_dsl", path = "assets/table.xml" }
    model in-ros as mjcf { ros pkg = "some_pkg", path = "assets/table.xml" }
    model no-file as mjcf { py { module: some.module, attr: table } }
}
"""


@pytest.fixture
def models(tmp_path: Path):
    """The scene graph, and each model's IRI by its name."""
    (tmp_path / "s.scene").write_text(SCENE)
    (tmp_path / "s.scenex").write_text(SCENEX)
    model = scenex_metamodel().model_from_file(str(tmp_path / "s.scenex"))
    graph = create_scenex_model_graph(model)
    return graph, {element.name: element.uri for element in model.scene_insts[0].models}


def test_a_relative_path_stays_as_authored(models) -> None:
    # The run searches for it from the working directory upwards and in the asset cache;
    # scene-dsl alone would read it against the .scenex.
    graph, uris = models
    assert model_file(graph, uris["relative"]) == "src/mj_kdl_wrapper/assets/my table.xml"


def test_absolute_and_package_paths_are_located(models) -> None:
    graph, uris = models
    assert model_file(graph, uris["absolute"]) == "/opt/assets/table.xml"
    in_python = Path(model_file(graph, uris["in-python"]))
    assert in_python == Path(scene_dsl.__file__).parent / "assets" / "table.xml"


def test_a_model_without_a_file_has_none(models) -> None:
    graph, uris = models
    assert model_file(graph, uris["no-file"]) is None


def test_a_ros_package_without_ros_says_to_source_it(models, monkeypatch) -> None:
    graph, uris = models
    # None in sys.modules fails the import, whether or not ROS is installed here.
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", None)
    with pytest.raises(RuntimeError, match="source /opt/ros"):
        model_file(graph, uris["in-ros"])
