# SPDX-License-Identifier: MPL-2.0
"""Motion-spec's adapter over scene-dsl's renderer-ready KDL IR."""

from pathlib import Path

from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.generation.scene_kdl import chain_for_iri, kdl_header_name, write_scene_kdl_header

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"


def test_scene_kdl_adapter_derives_solver_chains_and_writes_header(tmp_path: Path) -> None:
    scene = MODELS / "pick_place_single" / "pick_place_single.scenex"
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(scene))

    trees = build_kdl_trees(graph, scene.parent, strict_inertia=False)
    chain = next(chain for tree in trees for chain in tree["chains"])
    assert chain_for_iri(trees, chain["iri"])[2] == [f"joint_{number}" for number in range(1, 8)]

    header = write_scene_kdl_header(graph, tmp_path, scene.name, scene.parent)
    assert header.name == "pick_place_single.kdl.hpp"
    assert kdl_header_name("pick_place_single-app.ld.json") == header.name
    assert "make_chain_kinova_2f85_chain" in header.read_text()
